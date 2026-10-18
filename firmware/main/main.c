/**
 * WiFi Tomography Node - ESP-IDF with True CSI
 * For: ESP32-C3 (Seeed XIAO), ESP32-S3, and other ESP32 variants
 *
 * Optimized for ESP-IDF v5.x with:
 * - IRAM placement for time-critical code
 * - Ring buffer for CSI data (no missed packets)
 * - Persistent sockets (avoid create/close overhead)
 * - Disabled power save for lowest latency
 * - Full binary protocol (zero malloc overhead)
 *
 * ============================================================================
 * VERSION: 5.9.1-DEPLOYABLE
 * ============================================================================
 *
 * Features:
 * - Zero-malloc binary protocol (no JSON)
 * - IRAM-placed CSI callback for lowest latency
 * - Spinlock-protected trigger state (task-context safe)
 * - Task watchdog with 30s timeout
 * - WiFi auto-reconnect with exponential backoff
 * - Batch CSI streaming (up to 12 samples/packet)
 * - CRC32 validated packets
 * - On-die temperature telemetry in every heartbeat
 * - Loud CSI/BFI capture-init failures + self-reported capture counters
 * - Runtime-switchable burst transport: UDP (default), ESP-NOW broadcast,
 *   or raw-injection broadcast DATA frame (v5.8.0, PRIMARY round-5 path)
 *
 * v5.9.1 Deployable (2026-09-23, from the university-deployment audit and
 * the scheduler core review; prepared, not flashed):
 * - Wi-Fi: wifi_init_sta() waits for the AP without a timeout. The 30 s
 *   ESP_FAIL used to return from app_main before sockets and tasks
 *   existed: a node booting before its AP joined later and stayed
 *   silent until reset.
 * - Server address: adopt_server_addr() takes the source of any CRC-valid
 *   DISCOVER or TRIGGER as the server. The server now beacons DISCOVER
 *   every 10 s (mdns_advertiser.py), so mDNS and CONFIG_SERVER_IP are
 *   fallbacks, and a server that changes address is followed.
 * - Trigger rate limit 15 -> 5 ms. The server floors its slot at 15 ms
 *   (NODE_MIN_SLOT_MS) and puts the REAL collection window in slot_ms,
 *   5 ms shorter than the interval when no cycle delay supplies that
 *   slack. The old server wrote 20 ms once at start-up while focus mode
 *   triggered every 12 ms: every RX node fell 8 ms further behind per
 *   trigger until the 6-deep UDP mailbox overflowed (model: 0.2 % of
 *   links captured in 2-node focus). With the window shorter than the
 *   interval a node keeps up; the 15 ms limit would then have dropped
 *   the catch-up triggers it processes ~11 ms apart.
 *
 * likelihood-first choice after
 * docs/5-experiments/results/BENCH_V5_7_ESPNOW_2026_08_15.md found ESP-NOW
 * broadcast produces zero CSI at receivers -- prepared not flashed):
 * - PROBLEM: v5.7.0's ESP-NOW broadcast burst produces zero CSI at
 *   receivers, TX side fully confirmed working (init/peer/rate-pin all
 *   serial-verified) -- root cause not isolated, leading hypothesis is an
 *   unprecedented radio-mode combination (HE20-associated STA pinning
 *   ESP-NOW to an HT-class rate) that nothing in Espressif's own esp-csi
 *   repo has run.
 * - FIX (PRIMARY, burst_transport=2): sidesteps the question rather than
 *   resolving it. Constructs a raw 802.11 non-QoS DATA frame by hand
 *   (ToDS=0, FromDS=0, Address1=broadcast, Address2=this node's own MAC,
 *   Address3=BSSID) and injects it via esp_wifi_80211_tx() -- the SAME
 *   PPDU class (Data) this system has run CSI capture on since before
 *   v5.7.0 (the pre-v5.7.0 UDP-broadcast path is ALSO a Data frame, just
 *   ToDS=1 unicast-to-AP). Only the DS bits and destination address
 *   change, not the frame type -- why the PI ranked this PRIMARY over
 *   ESP-NOW for round-5. See the RAW-INJECTION section's own extensive
 *   comment (near raw_inject_burst_t) for the full frame-construction
 *   reasoning, including the corrected en_sys_seq=true requirement (the
 *   original round-5 order assumed false; esp_wifi_80211_tx()'s own doc
 *   comment requires true once the STA is connected) and the
 *   addr2-is-attribution verification against the EXISTING, unmodified
 *   csi_callback()/recent-trigger-TX logic.
 * - SECONDARY (burst_transport=1, unchanged default path, new
 *   espnow_rate_index=4): settles ESP-NOW for the record rather than
 *   leaving it an open question. Adds an HT40+WIFI_PHY_RATE_MCS0_LGI rate
 *   table entry matching Espressif's esp-crab reference's EXACT working
 *   ESP-NOW+CSI parameters (the investigation's precedent check), plus a
 *   no-op esp_now_register_recv_cb() (free, permanently closes the
 *   recv-cb hypothesis the investigation only refuted with moderate
 *   confidence).
 * - SAFETY GATES: raw-injection has no init-latch or rate-pin-confirmed
 *   gate analogous to ESP-NOW's (no equivalent init call needed; no
 *   confirmed rate-control mechanism exists for injected frames -- see
 *   the rate-control comment in the RAW-INJECTION section). Its slot-fit
 *   guard (raw_inject_burst_fits_slot()) uses a DELIBERATELY WIDER
 *   worst-case-per-frame bound than ESP-NOW's, reflecting genuine rate
 *   uncertainty, and is this transport's only safety gate. Its
 *   is_transmitting drain margin and throttled failure logging mirror
 *   ESP-NOW's shapes exactly, for the same structural reasons (documented
 *   ESP_ERR_WIFI_NO_MEM queue-full failure mode, same as ESP-NOW's
 *   ESP_ERR_ESPNOW_NO_MEM; process_trigger() adds this node's own MAC to
 *   its recent-trigger-TX set before deciding TX-vs-RX, same self-capture
 *   exposure).
 * - EVERYTHING ELSE UNCHANGED: v5.7.0's ESP-NOW broadcast transport
 *   (lazy init, rate-fail refusal, drain margin, self-MAC check),
 *   v5.6.0's TX-match attribution (K=12 recent-trigger-TX set, OR-composed
 *   batch predicate, derived attribution margin), heartbeat v5 counters,
 *   and the CSI delivery watchdog are untouched by this image -- verified
 *   by diff against the v5.7.0 HEAD this branched from.
 *
 * v5.8.0 TRIGGER_SEQ WIRE-EXPORT (2026-08-15, later that night, scoped
 * delta on top of the already-reviewed round-5 image above, before any
 * flash -- team-lead direction after a stop-the-line catch: the
 * "receivers-per-frame" methodology used throughout this project's prior
 * illumination-reach measurements (LOSS_DECOMPOSITION_2026_08_14.md's
 * "1.02 of 9", this session's own BENCH_V5_7_ESPNOW_2026_08_15.md
 * "~1.02-1.04") is INVALID, not just imprecise -- both grouped CSI
 * records by (tx, seq) across DIFFERENT receiving nodes, where `seq` is
 * g_batch_seq, a per-node LOCAL batch counter never read from the
 * over-the-air frame and never synchronized across nodes. See either
 * document's own 2026-08-15 retraction addendum for the full
 * verification (an empirical random-pair control found the "matched"
 * (tx,seq) groups statistically indistinguishable from chance):
 * - FIX: csi_packet_t.trigger_seq already existed (stamped from
 *   g_trigger_state.seq at capture time, for firmware-side attribution
 *   only) but was never exported onto the wire. It IS synchronized
 *   across nodes -- every node that captures a frame from a given
 *   server-issued trigger reads that SAME trigger packet's seq field off
 *   the wire. Appended to every outgoing CSI sample, unconditionally, at
 *   every transport: sample_v5_t grows 107->111 bytes
 *   (BINARY_SAMPLE_SIZE_V5_1), v6's per-sample layout grows
 *   3+num_subcarriers*2 -> 7+num_subcarriers*2
 *   (BINARY_V6_SAMPLE_TRIGGER_SEQ_EXTRA). Length-keyed detection
 *   server-side (no magic-byte bump), same None-not-0 discipline as
 *   heartbeat v5's csi_attribution_dropped field.
 * - WHY THIS MATTERS FOR THE BENCH: grouping by (tx_mac, trigger_seq)
 *   instead of (tx, seq) gives a genuine cross-node frame identifier, so
 *   this same bench can now measure the REAL current UDP-transport
 *   receivers-per-frame baseline (never actually measured before) AND
 *   this round's raw-injection delta against it, in one flash -- see
 *   docs/6-firmware/FLASH_RUNBOOK_V5_8_0.md section 6/2a.
 * - ALSO REMOVES (same team-lead direction, same night): the raw-injection
 *   path's best-effort esp_wifi_config_espnow_rate() rate hint
 *   (RAW_INJECT_RATE_HINT, g_raw_inject_rate_hint_attempted, and the call
 *   site in transmit_burst_raw_inject()). DON'T FIGHT THE API: the two
 *   APIs that could actually pin a rate
 *   (esp_wifi_config_80211_tx_rate()/esp_wifi_config_80211_tx()) are only
 *   callable before esp_wifi_start() -- incompatible with this firmware's
 *   lazy runtime-switchable design -- and every rate-control API here,
 *   pinned or best-effort, configures the WHOLE INTERFACE, risking the
 *   STA's own AP association if forced. Injected frames now go out at the
 *   chip's own driver default rate, unconditionally, observed (not
 *   controlled) via the frame_format field already persisted above. Safety
 *   is UNAFFECTED: RAW_INJECT_WORST_CASE_FRAME_AIRTIME_US's 500us bound
 *   was already unconditional regardless of the hint's success/failure --
 *   see its own updated comment. This re-scopes the bench's attack-0
 *   falsifier from "must pin to an acquired class" to "must observe and
 *   report the class," which the existing frame_format logging already
 *   satisfies.
 * - EVERYTHING ELSE FROM THE ROUND-5 BLOCK ABOVE UNCHANGED: the CSI batch
 *   sample wire format changes (sample_v5_t, add_sample_to_batch_v5()) and
 *   the rate-hint removal above are this delta's only touches; the raw-
 *   injection transport's frame construction, its slot-fit/drain-margin
 *   safety gates, and the ESP-NOW rate-table addition are otherwise
 *   untouched.
 *
 * v5.7.0 ROUND-2 (2026-08-15, directive-build pre-registered review --
 * attacks confirmed BEFORE any round actually ran against the pushed
 * image, 2a4a42e2; built to them rather than waiting to fail review):
 * - FOUND (attack a): the FIX bullet below, "init_espnow_broadcast()
 *   brings this up once, after WiFi STA is connected," was TRUE of
 *   2a4a42e2 but broke the "byte-identical to v5.6.0 when transport=0"
 *   claim at the DEVICE level -- every node called esp_now_init()/
 *   esp_now_add_peer() unconditionally at boot, touching WiFi driver
 *   state even on a node that never once receives burst_transport=1.
 * - FIX 1: init_espnow_broadcast() is now called LAZILY, from
 *   transmit_burst()'s dispatch, the first time (and only the first time
 *   -- g_espnow_init_attempted is a one-shot latch, regardless of
 *   outcome) this node is actually asked to send via ESP-NOW. Not called
 *   from wifi_init_sta() at all any more.
 * - FOUND (attack d): "a failed pin degrades to ESP-NOW's own default
 *   rate ... not fatal" (RATE PINNING bullet below) meant an unpinned
 *   send could go out at ESP-NOW's raw ~1 Mbps default. A 250-byte frame
 *   at 1 Mbps is ~2ms; a 10-frame burst (~20ms) overruns
 *   MIN_SLOT_MS=6ms (tdma_scheduler.py) many times over -- transmitting
 *   at that rate must be structurally impossible, not just unlikely.
 * - FIX 2: transmit_burst()'s dispatch now REFUSES the ESP-NOW path
 *   entirely (falls back to UDP + g_espnow_fallback_to_udp) unless
 *   g_espnow_rate_pinned is confirmed true for the requested index. This
 *   firmware never knowingly transmits at ESP-NOW's unpinned default.
 *   Coupled with a runtime slot-overrun guard, espnow_burst_fits_slot()
 *   (burst count and slot_ms are both wire-configurable, not compile-time
 *   constants, so this cannot be a _Static_assert) -- see its own comment
 *   and ESPNOW_WORST_CASE_FRAME_AIRTIME_US for the bound.
 * - FOUND (attack e): esp_now_send() is asynchronous (returns once
 *   queued, not once on-air). transmit_burst_espnow() cleared
 *   is_transmitting immediately after its send loop, same as
 *   transmit_burst_udp() -- but UDP's sendto() completion is a much
 *   closer proxy for on-air completion than ESP-NOW's queue-and-return.
 *   Since process_trigger() adds this node's OWN mac to its recent-
 *   trigger-TX set before deciding TX-vs-RX (see that call site's
 *   comment), a still-in-flight self-transmission could be re-admitted by
 *   this node's own csi_callback() the instant the flag cleared.
 * - FIX 3: a fixed, deliberately generous ESPNOW_TX_DRAIN_MARGIN_US delay
 *   before clearing is_transmitting on the ESP-NOW path. No
 *   esp_now_register_send_cb() is used -- broadcast frames' send
 *   callbacks carry no delivery status (ESP-NOW spec has no ACK for
 *   broadcast), so there is nothing trustworthy to wait on instead.
 * - FIX 4 (defense in depth for FIX 3, attack h): csi_callback() also
 *   does a cheap src_mac != own-MAC check, independent of the drain
 *   margin -- catches any residual gap regardless of how well-tuned the
 *   margin turns out to be.
 * - ADDED (attack c): a compile-time assertion that espnow_burst_t fits
 *   ESP_NOW_MAX_DATA_LEN -- no silent truncation path for a future field
 *   addition.
 * - DOCUMENTED, not fixable in code (mixed-fleet attribution): a comment
 *   in csi_callback() now states explicitly that info->mac is expected to
 *   report the true transmitter (802.11 addr2/TA), not the BSSID, for an
 *   ESP-NOW action frame -- this is BENCH-CHECKABLE, not verifiable from
 *   source; FLASH_RUNBOOK_V5_7_0.md section 6 makes it an explicit gate.
 *
 * v5.7.0 Mesh-Wide Illumination (PI order, prepared not flashed -- bench
 * validation is the round-5 gate before any fleet decision; see
 * docs/4-research/proposed/MESH_WIDE_ILLUMINATION_DESIGN.md):
 * - PROBLEM: the existing burst path (send_broadcast_packet(), a UDP
 *   sendto() to a broadcast IP while associated to an infrastructure AP)
 *   is, at the 802.11 MAC layer, a unicast DATA frame to the AP's BSSID
 *   (ToDS=1) -- never a genuine MAC-layer broadcast to other stations.
 *   Receiving nodes only opportunistically overhear a conversation not
 *   addressed to them. Measured mean 1.02 of 9 possible receivers captured
 *   CSI per burst frame post-v5.6.0-recreate seq capture, with a 47.8%
 *   same-batch duplicate rate explained by 802.11's automatic unicast
 *   retransmission (each retry an independently-overheard PHY transmission).
 *   This is a transport ceiling, not a v5.6.0 attribution-fix shortfall --
 *   see LOSS_DECOMPOSITION_2026_08_14.md's 2026-08-15 addendum.
 * - FIX: burst frames can now go out over ESP-NOW (Espressif's connection-
 *   less protocol on vendor 802.11 action frames) to the explicit broadcast
 *   peer ff:ff:ff:ff:ff:ff -- a true broadcast-destination frame, no ACK,
 *   no retry, every in-range receiver gets one equal simultaneous shot at
 *   decoding. init_espnow_broadcast() brings this up once, after WiFi STA
 *   is connected and CSI capture bring-up has already succeeded (see
 *   wifi_init_sta()); never fatal, falls back to the UDP path with a loud
 *   log on any failure.
 * - RUNTIME-SWITCHABLE, NOT A REFLASH: two new trigger-packet reserved
 *   bytes (binary_trigger_t.reserved[2] = burst_transport, [3] =
 *   espnow_rate_index; see the constants block above CONSTANTS for the
 *   full allocation and the 5 bytes still free after this). Compiled
 *   default and every pre-v5.7.0 server behavior is burst_transport=0
 *   (BURST_TRANSPORT_UDP) -- byte-identical to v5.6.0 until a server
 *   explicitly asks for espnow_broadcast (transmit_burst()'s dispatch,
 *   see its own comment).
 * - RATE PINNING: ESP-NOW's own documented default (1 Mbps) is an order
 *   of magnitude below this firmware's HT20 path and would eat most of
 *   the illumination gain in net information-per-airtime terms if left
 *   unpinned. Pinned via esp_now_set_peer_rate_config() -- the C5-correct
 *   per-peer API; esp_wifi_config_espnow_rate() is documented as not
 *   working on Wi-Fi 6 chips. Compiled default is the most conservative
 *   HT rate (MCS0 SGI, ~7.2 Mbps); a 3-entry lookup table (espnow_rate_
 *   index 1-3) lets the bench sweep faster rates without a reflash. A
 *   failed pin degrades to ESP-NOW's own default rate, logged loudly, not
 *   fatal -- the bench measures what is actually achieved.
 * - NEW STRUCT: espnow_burst_t (12 bytes: magic+type+tx_mac+seq) adds a
 *   seq field the existing binary_burst_t deliberately omits (production
 *   attribution is entirely receiver-side via local trigger-tracking, not
 *   payload content) -- seq here is ground truth for the bench's seq-
 *   capture validation methodology, not a production requirement.
 * - MEMORY: all new state is one broadcast ESP-NOW peer plus 5 fixed
 *   scalars/counters (g_espnow_ready, g_espnow_rate_pinned,
 *   g_espnow_last_rate_index, g_espnow_send_failures,
 *   g_espnow_fallback_to_udp) -- no dynamic allocation, no unbounded
 *   growth. is_transmitting's existing CSI-capture blackout during a
 *   node's own TX carries over unchanged to the ESP-NOW path.
 * - EVERYTHING ELSE UNCHANGED: v5.6.0's TX-match attribution (K=12
 *   recent-trigger-TX set, OR-composed batch predicate, derived
 *   attribution margin), heartbeat v5 counters, and the CSI delivery
 *   watchdog are untouched by this image -- verified by diff against the
 *   v5.6.0 HEAD this branched from.
 *
 * v5.6.0 TX-Match Attribution (PI order, prepared not flashed -- rollout
 * decision stays with the PI; NOT the fix for the field failures above,
 * this addresses LOSS_DECOMPOSITION_2026_08_14.md's Term 1, the 73-point
 * air-and-capture loss the decomposition found was NOT propagation, rho
 * = +0.220 wrong-signed against distance):
 * - PROBLEM: process_trigger()'s RX branch used to unconditionally drain
 *   and discard the CSI ring buffer at trigger start ("window-scoped
 *   retention"), keeping only frames that happened to arrive during this
 *   node's own vTaskDelay(slot_ms) wait. Every RX node processes the same
 *   broadcast trigger at a slightly different wall-clock time (network
 *   scheduling jitter), so burst frames from the CORRECT, correctly
 *   TX-filtered transmitter that arrived a few ms early -- before THIS
 *   node got around to draining -- were discarded, even though
 *   csi_callback() had already verified their src_mac matched the active
 *   trigger's expected transmitter at capture time. Measured: 2.69 of 10
 *   burst packets survive to the server per trigger-link.
 * - FIX (round-1 design; SEE "v5.6.0 ROUND-3" BELOW for the corrected,
 *   currently-shipping composition -- round-1's AND-of-three-layers
 *   design described immediately below was found self-cancelling by
 *   round-2's review and is kept here only as the historical record of
 *   what was tried first): the initial blind ring-clear is removed. The
 *   ring buffer is left intact between trigger start and batch collection.
 *   At batch time (after the same vTaskDelay(slot_ms) wait as before),
 *   each drained item was originally checked against THREE explicit
 *   layers, ALL required, before being added to the outgoing batch:
 *   (1) PRIMARY, exact identity: csi_packet_t.trigger_seq (new field,
 *       captured by csi_callback() from g_trigger_state.seq under the
 *       same spinlock as src_mac) equals this trigger's own seq. This is
 *       what actually separates two BACK-TO-BACK triggers to the SAME TX
 *       (2-node focus mode can re-trigger the same pair every
 *       ~MIN_SLOT_MS*2, ~12ms at the scheduler's floor) -- src_mac alone
 *       cannot, since both triggers share the same transmitter. Design
 *       correction from the first draft of this fix (adversarial review,
 *       directive-build): a mac-plus-time-only design forces the time
 *       bound to thread an impossibly thin needle between "above the
 *       burst's own span" and "below the tightest same-TX revisit
 *       interval"; an exact sequence-identity check removes that
 *       constraint by handling the common (non-racing) case precisely
 *       instead of probabilistically.
 *   (2) SECONDARY, cheap sanity check: src_mac equals this trigger's
 *       tx_mac_bytes (as before -- should always agree with (1) since
 *       both are copied from the same state, kept as defense-in-depth).
 *   (3) TERTIARY, staleness safety net (NOT the primary same-TX defense,
 *       that is (1)): age within ATTRIBUTION_TIME_MARGIN_US of this
 *       trigger's slot_ms window, numerically derived from the actual
 *       measured arrival distribution (see process_trigger()'s v5.6.0
 *       comment and ATTRIBUTION_TIME_MARGIN_US's own comment) --
 *       overridable per-trigger via the trigger packet's reserved bytes
 *       (0 = use the compiled default), so a wrong guess costs a config
 *       value on the next trigger, not a reflash.
 *   Items failing any layer are dropped, counted
 *   (g_csi_attribution_dropped), and logged -- never silently re-queued
 *   or guessed at, because misattribution (a frame credited to the wrong
 *   trigger) is a worse failure than loss (a frame credited to nothing).
 * - The ring buffer's own fixed size (CSI_RINGBUF_SIZE, pre-allocated at
 *   xRingbufferCreate(), RINGBUF_TYPE_NOSPLIT) is unchanged by this fix --
 *   removing the blind clear does not make the ring unbounded; a full
 *   ring still drops at capture time via the pre-existing g_csi_drops
 *   counter, exactly as before.
 * - HOST-SIDE VERIFICATION: the three-layer predicate was replayed against
 *   the actual measured arrival pattern from a 60s, 100-link capture
 *   (recordings/csi_1786742144, 58281 frames) before this was declared
 *   done -- see docs/8-operations/active-tasks/FIRMWARE_V5_6_0_PREP_2026_08_14.md
 *   for the script and results, including the caveat that capture predates
 *   the seq-persisting server change (schema v1, no per-packet seq), so it
 *   verifies arrival-time/link selection behavior, not per-burst identity
 *   -- the (1) layer's own correctness for identity is argued from the
 *   spinlock ordering above, not from that replay, and a full identity-
 *   level replay needs a post-recreate capture.
 * - COUPLED CHANGE (same image, PI order -- a shorter burst without this
 *   fix would just lose more): CONFIG_BURST_COUNT default cut from 10 to
 *   4. NOTE: this firmware-side default is a FALLBACK ONLY -- every
 *   trigger packet the server currently sends carries an explicit nonzero
 *   burst_count (BURSTS_PER_TRIGGER=10, backend/src/config.py), which the
 *   firmware always prefers over this default (see the MSG_TYPE_TRIGGER
 *   handler: `trig->burst_count ? trig->burst_count : CONFIG_BURST_COUNT`).
 *   This firmware-side change alone has NO live effect until the server
 *   side is also updated -- flagged to the PI/team-lead as a companion
 *   change this image does not make (out of scope: "prepare the firmware
 *   image", not a live server config change).
 *
 * v5.6.0 ROUND-2 (2026-08-14, directive-build adversarial review of
 * 10b9c352, before any flash -- flash-once is the acceptance bar):
 * - FOUND: the flat ATTRIBUTION_TIME_MARGIN_US=5000 margin was unsafe at
 *   the scheduler's TRUE slot floor (MIN_SLOT_MS=6, not the 12ms first
 *   assumed) and unconditionally broken for a 1-node focus set (reachable
 *   -- node 02:40 was wedged out of rotation this same week), which no
 *   margin value can fix. See ATTRIBUTION_TIME_MARGIN_US's own comment
 *   for the full k-node derivation.
 * - FIX 1 (this file): the margin is now DERIVED per trigger from the
 *   trigger's own slot_ms (min(cap, slot_ms/2)), not a flat constant.
 *   Restores safe headroom for k>=2 focus sets; k=1 is explicitly NOT
 *   fixable at this layer -- see fix 2.
 * - FIX 2 (backend, not this file, but LOAD-BEARING for fix 1's safety):
 *   backend/src/scheduler/tdma_scheduler.py's _apply_mode_change now
 *   refuses FOCUS_MODE with fewer than 2 validated online nodes, falling
 *   back to coverage. Must be live before this image reaches any node --
 *   see docs/6-firmware/FLASH_RUNBOOK_V5_6_0.md section 0c.
 * - FIX 3 (this file + backend): heartbeat v5 exports
 *   g_csi_attribution_dropped over the wire (see BINARY_HEARTBEAT_SIZE's
 *   comment) so the pilot's watch criteria are satisfiable from the
 *   server API, no serial console needed.
 *
 * v5.6.0 ROUND-3 (2026-08-14, directive-build's round-2 review of
 * 10b9c352 -- DECISIVE finding: round-1/round-2's fix recovered ZERO of
 * Term 1, despite round-2's real safety hardening):
 * - FOUND: the AND-composed batch predicate (round-1's three layers, ALL
 *   required) is self-cancelling for the exact population the ring-clear
 *   removal exists to recover. An early-arriving frame from trigger N,
 *   captured before THIS node's own g_trigger_state rolled over to N
 *   locally (still reads N-1 at capture time), gets pkt->trigger_seq !=
 *   N -- the seq-match layer always fails for it, by construction, no
 *   matter how the other two layers are tuned. WORSE, UPSTREAM: in the
 *   common (different-TX) case, csi_callback()'s PRE-EXISTING capture-time
 *   filter -- gated on g_trigger_state.tx_mac_bytes, this node's own
 *   stale view -- rejected the frame before it ever reached the ring, so
 *   round-1/round-2's batch-time layers never even saw it. The admitted
 *   set under 10b9c352 equaled the set pre-v5.6.0 firmware kept: zero
 *   recovery, despite a real (and still-needed) safety improvement to a
 *   mechanism that wasn't doing any recovery work yet.
 * - FIX 1 (this file, capture time): csi_callback()'s exact-current-
 *   trigger MAC filter is replaced by membership in a small, FIXED-SIZE
 *   set of the last K trigger TX MACs this node has locally processed
 *   (recent_trigger_tx_contains_locked(), see its own comment block; K=12
 *   as of this round -- see RECENT_TRIGGER_TX_COUNT's sizing rule, a
 *   post-hoc mode-measurement correction to an earlier K=4 draft that
 *   only covered focus mode and within-zone rotation, not zone-switch
 *   boundaries, the perimeter zone, or full coverage round-robin).
 *   This is the layer that actually recovers Term 1: an early frame now
 *   reaches the ring whenever its TX was ALSO a recent trigger's TX, not
 *   only when it exactly matches this node's current (possibly stale)
 *   view.
 * - FIX 2 (this file, batch time): the three-layer AND composition is
 *   replaced with a two-disjunct OR: (trigger_seq == seq) OR (src_mac ==
 *   tx_mac_bytes AND age <= max_age_us) -- see process_trigger()'s own
 *   comment for the full safety argument (why a stale same-TX leftover at
 *   k>=2 stays excluded by the time bound, and why k=1 is NOT defended
 *   against here, making the scheduler's <2-node guard MORE load-bearing
 *   under OR composition than it was under round-2's AND composition, not
 *   less).
 * - FIX 3 (backend, not this file): the round-2 pilot success metric
 *   (g_csi_attribution_dropped, a local/wire-exported drop counter) is
 *   demoted from primary to debug-aid. The reviewer showed it is
 *   structurally unable to distinguish "the fix is working" from "the fix
 *   is completely broken and every ambiguous item is being correctly
 *   refused" -- both look like a low, stable count. The pilot's PRIMARY
 *   criterion is now server-side: samples delivered per trigger-exchange,
 *   measured from received batches (current baseline 2.69 of a 10-packet
 *   burst; success is a substantial rise toward the burst count) -- see
 *   docs/6-firmware/FLASH_RUNBOOK_V5_6_0.md.
 *
 * v5.4.1 RF-Death Hardening (root-caused a field failure: node ran 12h with
 * heartbeats alive but zero CSI/BFI frames, fixed only by a power cycle):
 * - AUDIT FINDING: restart_capture() (fired on every WiFi (re)connect,
 *   including the very first one) could race wifi_init_sta()'s one-time CSI
 *   acquire-mask/callback setup from a different task, silently leaving CSI
 *   enabled before it was actually configured. Fixed with g_capture_configured
 *   gating restart_capture() until initial setup completes.
 * - CSI/BFI capture (re-)init calls now retry with backoff
 *   (RETRY_CAPTURE_CALL, CAPTURE_REINIT_MAX_RETRIES) instead of either
 *   panicking on the first blip (boot path) or silently giving up (reconnect
 *   path). Exhausting retries reboots -- the only recovery field data has
 *   shown to work for a wedged CSI/RF subsystem.
 * - Heartbeat (v3, 26 -> 34 bytes) now carries csi_frames_captured and
 *   bfi_frames_captured (cumulative since boot), so the backend can detect
 *   zero-progress capture exactly instead of inferring it from silence.
 * - SECOND AUDIT FINDING (from raw-session evidence: a node produced sparse
 *   but real CSI all afternoon, then went fully silent right after a backend
 *   container recreate/re-registration -- not a boot-time failure):
 *   process_trigger()'s TX branch (this node is the transmitter for a
 *   trigger) returned without clearing g_trigger_state.active. The CSI
 *   callback's peer filter then keeps matching against trigger_mac == our
 *   own MAC forever, discarding every subsequent CSI frame (a node never
 *   sees CSI attributed to itself) until another trigger happens to retarget
 *   this node as RX. Explains both symptoms: sparse-but-present CSI in normal
 *   round-robin operation (brief windows after each TX slot, self-healing on
 *   the next trigger of any kind) and total, permanent silence once this
 *   node stops being retriggered at all (e.g. dropped from the schedule
 *   after a server restart). Fixed at the source (TX branch now clears
 *   state via the same clear_trigger_state() helper as the RX path) plus a
 *   defense-in-depth staleness watchdog in trigger_task() that force-clears
 *   g_trigger_state.active if it's ever left set for TRIGGER_ACTIVE_STALE_MS.
 *
 * v5.4.0 On-Die Temperature Telemetry:
 * - Heartbeat carries die_temp_c_x10 (int16, °C * 10, INT16_MIN = unavailable)
 * - temperature_sensor driver installed once at boot, read every heartbeat
 *
 * v5.2.3 Thread Safety Fixes (based on external code review):
 * - Batch send: Added mutex to protect send buffer from concurrent access
 *   (finalize_and_send_batch_v5 called from trigger_task AND timer_task)
 * - CSI callback: Reverted ping-pong buffer (potential race if callbacks queue)
 * - CSI callback: Kept trigger state optimization (7 bytes vs 32+ bytes)
 *
 * v5.2.2 Performance Optimizations:
 * - Batch send: CRC calculation moved outside critical section (~20-30µs saved)
 * - CSI callback: trigger state check minimized (safe optimization kept)
 *
 * v5.2.1 Bug Fixes:
 * - Fixed rate limiter tick overflow (now uses 64-bit esp_timer)
 * - Fixed CSI callback using ISR macros in task context
 * - Fixed potential nested critical section in batch processing
 * - Added UDP send error tracking for debugging packet loss
 * - Added watchdog reset during WiFi scan
 * - Added bounds check for CRC append in batch buffer
 *
 * v5.2.0 Foveated Sensing Optimization:
 * - Reduced MIN_TRIGGER_INTERVAL_MS: 30ms → 15ms (enables 66 Hz theoretical max)
 * - Doubled CSI_RINGBUF_SIZE: 16 → 32 samples (prevents drops at 50 Hz)
 * - Focus mode target: 42-50 Hz (was ~30-33 Hz)
 *
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"       // v5.7.0: mesh-wide illumination, ESP-NOW broadcast burst path
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
// Pure binary protocol - zero JSON/malloc overhead
#include "esp_crc.h"
#include "rom/ets_sys.h"  // For esp_rom_delay_us
#include "esp_task_wdt.h"  // Task watchdog
#include "driver/temperature_sensor.h"  // On-die temperature sensor (C3 + C5)
#ifdef CONFIG_MDNS_DISCOVERY
#include "mdns.h"
#endif

// =============================================================================
// RELIABILITY CONFIGURATION
// =============================================================================
#define WATCHDOG_TIMEOUT_S      30      // Watchdog timeout in seconds
#define HEAP_WARNING_THRESHOLD  8192    // Warn if heap drops below this
#define HEAP_CRITICAL_THRESHOLD 4096    // Reboot if heap drops below this
#define WIFI_RECONNECT_DELAY_MS 5000    // Delay between WiFi reconnection attempts
#define SOCKET_TIMEOUT_MS       100     // Socket receive timeout (reduced from 5000ms for faster response)

// v5.4.1: CSI/BFI capture (re-)init calls (esp_wifi_set_csi, esp_wifi_set_promiscuous,
// esp_wifi_set_promiscuous_rx_cb) used to be either fatal-only (ESP_ERROR_CHECK at
// boot) or silently logged-and-ignored (restart_capture() on reconnect). Retry each
// call with backoff before giving up; on exhaustion the node self-heals the only way
// field data has shown works -- a reboot (see restart_capture() / wifi_init_sta()).
#define CAPTURE_REINIT_MAX_RETRIES    3      // attempts after the first failure
#define CAPTURE_REINIT_RETRY_BASE_MS  100    // backoff: 100, 200, 400 ms

// =============================================================================
// CONFIGURATION - Use menuconfig or override here
// =============================================================================
#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID "CHANGE_ME_SSID"
#endif

#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD "CHANGE_ME_PASSWORD"
#endif

#ifndef CONFIG_SERVER_IP
#define CONFIG_SERVER_IP "0.0.0.0"
#endif

#ifndef CONFIG_SERVER_PORT
#define CONFIG_SERVER_PORT 5000
#endif

#ifndef CONFIG_TRIGGER_PORT
#define CONFIG_TRIGGER_PORT 5001
#endif

#ifndef CONFIG_BURST_PORT
#define CONFIG_BURST_PORT 5555
#endif

// WiFi channel to force (0 = auto, 1-13 = specific channel)
// Set this to ensure all nodes connect to the same AP on the same channel
#ifndef CONFIG_WIFI_CHANNEL
#define CONFIG_WIFI_CHANNEL 11
#endif

// Number of burst packets to transmit. FALLBACK ONLY -- every trigger
// packet the server currently sends carries an explicit nonzero
// burst_count (BURSTS_PER_TRIGGER, backend/src/config.py), which the
// MSG_TYPE_TRIGGER handler always prefers over this default; changing
// this value alone has no live effect unless the server side changes too
// (see the v5.6.0 changelog entry above main() for the flagged coupling).
// v5.1.1: Reduced from 100 to 10 for faster trigger cycles.
// v5.6.0: Reduced from 10 to 4, coupled with the TX-match attribution fix
// (LOSS_DECOMPOSITION_2026_08_14.md Term 1) -- a shorter burst without
// that fix would only lose a larger fraction of a smaller burst; never
// ship one without the other.
#ifndef CONFIG_BURST_COUNT
#define CONFIG_BURST_COUNT 4
#endif

// Delay between burst packets in microseconds
// - PHY minimum: ~100 µs (preamble + DIFS)
// - Safe range: 100-500 µs
// - v5.2: Reduced to 100µs (PHY minimum) for faster burst cycles
// - With CONFIG_BURST_COUNT=10 (pre-v5.6.0): 10 x 100us = 1ms burst time
// - v5.6.0, CONFIG_BURST_COUNT=4 (fallback default; see the coupling note
//   at CONFIG_BURST_COUNT): 4 x 100us = 400us burst time
#ifndef CONFIG_BURST_DELAY_US
#define CONFIG_BURST_DELAY_US 100
#endif

// v5.6.0: TX-match attribution's TERTIARY (staleness safety net) time-bound
// margin CAP, in microseconds, added on top of the current trigger's own
// slot_ms window (process_trigger()'s max_age_us = slot_ms*1000 + this).
// NOT the primary defense against back-to-back same-TX triggers -- that is
// csi_packet_t.trigger_seq's exact-identity check (process_trigger()'s
// v5.6.0 comment block has the full three-layer design).
//
// 2026-08-14 FIX (directive-build round-2 adversarial review): this used to
// be the margin itself, flat, for every slot_ms. That is unsafe at the
// scheduler's true slot floor (MIN_SLOT_MS=6 -- tdma_scheduler.py -- not
// the 12ms this fix originally assumed): a flat 5ms margin gives a 2-node
// focus set an admission window of slot_ms+margin=11ms against a 12ms
// same-TX revisit cadence, 1ms of headroom against measured jitter where
// 23% of arrivals exceed 2ms spacing. process_trigger() now DERIVES the
// actual per-trigger default from the trigger's own slot_ms instead of
// using this constant directly:
//
//   effective_margin_us = min(ATTRIBUTION_TIME_MARGIN_US, slot_ms_us / 2)
//
// GENERAL INVARIANT (k = number of nodes in the active focus set; a
// k-node round-robin puts k-1 other slots between two turns of the same
// node, so it revisits the same TX every k*slot_ms -- a full rotation).
// The admission window must stay below that revisit interval:
//
//       margin < (k-1) * slot_ms
//
// For k=1 this reads margin < 0, which NO margin satisfies -- a
// single-node focus set re-triggers the same TX every slot (revisit
// interval = slot_ms, not (k-1)*slot_ms = 0), so this layer is
// STRUCTURALLY UNABLE to exclude the previous cycle's frames for ANY
// margin > 0. That is not a smaller-margin problem and this layer cannot
// fix it. k=1 focus sets are instead refused entirely by the scheduler
// (backend/src/scheduler/tdma_scheduler.py's _apply_mode_change(), the
// <2-validated-node guard added 2026-08-14) -- that guard is LOAD-BEARING
// for this invariant, not belt-and-braces defense-in-depth. Without it, no
// firmware margin choice makes k=1 safe.
//
// For k>=2, (k-1)*slot_ms >= slot_ms, so margin < slot_ms; a 2x safety
// factor gives margin <= slot_ms/2, the formula above. Numeric example at
// the scheduler's floor (MIN_SLOT_MS=6, k=2): margin = min(5000, 3000) =
// 3ms, admission window = 9ms, versus a 12ms same-TX revisit cadence -- 3ms
// of headroom, versus the flat-5ms design's 1ms.
//
// This constant remains the CAP (unchanged from the original flat-5ms
// value, still ~2.5x the ~2ms nominal within-burst span measured from
// recordings/csi_1786742144 -- see process_trigger()'s layer-3 comment),
// so large-slot_ms triggers are not given an unnecessarily wide window.
// Overridable per-trigger via the trigger packet's reserved bytes
// (attribution_margin_us, 0 = use the derived default), so a wrong guess
// costs a config value on the next trigger, not a reflash.
#ifndef ATTRIBUTION_TIME_MARGIN_US
#define ATTRIBUTION_TIME_MARGIN_US 5000
#endif

// =============================================================================
// v5.7.0: MESH-WIDE ILLUMINATION -- ESP-NOW BROADCAST BURST PATH
// =============================================================================
// docs/4-research/proposed/MESH_WIDE_ILLUMINATION_DESIGN.md. Mechanism: the
// existing burst path (send_broadcast_packet(), transmit_burst_udp() below)
// is UDP-broadcast over infrastructure WiFi, which at the MAC layer is
// unicast-to-AP (ToDS=1), not a true 802.11 broadcast -- measured mean 1.02
// of 9 receivers per burst frame. A true broadcast-destination frame
// (ff:ff:ff:ff:ff:ff at the MAC layer, which 802.11 does not ACK or retry)
// gives every receiver in range one simultaneous, equal shot at decoding.
// ESP-NOW is Espressif's own connectionless protocol built on vendor
// action frames with explicit broadcast-peer support, coexisting with the
// existing STA association (the control plane and CSI/BFI/heartbeat
// delivery to the server are unaffected -- only the CSI-bearing burst
// itself moves transport).
//
// Runtime-switchable, not a reflash: BINARY_TRIGGER_SIZE's reserved[9] had
// 7 bytes free after round-2's attribution_margin_us (2 bytes). This adds
// two more: reserved[2] = burst_transport (0 = udp, compiled default and
// byte-identical to pre-v5.7 behavior; 1 = espnow_broadcast; 2 =
// raw_inject, added v5.8.0 -- see the RAW-INJECTION section below),
// reserved[3] = espnow_rate_index (0 = compiled default rate; 1-3 index a
// small lookup table below; unused by raw_inject, which has no comparable
// confirmed rate-control mechanism -- see that section). 5 bytes remain
// free after this.
#define BURST_TRANSPORT_UDP              0
#define BURST_TRANSPORT_ESPNOW_BROADCAST 1
#define BURST_TRANSPORT_RAW_INJECT       2

// ESP-NOW's own documented default bit rate is 1 Mbps
// (esp_now.rst "Frame Format") -- an order of magnitude below this
// firmware's existing HT20 path and enough to eat most of the receiver-
// count gain in net information-per-airtime terms if left unpinned. Pinned
// via esp_now_set_peer_rate_config() (NOT the deprecated
// esp_wifi_config_espnow_rate(), which the ESP-IDF header itself documents
// as not working on Wi-Fi 6 chips -- the C5 needs the newer per-peer API).
//
// Compiled default is deliberately the MOST CONSERVATIVE (lowest, most
// robust) HT MCS rather than the fastest available: this is a new,
// unvalidated-on-C5 path (see the round-5 gate in the design doc, section
// 3.3), and broadcast frames get no retry at all if a receiver fails to
// decode one -- unlike the aggressive-rate-with-retry trade the old
// unicast path made implicitly. MCS0 with short GI is still ~7.2 Mbps,
// ~7x ESP-NOW's raw default, while being the most decode-robust HT rate
// available. The bench (design doc section 3.6) is what determines
// whether a faster index below is actually safe to prefer.
#define ESPNOW_RATE_PHYMODE  WIFI_PHY_MODE_HT20
#define ESPNOW_RATE_DEFAULT  WIFI_PHY_RATE_MCS0_SGI

// v5.8.0 (round-5 SECONDARY, settles ESP-NOW for the record per
// docs/5-experiments/results/BENCH_V5_7_ESPNOW_2026_08_15.md): the rate
// table is now phymode+rate PAIRS, not a flat rate array sharing a single
// compile-time ESPNOW_RATE_PHYMODE -- indices 1-3 keep the original
// HT20/SGI family (this file's own conservative default family, bench
// room to sweep MCS within it), and NEW index 4 pairs HT40 with
// WIFI_PHY_RATE_MCS0_LGI (Long GI), matching Espressif's own esp-crab
// reference (examples/esp-crab/{master_recv,slave_recv,slave_send}) EXACT
// working ESP-NOW+CSI parameters byte-for-byte -- the investigation doc's
// leading hypothesis is that THIS specific phymode/GI combination, not
// just the MCS index, is what's untested on our HE20-associated STA.
// Index 0 (compiled default, ESPNOW_RATE_PHYMODE/ESPNOW_RATE_DEFAULT
// above) is unchanged -- HT20/SGI stays the safe default for anyone not
// deliberately testing the esp-crab-matched entry.
#define ESPNOW_RATE_TABLE_SIZE 4
typedef struct {
    wifi_phy_mode_t phymode;
    wifi_phy_rate_t rate;
} espnow_rate_entry_t;
static const espnow_rate_entry_t ESPNOW_RATE_TABLE[ESPNOW_RATE_TABLE_SIZE] = {
    { WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS0_SGI },  // index 1: same as compiled default, explicit pin
    { WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS3_SGI },  // index 2: ~28.9 Mbps, moderate
    { WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS7_SGI },  // index 3: ~72.2 Mbps, aggressive -- bench validates before fleet use
    { WIFI_PHY_MODE_HT40, WIFI_PHY_RATE_MCS0_LGI },  // index 4: esp-crab's exact working ESP-NOW+CSI params -- see FLASH_RUNBOOK_V5_8_0.md's transport=1@new-rate step
};

// v5.7.0 (directive-build pre-registered review, attack d): slot-overrun
// guard. ESP-NOW's UNPINNED default rate (~1 Mbps, see above) would make a
// multi-frame burst overrun the scheduler's own slot floor -- a 250-byte
// frame at 1 Mbps is already ~2ms, so a 10-frame burst (~20ms) blows
// through MIN_SLOT_MS=6ms (tdma_scheduler.py) many times over. The
// dispatcher (transmit_burst() below) makes this structurally impossible,
// not just measured-safe: ESP-NOW is only ever used once
// g_espnow_rate_pinned is true (one of THIS file's own controlled rates,
// never ESP-NOW's raw default -- see the refusal logic in transmit_burst()),
// and this constant bounds the worst case for that pinned-only population.
//
// ESPNOW_WORST_CASE_FRAME_AIRTIME_US is a deliberately generous, ROUNDED
// bound, not a measured value (this file has no send-completion signal to
// measure against -- broadcast frames get no ACK and this codebase does
// not key logic off esp_now_send()'s callback status, see
// transmit_burst_espnow()'s own comment). A real frame (espnow_burst_t's
// 12-byte payload plus 802.11 header/vendor-IE overhead, on-air at
// MCS0 SGI -- the SLOWEST rate this file ever pins, ESPNOW_RATE_DEFAULT --
// is under 100us including preamble/SIFS; 200us is a 2x safety margin over
// that estimate, chosen round rather than tight.
#define ESPNOW_WORST_CASE_FRAME_AIRTIME_US 200

// Runtime check (burst_count and slot_ms are both wire-configurable, so
// this cannot be a _Static_assert): see espnow_burst_fits_slot() below,
// called from transmit_burst()'s dispatch before committing to the
// ESP-NOW path.
static inline bool espnow_burst_fits_slot(uint32_t count, uint16_t slot_ms)
{
    uint64_t worst_case_us = (uint64_t)count * ESPNOW_WORST_CASE_FRAME_AIRTIME_US;
    return worst_case_us < ((uint64_t)slot_ms * 1000);
}

// v5.7.0 (directive-build pre-registered review, attack e): fixed drain
// margin added after the last esp_now_send() in a burst, before
// is_transmitting is cleared -- see transmit_burst_espnow()'s own comment
// for the full reasoning (esp_now_send() is async; no completion signal
// exists to wait on for broadcast frames).
//
// WHY 2000us BOUNDS THE ACTUAL PENDING QUEUE, not just "10x one frame":
// each esp_now_send() call in the burst loop is followed by
// CONFIG_BURST_DELAY_US (100us) of esp_rom_delay_us() before the next
// call is issued. A single frame's on-air time at any rate this file ever
// pins (MCS0 SGI or faster -- see ESPNOW_WORST_CASE_FRAME_AIRTIME_US, our
// own generous per-frame bound) is under that 100us inter-send delay, so
// in steady state the driver's send queue should rarely hold more than
// ~1-2 frames at the moment any given esp_now_send() call returns -- the
// 100us gap is normally enough for the previous frame to have cleared.
// The margin only needs to cover whatever residual queue depth exists at
// the SPECIFIC instant the loop's LAST send returns, which by this
// argument is a small, bounded number of frames, not the whole burst.
// 2000us is a 5x-to-10x safety factor over that few-hundred-microsecond
// estimate (not a measured value -- there is no completion signal to
// measure against, see above), chosen to survive scheduling jitter and
// any single anomalously slow frame without needing the bench to tune it
// before the first real ESP-NOW burst.
#define ESPNOW_TX_DRAIN_MARGIN_US 2000

// =============================================================================
// CONSTANTS
// =============================================================================
static const char *TAG = "CSI_NODE";
#if CONFIG_SOC_WIFI_HE_SUPPORT
#define MAX_CSI_DATA_LEN     512   // HE20: 242 subcarriers x 2 = 484, aligned to 512
#else
#define MAX_CSI_DATA_LEN     384   // HT20: 52 subcarriers x 2 = 104 (with headroom)
#endif
// Ring buffer size: larger on S3/HE (more RAM) for higher sample rates.
// HE path (C5/C6): 64 samples × 512 B = 32 KB (within 320 KB SRAM budget,
// avoids drops if processing task is preempted at 100+ Hz).
#if CONFIG_IDF_TARGET_ESP32S3
#define CSI_RINGBUF_SIZE     (MAX_CSI_DATA_LEN * 64)  // S3: 64 samples for 100+ Hz headroom
#elif CONFIG_SOC_WIFI_HE_SUPPORT
#define CSI_RINGBUF_SIZE     (MAX_CSI_DATA_LEN * 64)  // C6/C5: 64 samples for 100+ Hz headroom
#else
#define CSI_RINGBUF_SIZE     (MAX_CSI_DATA_LEN * 32)  // C3: 32 samples for 50 Hz headroom
#endif
#define HEARTBEAT_INTERVAL_MS 2000
#define ANNOUNCE_INTERVAL_MS  5000
// How often a node still waiting for its AP at boot logs that it is waiting.
#define WIFI_CONNECT_WAIT_LOG_MS 30000

// On-die temperature sensor calibration range (driver requirement, not a
// physical limit -- covers the indoor deployment envelope with margin).
#define DIE_TEMP_RANGE_MIN_C  -10
#define DIE_TEMP_RANGE_MAX_C   80
// Sentinel for "sensor unavailable/read failed" in the heartbeat's
// int16 x10-scaled field (outside any real die_temp_c_x10 range).
#define DIE_TEMP_READ_FAILED_X10  INT16_MIN

// Trigger rate limiting - ignore triggers that arrive too fast.
// v5.9.1: the server owns the pacing. It never sends triggers closer than
// its NODE_MIN_SLOT_MS (15 ms, tdma_scheduler.py), and the wire slot_ms is
// now the real collection window, shorter than that interval by the guard
// the batch build and send need. This limit therefore only has to reject a
// storm, and it must sit BELOW the window: the check compares processing
// times, and a node catching up on a queued trigger processes it window +
// overhead after the previous one (~11 ms). The former 15 ms here dropped
// exactly those catch-up triggers.
#define MIN_TRIGGER_INTERVAL_MS 5

// v5.4.1: g_trigger_state.active gates the CSI callback's peer filter (see
// csi_callback()) -- while true, only CSI attributed to trigger_mac is kept,
// everything else is silently dropped. process_trigger() clears it once RX
// processing finishes; TRIGGER_ACTIVE_STALE_MS bounds how long it can be left
// set by any path that doesn't (a confirmed bug fixed this release -- see
// process_trigger()'s TX branch). Comfortably above the default 80ms slot_ms
// and any legitimate burst/RX-wait duration.
#define TRIGGER_ACTIVE_STALE_MS  3000

#if CONFIG_SOC_WIFI_HE_SUPPORT
// HE20 pilot positions in 242-subcarrier array (approximate -- depends on RU allocation)
// For full-band 242-tone RU: pilots at +/-{12, 40, 68, 96}
// In array order (0-based): 26, 54, 82, 110, 134, 162, 190, 218
#define HE_PILOT_COUNT 8
static const uint8_t __attribute__((unused)) he_pilot_indices[HE_PILOT_COUNT] = {26, 54, 82, 110, 134, 162, 190, 218};
#endif
// HT20 pilot subcarrier indices (in 52-subcarrier array, 0-indexed)
// These correspond to subcarriers -21, -7, +7, +21 relative to center
#define PILOT_SC_1  6   // subcarrier -21
#define PILOT_SC_2  20  // subcarrier -7
#define PILOT_SC_3  32  // subcarrier +7
#define PILOT_SC_4  46  // subcarrier +21

// WiFi event group bits
#define WIFI_CONNECTED_BIT    BIT0

// =============================================================================
// CSI DATA STRUCTURE (packed for ring buffer) - Enhanced with signal info
// =============================================================================
typedef struct __attribute__((packed)) {
    int64_t timestamp_us;
    int8_t  rssi;
    int8_t  noise_floor;
    uint8_t channel;
    uint8_t secondary_channel;
    uint8_t rate;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    uint8_t bb_format;          // cur_bb_format on C6 (0=non-HT,1=HT,3=HE-SU,4=HE-MU)
    uint8_t csi_valid;          // rx_channel_estimate_info_vld
#else
    uint8_t sig_mode;           // 0=non-HT, 1=HT, 3=VHT
    uint8_t cwb;                // Channel bandwidth (0=20MHz, 1=40MHz)
#endif
    uint8_t aggregation;
    uint8_t stbc;
    uint8_t fec_coding;
    uint8_t sgi;                // Short Guard Interval
    uint8_t rx_state;           // RX state info
    uint16_t len;
    uint8_t src_mac[6];
    // v5.6.0: which trigger instance this frame was captured under (the
    // ACTIVE g_trigger_state.seq at capture time, 0 if no trigger was
    // active). Exact-identity discriminator for TX-match attribution --
    // see process_trigger()'s v5.6.0 comment block. src_mac alone cannot
    // separate two back-to-back triggers to the SAME transmitter; seq can,
    // in the non-racing case (this node's csi_callback() observes a
    // trigger_seq update strictly before that trigger's own batch
    // collection runs, which holds unless this node's own processing of
    // trigger N is still mid-flight when trigger N+1 to the same TX
    // already landed -- the time-bound predicate is the safety net for
    // that residual race, not the primary defense).
    uint32_t trigger_seq;
    int8_t  data[MAX_CSI_DATA_LEN];
} csi_packet_t;

// =============================================================================
// BINARY PROTOCOL - Zero-malloc, fixed-size packets for all messages
// =============================================================================
// Protocol magic byte (first byte of all packets)
#define BINARY_PROTO_MAGIC      0xC5

// Message types (second byte)
#define MSG_TYPE_HELLO          0x01    // Firmware → Server: Node registration
#define MSG_TYPE_HEARTBEAT      0x02    // Firmware → Server: Keepalive
#define MSG_TYPE_CSI            0x03    // Firmware → Server: CSI data (v2)
#define MSG_TYPE_TRIGGER        0x10    // Server → Firmware: Start measurement
#define MSG_TYPE_DISCOVER       0x11    // Server → Firmware: Request HELLO
#define MSG_TYPE_TEST           0xFF    // Connectivity test
#define MSG_TYPE_BURST          0x30    // Burst packet (binary, replaces text "BURST:")
#define MSG_TYPE_BFI            0x40    // Firmware → Server: captured beamforming feedback (BFI)

// =============================================================================
// BINARY BURST PACKET - 8 bytes, sent during TX burst (OPT-1)
// =============================================================================
#define BINARY_BURST_SIZE       8

typedef struct __attribute__((packed)) {
    uint8_t  magic;             // 0xC5 (1)
    uint8_t  type;              // 0x30 = BURST (1)
    uint8_t  tx_mac[6];         // Transmitter MAC (6)
} binary_burst_t;               // Total: 8 bytes (vs ~40 bytes text)

_Static_assert(sizeof(binary_burst_t) == BINARY_BURST_SIZE,
               "Binary BURST packet must be exactly 8 bytes");

// =============================================================================
// v5.7.0: ESP-NOW BROADCAST BURST PACKET - 12 bytes
// =============================================================================
// Deliberately NOT byte-identical to binary_burst_t, and not required to
// be: this frame never passes through the server's UDP receiver (ESP-NOW
// is a direct node-to-node 802.11 action frame; the server never sees it),
// so its payload's only job is (a) be a valid on-air frame for CSI
// extraction to fire against, which csi_callback() gets from PHY/MAC
// metadata (info->mac, populated by the driver) regardless of payload
// content -- same as the existing UDP path never reads binary_burst_t's
// content either -- and (b) carry ground truth for the seq-capture bench
// methodology (design doc section 3.6), which needs to identify "the same
// physical transmission across independently-processed receiver captures"
// without depending on any one receiver's own local (possibly stale)
// g_trigger_state.seq. tx_mac is redundant with the 802.11 source address
// ESP-NOW already carries, kept for parity with binary_burst_t's
// convention and cheap cross-checking.
#define ESPNOW_BURST_SIZE       12

typedef struct __attribute__((packed)) {
    uint8_t  magic;             // 0xC5 (1)
    uint8_t  type;              // 0x30 = BURST, same MSG_TYPE_BURST as the UDP path (1)
    uint8_t  tx_mac[6];         // Transmitter MAC (6) -- redundant with 802.11 source addr
    uint32_t seq;                // Trigger seq -- bench ground truth (4)
} espnow_burst_t;               // Total: 12 bytes


// =============================================================================
// v5.8.0: RAW-INJECTION BROADCAST DATA FRAME PATH (PRIMARY, round-5)
// =============================================================================
// docs/5-experiments/results/BENCH_V5_7_ESPNOW_2026_08_15.md found ESP-NOW
// broadcast produces zero CSI at receivers on this hardware/radio-mode
// combination (root cause not yet isolated -- an HE20-negotiated STA
// pinning ESP-NOW to an HT-class rate has no working precedent anywhere in
// Espressif's own esp-csi repo). This path sidesteps the question entirely
// rather than resolving it: instead of ESP-NOW, construct a raw 802.11
// non-QoS DATA frame by hand and inject it via esp_wifi_80211_tx()
// (explicitly documented as supporting non-QoS data frames -- see that
// API's own doc comment in esp_wifi.h) with ToDS=0, FromDS=0, and a
// broadcast destination -- the SAME PPDU class (Data) this entire system
// has run CSI capture on since before v5.7.0 (the pre-v5.7.0
// UDP-broadcast-over-STA path is ALSO a Data frame, just ToDS=1
// unicast-to-AP; this changes ONLY the DS bits and destination address,
// not the frame type/PPDU class our CSI engine has always processed).
// This is why the PI ranked this PRIMARY over ESP-NOW for round-5: it
// changes the minimum number of variables relative to a mechanism already
// proven CSI-visible, rather than introducing an entirely untested frame
// class (ESP-NOW action frames).
//
// FRAME LAYOUT (802.11 MAC header, non-QoS Data, no Address4 -- ToDS=0
// and FromDS=0 both being 0 means no 4-address extension):
//   frame_control (2) | duration (2) | addr1/RA-DA (6) | addr2/TA-SA (6)
//   | addr3/BSSID (6) | seq_ctrl (2) | [frame body]
// = 24 bytes of header, matching esp_wifi_80211_tx()'s own documented
// minimum length (len must be >= 24 bytes). Frame body reuses
// espnow_burst_t verbatim (not a near-duplicate struct) -- team-lead's own
// framing, and it is already exactly the right shape (magic+type+tx_mac+seq).
//
// ADDR1 = broadcast (ff:ff:ff:ff:ff:ff): per 802.11 Table 9-26, when
// ToDS=0 and FromDS=0, Address1=DA (Destination Address) -- this is what
// makes the frame genuinely broadcast at the MAC layer, unlike the
// pre-v5.7.0 ToDS=1 path where Address1=BSSID (the AP), never broadcast.
//
// ADDR2 = this node's own MAC -- per the same table, Address2=SA (Source
// Address) for ToDS=0/FromDS=0. THIS IS THE ATTRIBUTION FIELD (team-lead's
// explicit ask: verify against the working code, not just assert it).
// Verified: csi_callback()'s existing, UNMODIFIED attribution logic reads
// info->mac (see recent_trigger_tx_contains_locked(info->mac) and
// csi_packet_t.src_mac's own comment) -- info->mac is populated by the
// WiFi driver from the MAC header's Address2 FIELD POSITION, not from any
// ToDS/FromDS-aware semantic parsing. This is the exact mechanism the
// pre-v5.7.0 system has relied on for its entire history (a ToDS=1
// unicast-to-AP frame's Address2 is ALSO the sending node, not the AP --
// info->mac has never meant "BSSID" for us) and what the v5.6.0 TX-match
// attribution predicate was validated against (58281 real frames,
// scripts/firmware_attribution_replay_test.py). Switching ToDS from 1 to 0
// does not move which header position carries our MAC; it only changes
// what's carried in Address1 and Address3. No RX-path code changes needed
// for attribution to keep working -- this is why this transport is a
// receive-side no-op, exactly like ESP-NOW's promiscuous CSI callback
// needed no changes.
//
// ADDR3 = BSSID (this node's associated AP's MAC), per the same table's
// ToDS=0/FromDS=0 convention -- NOT broadcast. This identifies the frame
// as belonging to our BSS to anything that inspects it, without routing
// it through the AP (ToDS=0 means the AP has no obligation to process or
// forward a frame not addressed to it -- Address1 is broadcast, not the
// AP's own address, so associated-STA-sends-an-odd-broadcast is not a
// protocol violation the AP would need to react to, e.g. by
// disassociating this station). This resolves "addr3=BSSID or broadcast,
// per what keeps the associated STA happy" in favor of BSSID: it is the
// standards-referenced value for this DS-bit combination, not a guess.
//
// SEQUENCE CONTROL: esp_wifi_80211_tx()'s own doc comment is explicit and
// load-bearing here -- "if the API is called after the Wi-Fi connection
// has been set up, en_sys_seq must be true, otherwise ESP_ERR_INVALID_ARG
// is returned." Every node calling this is STA-connected by construction
// (the trigger loop that would call this only runs post-connection), so
// en_sys_seq=true is REQUIRED, not a design choice -- this corrects an
// assumption in the original round-5 order ("likely false so we control
// seq for attribution"). With en_sys_seq=true the driver overwrites
// whatever we put in seq_ctrl with its own internal sequence number
// (shared with the STA's other outgoing traffic) -- we have zero control
// over it and it carries no bench-attributable information. This is
// exactly why the bench seq lives in the frame BODY (espnow_burst_t.seq),
// which was already the plan and needs no adjustment.
//
// FRAME CONTROL: Type=Data(2), Subtype=0 (non-QoS Data -- the ONLY Data
// subtype esp_wifi_80211_tx() documents support for), ToDS=0, FromDS=0,
// all other bits 0. Verified against the standard field-packing
// convention (subtype occupies the high nibble of byte 0, type the next
// two bits, version the low two bits -- cross-checked against a known
// reference value, the beacon frame's well-known FC=0x0080 for
// Type=Management(0)/Subtype=Beacon(8)): byte0 = (0<<4)|(2<<2)|(0) =
// 0x08, byte1 (ToDS/FromDS/etc, all 0) = 0x00 -> frame_control = 0x0008
// as a native-endian (little-endian on this hardware, matching 802.11's
// own on-the-wire byte order for this field) uint16_t.
#define RAW_INJECT_FRAME_CONTROL_DATA_NONQOS  0x0008

// Duration/ID: 0. Real directed (unicast, individually-addressed) data
// frames set this to the NAV protection duration for their expected ACK;
// broadcast/multicast-destination frames get no ACK under 802.11 (same
// no-ACK/no-retry property this whole design already relies on for
// ESP-NOW broadcast), so there is nothing to protect -- 0 is the correct,
// standard value, not a placeholder.
#define RAW_INJECT_DURATION 0

typedef struct __attribute__((packed)) {
    uint16_t frame_control;
    uint16_t duration;
    uint8_t  addr1[6];   // RA/DA -- broadcast
    uint8_t  addr2[6];   // TA/SA -- this node's own MAC, THE attribution field
    uint8_t  addr3[6];   // BSSID
    uint16_t seq_ctrl;   // driver-overwritten (en_sys_seq=true, mandatory once connected) -- zero-filled, not meaningful to us
    espnow_burst_t body; // reused verbatim, not duplicated -- magic+type+tx_mac+seq
} raw_inject_burst_t;

#define RAW_INJECT_MAC_HEADER_SIZE 24
#define RAW_INJECT_FRAME_SIZE (RAW_INJECT_MAC_HEADER_SIZE + ESPNOW_BURST_SIZE)

_Static_assert(sizeof(raw_inject_burst_t) == RAW_INJECT_FRAME_SIZE,
               "raw_inject_burst_t must be exactly header+body, no padding");
_Static_assert(RAW_INJECT_FRAME_SIZE >= 24 && RAW_INJECT_FRAME_SIZE <= 1500,
               "esp_wifi_80211_tx() requires 24 <= len <= 1500 bytes");

// RATE CONTROL -- v5.8.0 originally attempted a best-effort rate hint via
// esp_wifi_config_espnow_rate() here (see git history / the trigger_seq
// changelog entry above main() for the removal rationale). REMOVED
// 2026-08-15, same night, per team-lead direction after review: the two
// APIs that could actually PIN a rate --
// esp_wifi_config_80211_tx_rate()/esp_wifi_config_80211_tx() -- are only
// callable BEFORE esp_wifi_start(), architecturally incompatible with this
// firmware's lazy, runtime-switchable transport design, and (per their own
// doc comments) configure the WHOLE INTERFACE's rate, not scoped to
// injected frames -- forcing either risks the STA's own AP association,
// which is unacceptable for a bench that must stay on the network to
// report results. esp_wifi_config_espnow_rate() (what was attempted here)
// doesn't solve this either: its own signature takes an INTERFACE, not a
// peer, so it's the same whole-interface risk in a different shape, and
// its effect on esp_wifi_80211_tx()-injected frames was never confirmed
// even when the call reported success -- a best-effort hint with no
// completion/telemetry signal to verify it worked, on an API whose scope
// was itself uncertain.
//
// DECISION: don't fight it. This bench does not attempt to control the
// injected rate at all -- frames go out at whatever this chip's driver
// default/basic rate is for injected non-QoS Data frames. What the bench
// DOES do is OBSERVE, not assume: the frame_format field persisted in
// every delivered sample (this firmware's v6 batch header, `bb_format` at
// capture time -- see BinaryCSIPacket.sig_mode server-side) reports which
// PPDU class the receiver actually acquired CSI from. A cleaner
// measurement bench, not a worse one: if the driver default turns out to
// be legacy-class, that is directly informative (a broader-compatibility
// class than the HT-class rate ESP-NOW's own pin attempt failed at), and
// either way the bench learns the true answer instead of an assumed one.
// Fleet-use rate/class control, if ever needed, is deferred -- out of
// scope for a measurement bench.

// Worst-case per-frame airtime bound, DELIBERATELY MORE CONSERVATIVE than
// ESPNOW_WORST_CASE_FRAME_AIRTIME_US (200us) because the injected rate is
// the chip's own driver default, deliberately not pinned (see RATE
// CONTROL above) -- this bounds against the possibility the frame goes
// out at whatever this chip's default/basic rate for injected non-QoS
// data frames turns out to be, which could be far slower than HT.
// RAW_INJECT_FRAME_SIZE (36 bytes = 288 bits) at a conservative 1 Mbps
// floor is 288us; with preamble/SIFS overhead, 500us is a generous round
// bound -- 2.5x ESP-NOW's own bound, reflecting the extra uncertainty, not
// a measured value. UNCONDITIONAL regardless of what class frame_format
// later reports the frame actually went out as -- this bound is a
// pre-transmission safety margin, not a post-hoc measurement.
#define RAW_INJECT_WORST_CASE_FRAME_AIRTIME_US 500

static inline bool raw_inject_burst_fits_slot(uint32_t count, uint16_t slot_ms)
{
    uint64_t worst_case_us = (uint64_t)count * RAW_INJECT_WORST_CASE_FRAME_AIRTIME_US;
    return worst_case_us < ((uint64_t)slot_ms * 1000);
}

// Drain margin: esp_wifi_80211_tx()'s own doc comment lists
// ESP_ERR_WIFI_NO_MEM ("out of memory") as a documented failure mode --
// the same shape of error ESP-NOW's esp_now_send() documents
// (ESP_ERR_ESPNOW_NO_MEM) for its own async send queue filling up. No
// stronger synchronicity guarantee is documented for esp_wifi_80211_tx()
// than for esp_now_send(), so this file treats it with the SAME
// conservative assumption (queue-and-return, not guaranteed on-air before
// return) for exactly the same reason ESP-NOW needed a drain margin:
// process_trigger() adds this node's own MAC to its recent-trigger-TX set
// before deciding TX-vs-RX, so a too-early is_transmitting clear risks
// this node re-admitting its own still-in-flight broadcast. Sized larger
// than ESPNOW_TX_DRAIN_MARGIN_US in proportion to this transport's larger
// worst-case per-frame bound above.
#define RAW_INJECT_TX_DRAIN_MARGIN_US 4000

_Static_assert(sizeof(espnow_burst_t) == ESPNOW_BURST_SIZE,
               "ESP-NOW BURST packet must be exactly 12 bytes");
// v5.7.0 (directive-build pre-registered review, attack c): no silent
// truncation path. esp_now_send() itself would refuse an oversized buffer
// at runtime, but a compile-time bound on the payload struct catches a
// future field addition here before it ever reaches that call.
_Static_assert(sizeof(espnow_burst_t) <= ESP_NOW_MAX_DATA_LEN,
               "ESP-NOW burst payload exceeds ESP_NOW_MAX_DATA_LEN");

// Chip type - auto-detected at runtime
// Values from esp_chip_info.h: ESP32=1, S2=2, C3=5(!), S3=9, C2=12, C6=13, H2=16, C5=23
// But binary protocol uses: ESP32=1, S2=2, C3=3, S3=4, C2=5, C6=6, H2=7, C5=8
// We map IDF values to protocol values
#include "esp_chip_info.h"
static uint8_t get_chip_type_for_protocol(void) {
    esp_chip_info_t info;
    esp_chip_info(&info);
    switch (info.model) {
        case CHIP_ESP32:    return 0x01;
        case CHIP_ESP32S2:  return 0x02;
        case CHIP_ESP32C3:  return 0x03;
        case CHIP_ESP32S3:  return 0x04;
        case CHIP_ESP32C2:  return 0x05;
        case CHIP_ESP32C6:  return 0x06;
        case CHIP_ESP32H2:  return 0x07;
        case CHIP_ESP32C5:  return 0x08;
        default:            return 0x03;  // Default to C3
    }
}

// Check if running on multi-core chip (S3, original ESP32)
static bool is_multicore(void) {
    esp_chip_info_t info;
    esp_chip_info(&info);
    return info.cores > 1;
}

// Firmware version - v5.8.0-RAWINJECT (adds raw-injection broadcast DATA
// frame burst transport, PRIMARY round-5 path; see v5.8.0 changelog entry
// above main(). v5.7.0's ESP-NOW broadcast transport, v5.6.0's TX-match
// attribution, burst count default cut 10->4, etc. are unchanged -- see
// those versions' own changelog entries, also above main())
#define FW_VERSION_MAJOR        5
#define FW_VERSION_MINOR        9
#define FW_VERSION_PATCH        1

// =============================================================================
// BINARY HELLO PACKET - 40 bytes, sent on boot and periodically
// =============================================================================
#define BINARY_HELLO_SIZE       40

typedef struct __attribute__((packed)) {
    uint8_t  magic;             // 0xC5 (1)
    uint8_t  type;              // 0x01 = HELLO (1)
    uint8_t  mac[6];            // Node MAC address (6)
    uint8_t  ip[4];             // IP address bytes (4)
    uint8_t  chip_type;         // CHIP_ESP32_C3 = 0x03 (1)
    uint8_t  fw_major;          // Firmware major (1)
    uint8_t  fw_minor;          // Firmware minor (1)
    uint8_t  fw_patch;          // Firmware patch (1)
    char     fw_tag[8];         // "BINARY\0\0" (8)
    uint32_t uptime_s;          // Uptime in seconds (4)
    uint32_t free_heap;         // Free heap bytes (4)
    uint32_t min_heap;          // Minimum heap seen (4)
    uint32_t crc32;             // CRC32 checksum (4)
} binary_hello_t;               // Total: 40 bytes

_Static_assert(sizeof(binary_hello_t) == BINARY_HELLO_SIZE,
               "Binary HELLO packet must be exactly 40 bytes");

// =============================================================================
// BINARY HEARTBEAT PACKET - 34 bytes, sent every 2 seconds
// =============================================================================
// v5.4.0: appended die_temp_c_x10 before crc32 (24 -> 26 bytes).
// v5.4.1: appended csi_frames_captured + bfi_frames_captured before crc32
// (26 -> 34 bytes), cumulative since boot -- lets the backend detect a node
// whose CSI/BFI capture has silently stalled by comparing counters across
// heartbeats instead of trusting the node's own health view. bfi_frames_captured
// is always 0 on builds without CONFIG_ENABLE_BFI_CAPTURE.
// Old firmware still sends the shorter v1 (24B) / v2 (26B) layouts; the server
// accepts all three, keyed on packet length -- see BINARY_HEARTBEAT_SIZE_V2 /
// BINARY_HEARTBEAT_SIZE_V3 in protocol_constants.py.
// v5.4.2 (heartbeat v4): appended the four DROP counters before crc32
// (34 -> 50 bytes). Cumulative since boot, same convention as the capture
// counters above.
//
// WHY. The capture counters say how many frames the node ENQUEUED; nothing
// said how many it then failed to deliver, so per-node capture YIELD was not
// computable and a node losing most of its frames was indistinguishable from
// a quiet one. On 2026-08-12 that gap cost six days: the server's UDP socket
// had silently discarded 1.34 M datagrams (about half the CSI and three
// quarters of the BFI stream) while every health surface reported green, and
// the only way to find it was to compare firmware counters against ingested
// packets by hand. With these four fields the backend can compute yield per
// node per heartbeat and alarm on it directly.
//
// Old firmware keeps sending the shorter v1 (24B) / v2 (26B) / v3 (34B)
// layouts; the server accepts all four, keyed on packet length, so a fleet
// may be flashed progressively. See BINARY_HEARTBEAT_SIZE_V4 in
// protocol_constants.py.
//
// v5.6.0 (heartbeat v5, 2026-08-14): appended csi_attribution_dropped
// before crc32 (50 -> 54 bytes). Every earlier length threshold and byte
// offset is UNCHANGED -- this is append-only, so the four v4 fields stay
// at bytes 30:46 exactly as before, and a v4-length (50B) datagram from a
// not-yet-flashed node still parses correctly; the server reports the new
// field as None (never 0) when it is absent, matching the None-vs-0
// convention the v4 counters themselves established. See
// BINARY_HEARTBEAT_SIZE_V5 in protocol_constants.py.
//
// WHY. The TX-match attribution fix (this same image; see the v5.6.0
// changelog above main()) added a THIRD failure mode alongside capture
// drops (g_csi_drops) and delivery drops (the v4 quartet): a frame that
// arrived and was captured, but that process_trigger()'s three-layer
// predicate refused to attribute to any trigger (g_csi_attribution_dropped).
// That counter was log-only in the first draft of this fix (ESP_LOGW only,
// serial-console-only visibility) -- insufficient for the pilot rollout's
// watch criteria, which needs it readable from the server API without a
// serial console per node. Caveat (directive-build round-2 review): this
// counter measures the guard FIRING -- i.e. correctly refusing an
// ambiguous item -- NOT the guard FAILING. It cannot detect actual
// misattribution that slipped through layer 1's race window (see that
// layer's comment). Misattribution detection is the post-flash
// seq-capture audit's job, not this counter's.
#define BINARY_HEARTBEAT_SIZE   62

typedef struct __attribute__((packed)) {
    uint8_t  magic;                 // 0xC5 (1)
    uint8_t  type;                  // 0x02 = HEARTBEAT (1)
    uint8_t  mac[6];                // Node MAC address (6)
    uint32_t uptime_s;              // Uptime in seconds (4)
    uint32_t free_heap;             // Free heap bytes (4)
    uint32_t min_heap;              // Minimum heap seen (4)
    int16_t  die_temp_c_x10;        // On-die temp * 10, INT16_MIN = unavailable (2)
    uint32_t csi_frames_captured;   // Cumulative CSI frames captured since boot (4)
    uint32_t bfi_frames_captured;   // Cumulative BFI frames captured since boot (4)
    uint32_t csi_drops;             // v4: CSI ring buffer full (4)
    uint32_t udp_send_failures;     // v4: sendto() failed (4)
    uint32_t udp_mutex_timeouts;    // v4: UDP mutex timed out (4)
    uint32_t batch_drops;           // v4: whole batch discarded, mutex unavailable (4)
    uint32_t csi_attribution_dropped; // v5: TX-match attribution refused item (4)
    // v6 RF ASSOCIATION TELEMETRY. A node can be associated, heartbeating and
    // fully "online" while hearing almost nothing, and nothing in the previous
    // 54 bytes could tell that apart from a healthy node. Measured 2026-08-28:
    // three nodes delivered 4.9-6.4 CSI frames/s against a fleet flat at
    // 192-213, with die temperatures 44-50 C against 56-63 -- radios doing a
    // fraction of the work -- and every existing field read normal. One of
    // them sat 40 cm from the occupant, so it was not distance. Without
    // channel/BSSID/RSSI the difference between "deaf" and "quiet room" is not
    // observable from the server at all, which is how this starved half the
    // corpus baseline fit unnoticed.
    int8_t   ap_rssi_dbm;           // v6: STA RSSI to its AP, 0 = unavailable (1)
    uint8_t  ap_channel;            // v6: primary channel, 0 = unavailable (1)
    uint8_t  ap_bssid[6];           // v6: associated AP BSSID, all-zero = unavailable (6)
    uint32_t crc32;                 // CRC32 checksum (4)
} binary_heartbeat_t;               // Total: 62 bytes

_Static_assert(sizeof(binary_heartbeat_t) == BINARY_HEARTBEAT_SIZE,
               "Binary HEARTBEAT packet must be exactly 54 bytes");

// =============================================================================
// BINARY TRIGGER PACKET - 32 bytes, received from server
// =============================================================================
#define BINARY_TRIGGER_SIZE     32

typedef struct __attribute__((packed)) {
    uint8_t  magic;             // 0xC5 (1)
    uint8_t  type;              // 0x10 = TRIGGER (1)
    uint8_t  tx_mac[6];         // MAC of transmitting node (6)
    uint32_t seq;               // Sequence number (4)
    uint16_t burst_count;       // Number of burst packets (2)
    uint16_t slot_ms;           // Slot duration in ms (2)
    uint8_t  samples_per_meas;  // Samples to collect (1)
    uint8_t  send_all_samples;  // 0=best only, 1=send all (1)
    uint8_t  slot_id;           // v5.0: Slot identifier for TX→RX mapping (1)
    uint8_t  reserved[9];       // Future use (9)
    uint32_t crc32;             // CRC32 checksum (4)
} binary_trigger_t;             // Total: 32 bytes

_Static_assert(sizeof(binary_trigger_t) == BINARY_TRIGGER_SIZE,
               "Binary TRIGGER packet must be exactly 32 bytes");

// =============================================================================
// BINARY DISCOVER PACKET - 8 bytes, received from server
// =============================================================================
#define BINARY_DISCOVER_SIZE    8

typedef struct __attribute__((packed)) {
    uint8_t  magic;             // 0xC5 (1)
    uint8_t  type;              // 0x11 = DISCOVER (1)
    uint16_t reserved;          // Padding (2)
    uint32_t crc32;             // CRC32 checksum (4)
} binary_discover_t;            // Total: 8 bytes

_Static_assert(sizeof(binary_discover_t) == BINARY_DISCOVER_SIZE,
               "Binary DISCOVER packet must be exactly 8 bytes");

// =============================================================================
// BINARY CSI BATCH V5.0 - Adaptive streaming with minimal headers
// =============================================================================
#define BINARY_CSI_MAGIC_V5_0   0xC5
#define BINARY_CSI_MAGIC_V5_1   0x07  // Version 5.0
#define BINARY_BATCH_HEADER_SIZE 8
#define BINARY_SAMPLE_SIZE_V5   107   // pre-v5.8.0 sample width (no trigger_seq)
// v5.8.0: appends trigger_seq (uint32) to every sample -- the cross-node
// frame identifier (see csi_packet_t.trigger_seq's own comment: stamped
// from g_trigger_state.seq, identical across every node that captured a
// frame from the SAME server-issued trigger, unlike this batch header's
// own `seq` field two lines below, which is a per-node local batch
// counter and is NOT comparable across nodes). The server detects which
// width a given packet uses purely from packet LENGTH (no separate magic
// byte) -- see protocol_constants.py's BINARY_SAMPLE_SIZE_V5_1 for the
// matching server-side constant and detection logic.
#define BINARY_SAMPLE_SIZE_V5_1 111   // BINARY_SAMPLE_SIZE_V5 + trigger_seq(4)
#define MAX_SAMPLES_PER_BATCH   12    // (1400 MTU - 8 header - 4 CRC) / 111
#define BATCH_BUFFER_SIZE       1400  // MTU-sized buffer
#define SILENCE_TIMEOUT_US      1500  // v5.2: 1.5ms silence = batch complete (was 3ms)

// Batch header (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];          // 0xC5, 0x07 (2)
    uint8_t  slot_id;           // TX identity from trigger (1)
    uint8_t  sample_count;      // Number of samples in payload (1)
    uint16_t seq;               // Batch sequence number (2)
    uint16_t timestamp_lo;      // Low 16 bits of first sample timestamp (2)
} batch_header_v5_t;            // Total: 8 bytes

_Static_assert(sizeof(batch_header_v5_t) == BINARY_BATCH_HEADER_SIZE,
               "Batch header v5 must be exactly 8 bytes");

// Per-sample data (111 bytes, v5.8.0+)
typedef struct __attribute__((packed)) {
    uint16_t timestamp_delta;   // µs offset from batch timestamp (2)
    int8_t   rssi;              // Signal strength (1)
    int8_t   iq_data[104];      // 52 subcarriers × 2 (I/Q pairs) (104)
    uint32_t trigger_seq;       // v5.8.0: cross-node frame identifier (4)
} sample_v5_t;                  // Total: 111 bytes

_Static_assert(sizeof(sample_v5_t) == BINARY_SAMPLE_SIZE_V5_1,
               "Sample v5.1 must be exactly 111 bytes");

// =============================================================================
// BINARY CSI BATCH V6.0 - Variable-width CSI for WiFi 6 (ESP32-C6)
// =============================================================================
#if CONFIG_SOC_WIFI_HE_SUPPORT
#define BINARY_CSI_MAGIC_V6     0x08  // Version 6.0 (second magic byte)
#define BINARY_V6_HEADER_SIZE   12    // 8 + 4 extra bytes

// v6 batch header (12 bytes)
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];          // 0xC5, 0x08 (2)
    uint8_t  slot_id;           // TX identity (1)
    uint8_t  sample_count;      // Number of samples (1)
    uint16_t seq;               // Batch sequence number (2)
    uint16_t timestamp_lo;      // Low 16 bits of first timestamp (2)
    uint16_t num_subcarriers;   // Subcarriers per sample (2) -- NEW
    uint8_t  frame_format;      // 0=non-HT,1=HT,3=HE-SU,4=HE-MU (1) -- NEW
    uint8_t  reserved;          // (1)
} batch_header_v6_t;

_Static_assert(sizeof(batch_header_v6_t) == BINARY_V6_HEADER_SIZE,
               "Batch header v6 must be exactly 12 bytes");

// v6 per-sample: variable width
// Layout (pre-v5.8.0): [timestamp_delta:2][rssi:1][iq_data:num_subcarriers*2]
//   Size = 3 + num_subcarriers * 2
// Layout (v5.8.0+): same, plus [trigger_seq:4] appended
//   Size = 7 + num_subcarriers * 2
// Same cross-node frame identifier as sample_v5_t's own trigger_seq field --
// see its comment above.
#define BINARY_V6_SAMPLE_TRIGGER_SEQ_EXTRA 4
#endif

// =============================================================================
// TRIGGER STATE STRUCTURE (for thread-safe access)
// =============================================================================
typedef struct {
    uint8_t tx_mac_bytes[6];    // Binary MAC for IRAM-safe comparison
    char    tx_mac_str[18];     // String MAC for logging
    uint32_t seq;               // Sequence number
    bool    active;             // Whether filtering is active
    TickType_t activated_at;    // v5.4.1: tick count when active was last set true
                                 // (staleness watchdog, see TRIGGER_ACTIVE_STALE_MS)
} trigger_state_t;

// =============================================================================
// GLOBAL STATE
// =============================================================================
static EventGroupHandle_t s_wifi_event_group = NULL;
static char node_mac[18] = {0};
static uint8_t node_mac_bytes[6] = {0};  // Binary MAC for efficient packing
// v5.8.0: this node's associated AP's BSSID -- used only by the
// raw-injection burst path's Address3 field (see the RAW-INJECTION
// section). Populated once, post-connection (wifi_init_sta(), right after
// g_capture_configured is set) via esp_wifi_sta_get_ap_info(), the robust
// general query for the CURRENT negotiated BSSID -- not read from the
// pre-connection scan_for_ap_on_channel()'s target_bssid local, which is
// only populated on the CONFIG_WIFI_CHANNEL>0 forced-connection branch and
// reflects the scan target, not necessarily the connection actually made.
static uint8_t node_bssid_bytes[6] = {0};
static char node_ip[16] = {0};
static int udp_sock = -1;
static int trigger_sock = -1;
static int broadcast_sock = -1;  // Persistent broadcast socket
static int64_t boot_time_us = 0;
static atomic_bool is_transmitting = false;  // Atomic for cross-task visibility
static int wifi_retry_count = 0;
static int64_t s_last_trigger_time_us = 0;  // For trigger rate limiting (64-bit to avoid overflow)

// Trigger state with spinlock protection for ISR-safe access
static trigger_state_t g_trigger_state = {0};
static portMUX_TYPE trigger_spinlock = portMUX_INITIALIZER_UNLOCKED;
// v5.4.1: cumulative count of times the trigger_task staleness watchdog had
// to force-clear a stuck g_trigger_state.active (see trigger_task()). Should
// be 0 in normal operation now that process_trigger()'s TX branch clears the
// state itself; a nonzero count means some other path is still leaving it set.
static volatile uint32_t g_trigger_stale_recoveries = 0;

// Clear g_trigger_state.active under the spinlock. Shared by process_trigger()
// (both the TX and RX completion paths) and the trigger_task() staleness
// watchdog, so there is exactly one place that un-sets the CSI callback's
// peer filter.
static inline void clear_trigger_state(void)
{
    portENTER_CRITICAL(&trigger_spinlock);
    g_trigger_state.active = false;
    portEXIT_CRITICAL(&trigger_spinlock);
}

// v5.1.1 SAFETY: Static CSI packet buffer to avoid stack overflow in WiFi callback
// The csi_packet_t is ~412 bytes - too large for WiFi task stack
// v5.2.3 FIX: Reverted to single buffer (ping-pong had race condition risk)
static csi_packet_t g_csi_pkt_buffer;
static portMUX_TYPE csi_pkt_spinlock = portMUX_INITIALIZER_UNLOCKED;

// v5.2.0: CSI drop counter for diagnostics (ring buffer full)
static volatile uint32_t g_csi_drops = 0;
// v5.6.0: items drained from the ring at batch time but rejected by the
// TX-match/time-bound attribution check in process_trigger() -- a
// DIFFERENT failure mode from g_csi_drops (which is capture-time, ring
// full). Logged (ESP_LOGD in process_trigger(), periodic ESP_LOGW below
// like g_csi_drops) AND exported over the wire as of heartbeat v5
// (2026-08-14, BINARY_HEARTBEAT_SIZE's v5.6.0 comment) -- the pilot
// rollout's watch criteria needs this readable from the server API, not
// serial-console-only. Cumulative since boot. Reminder of the reviewer's
// caveat: this counts the guard FIRING (refusing an ambiguous item), not
// the guard FAILING -- it cannot detect misattribution that slipped
// through; that is the post-flash seq-capture audit's job.
static volatile uint32_t g_csi_attribution_dropped = 0;
// v5.4.1: cumulative successful CSI captures since boot, mirrors g_bfi_captured
// below. Reported in the heartbeat so the backend can detect a node whose CSI
// subsystem has gone silently dead (heartbeats keep flowing, this counter does
// not) without relying on the node's own view of its health.
static volatile uint32_t g_csi_captured = 0;

// v5.2.1: UDP send error counters for debugging packet loss
static volatile uint32_t g_udp_send_failures = 0;
static volatile uint32_t g_udp_mutex_timeouts = 0;

// v5.7.0: ESP-NOW broadcast burst state. All bounded, no dynamic
// allocation -- see the memory-guards note in the v5.7.0 changelog above
// main() for the full audit.
static bool g_espnow_ready = false;                  // init + broadcast peer add succeeded
// v5.7.0 (directive-build pre-registered review, attack a): lazy init.
// esp_now_init()/esp_now_add_peer() must NOT run at boot on every node --
// the "byte-identical to v5.6.0 when transport=0" claim is about the wire
// default, not about what runs on-device, and an unconditional boot-time
// init would touch the WiFi driver's state even on a node that never once
// receives burst_transport=1. init_espnow_broadcast() is instead called
// lazily, from transmit_burst()'s dispatch, the first time (and only the
// first time -- see g_espnow_init_attempted below) this node is actually
// asked to send via ESP-NOW.
// ONE-SHOT, NOT RETRIED, on either outcome -- deliberately different from
// this file's RETRY_CAPTURE_CALL pattern for CSI/BFI capture bring-up
// (bounded retries, then reboot on exhaustion). That pattern exists
// because a wedged CSI/RF subsystem means total sensing blindness, which
// justifies escalating all the way to a reboot. A failed ESP-NOW init has
// a much cheaper, already-safe outcome: this node's burst_transport=espnow
// requests fall back to UDP for the rest of this boot, which is exactly
// this image's pre-v5.7.0 behavior -- nothing is lost, so there is
// nothing worth spending retries or a reboot on. A transient failure that
// would have succeeded on a second attempt is recoverable the ordinary
// way instead: the node's next reboot (which happens for unrelated
// reasons periodically anyway) gets a fresh, un-latched attempt.
static bool g_espnow_init_attempted = false;          // one-shot lazy-init latch, set on first attempt regardless of outcome
static bool g_espnow_rate_pinned = false;             // last esp_now_set_peer_rate_config() call succeeded
static uint8_t g_espnow_last_rate_index = 0xFF;       // 0xFF = "never applied" sentinel, not a valid index (table is 0-3)
static volatile uint32_t g_espnow_send_failures = 0;  // esp_now_send() returned != ESP_OK
static volatile uint32_t g_espnow_fallback_to_udp = 0; // burst_transport asked for espnow but it was not usable (not ready, or rate not pinned -- see transmit_burst())

// v5.8.0: raw-injection state. No init-latch/rate-pinned pair analogous to
// ESP-NOW's -- esp_wifi_80211_tx() needs no separate init call (unlike
// esp_now_init()) and has no confirmed rate-pin mechanism to gate on (see
// the RAW-INJECTION section's rate-control comment). Kept as its OWN
// counters, not shared with ESP-NOW's, so the two transports' diagnostics
// don't conflate (e.g. raw-inject failing while ESP-NOW is fine, or vice
// versa, should be distinguishable from the heartbeat/log alone).
static volatile uint32_t g_raw_inject_send_failures = 0;   // esp_wifi_80211_tx() returned != ESP_OK
static volatile uint32_t g_raw_inject_fallback_to_udp = 0; // burst_transport asked for raw_inject but the slot-fit guard refused it

// v5.4.2: whole-batch drops (batch_send_mutex unavailable or timed out).
// Previously the only uninstrumented loss path in the node: no counter, no
// log. See the heartbeat v4 comment for why these now leave the board.
static volatile uint32_t g_batch_drops = 0;
// v5.5: CSI batches dispatched toward the server (incremented at the
// send_to_server call in finalize_and_send_batch_v5). Watched by the CSI
// delivery watchdog in heartbeat_task: field incident 2026-08-13 (node
// A5:02:40) had capture ALIVE (csi_frames_captured growing, heartbeats
// delivered) while no CSI batch left the node for 7 days -- a wedge mode
// invisible to both the task watchdog and the BFI/capture watchdog, and
// only a power cycle recovered it. NOTE: the registry's packets_sent field
// is 0 fleet-wide (never incremented anywhere); do not key any health check
// on it.
static volatile uint32_t g_csi_batches_sent = 0;

// CSI ring buffer for reliable data capture
static RingbufHandle_t csi_ringbuf = NULL;

// On-die temperature sensor handle, installed once in app_main()
static temperature_sensor_handle_t temp_sensor_handle = NULL;

// =============================================================================
// BFI (BEAMFORMING FEEDBACK) CAPTURE - opportunistic, coexists with CSI
// =============================================================================
// When CONFIG_ENABLE_BFI_CAPTURE is set, the node ALSO passively captures
// 802.11 Compressed Beamforming Report Action frames (VHT cat 21 / HE cat 30)
// overheard on its channel and forwards the raw body as MSG_TYPE_BFI. This
// reuses the promiscuous RX path already enabled for CSI: a second callback
// on the same radio, same channel dwell. CSI path is untouched. Additive
// protocol -- old servers ignore MSG_TYPE_BFI, new servers add one handler.
#ifdef CONFIG_ENABLE_BFI_CAPTURE
#define BFI_MAX_BODY      512   // action-body cap (prototype reports ~200 B)
#define MAX_BFI_ITEMS     32   // absorb bursts between ~5s heartbeat drains
typedef struct {
    int64_t  timestamp_us;
    uint8_t  src_mac[6];      // beamformee (STA that computed the report)
    uint8_t  dst_mac[6];      // beamformer (usually the AP)
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  bb_format;       // cur_bb_format (HE) / sig_mode (legacy)
    uint8_t  subtype;         // 13 = Action, 14 = Action No Ack
    uint8_t  category;        // 21 = VHT, 30 = HE
    uint8_t  action;
    uint16_t body_len;        // bytes of action body captured
    uint8_t  body[BFI_MAX_BODY];
} bfi_capture_t;
#define BFI_RINGBUF_SIZE (MAX_BFI_ITEMS * (sizeof(bfi_capture_t) + 16))
// BFI watchdog: if connected but g_bfi_captured has not advanced for this
// long, the promiscuous RX path has stalled without a re-arm event firing
// (e.g. a re-association that kept the IP, or a channel event). Re-arm
// proactively from heartbeat_task. Re-arming is cheap and idempotent, so a
// false trigger during a genuinely quiet window is harmless.
#define BFI_WATCHDOG_STALL_S  45
// CSI delivery watchdog (v5.5): if capture is advancing (we hear the mesh
// sounding) but no CSI batch has been dispatched for this long, the delivery
// path is wedged (2026-08-13 A5:02:40 incident: 7 days in that state, only a
// power cycle recovered). First expiry re-arms capture; a second consecutive
// expiry reboots, mirroring the restart_capture() escalation philosophy:
// a reboot is the only recovery field data has shown to work.
#define CSI_DELIVERY_WATCHDOG_S  180
static RingbufHandle_t bfi_ringbuf = NULL;
static volatile uint32_t g_bfi_captured = 0;
static volatile uint32_t g_bfi_drops = 0;

// Wire header (36 bytes) prepended to the raw action body on the network.
typedef struct __attribute__((packed)) {
    uint8_t  magic;           // 0xC5
    uint8_t  type;            // 0x40 = MSG_TYPE_BFI
    uint8_t  node_mac[6];     // capturing node
    uint8_t  src_mac[6];      // beamformee
    uint8_t  dst_mac[6];      // beamformer
    int64_t  timestamp_us;
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  bb_format;
    uint8_t  subtype;
    uint8_t  category;
    uint8_t  action;
    uint16_t body_len;        // followed by body_len bytes, then uint32 CRC32
} bfi_header_t;
_Static_assert(sizeof(bfi_header_t) == 36, "bfi_header_t must be 36 bytes");
#endif  // CONFIG_ENABLE_BFI_CAPTURE

// Socket mutex for thread-safe UDP sends
static SemaphoreHandle_t socket_mutex = NULL;

// Server address (cached)
static struct sockaddr_in server_addr;
static struct sockaddr_in broadcast_addr;
// v5.9.0: burst destination is an airtime lever, not a sensing one. The CSI
// receivers overhear the transmitter's STA->AP UPLINK (info->mac = a node,
// verified: the src_mac filter rejects every AP-sourced frame), so the burst
// datagram's destination does NOT affect illumination -- only whether the AP
// wastefully REBROADCASTS it to the whole BSS at a basic rate (that
// rebroadcast is captured-then-rejected, pure airtime waste). g_burst_unicast
// (from trigger reserved[4], default 0 = broadcast = byte-identical to
// pre-v5.9) routes bursts UNICAST to the server instead, on the burst port,
// eliminating the AP rebroadcast while preserving the identical overhearable
// uplink. The server silently ignores MSG_TYPE_BURST at any port.
static struct sockaddr_in burst_unicast_addr;
static volatile uint8_t g_burst_unicast = 0;

// mDNS-resolved server IP (empty = not resolved, use Kconfig fallback)
#ifdef CONFIG_MDNS_DISCOVERY
static char resolved_server_ip[16] = {0};
#endif

// =============================================================================
// BATCH CSI STATE
// =============================================================================
static uint8_t g_batch_buffer[BATCH_BUFFER_SIZE];
static size_t g_batch_offset = BINARY_BATCH_HEADER_SIZE;  // Start after header
static uint8_t g_current_slot_id = 0;
static uint16_t g_batch_seq = 0;
static uint32_t g_batch_first_timestamp = 0;
static uint8_t g_batch_sample_count = 0;
static esp_timer_handle_t silence_timer = NULL;
static portMUX_TYPE batch_spinlock = portMUX_INITIALIZER_UNLOCKED;

// v5.2.3 FIX: Mutex to protect send buffer during copy-CRC-send operation
// Required because finalize_and_send_batch_v5() can be called from:
// 1. trigger_task (via process_trigger)
// 2. timer_task (via silence_timer_callback)
static SemaphoreHandle_t batch_send_mutex = NULL;

#if CONFIG_SOC_WIFI_HE_SUPPORT
// v6 batch state for WiFi 6 variable-width CSI
static bool g_batch_is_v6 = false;           // Current batch uses v6 format
static uint16_t g_batch_v6_num_subcarriers = 0;  // Subcarriers in current v6 batch
static uint8_t g_batch_v6_frame_format = 0;  // Frame format of current v6 batch
#endif

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
static void send_hello(void);
// v5.7.0: defined with the rest of the ESP-NOW broadcast init/rate-pinning
// block below (near transmit_burst_espnow). Called LAZILY from
// transmit_burst()'s dispatch (also below), not from wifi_init_sta() --
// see g_espnow_init_attempted's comment for why. This forward declaration
// exists only because transmit_burst()'s dispatch wrapper is defined
// before init_espnow_broadcast() in the file, same as any other forward
// reference here.
static void init_espnow_broadcast(void);

// =============================================================================
// IRAM FUNCTIONS - Time-critical code
// =============================================================================

/**
 * IRAM-safe 6-byte MAC comparison
 * Returns true if MACs match
 */
static inline bool IRAM_ATTR mac_bytes_equal(const uint8_t *mac1, const uint8_t *mac2)
{
    return (mac1[0] == mac2[0] && mac1[1] == mac2[1] && mac1[2] == mac2[2] &&
            mac1[3] == mac2[3] && mac1[4] == mac2[4] && mac1[5] == mac2[5]);
}

// =============================================================================
// v5.6.0 ROUND-3: recent-trigger-TX set (capture-time gate)
// =============================================================================
// 2026-08-14, directive-build round-2 review of 10b9c352: the ring-clear
// removal (v5.6.0's whole point) recovers NOTHING, because csi_callback()'s
// PRE-EXISTING capture-time filter gated on g_trigger_state.tx_mac_bytes --
// THIS node's own, possibly-stale, view of "the current trigger's TX" --
// and rejected the frame before it ever reached the ring, upstream of
// every batch-time layer this file's v5.6.0/round-2 work touched. Network
// jitter means the actual TX node can start transmitting trigger N's burst
// before THIS node's own trigger_task has processed trigger N locally (its
// g_trigger_state still reflects N-1). In the COMMON case (N's TX differs
// from N-1's TX, true for any round-robin/rotation, not just back-to-back
// same-TX triggers), the exact single-MAC filter rejected 100% of those
// early frames, unconditionally -- the exact population the ring-clear fix
// exists to recover.
//
// FIX: replace the exact-current-trigger filter with a small, FIXED-SIZE
// set of the last K trigger TX MACs this node has locally processed
// (bounded array, no dict -- PI memory rule, see the memory-guards section
// of the prep doc). A frame is admitted into the ring if its src_mac
// matches ANY of these, not just the CURRENT trigger's specific TX --
// recovering the early-arrival case whenever trigger N's TX was ALSO a
// recent trigger's TX.
//
// SIZING RULE (2026-08-14, directive-build's mode measurement, round-3
// review): K MUST EXCEED THE WIDEST ROTATION THE SCHEDULER CAN PRODUCE,
// not just the smallest common case. An earlier draft set K=4, sized for
// 2-node focus mode alone (only 2 possible TXs, always within any K>=2).
// Measured against the fleet's actual mode mix: K=4 covered focus fully
// and zone rotation WITHIN a zone (site zones are 2/2/1/3/2 nodes, so
// within-zone rotation stays inside a 4-wide window) -- but LOST recovery
// at zone-switch boundaries, on the perimeter zone (>4 boundary nodes),
// and during full coverage round-robin (up to 10 transmitters). Recovery
// that only works in SOME scheduler modes is not an acceptable basis for
// the pilot's own success criterion (server-side delivered rate), which
// assumes the fix works regardless of which mode is active when the pilot
// happens to be measured. K is set below to exceed the fleet's widest
// rotation (10 transmitters) with headroom, making recovery
// MODE-INDEPENDENT rather than mode-lucky. Cost: a fixed 84-byte increase
// in this array's .bss footprint (K entries x 7 bytes/entry, see this
// file's memory-guards documentation) against ~190KB of remaining SRAM --
// negligible.
//
// Deliberately still NOT "accept every MAC" at any K -- the ambient
// promiscuous-capture river runs ~3200 frames/s and would flood a
// same-order-of-magnitude-or-larger fixed-size ring, evicting real burst
// frames instead of admitting them: a NEW loss mode, not a fix. K stays
// bounded to the fleet's known rotation width, not open-ended.
//
// This broadens WHICH frames reach the ring; it does NOT change how they
// are stamped (still exactly g_trigger_state.seq/src_mac at capture time,
// as before) or how batch-time attribution decides which trigger's batch
// they belong to (see process_trigger()'s OR-composed admission,
// downstream of this set entirely).
#define RECENT_TRIGGER_TX_COUNT 12

typedef struct {
    uint8_t mac[6];
    bool valid;
} recent_trigger_tx_entry_t;

// Protected by trigger_spinlock (the same lock g_trigger_state already
// uses -- both are written together in process_trigger(), so sharing the
// lock avoids a second one in csi_callback()'s time-critical path).
static recent_trigger_tx_entry_t g_recent_trigger_tx[RECENT_TRIGGER_TX_COUNT] = {0};
static uint8_t g_recent_trigger_tx_next = 0;

// Caller must already hold trigger_spinlock.
static inline void IRAM_ATTR recent_trigger_tx_add_locked(const uint8_t *mac_bytes)
{
    memcpy(g_recent_trigger_tx[g_recent_trigger_tx_next].mac, mac_bytes, 6);
    g_recent_trigger_tx[g_recent_trigger_tx_next].valid = true;
    g_recent_trigger_tx_next = (uint8_t)((g_recent_trigger_tx_next + 1) % RECENT_TRIGGER_TX_COUNT);
}

// Caller must already hold trigger_spinlock. Bounded loop
// (RECENT_TRIGGER_TX_COUNT iterations), IRAM-safe.
static inline bool IRAM_ATTR recent_trigger_tx_contains_locked(const uint8_t *mac_bytes)
{
    for (int i = 0; i < RECENT_TRIGGER_TX_COUNT; i++) {
        if (g_recent_trigger_tx[i].valid &&
            mac_bytes_equal(g_recent_trigger_tx[i].mac, mac_bytes)) {
            return true;
        }
    }
    return false;
}

/**
 * CSI callback - runs in WiFi task context (NOT ISR!)
 * IRAM_ATTR ensures this runs from RAM for lowest latency
 * Enhanced to capture AGC gain, noise floor, and signal mode for accuracy
 *
 * Per ESP-IDF docs: "The CSI receiving callback function runs from Wi-Fi task."
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html
 *
 * THREAD SAFETY:
 * - Uses spinlock-protected local copy of trigger state
 * - Uses atomic_load for is_transmitting flag
 * - All comparisons use binary data (no string functions)
 * - v5.2.1: Uses task-context critical section macros (not ISR variants)
 * - v5.2.3 FIX: Ring buffer send kept inside spinlock for safety
 *
 * v5.2.2 OPTIMIZATION (kept):
 * - Trigger state check: only copies active flag + MAC (7 bytes vs 32+ bytes)
 *
 * v5.2.3 FIX: Reverted ping-pong buffer optimization
 * - Ring buffer send moved back inside spinlock for thread safety
 * - Ping-pong was unsafe if callbacks could queue faster than send completes
 */
static void IRAM_ATTR csi_callback(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || atomic_load(&is_transmitting)) {
        return;
    }

    // v5.7.0 (directive-build pre-registered review, attack h): cheap
    // early-out for a node hearing its own broadcast. process_trigger()
    // adds this node's own MAC to the recent-trigger-TX set BEFORE
    // deciding whether it is the TX or RX for a trigger (see that call
    // site's comment), so recent_trigger_tx_contains_locked() below would
    // otherwise admit this node's own frames for the duration of any TX
    // burst it just sent. The primary defense against that is
    // is_transmitting's blackout covering the whole on-air window (see
    // transmit_burst_espnow()'s drain-margin comment for why that needs
    // extra care on the ESP-NOW path specifically) -- this is a second,
    // independent, near-free layer: a 6-byte compare, before the
    // spinlock-protected admit_to_ring check below, not a replacement for
    // getting the blackout window right.
    if (mac_bytes_equal(info->mac, node_mac_bytes)) {
        return;
    }

    // v5.6.0 round-3 (2026-08-14, directive-build round-2 review): gate on
    // the recent-trigger-TX set (see its comment block above), NOT on this
    // node's own current/possibly-stale g_trigger_state.tx_mac_bytes -- the
    // old exact-current-trigger filter here was upstream of every batch-
    // time layer this file's v5.6.0 work touched, and rejected the exact
    // early-arrival frames the ring-clear fix exists to recover, in the
    // common (different-TX) case. trigger_seq_local is still stamped from
    // g_trigger_state.seq unconditionally (clear_trigger_state() only
    // clears .active, never .seq -- see its own comment -- so this is
    // always "the most recent trigger this node has locally processed,"
    // which is what process_trigger()'s batch-time seq-match layer needs).
    //
    // v5.7.0 EXPECTATION (directive-build pre-registered review, "mixed-
    // fleet attribution"): info->mac must report the ACTUAL TRANSMITTER
    // (802.11 addr2/TA), not the BSSID (addr3), for an ESP-NOW action
    // frame -- exactly as it already does for this file's existing
    // UDP-broadcast-path frames (send_broadcast_packet()'s DATA frames,
    // where this same field has always been the sender). If a future IDF
    // version or this chip's driver ever reports addr3 for ESP-NOW
    // reception instead, recent_trigger_tx_contains_locked() and every
    // downstream tx_match check silently stop matching for the espnow
    // path specifically (UDP is unaffected) -- this is BENCH-CHECKABLE,
    // not verifiable from source alone; see
    // docs/6-firmware/FLASH_RUNBOOK_V5_7_0.md's phase-1 gate.
    uint32_t trigger_seq_local;
    bool admit_to_ring;
    portENTER_CRITICAL(&trigger_spinlock);
    trigger_seq_local = g_trigger_state.seq;
    admit_to_ring = recent_trigger_tx_contains_locked(info->mac);
    portEXIT_CRITICAL(&trigger_spinlock);

    if (!admit_to_ring) {
        return;
    }

#if CONFIG_SOC_WIFI_HE_SUPPORT
    // C6: Must check validity flag before processing CSI
    if (!info->rx_ctrl.rx_channel_estimate_info_vld) {
        return;  // Discard invalid CSI
    }
#endif

    // v5.2.3 FIX: Use single buffer with ring buffer send inside spinlock
    // This is safer than ping-pong which had potential race if callbacks queued
    portENTER_CRITICAL(&csi_pkt_spinlock);

    // Build packet in static buffer - capture all available metadata
    g_csi_pkt_buffer.timestamp_us = esp_timer_get_time();
#if CONFIG_SOC_WIFI_HE_SUPPORT
    // C6: WiFi 6 rx_ctrl fields
    g_csi_pkt_buffer.bb_format = info->rx_ctrl.cur_bb_format;
    g_csi_pkt_buffer.csi_valid = 1;
    g_csi_pkt_buffer.rssi = info->rx_ctrl.rssi;
    g_csi_pkt_buffer.noise_floor = info->rx_ctrl.noise_floor;
    g_csi_pkt_buffer.channel = info->rx_ctrl.channel;
    g_csi_pkt_buffer.secondary_channel = info->rx_ctrl.second;
    g_csi_pkt_buffer.rate = info->rx_ctrl.rate;
    // Fields not available on C6 -- zero them
    g_csi_pkt_buffer.aggregation = 0;
    g_csi_pkt_buffer.stbc = 0;
    g_csi_pkt_buffer.fec_coding = 0;
    g_csi_pkt_buffer.sgi = 0;
#else
    // C3/S3: Legacy HT rx_ctrl fields
    g_csi_pkt_buffer.rssi = info->rx_ctrl.rssi;
    g_csi_pkt_buffer.noise_floor = info->rx_ctrl.noise_floor;
    g_csi_pkt_buffer.channel = info->rx_ctrl.channel;
    g_csi_pkt_buffer.secondary_channel = info->rx_ctrl.secondary_channel;
    g_csi_pkt_buffer.rate = info->rx_ctrl.rate;
    g_csi_pkt_buffer.sig_mode = info->rx_ctrl.sig_mode;
    g_csi_pkt_buffer.cwb = info->rx_ctrl.cwb;
    g_csi_pkt_buffer.aggregation = info->rx_ctrl.aggregation;
    g_csi_pkt_buffer.stbc = info->rx_ctrl.stbc;
    g_csi_pkt_buffer.fec_coding = info->rx_ctrl.fec_coding;
    g_csi_pkt_buffer.sgi = info->rx_ctrl.sgi;
#endif
    g_csi_pkt_buffer.rx_state = 0;  // Critical for amplitude correction
    g_csi_pkt_buffer.len = (info->len > MAX_CSI_DATA_LEN) ? MAX_CSI_DATA_LEN : info->len;
    memcpy(g_csi_pkt_buffer.src_mac, info->mac, 6);
    // v5.6.0 round-3: "most recent trigger this node has locally processed"
    // (0 only before the very first trigger since boot -- clear_trigger_state()
    // never resets .seq, so this stays meaningful across trigger boundaries,
    // not just while a trigger is nominally "active").
    g_csi_pkt_buffer.trigger_seq = trigger_seq_local;
    memcpy(g_csi_pkt_buffer.data, info->buf, g_csi_pkt_buffer.len);

#if CONFIG_SOC_WIFI_HE_SUPPORT
    // C6 hardware bug: first 4 bytes of CSI buffer may be invalid.
    // If first_word_invalid is set, zero them in our local buffer.
    if (info->first_word_invalid && g_csi_pkt_buffer.len >= 4) {
        memset((void *)g_csi_pkt_buffer.data, 0, 4);
    }
#endif

    // Send to ring buffer (non-blocking, timeout=0)
    // v5.2.3: Kept inside spinlock for thread safety
    if (xRingbufferSend(csi_ringbuf, &g_csi_pkt_buffer, sizeof(csi_packet_t), 0) != pdTRUE) {
        g_csi_drops++;
    } else {
        g_csi_captured++;
    }

    portEXIT_CRITICAL(&csi_pkt_spinlock);
}

#ifdef CONFIG_ENABLE_BFI_CAPTURE
// Static capture buffer for the promiscuous callback (bfi_capture_t is large;
// keep it off the WiFi task stack, same pattern as the CSI callback).
static bfi_capture_t g_bfi_pkt_buffer;
static portMUX_TYPE bfi_pkt_spinlock = portMUX_INITIALIZER_UNLOCKED;

/**
 * Promiscuous RX callback - coexists with the CSI callback on the same radio.
 * Filters (in software, so the promiscuous FILTER is left at default and the
 * CSI path is not disturbed) for Compressed Beamforming Report Action frames
 * and queues the raw body for forwarding. Runs in WiFi task context; kept
 * minimal (copy + ring send), UDP happens off-callback in heartbeat_task.
 */
static void bfi_promisc_cb(void *buf, wifi_promiscuous_pkt_type_t ptype)
{
    // Cheap early-outs: only management frames, never during our own TX burst.
    if (ptype != WIFI_PKT_MGMT || atomic_load(&is_transmitting)) {
        return;
    }
    const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
    const uint8_t *d = p->payload;
    uint16_t flen = p->rx_ctrl.sig_len;
    if (flen < 30) return;                       // need 24 hdr + cat + action + FCS

    uint8_t ftype   = (d[0] >> 2) & 0x03;
    uint8_t subtype = (d[0] >> 4) & 0x0F;
    if (ftype != 0 || (subtype != 13 && subtype != 14)) return;   // Action / No-Ack

    uint8_t category = d[24];                     // body starts after 24-byte mgmt hdr
    if (category != 21 && category != 30) return; // VHT(21) / HE(30) beamforming only

    portENTER_CRITICAL(&bfi_pkt_spinlock);
    g_bfi_pkt_buffer.timestamp_us = esp_timer_get_time();
    memcpy(g_bfi_pkt_buffer.dst_mac, d + 4, 6);   // addr1 = RA (beamformer/AP)
    memcpy(g_bfi_pkt_buffer.src_mac, d + 10, 6);  // addr2 = TA (beamformee/STA)
    g_bfi_pkt_buffer.rssi     = p->rx_ctrl.rssi;
    g_bfi_pkt_buffer.channel  = p->rx_ctrl.channel;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    g_bfi_pkt_buffer.bb_format = p->rx_ctrl.cur_bb_format;
#else
    g_bfi_pkt_buffer.bb_format = p->rx_ctrl.sig_mode;
#endif
    g_bfi_pkt_buffer.subtype  = subtype;
    g_bfi_pkt_buffer.category = category;
    g_bfi_pkt_buffer.action   = d[25];
    uint16_t body_len = flen - 28;                // minus 24 hdr + 4 FCS
    if (body_len > BFI_MAX_BODY) body_len = BFI_MAX_BODY;
    g_bfi_pkt_buffer.body_len = body_len;
    memcpy(g_bfi_pkt_buffer.body, d + 24, body_len);

    if (xRingbufferSend(bfi_ringbuf, &g_bfi_pkt_buffer, sizeof(bfi_capture_t), 0) == pdTRUE) {
        g_bfi_captured++;
    } else {
        g_bfi_drops++;
    }
    portEXIT_CRITICAL(&bfi_pkt_spinlock);
}
#endif  // CONFIG_ENABLE_BFI_CAPTURE

// =============================================================================
// RE-ARM CAPTURE ON (RE)CONNECT
// =============================================================================
// v5.4.1 AUDIT FINDING (root-cause candidate for "heartbeats alive, zero CSI"
// field failures): WIFI_EVENT_STA_CONNECTED fires restart_capture() on the
// very FIRST association too, from the WiFi event task -- concurrently with
// wifi_init_sta()'s own one-time esp_wifi_set_csi_config()/set_csi_rx_cb()
// calls, which run on the main task only after xEventGroupWaitBits() wakes
// (itself racy: GOT_IP sets WIFI_CONNECTED_BIT *before* calling
// restart_capture(), so the main task can resume and start its own setup
// while the event task's restart_capture() is still running). Two tasks
// touching esp_wifi_set_csi()/esp_wifi_set_promiscuous() with no ordering
// guarantee between them can enable CSI acquisition before the acquire mask
// and RX callback are actually configured -- no error is returned, so the
// node comes up looking healthy (WiFi connected, heartbeats flowing) while
// producing zero CSI. g_capture_configured closes the race: restart_capture()
// is a no-op until wifi_init_sta() has completed its one-time setup, so only
// genuine re-arm-on-reconnect calls (which come strictly after that point)
// ever touch the capture path here.
static volatile bool g_capture_configured = false;

// Capture (re-)init health, set false after CAPTURE_REINIT_MAX_RETRIES
// consecutive failures on any stage of restart_capture(); cleared on the next
// fully successful call. Not transmitted on its own -- the backend infers
// RF-death from stalled csi_frames_captured/bfi_frames_captured heartbeat
// counters instead, which do not depend on the node's own view of its health.
static volatile bool g_capture_healthy = true;
static volatile uint32_t g_capture_reinit_failures = 0;

// Retry `call_expr` (an esp_err_t-returning expression) up to
// CAPTURE_REINIT_MAX_RETRIES times with exponential backoff before giving up.
// Clears `ok_flag` to false only on final exhaustion, so one macro invocation
// per capture-path stage composes into an overall success/failure verdict.
#define RETRY_CAPTURE_CALL(call_expr, what, ok_flag) do {                          \
    esp_err_t _e = (call_expr);                                                    \
    for (int _attempt = 1; _e != ESP_OK && _attempt <= CAPTURE_REINIT_MAX_RETRIES; \
         _attempt++) {                                                             \
        vTaskDelay(pdMS_TO_TICKS(CAPTURE_REINIT_RETRY_BASE_MS << (_attempt - 1)));  \
        _e = (call_expr);                                                          \
        if (_e == ESP_OK) {                                                        \
            ESP_LOGI(TAG, "%s: %s recovered on retry %d/%d",                       \
                     __func__, (what), _attempt, CAPTURE_REINIT_MAX_RETRIES);      \
        }                                                                           \
    }                                                                               \
    if (_e != ESP_OK) {                                                            \
        ESP_LOGE(TAG, "%s: %s failed after %d retries: %s",                        \
                 __func__, (what), CAPTURE_REINIT_MAX_RETRIES, esp_err_to_name(_e)); \
        (ok_flag) = false;                                                         \
    }                                                                               \
} while (0)

// After a WiFi re-association the CSI config persists, but the promiscuous /
// BFI callback path does not reliably survive, so BFI capture silently dies
// until reboot. Re-assert the whole capture path on every GOT_IP. Retries
// each stage with backoff (RETRY_CAPTURE_CALL); if the whole path still
// cannot be re-armed, this is no longer a transient blip -- self-heal with a
// reboot, since a power cycle is the only recovery field data has shown to
// work for a wedged CSI/RF subsystem. Idempotent and safe to call repeatedly.
static void restart_capture(void)
{
    if (!g_capture_configured) {
        // Initial connection: wifi_init_sta() owns first-time setup (acquire
        // mask + RX callback registration). Re-arming here would race it --
        // see the g_capture_configured comment above.
        ESP_LOGD(TAG, "restart_capture: skipped, initial setup not complete yet");
        return;
    }

    bool ok = true;
    RETRY_CAPTURE_CALL(esp_wifi_set_promiscuous(true), "set_promiscuous", ok);
#ifdef CONFIG_ENABLE_BFI_CAPTURE
    RETRY_CAPTURE_CALL(esp_wifi_set_promiscuous_rx_cb(bfi_promisc_cb), "set_promiscuous_rx_cb", ok);
#endif
    RETRY_CAPTURE_CALL(esp_wifi_set_csi(true), "set_csi", ok);

    if (ok) {
        g_capture_healthy = true;
        ESP_LOGI(TAG, "capture re-armed (CSI + promiscuous/BFI) after (re)connect");
        return;
    }

    g_capture_reinit_failures++;
    g_capture_healthy = false;
    ESP_LOGE(TAG, "capture re-init FAILED after retries (failure #%lu) -- "
                   "CSI/BFI subsystem wedged, rebooting to recover",
             (unsigned long)g_capture_reinit_failures);
    vTaskDelay(pdMS_TO_TICKS(100));  // let the log line flush, same pattern as HEAP_CRITICAL_THRESHOLD
    esp_restart();
}

// =============================================================================
// WIFI EVENT HANDLER
// =============================================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                // Don't auto-connect here - we connect manually after channel scan
                ESP_LOGI(TAG, "WiFi STA started");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                wifi_retry_count++;
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

                // Use exponential backoff: 1s, 2s, 4s, 5s (max), 5s, 5s...
                uint32_t delay_ms = (wifi_retry_count < 3) ? (1000 << wifi_retry_count) : WIFI_RECONNECT_DELAY_MS;
                if (delay_ms > WIFI_RECONNECT_DELAY_MS) delay_ms = WIFI_RECONNECT_DELAY_MS;

                ESP_LOGW(TAG, "WiFi disconnected, retry #%d in %lu ms",
                         wifi_retry_count, (unsigned long)delay_ms);

                vTaskDelay(pdMS_TO_TICKS(delay_ms));
                esp_wifi_connect();  // Always keep trying - reliability is key
                break;

            case WIFI_EVENT_STA_CONNECTED:
                wifi_retry_count = 0;
                ESP_LOGI(TAG, "WiFi connected to AP");
                // Re-arm capture on every association, not just GOT_IP: a
                // re-association that retains the DHCP lease fires CONNECTED
                // without a new GOT_IP, and would otherwise leave the
                // promiscuous/BFI path dead until reboot.
                restart_capture();
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(node_ip, sizeof(node_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", node_ip);
        wifi_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        // Re-arm CSI + promiscuous/BFI capture on every (re)connect.
        restart_capture();
    }
}

// =============================================================================
// SCAN FOR AP ON PREFERRED CHANNEL
// =============================================================================
#if CONFIG_WIFI_CHANNEL > 0
static bool scan_for_ap_on_channel(uint8_t *out_bssid, uint8_t *out_channel)
{
    ESP_LOGI(TAG, "Scanning for SSID '%s' on channel %d...", CONFIG_WIFI_SSID, CONFIG_WIFI_CHANNEL);

    wifi_scan_config_t scan_config = {
        .ssid = (uint8_t *)CONFIG_WIFI_SSID,
        .bssid = NULL,
        .channel = 0,  // Scan all channels
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    // v5.2.1: WiFi scan can take several seconds - feed watchdog before and after
    // to prevent timeout if watchdog is already configured
    esp_task_wdt_reset();
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    esp_task_wdt_reset();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi scan failed: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);

    if (ap_count == 0) {
        ESP_LOGW(TAG, "No APs found");
        return false;
    }

    wifi_ap_record_t *ap_records = malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (!ap_records) {
        ESP_LOGE(TAG, "Failed to allocate memory for scan results");
        return false;
    }

    esp_wifi_scan_get_ap_records(&ap_count, ap_records);

    // Find AP on preferred channel, or best signal as fallback
    int preferred_idx = -1;
    int best_idx = -1;
    int8_t best_rssi = -127;

    for (int i = 0; i < ap_count; i++) {
        if (strcmp((char *)ap_records[i].ssid, CONFIG_WIFI_SSID) == 0) {
            ESP_LOGI(TAG, "  Found: ch=%d, rssi=%d, bssid=%02X:%02X:%02X:%02X:%02X:%02X",
                     ap_records[i].primary,
                     ap_records[i].rssi,
                     ap_records[i].bssid[0], ap_records[i].bssid[1], ap_records[i].bssid[2],
                     ap_records[i].bssid[3], ap_records[i].bssid[4], ap_records[i].bssid[5]);

            // Check if on preferred channel
            if (ap_records[i].primary == CONFIG_WIFI_CHANNEL) {
                if (preferred_idx < 0 || ap_records[i].rssi > ap_records[preferred_idx].rssi) {
                    preferred_idx = i;
                }
            }

            // Track best signal overall
            if (ap_records[i].rssi > best_rssi) {
                best_rssi = ap_records[i].rssi;
                best_idx = i;
            }
        }
    }

    int selected_idx = (preferred_idx >= 0) ? preferred_idx : best_idx;

    if (selected_idx >= 0) {
        memcpy(out_bssid, ap_records[selected_idx].bssid, 6);
        *out_channel = ap_records[selected_idx].primary;

        if (preferred_idx >= 0) {
            ESP_LOGI(TAG, "Selected AP on preferred channel %d", *out_channel);
        } else {
            ESP_LOGW(TAG, "No AP on channel %d, using best signal on channel %d",
                     CONFIG_WIFI_CHANNEL, *out_channel);
        }

        free(ap_records);
        return true;
    }

    ESP_LOGE(TAG, "No matching AP found for SSID '%s'", CONFIG_WIFI_SSID);
    free(ap_records);
    return false;
}
#endif

// =============================================================================
// INITIALIZE WIFI WITH CSI
// =============================================================================
static esp_err_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // Increase WiFi task stack for CSI processing
    cfg.static_rx_buf_num = 16;
    cfg.dynamic_rx_buf_num = 64;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Disable power save for lowest latency
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Regulatory domain: Austria (EU). Required for 5 GHz operation on C5.
    // 2.4 GHz: chans 1-13. 5 GHz UNII-1 (36-48): no DFS. UNII-2 (52+): DFS.
    // ieee80211d_enabled=false: our config is authoritative, AP country IE
    // cannot override (deterministic testbed behavior).
    esp_err_t cc_err = esp_wifi_set_country_code("AT", false);
    if (cc_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_country_code failed: %s", esp_err_to_name(cc_err));
    }

#if CONFIG_SOC_WIFI_HE_SUPPORT
    // Enable WiFi 6 (802.11ax) so the PHY produces HE-LTF CSI (242 subcarriers)
    // instead of legacy HT (52). Without this the chip downgrades to HT mode.
    //
    // Dual-band parts (C5) run in band-mode AUTO and REJECT the singular
    // esp_wifi_set_protocol() with ESP_ERR_NOT_SUPPORTED -- they require the
    // per-band esp_wifi_set_protocols(). Single-band HE parts (C6) keep the
    // singular API. Guarded by CONFIG_SOC_WIFI_SUPPORT_5G (set on C5, not C6).
#if CONFIG_SOC_WIFI_SUPPORT_5G
    wifi_protocols_t proto_cfg = {
        // 2.4 GHz: b/g/n/ax
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                  WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        // 5 GHz: a/n/ac/ax (b/g are invalid on 5 GHz)
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N |
                  WIFI_PROTOCOL_11AC | WIFI_PROTOCOL_11AX,
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(WIFI_IF_STA, &proto_cfg));
#else
    uint8_t protocols = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                        WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX;
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, protocols));
#endif
    ESP_LOGI(TAG, "WiFi 6 (802.11ax) protocol enabled for HE-LTF CSI");
#endif

    // Start WiFi first (required for scanning)
    ESP_ERROR_CHECK(esp_wifi_start());

    // Get MAC address (both string and binary forms)
    esp_wifi_get_mac(WIFI_IF_STA, node_mac_bytes);
    snprintf(node_mac, sizeof(node_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             node_mac_bytes[0], node_mac_bytes[1], node_mac_bytes[2],
             node_mac_bytes[3], node_mac_bytes[4], node_mac_bytes[5]);
    ESP_LOGI(TAG, "Node MAC: %s", node_mac);

#if CONFIG_WIFI_CHANNEL > 0
    // Scan for AP on preferred channel
    uint8_t target_bssid[6];
    uint8_t target_channel;

    if (scan_for_ap_on_channel(target_bssid, &target_channel)) {
        // Set BSSID to force connection to specific AP
        memcpy(wifi_config.sta.bssid, target_bssid, 6);
        wifi_config.sta.bssid_set = true;
        wifi_config.sta.channel = target_channel;

        ESP_LOGI(TAG, "Forcing connection to BSSID %02X:%02X:%02X:%02X:%02X:%02X on channel %d",
                 target_bssid[0], target_bssid[1], target_bssid[2],
                 target_bssid[3], target_bssid[4], target_bssid[5],
                 target_channel);
    } else {
        ESP_LOGW(TAG, "Channel scan failed, using default connection");
    }
#endif

    // Apply WiFi config and connect
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Wait for connection, indefinitely. The event handler keeps reconnecting
    // with backoff, so an AP that comes up after the node (building power
    // cycle) is joined late rather than never: returning ESP_FAIL here used
    // to leave app_main without sockets or tasks, a node that associates
    // later but stays silent until someone resets it.
    ESP_LOGI(TAG, "Connecting to WiFi...");
    EventBits_t bits = 0;
    while (!(bits & WIFI_CONNECTED_BIT)) {
        bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                   pdFALSE, pdFALSE,
                                   pdMS_TO_TICKS(WIFI_CONNECT_WAIT_LOG_MS));
        if (!(bits & WIFI_CONNECTED_BIT)) {
            ESP_LOGW(TAG, "Still waiting for WiFi '%s' (retry #%d), not giving up",
                     CONFIG_WIFI_SSID, wifi_retry_count);
        }
    }

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to %s", CONFIG_WIFI_SSID);

        // Set bandwidth to 20MHz (HE20 = 242 subcarriers; HE40 does not exist).
        // Dual-band C5 requires the per-band esp_wifi_set_bandwidths(); the
        // singular esp_wifi_set_bandwidth() is rejected under band-mode AUTO.
#if CONFIG_SOC_WIFI_SUPPORT_5G
        wifi_bandwidths_t bw_cfg = { .ghz_2g = WIFI_BW_HT20, .ghz_5g = WIFI_BW_HT20 };
        ESP_ERROR_CHECK(esp_wifi_set_bandwidths(WIFI_IF_STA, &bw_cfg));
#else
        ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
#endif

        // v5.4.1: initial capture bring-up now retries with backoff (via
        // RETRY_CAPTURE_CALL, shared with restart_capture()) instead of
        // aborting on the first transient failure. `ok` accumulates the
        // verdict across every stage below.
        bool capture_ok = true;

        // Enable promiscuous mode for better CSI capture
        RETRY_CAPTURE_CALL(esp_wifi_set_promiscuous(true), "set_promiscuous (init)", capture_ok);

        // Configure CSI
#if CONFIG_SOC_WIFI_HE_SUPPORT
        // ESP32-C6/C5: WiFi 6 HE-LTF CSI.
        // Lean acquire mask: only frame types our peer-to-peer bursts
        // actually emit. MU OFDMA is AP-downlink only; DCM is rare (low
        // MCS); beamformed adds CSI for AP traffic we do not analyze.
        // Disabling them reduces ISR pressure -> higher sustained rate.
        //
        // NOTE: wifi_csi_config_t is an alias for wifi_csi_acquire_config_t,
        // and its layout differs between the C6 (MAC v2) and C5 (MAC v3).
        // On the C5 the STBC field is `acquire_csi_he_stbc_mode` (2-bit),
        // not `acquire_csi_he_stbc` -- naming the wrong field is a hard
        // compile error, so the two targets need separate initializers.
        // Unlisted C5-only fields (acquire_csi_force_lltf, acquire_csi_vht)
        // are zero-initialized by the designated initializer.
        // Selector mirrors esp_wifi_he_types.h exactly: the v3 struct is
        // chosen by CONFIG_SOC_WIFI_MAC_VERSION_NUM == 3 (C5), not by target.
#if CONFIG_SOC_WIFI_MAC_VERSION_NUM == 3
        wifi_csi_config_t csi_config = {
            .enable = 1,
            .acquire_csi_legacy = 1,        // probe/control fallback
            .acquire_csi_ht20 = 1,          // HT20 fallback if AP downgrades
            .acquire_csi_ht40 = 0,
            .acquire_csi_su = 1,            // HE20 SU - primary path
            .acquire_csi_mu = 0,            // disabled: not used in mesh
            .acquire_csi_dcm = 0,           // disabled: rare, low-MCS only
            .acquire_csi_beamformed = 0,    // disabled: AP-only, off-target
            .acquire_csi_he_stbc_mode = 0,  // C5: 2-bit STBC LTF selector
            .val_scale_cfg = 0,             // C5: 4-bit (0-8), 0 = auto
            .dump_ack_en = 0,
        };
#else
        // ESP32-C6 (MAC v2)
        wifi_csi_config_t csi_config = {
            .enable = 1,
            .acquire_csi_legacy = 1,      // probe/control fallback
            .acquire_csi_ht20 = 1,        // HT20 fallback if AP downgrades
            .acquire_csi_ht40 = 0,
            .acquire_csi_su = 1,          // HE20 SU - primary path
            .acquire_csi_mu = 0,          // disabled: not used in mesh
            .acquire_csi_dcm = 0,         // disabled: rare, low-MCS only
            .acquire_csi_beamformed = 0,  // disabled: AP-only, off-target
            .acquire_csi_he_stbc = 0,
            .val_scale_cfg = 0,
            .dump_ack_en = 0,
        };
#endif
#else
        // ESP32-C3/S3: Legacy HT CSI
        wifi_csi_config_t csi_config = {
            .lltf_en = true,            // Legacy Long Training Field
            .htltf_en = true,           // HT-LTF
            .stbc_htltf2_en = true,     // STBC HT-LTF2
            .ltf_merge_en = true,       // Merge LTF
            .channel_filter_en = true,  // Enable channel filter
            .manu_scale = false,
            .shift = 0,                 // Auto scale (uint8_t, 0-15)
            .dump_ack_en = false,       // Don't dump ACK frames
        };
#endif

        RETRY_CAPTURE_CALL(esp_wifi_set_csi_config(&csi_config), "set_csi_config (init)", capture_ok);
        RETRY_CAPTURE_CALL(esp_wifi_set_csi_rx_cb(csi_callback, NULL), "set_csi_rx_cb (init)", capture_ok);
        RETRY_CAPTURE_CALL(esp_wifi_set_csi(true), "set_csi (init)", capture_ok);

        ESP_LOGI(TAG, "CSI capture enabled (HT20, promiscuous mode)");

#ifdef CONFIG_ENABLE_BFI_CAPTURE
        // Coexist: register a promiscuous packet callback alongside the CSI
        // callback. The promiscuous FILTER is deliberately left at its default
        // (the callback filters management frames in software) so CSI
        // acquisition is not affected. Promiscuous mode itself is already on.
        RETRY_CAPTURE_CALL(esp_wifi_set_promiscuous_rx_cb(bfi_promisc_cb), "set_promiscuous_rx_cb (init)", capture_ok);
        ESP_LOGI(TAG, "BFI capture enabled: forwarding VHT/HE beamforming reports as MSG_TYPE_BFI");
#endif

        if (!capture_ok) {
            // Same escalation as restart_capture(): retries exhausted means
            // this isn't transient. Reboot rather than limp along with a
            // half-configured (or, per the g_capture_configured race this
            // release also closes, potentially never-configured) CSI path
            // that would otherwise report healthy heartbeats forever.
            ESP_LOGE(TAG, "Initial CSI/BFI capture bring-up failed after retries, rebooting to recover");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }

        // Only now is it safe for restart_capture() (fired from the WiFi
        // event handler on every future reconnect) to actually re-arm the
        // capture path -- see the g_capture_configured comment above.
        g_capture_configured = true;

        // v5.8.0: capture the associated AP's BSSID for the raw-injection
        // burst path's Address3 field (node_bssid_bytes' own comment has
        // the full reasoning for using this API here rather than the
        // pre-connection scan result). Never fatal: on failure,
        // node_bssid_bytes stays zero-initialized (all-zero MAC) -- a raw
        // burst would carry an all-zero BSSID, which does not affect
        // reception/attribution (Address1/Address2 are what matter for
        // that) but is logged loudly since a real deployment should never
        // see this.
        wifi_ap_record_t ap_info = {0};
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            memcpy(node_bssid_bytes, ap_info.bssid, 6);
        } else {
            ESP_LOGE(TAG, "esp_wifi_sta_get_ap_info FAILED -- node_bssid_bytes "
                           "stays zero; raw-inject bursts (if ever requested) "
                           "will carry an all-zero Address3");
        }

        // v5.7.0: deliberately NOT calling init_espnow_broadcast() here.
        // Earlier drafts of this image did (unconditionally, at boot,
        // regardless of whether this node would ever be asked to use
        // ESP-NOW) -- directive-build's pre-registered review flagged this
        // as breaking the "byte-identical to v5.6.0 when transport=0" claim
        // at the DEVICE level: eager esp_now_init()/esp_now_add_peer()
        // touches WiFi driver state on every node, not just the ones that
        // end up using it. init_espnow_broadcast() is now called lazily,
        // from transmit_burst()'s dispatch, on the first (and only the
        // first) burst that actually requests burst_transport=espnow --
        // see g_espnow_init_attempted's comment.
        return ESP_OK;
    }

    ESP_LOGE(TAG, "WiFi connection timeout");
    return ESP_FAIL;
}

// =============================================================================
// mDNS SERVER DISCOVERY
// =============================================================================
#ifdef CONFIG_MDNS_DISCOVERY
static void resolve_server_via_mdns(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mDNS: init failed (%s), will use Kconfig fallback", esp_err_to_name(err));
        return;
    }

    // Set this node's hostname to its MAC (colons stripped) for discoverability
    char hostname[13];  // 12 hex chars + null
    snprintf(hostname, sizeof(hostname), "%02x%02x%02x%02x%02x%02x",
             node_mac_bytes[0], node_mac_bytes[1], node_mac_bytes[2],
             node_mac_bytes[3], node_mac_bytes[4], node_mac_bytes[5]);
    mdns_hostname_set(hostname);

    ESP_LOGI(TAG, "mDNS: querying _csi._udp.local (timeout %d ms)...", CONFIG_MDNS_QUERY_TIMEOUT_MS);

    mdns_result_t *results = NULL;
    err = mdns_query_ptr("_csi", "_udp", CONFIG_MDNS_QUERY_TIMEOUT_MS, 1, &results);
    if (err != ESP_OK || results == NULL) {
        ESP_LOGW(TAG, "mDNS: query timeout, will use Kconfig fallback (%s)", CONFIG_SERVER_IP);
        if (results) mdns_query_results_free(results);
        return;
    }

    // Resolve the first result to get its A record.
    mdns_result_t *r = results;
    if (r->addr && r->addr->addr.type == ESP_IPADDR_TYPE_V4) {
        snprintf(resolved_server_ip, sizeof(resolved_server_ip),
                 IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));
        ESP_LOGI(TAG, "mDNS: resolved _csi._udp.local -> %s", resolved_server_ip);
    } else if (r->hostname) {
        // A PTR query on ESP-IDF frequently returns the service instance
        // WITHOUT the A record populated (the address arrives in a separate
        // packet). Follow up with an explicit A-record query on the SRV
        // target hostname to actually obtain the server's IPv4. Without this,
        // discovery "finds" the service but never learns its IP, silently
        // falling back to the hardcoded CONFIG_SERVER_IP.
        // Bounded timeout: a responsive server answers an A query in well
        // under a second; capping at 1500 ms avoids a long boot stall on the
        // failure path instead of inheriting the full PTR-query timeout.
        esp_ip4_addr_t addr4 = { 0 };
        esp_err_t aerr = mdns_query_a(r->hostname, 1500, &addr4);
        if (aerr == ESP_OK && addr4.addr != 0) {
            snprintf(resolved_server_ip, sizeof(resolved_server_ip), IPSTR, IP2STR(&addr4));
            ESP_LOGI(TAG, "mDNS: resolved host '%s' -> %s (A query)",
                     r->hostname, resolved_server_ip);
        } else {
            ESP_LOGW(TAG, "mDNS: A query for host '%s' failed (%s), using Kconfig fallback",
                     r->hostname, esp_err_to_name(aerr));
        }
    } else {
        ESP_LOGW(TAG, "mDNS: service found but no hostname/IPv4, using Kconfig fallback");
    }

    mdns_query_results_free(results);
}
#endif

// =============================================================================
// TEMPERATURE SENSOR INITIALIZATION
// =============================================================================
/**
 * Install and enable the on-die temperature sensor once at boot.
 * C3, S3, and C5 targets all expose the same esp_driver_tsens API
 * (SOC_TEMP_SENSOR_SUPPORTED=1 on all three, verified against soc_caps.h),
 * so no per-chip conditional is needed.
 * Failure is non-fatal: read_die_temp_c_x10() reports "unavailable" if the
 * handle was never installed.
 */
static esp_err_t init_temp_sensor(void)
{
    temperature_sensor_config_t temp_sensor_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(DIE_TEMP_RANGE_MIN_C, DIE_TEMP_RANGE_MAX_C);

    esp_err_t err = temperature_sensor_install(&temp_sensor_config, &temp_sensor_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Temperature sensor install failed: %s", esp_err_to_name(err));
        temp_sensor_handle = NULL;
        return err;
    }

    err = temperature_sensor_enable(temp_sensor_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Temperature sensor enable failed: %s", esp_err_to_name(err));
        temperature_sensor_uninstall(temp_sensor_handle);
        temp_sensor_handle = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Temperature sensor enabled (range %d..%d C)",
             DIE_TEMP_RANGE_MIN_C, DIE_TEMP_RANGE_MAX_C);
    return ESP_OK;
}

/**
 * Read the on-die temperature, scaled x10 for lossless int16 transport.
 * Returns DIE_TEMP_READ_FAILED_X10 if the sensor was never installed or
 * the read fails (e.g. reading is momentarily out of the configured range).
 */
static int16_t read_die_temp_c_x10(void)
{
    if (temp_sensor_handle == NULL) {
        return DIE_TEMP_READ_FAILED_X10;
    }

    float celsius = 0.0f;
    esp_err_t err = temperature_sensor_get_celsius(temp_sensor_handle, &celsius);
    if (err != ESP_OK) {
        return DIE_TEMP_READ_FAILED_X10;
    }

    return (int16_t)(celsius * 10.0f);
}

// =============================================================================
// SOCKET INITIALIZATION
// =============================================================================
static esp_err_t init_sockets(void)
{
    // Choose server IP: mDNS-resolved takes priority, Kconfig fallback otherwise
#ifdef CONFIG_MDNS_DISCOVERY
    const char *server_ip = resolved_server_ip[0] ? resolved_server_ip : CONFIG_SERVER_IP;
    const char *ip_source = resolved_server_ip[0] ? "mDNS" : "Kconfig";
#else
    const char *server_ip = CONFIG_SERVER_IP;
    const char *ip_source = "Kconfig";
#endif

    ESP_LOGI(TAG, "Initializing sockets...");
    ESP_LOGI(TAG, "  Server: %s:%d (source: %s)", server_ip, CONFIG_SERVER_PORT, ip_source);
    ESP_LOGI(TAG, "  Trigger port: %d", CONFIG_TRIGGER_PORT);
    ESP_LOGI(TAG, "  Burst port: %d", CONFIG_BURST_PORT);

    // Cache server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(CONFIG_SERVER_PORT);
    if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) != 1) {
        ESP_LOGE(TAG, "Invalid server IP address: %s", server_ip);
        return ESP_FAIL;
    }

    // Cache broadcast address
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(CONFIG_BURST_PORT);
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    // v5.9.0: unicast burst destination = server IP on the burst port. Same
    // resolved server_ip; used only when g_burst_unicast is set (trigger
    // reserved[4]). Reuses server_addr's already-validated sin_addr.
    memset(&burst_unicast_addr, 0, sizeof(burst_unicast_addr));
    burst_unicast_addr.sin_family = AF_INET;
    burst_unicast_addr.sin_port = htons(CONFIG_BURST_PORT);
    burst_unicast_addr.sin_addr = server_addr.sin_addr;

    // Create main UDP socket
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return ESP_FAIL;
    }

    // Set socket options
    int opt = 1;
    setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Create persistent broadcast socket
    broadcast_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (broadcast_sock < 0) {
        ESP_LOGE(TAG, "Failed to create broadcast socket: errno %d", errno);
        return ESP_FAIL;
    }

    int broadcast = 1;
    setsockopt(broadcast_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    setsockopt(broadcast_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Create trigger socket
    trigger_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (trigger_sock < 0) {
        ESP_LOGE(TAG, "Failed to create trigger socket: errno %d", errno);
        return ESP_FAIL;
    }

    setsockopt(trigger_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    setsockopt(trigger_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in trigger_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_TRIGGER_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(trigger_sock, (struct sockaddr *)&trigger_addr, sizeof(trigger_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind trigger socket: errno %d", errno);
        return ESP_FAIL;
    }

    // Set receive timeout on trigger socket to prevent blocking forever
    struct timeval tv = {
        .tv_sec = SOCKET_TIMEOUT_MS / 1000,
        .tv_usec = (SOCKET_TIMEOUT_MS % 1000) * 1000
    };
    setsockopt(trigger_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "Sockets initialized (server=%s:%d, trigger=%d, burst=%d)",
             server_ip, CONFIG_SERVER_PORT, CONFIG_TRIGGER_PORT, CONFIG_BURST_PORT);
    return ESP_OK;
}

// =============================================================================
// UDP SEND HELPERS (Thread-safe with mutex)
// v5.2.1: Added error tracking for debugging packet loss
// =============================================================================
static inline void send_to_server(const char *data, size_t len)
{
    if (udp_sock >= 0 && socket_mutex) {
        if (xSemaphoreTake(socket_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            int ret = sendto(udp_sock, data, len, 0,
                             (struct sockaddr *)&server_addr, sizeof(server_addr));
            if (ret < 0) {
                g_udp_send_failures++;
            }
            xSemaphoreGive(socket_mutex);
        } else {
            g_udp_mutex_timeouts++;
        }
    }
}

static inline void send_broadcast_packet(const char *data, size_t len)
{
    if (broadcast_sock >= 0 && socket_mutex) {
        if (xSemaphoreTake(socket_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // v5.9.0: default (g_burst_unicast==0) -> broadcast_addr, exactly
            // as before. When set, unicast to the server (no AP rebroadcast);
            // the overhearable STA->AP uplink -- the only thing sensing uses --
            // is identical either way.
            const struct sockaddr_in *dest = g_burst_unicast ? &burst_unicast_addr
                                                             : &broadcast_addr;
            int ret = sendto(broadcast_sock, data, len, 0,
                             (struct sockaddr *)dest, sizeof(*dest));
            if (ret < 0) {
                g_udp_send_failures++;
            }
            xSemaphoreGive(socket_mutex);
        } else {
            g_udp_mutex_timeouts++;
        }
    }
}

#ifdef CONFIG_ENABLE_BFI_CAPTURE
// =============================================================================
// BFI SEND / DRAIN
// =============================================================================
// Build a MSG_TYPE_BFI packet (36-byte header + raw action body + CRC32) and
// send it to the server, reusing the CSI/HELLO UDP plumbing.
static void send_bfi(const bfi_capture_t *cap)
{
    static uint8_t sbuf[sizeof(bfi_header_t) + BFI_MAX_BODY + 4];
    bfi_header_t *h = (bfi_header_t *)sbuf;
    h->magic = BINARY_PROTO_MAGIC;
    h->type  = MSG_TYPE_BFI;
    memcpy(h->node_mac, node_mac_bytes, 6);
    memcpy(h->src_mac,  cap->src_mac, 6);
    memcpy(h->dst_mac,  cap->dst_mac, 6);
    h->timestamp_us = cap->timestamp_us;
    h->rssi      = cap->rssi;
    h->channel   = cap->channel;
    h->bb_format = cap->bb_format;
    h->subtype   = cap->subtype;
    h->category  = cap->category;
    h->action    = cap->action;
    h->body_len  = cap->body_len;

    size_t off = sizeof(bfi_header_t);
    memcpy(sbuf + off, cap->body, cap->body_len);
    off += cap->body_len;
    uint32_t crc = esp_crc32_le(0, sbuf, off);
    memcpy(sbuf + off, &crc, sizeof(crc));
    off += sizeof(crc);

    send_to_server((const char *)sbuf, off);
}

// Drain queued BFI captures (called from heartbeat_task; BFI is infrequent).
static void drain_bfi(void)
{
    if (!bfi_ringbuf) return;
    size_t item_size;
    void *item;
    int n = 0;
    while (n < MAX_BFI_ITEMS &&
           (item = xRingbufferReceive(bfi_ringbuf, &item_size, 0)) != NULL) {
        send_bfi((const bfi_capture_t *)item);
        vRingbufferReturnItem(bfi_ringbuf, item);
        n++;
    }
}
#endif  // CONFIG_ENABLE_BFI_CAPTURE

// =============================================================================
// HELPER: Parse IP string "a.b.c.d" to 4 bytes
// =============================================================================
static void ip_str_to_bytes(const char *ip_str, uint8_t *ip_bytes)
{
    unsigned int a, b, c, d;
    if (sscanf(ip_str, "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
        ip_bytes[0] = (uint8_t)a;
        ip_bytes[1] = (uint8_t)b;
        ip_bytes[2] = (uint8_t)c;
        ip_bytes[3] = (uint8_t)d;
    } else {
        memset(ip_bytes, 0, 4);
    }
}

// =============================================================================
// PACKET BUILDERS - Binary protocol (zero malloc)
// =============================================================================

/**
 * Send HELLO packet - 40 bytes, no malloc
 * Used for node registration and periodic re-announcement
 */
static void send_hello(void)
{
    binary_hello_t pkt = {0};  // Zero-initialize

    // Header
    pkt.magic = BINARY_PROTO_MAGIC;
    pkt.type = MSG_TYPE_HELLO;

    // Node identification
    memcpy(pkt.mac, node_mac_bytes, 6);
    ip_str_to_bytes(node_ip, pkt.ip);

    // Chip and firmware info (auto-detected)
    pkt.chip_type = get_chip_type_for_protocol();
    pkt.fw_major = FW_VERSION_MAJOR;
    pkt.fw_minor = FW_VERSION_MINOR;
    pkt.fw_patch = FW_VERSION_PATCH;
    strncpy(pkt.fw_tag, "RFWD", sizeof(pkt.fw_tag) - 1);  // v5.4.1-RFWD

    // Runtime stats
    pkt.uptime_s = (uint32_t)((esp_timer_get_time() - boot_time_us) / 1000000);
    pkt.free_heap = esp_get_free_heap_size();
    pkt.min_heap = esp_get_minimum_free_heap_size();

    // CRC32 (over packet excluding CRC field itself)
    pkt.crc32 = esp_crc32_le(0, (const uint8_t *)&pkt,
                              sizeof(binary_hello_t) - sizeof(uint32_t));

    send_to_server((const char *)&pkt, sizeof(binary_hello_t));
    ESP_LOGD(TAG, "HELLO sent (40 bytes, binary)");
}

/**
 * Send HEARTBEAT packet - 34 bytes, no malloc
 * Used for keepalive and health monitoring
 */
static void send_heartbeat(void)
{
    binary_heartbeat_t pkt = {0};  // Zero-initialize

    // Header
    pkt.magic = BINARY_PROTO_MAGIC;
    pkt.type = MSG_TYPE_HEARTBEAT;

    // Node identification
    memcpy(pkt.mac, node_mac_bytes, 6);

    // Runtime stats
    pkt.uptime_s = (uint32_t)((esp_timer_get_time() - boot_time_us) / 1000000);
    pkt.free_heap = esp_get_free_heap_size();
    pkt.min_heap = esp_get_minimum_free_heap_size();
    pkt.die_temp_c_x10 = read_die_temp_c_x10();
    pkt.csi_frames_captured = g_csi_captured;
#ifdef CONFIG_ENABLE_BFI_CAPTURE
    pkt.bfi_frames_captured = g_bfi_captured;
#else
    pkt.bfi_frames_captured = 0;
#endif
    // v4 drop counters: what the node FAILED to deliver, so the backend can
    // compute yield instead of assuming it.
    pkt.csi_drops = g_csi_drops;
    pkt.udp_send_failures = g_udp_send_failures;
    pkt.udp_mutex_timeouts = g_udp_mutex_timeouts;
    pkt.batch_drops = g_batch_drops;
    // v5: TX-match attribution guard firing (not a delivery/capture drop --
    // see BINARY_HEARTBEAT_SIZE's v5.6.0 comment for the firing-vs-failing
    // caveat).
    pkt.csi_attribution_dropped = g_csi_attribution_dropped;

    // v6: RF ASSOCIATION. Read fresh every heartbeat rather than reusing the
    // BSSID cached at connect: the question this answers is whether the node
    // is STILL where it thinks it is -- a silent roam to another AP or channel
    // is exactly the failure mode that leaves a node associated and deaf, and
    // a value cached at boot could never show it.
    //
    // Sentinels are deliberate and distinguishable from real readings: RSSI 0
    // dBm is not a value a real STA reports, channel 0 is not a real channel,
    // and an all-zero BSSID is already used elsewhere in this file to mean
    // "no association". A failed call must not look like a strong signal on
    // channel 1.
    {
        wifi_ap_record_t hb_ap = {0};
        if (esp_wifi_sta_get_ap_info(&hb_ap) == ESP_OK) {
            pkt.ap_rssi_dbm = (int8_t)hb_ap.rssi;
            pkt.ap_channel  = hb_ap.primary;
            memcpy(pkt.ap_bssid, hb_ap.bssid, 6);
        } else {
            pkt.ap_rssi_dbm = 0;
            pkt.ap_channel  = 0;
            memset(pkt.ap_bssid, 0, 6);
        }
    }

    // CRC32
    pkt.crc32 = esp_crc32_le(0, (const uint8_t *)&pkt,
                              sizeof(binary_heartbeat_t) - sizeof(uint32_t));

    send_to_server((const char *)&pkt, sizeof(binary_heartbeat_t));
}

// OPT-4: Inline macro for pilot phase extraction (eliminates function call overhead)
// Pilot subcarriers are at indices 6, 20, 32, 46 in the 52-subcarrier array
#define EXTRACT_PILOT_PHASES(iq, pilots) do { \
    (pilots)[0] = (iq)[PILOT_SC_1 * 2];         \
    (pilots)[1] = (iq)[PILOT_SC_1 * 2 + 1];     \
    (pilots)[2] = (iq)[PILOT_SC_2 * 2];         \
    (pilots)[3] = (iq)[PILOT_SC_2 * 2 + 1];     \
    (pilots)[4] = (iq)[PILOT_SC_3 * 2];         \
    (pilots)[5] = (iq)[PILOT_SC_3 * 2 + 1];     \
    (pilots)[6] = (iq)[PILOT_SC_4 * 2];         \
    (pilots)[7] = (iq)[PILOT_SC_4 * 2 + 1];     \
} while(0)

// =============================================================================
// BATCH CSI FUNCTIONS
// =============================================================================

/**
 * Finalize and send the current batch
 * Called from silence timer or when batch is full
 *
 * v5.2.2 OPT: CRC calculation moved outside critical section
 * v5.2.3 FIX: Added mutex to protect send buffer from concurrent access
 *
 * THREAD SAFETY:
 * - batch_spinlock: protects batch state (short duration, ISR-safe)
 * - batch_send_mutex: protects send buffer during copy-CRC-send (longer duration)
 * - This function can be called from trigger_task OR timer_task
 */
static void finalize_and_send_batch_v5(void)
{
    // v5.2.2 OPT: Local buffer for CRC calculation outside spinlock
    // v5.2.3 FIX: Now protected by mutex since called from multiple task contexts
    static uint8_t s_send_buffer[BATCH_BUFFER_SIZE + 4];  // +4 for CRC
    size_t data_size;
    uint8_t sample_count;
    uint8_t slot_id;

    // v5.2.3 FIX: Acquire mutex before any buffer operations
    // This prevents race between trigger_task and timer_task
    if (batch_send_mutex == NULL ||
        xSemaphoreTake(batch_send_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        // Mutex not ready or timeout - skip this send to avoid corruption.
        // COUNTED since v5.4.2: this discards a WHOLE BATCH and was previously
        // silent, with no counter and no log, so a node losing batches looked
        // identical to a node with nothing to send.
        g_batch_drops++;
        return;
    }

    portENTER_CRITICAL(&batch_spinlock);

    if (g_batch_sample_count == 0) {
        portEXIT_CRITICAL(&batch_spinlock);
        xSemaphoreGive(batch_send_mutex);
        return;
    }

#if CONFIG_SOC_WIFI_HE_SUPPORT
    // C6: Build v5 or v6 header depending on batch type
    if (g_batch_is_v6) {
        batch_header_v6_t *header = (batch_header_v6_t *)g_batch_buffer;
        header->magic[0] = BINARY_CSI_MAGIC_V5_0;  // Same first magic byte (0xC5)
        header->magic[1] = BINARY_CSI_MAGIC_V6;
        header->slot_id = g_current_slot_id;
        header->sample_count = g_batch_sample_count;
        header->seq = g_batch_seq++;
        header->timestamp_lo = (uint16_t)(g_batch_first_timestamp & 0xFFFF);
        header->num_subcarriers = g_batch_v6_num_subcarriers;
        header->frame_format = g_batch_v6_frame_format;
        header->reserved = 0;
    } else {
        batch_header_v5_t *header = (batch_header_v5_t *)g_batch_buffer;
        header->magic[0] = BINARY_CSI_MAGIC_V5_0;
        header->magic[1] = BINARY_CSI_MAGIC_V5_1;
        header->slot_id = g_current_slot_id;
        header->sample_count = g_batch_sample_count;
        header->seq = g_batch_seq++;
        header->timestamp_lo = (uint16_t)(g_batch_first_timestamp & 0xFFFF);
    }
#else
    // Build header in place
    batch_header_v5_t *header = (batch_header_v5_t *)g_batch_buffer;
    header->magic[0] = BINARY_CSI_MAGIC_V5_0;
    header->magic[1] = BINARY_CSI_MAGIC_V5_1;
    header->slot_id = g_current_slot_id;
    header->sample_count = g_batch_sample_count;
    header->seq = g_batch_seq++;
    header->timestamp_lo = (uint16_t)(g_batch_first_timestamp & 0xFFFF);
#endif

    data_size = g_batch_offset;
    sample_count = g_batch_sample_count;
    slot_id = g_current_slot_id;

    // v5.2.1: Bounds check before copy to prevent buffer overflow
    if (data_size > BATCH_BUFFER_SIZE) {
        ESP_LOGE(TAG, "FATAL: Batch buffer overflow prevented (size=%d)", data_size);
        portEXIT_CRITICAL(&batch_spinlock);
        xSemaphoreGive(batch_send_mutex);
        return;
    }

    // v5.2.2 OPT: Copy to local buffer, then release spinlock
    memcpy(s_send_buffer, g_batch_buffer, data_size);

    // Reset for next batch (allows new samples to accumulate immediately)
    g_batch_offset = BINARY_BATCH_HEADER_SIZE;
    g_batch_sample_count = 0;
    g_batch_first_timestamp = 0;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    g_batch_is_v6 = false;
    g_batch_v6_num_subcarriers = 0;
    g_batch_v6_frame_format = 0;
#endif

    portEXIT_CRITICAL(&batch_spinlock);

    // v5.2.2 OPT: CRC calculation now OUTSIDE critical section
    // CRC over 1400 bytes takes ~20-30µs - no longer blocks other tasks
    uint32_t crc = esp_crc32_le(0, s_send_buffer, data_size);
    memcpy(s_send_buffer + data_size, &crc, 4);
    size_t total_size = data_size + 4;

    // Send outside critical section
    send_to_server((const char *)s_send_buffer, total_size);
    g_csi_batches_sent++;  // delivery-watchdog heartbeat (send failures are
                           // counted separately in g_udp_send_failures)

    // v5.2.3 FIX: Release mutex after send completes
    xSemaphoreGive(batch_send_mutex);

    ESP_LOGD(TAG, "v5.0 batch sent: slot=%d, samples=%d, size=%d",
             slot_id, sample_count, total_size);
}

/**
 * Silence timer callback - flush batch after 3ms of no CSI
 */
static void silence_timer_callback(void *arg)
{
    finalize_and_send_batch_v5();
}

/**
 * Initialize v5.0 batch mode and silence timer
 */
static esp_err_t init_batch_mode_v5(void)
{
    // v5.2.3 FIX: Create mutex for send buffer protection
    batch_send_mutex = xSemaphoreCreateMutex();
    if (!batch_send_mutex) {
        ESP_LOGE(TAG, "Failed to create batch send mutex");
        return ESP_FAIL;
    }

    esp_timer_create_args_t timer_args = {
        .callback = silence_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "silence_timer"
    };

    esp_err_t ret = esp_timer_create(&timer_args, &silence_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create silence timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Protocol v5.0 batch mode initialized (timeout=%dµs, send_mutex=OK)", SILENCE_TIMEOUT_US);
    return ESP_OK;
}

/**
 * Start new batch for a TDMA slot
 */
static void start_new_batch_v5(uint8_t slot_id)
{
    // Flush any pending batch first
    finalize_and_send_batch_v5();

    portENTER_CRITICAL(&batch_spinlock);
    g_current_slot_id = slot_id;
    g_batch_offset = BINARY_BATCH_HEADER_SIZE;
    g_batch_sample_count = 0;
    g_batch_first_timestamp = 0;
    portEXIT_CRITICAL(&batch_spinlock);

    ESP_LOGD(TAG, "v5.0 new batch started: slot=%d", slot_id);
}

/**
 * Add CSI sample to current batch
 * Returns true if sample added, false if batch is full (caller should flush)
 *
 * v5.2.1: Restructured to avoid nested critical sections.
 * Check-and-flush happens outside critical section to prevent deadlock.
 *
 * v6.0: On C6, detects HE frames (large pkt->len) and uses v6 variable-width
 * protocol. HT frames on C6 still use v5. C3/S3 always use v5.
 */
static bool add_sample_to_batch_v5(const csi_packet_t *pkt)
{
#if CONFIG_SOC_WIFI_HE_SUPPORT
    // C6: Detect subcarrier count from CSI data length
    // HE20 full-band: 242 subcarriers * 2 = 484 bytes
    // HT20: 52 subcarriers * 2 = 104 bytes
    // Threshold: >200 bytes means HE frame
    uint16_t num_subcarriers = pkt->len / 2;
    bool is_he_frame = (pkt->len > 200);
    size_t sample_size;

    if (is_he_frame) {
        // v6 per-sample: [timestamp_delta:2][rssi:1][iq_data:num_subcarriers*2][trigger_seq:4]
        sample_size = 3 + (uint32_t)num_subcarriers * 2 + BINARY_V6_SAMPLE_TRIGGER_SEQ_EXTRA;
    } else {
        sample_size = BINARY_SAMPLE_SIZE_V5_1;
    }

    // Check if flush is needed BEFORE taking spinlock
    bool needs_flush = false;
    portENTER_CRITICAL(&batch_spinlock);
    if (g_batch_sample_count >= MAX_SAMPLES_PER_BATCH ||
        (g_batch_offset + sample_size) > BATCH_BUFFER_SIZE - 4) {  // -4 for CRC
        needs_flush = true;
    }
    // Also flush if switching between v5 and v6 mid-batch
    if (g_batch_sample_count > 0 && g_batch_is_v6 != is_he_frame) {
        needs_flush = true;
    }
    portEXIT_CRITICAL(&batch_spinlock);

    if (needs_flush) {
        finalize_and_send_batch_v5();
    }

    // Now add the sample
    portENTER_CRITICAL(&batch_spinlock);

    // Record first timestamp and set batch format
    if (g_batch_sample_count == 0) {
        g_batch_first_timestamp = (uint32_t)(pkt->timestamp_us & 0xFFFFFFFF);
        g_batch_is_v6 = is_he_frame;
        if (is_he_frame) {
            g_batch_v6_num_subcarriers = num_subcarriers;
            g_batch_v6_frame_format = pkt->bb_format;
            g_batch_offset = BINARY_V6_HEADER_SIZE;  // v6 header is 12 bytes
        } else {
            g_batch_offset = BINARY_BATCH_HEADER_SIZE;  // v5 header is 8 bytes
        }
    }

    if (is_he_frame) {
        // v6: variable-width sample layout
        uint8_t *dest = g_batch_buffer + g_batch_offset;
        uint16_t ts_delta = (uint16_t)((pkt->timestamp_us - g_batch_first_timestamp) & 0xFFFF);
        memcpy(dest, &ts_delta, 2);
        dest[2] = pkt->rssi;
        size_t iq_bytes = (size_t)num_subcarriers * 2;
        if (iq_bytes > pkt->len) iq_bytes = pkt->len;
        memcpy(dest + 3, pkt->data, iq_bytes);
        // trigger_seq goes at the DECLARED offset (3 + num_subcarriers*2),
        // not 3 + iq_bytes -- the two only differ if pkt->len undershoots
        // num_subcarriers*2, and the wire layout must match num_subcarriers
        // from the header regardless, so the receiver's fixed offset math
        // (protocol_constants.py's BINARY_V6_SAMPLE_TRIGGER_SEQ_EXTRA) stays valid.
        memcpy(dest + 3 + (size_t)num_subcarriers * 2, &pkt->trigger_seq, 4);
        g_batch_offset += sample_size;
    } else {
        // v5: fixed 111-byte samples (HT frames on C6, or legacy)
        sample_v5_t *sample = (sample_v5_t *)(g_batch_buffer + g_batch_offset);
        sample->timestamp_delta = (uint16_t)((pkt->timestamp_us - g_batch_first_timestamp) & 0xFFFF);
        sample->rssi = pkt->rssi;
        size_t iq_bytes = (pkt->len < 104) ? pkt->len : 104;
        memcpy(sample->iq_data, pkt->data, iq_bytes);
        if (iq_bytes < 104) {
            memset(sample->iq_data + iq_bytes, 0, 104 - iq_bytes);
        }
        sample->trigger_seq = pkt->trigger_seq;
        g_batch_offset += BINARY_SAMPLE_SIZE_V5_1;
    }
    g_batch_sample_count++;

    portEXIT_CRITICAL(&batch_spinlock);

#else
    // C3/S3: Always use v5 fixed-width protocol

    // v5.2.1: Check if flush is needed BEFORE taking spinlock
    // This avoids nested locking since finalize_and_send_batch_v5() also takes batch_spinlock
    bool needs_flush = false;
    portENTER_CRITICAL(&batch_spinlock);
    if (g_batch_sample_count >= MAX_SAMPLES_PER_BATCH ||
        (g_batch_offset + BINARY_SAMPLE_SIZE_V5_1) > BATCH_BUFFER_SIZE - 4) {  // -4 for CRC
        needs_flush = true;
    }
    portEXIT_CRITICAL(&batch_spinlock);

    // Flush outside critical section if needed
    if (needs_flush) {
        finalize_and_send_batch_v5();
    }

    // Now add the sample
    portENTER_CRITICAL(&batch_spinlock);

    // Record first timestamp
    if (g_batch_sample_count == 0) {
        g_batch_first_timestamp = (uint32_t)(pkt->timestamp_us & 0xFFFFFFFF);
    }

    // Build sample at current offset
    sample_v5_t *sample = (sample_v5_t *)(g_batch_buffer + g_batch_offset);
    sample->timestamp_delta = (uint16_t)((pkt->timestamp_us - g_batch_first_timestamp) & 0xFFFF);
    sample->rssi = pkt->rssi;

    // Copy I/Q data (max 104 bytes = 52 subcarriers x 2)
    size_t iq_bytes = (pkt->len < 104) ? pkt->len : 104;
    memcpy(sample->iq_data, pkt->data, iq_bytes);
    if (iq_bytes < 104) {
        memset(sample->iq_data + iq_bytes, 0, 104 - iq_bytes);
    }
    sample->trigger_seq = pkt->trigger_seq;

    g_batch_offset += BINARY_SAMPLE_SIZE_V5_1;
    g_batch_sample_count++;

    portEXIT_CRITICAL(&batch_spinlock);
#endif

    // Restart silence timer
    // NOTE: Must be OUTSIDE critical section - esp_timer uses internal spinlocks
    // and calling from within portENTER_CRITICAL causes nested locking issues.
    // Small race window here is acceptable; worst case is a redundant flush.
    esp_timer_stop(silence_timer);
    esp_timer_start_once(silence_timer, SILENCE_TIMEOUT_US);

    return true;
}

// =============================================================================
// v5.7.0: ESP-NOW BROADCAST INIT AND RATE PINNING
// =============================================================================
static const uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Pin the ESP-NOW broadcast peer's TX rate. Called once at init
// (rate_index=0, compiled default) and again from the TX dispatch path
// only when a trigger's espnow_rate_index actually differs from what is
// currently applied -- esp_now_set_peer_rate_config() configures a PEER,
// not a per-send parameter, so re-issuing it on every burst (up to
// ~45-60 triggers/s) would be wasted API overhead for no behavioral
// change on the common case of every trigger requesting the same index.
static void apply_espnow_rate(uint8_t rate_index)
{
    if (!g_espnow_ready) {
        return;
    }
    if (rate_index == g_espnow_last_rate_index) {
        return;  // already applied -- this is the common case, cheap no-op
    }

    // v5.8.0: phymode now varies per-index too (index 4 pairs HT40 with
    // MCS0_LGI, matching esp-crab -- see the table's own comment), not
    // just the rate. rate_index==0 (compiled default) and any
    // out-of-range index both fall through to the compiled
    // phymode+default pair -- a malformed trigger field degrades to the
    // safe default, not an undefined table read.
    wifi_phy_mode_t phymode = ESPNOW_RATE_PHYMODE;
    wifi_phy_rate_t rate = ESPNOW_RATE_DEFAULT;
    if (rate_index >= 1 && rate_index <= ESPNOW_RATE_TABLE_SIZE) {
        phymode = ESPNOW_RATE_TABLE[rate_index - 1].phymode;
        rate = ESPNOW_RATE_TABLE[rate_index - 1].rate;
    }

    wifi_tx_rate_config_t rate_cfg = {
        .phymode = phymode,
        .rate = rate,
        .ersu = false,
        .dcm = false,
    };
    esp_err_t err = esp_now_set_peer_rate_config(ESPNOW_BROADCAST_MAC, &rate_cfg);
    // g_espnow_last_rate_index is updated on BOTH branches, not only on
    // success. This function's whole point is to avoid re-issuing the API
    // call every burst when nothing changed (see the comment above); if a
    // FAILURE didn't also count as "handled for this index", a persistent
    // failure (e.g. the API genuinely rejected on this chip) would retry
    // -- and re-log at ESP_LOGE -- on every single burst dispatch at
    // trigger rate (up to ~45-60 Hz), instead of once per distinct
    // requested index. g_espnow_rate_pinned is still the accurate
    // success/fail signal for anything that needs to know whether the
    // rate actually took.
    g_espnow_last_rate_index = rate_index;
    if (err == ESP_OK) {
        g_espnow_rate_pinned = true;
        ESP_LOGI(TAG, "ESP-NOW broadcast rate pinned: index=%u", rate_index);
    } else {
        // Graceful degradation, per design: rate pinning is a tuning
        // nicety, not a prerequisite for ESP-NOW broadcast to function.
        // A failed pin leaves the peer at whatever rate ESP-NOW's own
        // default applies (documented 1 Mbps) -- logged loudly (once per
        // distinct requested index, not once per burst -- see above)
        // since this directly determines whether the illumination gain
        // survives in net information-per-airtime terms, but NOT fatal:
        // the bench measures what is actually achieved, not what was
        // requested.
        g_espnow_rate_pinned = false;
        ESP_LOGE(TAG, "esp_now_set_peer_rate_config FAILED (%s) for index=%u "
                       "-- broadcast bursts will run at ESP-NOW's own default "
                       "rate, not the pinned one. Not fatal; the bench "
                       "measures the actual result, not the request.",
                 esp_err_to_name(err), rate_index);
    }
}

// Called LAZILY (see g_espnow_init_attempted's comment), the first time
// this node is asked to send burst_transport=espnow_broadcast, from
// transmit_burst()'s dispatch -- NOT from wifi_init_sta() at boot. By the
// time any trigger can request espnow, WiFi STA is already connected and
// CSI capture bring-up has already succeeded (both are boot-time
// prerequisites of the trigger loop even running). Never fatal: if
// ESP-NOW cannot be initialized, g_espnow_ready stays false and the TX
// dispatch path (transmit_burst() below) falls back to the UDP path with
// a loud log, exactly as if the server had never asked for
// espnow_broadcast at all. Does NOT pin a rate itself -- transmit_burst()
// always calls apply_espnow_rate() right after confirming g_espnow_ready,
// for whatever index that specific trigger actually requested, so pinning
// here would either be redundant (common case, index 0) or briefly pin
// the wrong index before the dispatcher corrects it (first call requests
// a non-zero index) -- neither worth the extra API call.
//
// v5.8.0: registers a no-op esp_now_register_recv_cb() (see
// espnow_recv_noop_cb() just below) -- REVERSES the earlier "deliberately
// no recv_cb" stance from v5.7.0. That stance's reasoning (every
// receiving node's job is to extract CSI from the on-air frame, which the
// existing promiscuous CSI RX callback already does at the PHY/MAC layer
// regardless of transport or payload, so no node needs ESP-NOW's own
// application-layer delivery) is still believed CORRECT -- the
// investigation doc's own precedent check found the architecturally
// closest working Espressif example (esp-crab) does NOT register one
// either, refuting the hypothesis with moderate confidence. This is
// registered anyway, for free, because "moderate confidence" is not
// "closed for good": esp-crab's broadcast peer is also added despite
// being "not strictly required for RX... but more tolerant" per that
// example's own comment -- the same defensive-belt-and-braces philosophy
// applies here. Costs nothing (a no-op callback), and permanently removes
// this specific variable from any future re-litigation of why ESP-NOW CSI
// didn't fire.
static void espnow_recv_noop_cb(const esp_now_recv_info_t *recv_info,
                                 const uint8_t *data, int data_len)
{
    (void)recv_info;
    (void)data;
    (void)data_len;
    // Deliberately empty -- see the registration comment above.
}

static void init_espnow_broadcast(void)
{
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init FAILED (%s) -- burst_transport="
                       "espnow_broadcast will fall back to UDP for this "
                       "node until reboot. Not fatal.", esp_err_to_name(err));
        return;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, ESPNOW_BROADCAST_MAC, 6);
    peer.channel = 0;       // 0 = current channel; matches this node's STA association, no separate channel-hop
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;   // broadcast frames cannot be encrypted (ESP-NOW spec: "encrypting multicast... is not supported")
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_add_peer(broadcast) FAILED (%s) -- same "
                       "UDP fallback as an init failure.", esp_err_to_name(err));
        return;
    }

    // v5.8.0: best-effort, non-fatal -- see espnow_recv_noop_cb()'s own
    // comment. A failure here does not block ESP-NOW broadcast use
    // (init+peer already succeeded, which is what g_espnow_ready gates
    // on); logged, not escalated.
    err = esp_now_register_recv_cb(espnow_recv_noop_cb);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_register_recv_cb FAILED (%s) -- proceeding "
                       "without it (see espnow_recv_noop_cb's comment: this "
                       "was always believed unnecessary, not required).",
                 esp_err_to_name(err));
    }

    g_espnow_ready = true;
    ESP_LOGI(TAG, "ESP-NOW broadcast ready (peer=ff:ff:ff:ff:ff:ff, channel=current)");
}

// =============================================================================
// TRANSMIT BURST (UDP path) - Precise timing with configurable delay
// =============================================================================
// v5.7.0: renamed from transmit_burst() to transmit_burst_udp() -- this is
// now one of two burst-transport implementations, dispatched by
// transmit_burst() below based on the trigger's burst_transport field.
// Behavior is BYTE-IDENTICAL to pre-v5.7.0: this function's body is
// unchanged, only its name and call site moved.
static void transmit_burst_udp(uint32_t count, uint32_t seq)
{
    (void)seq;  // seq not needed in binary burst (8 bytes vs 40 bytes)
    atomic_store(&is_transmitting, true);

    // Use configured burst count if 0 is passed
    if (count == 0) {
        count = CONFIG_BURST_COUNT;
    }

    // Pre-build binary burst packet (only 8 bytes vs ~40 bytes text)
    binary_burst_t pkt = {
        .magic = BINARY_PROTO_MAGIC,
        .type = MSG_TYPE_BURST,
    };
    memcpy(pkt.tx_mac, node_mac_bytes, 6);

    ESP_LOGD(TAG, "Starting burst TX: %" PRIu32 " packets (binary, UDP), %d us delay",
             count, CONFIG_BURST_DELAY_US);

    for (uint32_t i = 0; i < count; i++) {
        send_broadcast_packet((const char *)&pkt, sizeof(binary_burst_t));

        // Precise inter-packet delay for consistent timing
        if (CONFIG_BURST_DELAY_US > 0) {
            esp_rom_delay_us(CONFIG_BURST_DELAY_US);
        }

        // Yield periodically to prevent watchdog timeout (every 32 packets)
        if ((i & 0x1F) == 0x1F) {
            vTaskDelay(1);
        }
    }

    atomic_store(&is_transmitting, false);

    // v5.1.1 SAFETY: Always yield after burst to let other tasks run
    // Critical for small burst counts that don't trigger the periodic yield
    vTaskDelay(1);

    ESP_LOGD(TAG, "Burst TX complete: %" PRIu32 " packets (UDP)", count);
}

// =============================================================================
// v5.7.0: TRANSMIT BURST (ESP-NOW broadcast path)
// =============================================================================
// Mirrors transmit_burst_udp()'s structure and timing (same
// CONFIG_BURST_DELAY_US inter-packet spacing, same watchdog-yield cadence)
// so the two paths are comparable in the bench A/B -- only the actual
// transmission call, payload, and the is_transmitting drain margin below
// differ. Does NOT pin a rate itself -- the caller (transmit_burst()'s
// dispatch) must already have confirmed g_espnow_rate_pinned before
// calling this; sending here always assumes the rate currently applied to
// ESPNOW_BROADCAST_MAC is the one the caller wanted.
static void transmit_burst_espnow(uint32_t count, uint32_t seq)
{
    atomic_store(&is_transmitting, true);

    if (count == 0) {
        count = CONFIG_BURST_COUNT;
    }

    espnow_burst_t pkt = {
        .magic = BINARY_PROTO_MAGIC,
        .type = MSG_TYPE_BURST,
        .seq = seq,
    };
    memcpy(pkt.tx_mac, node_mac_bytes, 6);

    ESP_LOGD(TAG, "Starting burst TX: %" PRIu32 " packets (binary, ESP-NOW "
                   "broadcast), %d us delay", count, CONFIG_BURST_DELAY_US);

    // v5.7.0 (directive-build review, BLOCKER 2): throttled failure
    // logging. A silent esp_now_send() failure (e.g. ESP_ERR_ESPNOW_NO_MEM
    // under a full send queue -- plausible in a tight burst loop, see
    // attack e's own comment on esp_now_send()'s async queue) would look,
    // from the bench's phase-1 counters alone, identical to "CSI never
    // fires" -- a fake phase-1 FAIL with the real cause invisible on the
    // attached serial console. Logged once per burst (first occurrence),
    // not once per failed frame, to avoid the log itself flooding at
    // burst rate; the summary line after the loop reports how many of
    // this burst's frames failed.
    uint32_t burst_send_failures = 0;
    for (uint32_t i = 0; i < count; i++) {
        esp_err_t err = esp_now_send(ESPNOW_BROADCAST_MAC,
                                      (const uint8_t *)&pkt, sizeof(pkt));
        if (err != ESP_OK) {
            if (burst_send_failures == 0) {
                ESP_LOGW(TAG, "esp_now_send FAILED (%s) on frame %" PRIu32
                               "/%" PRIu32 " of this burst -- not fatal, "
                               "continuing (see the summary line after this "
                               "burst for the total)",
                         esp_err_to_name(err), i + 1, count);
            }
            burst_send_failures++;
            g_espnow_send_failures++;
        }

        if (CONFIG_BURST_DELAY_US > 0) {
            esp_rom_delay_us(CONFIG_BURST_DELAY_US);
        }

        if ((i & 0x1F) == 0x1F) {
            vTaskDelay(1);
        }
    }
    if (burst_send_failures > 0) {
        ESP_LOGW(TAG, "Burst TX: %" PRIu32 "/%" PRIu32 " ESP-NOW sends failed "
                       "this burst (cumulative g_espnow_send_failures=%lu)",
                 burst_send_failures, count, (unsigned long)g_espnow_send_failures);
    }

    // v5.7.0 (directive-build pre-registered review, attack e): esp_now_send()
    // is ASYNCHRONOUS -- it returns once the frame is queued to the driver,
    // not once it is actually on-air. Clearing is_transmitting immediately
    // after the loop (as transmit_burst_udp() does, where sendto()
    // completion is a much closer proxy for on-air completion) would let
    // this node's own CSI callback re-admit frames -- INCLUDING its own
    // still-in-flight broadcast, since process_trigger() already added
    // this node's own MAC to its recent-trigger-TX set before calling
    // transmit_burst() (see that call site's comment) -- while the last
    // queued frame(s) are still finishing. No esp_now_register_send_cb()
    // is used to wait for actual completion: broadcast frames' send
    // callbacks carry no delivery status (ESP-NOW spec has no ACK for
    // broadcast), so there is nothing trustworthy to key a wait on. This
    // fixed, deliberately generous drain margin stands in instead --
    // ESPNOW_WORST_CASE_FRAME_AIRTIME_US's own comment has the bound this
    // is generous relative to. csi_callback()'s cheap src-mac self-check
    // (see its own comment) is a second, independent layer against
    // whatever residual gap remains.
    esp_rom_delay_us(ESPNOW_TX_DRAIN_MARGIN_US);
    atomic_store(&is_transmitting, false);
    vTaskDelay(1);

    ESP_LOGD(TAG, "Burst TX complete: %" PRIu32 " packets (ESP-NOW broadcast)", count);
}

// =============================================================================
// v5.8.0: TRANSMIT BURST (raw-injection broadcast DATA frame path)
// =============================================================================
// Mirrors transmit_burst_espnow()'s structure (same is_transmitting
// guarding, same CONFIG_BURST_DELAY_US inter-packet spacing, same
// watchdog-yield cadence, same throttled failure logging shape) so the
// three paths stay comparable in the bench. Builds one raw_inject_burst_t
// per frame (magic/type/seq change per-frame via the reused
// espnow_burst_t body; the 802.11 header is otherwise identical for
// every frame in this burst, so it is built once outside the loop).
static void transmit_burst_raw_inject(uint32_t count, uint32_t seq)
{
    // No rate hint/pin attempted here (see the RATE CONTROL comment
    // above raw_inject_burst_fits_slot() for the removal rationale,
    // 2026-08-15) -- injected frames go out at the driver's own default
    // rate for esp_wifi_80211_tx()-injected non-QoS Data frames, observed
    // (not controlled) via the delivered frame_format field.

    atomic_store(&is_transmitting, true);

    if (count == 0) {
        count = CONFIG_BURST_COUNT;
    }

    raw_inject_burst_t frame = {
        .frame_control = RAW_INJECT_FRAME_CONTROL_DATA_NONQOS,
        .duration = RAW_INJECT_DURATION,
        .seq_ctrl = 0,  // driver-overwritten, en_sys_seq=true -- see the struct's own comment
    };
    memset(frame.addr1, 0xFF, 6);           // broadcast DA
    memcpy(frame.addr2, node_mac_bytes, 6); // SA -- the attribution field
    memcpy(frame.addr3, node_bssid_bytes, 6);  // BSSID
    frame.body.magic = BINARY_PROTO_MAGIC;
    frame.body.type = MSG_TYPE_BURST;
    frame.body.seq = seq;  // trigger seq, same for every frame in this burst
                            // (identifies the trigger, not the frame-within-burst
                            // -- matches transmit_burst_espnow()'s identical
                            // semantics for espnow_burst_t.seq) -- set once here,
                            // not per-iteration below, since it's loop-invariant;
                            // header seq_ctrl is not ours to set (en_sys_seq=true).
    memcpy(frame.body.tx_mac, node_mac_bytes, 6);

    ESP_LOGD(TAG, "Starting burst TX: %" PRIu32 " packets (binary, raw-inject "
                   "broadcast DATA), %d us delay", count, CONFIG_BURST_DELAY_US);

    // Same throttled-logging shape as transmit_burst_espnow()'s -- see that
    // function's own comment for why (a silent failure here would produce
    // an identical-looking false phase-1 FAIL on the bench).
    uint32_t burst_send_failures = 0;
    for (uint32_t i = 0; i < count; i++) {
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, &frame, sizeof(frame), true);
        if (err != ESP_OK) {
            if (burst_send_failures == 0) {
                ESP_LOGW(TAG, "esp_wifi_80211_tx FAILED (%s) on frame %" PRIu32
                               "/%" PRIu32 " of this burst -- not fatal, "
                               "continuing (see the summary line after this "
                               "burst for the total)",
                         esp_err_to_name(err), i + 1, count);
            }
            burst_send_failures++;
            g_raw_inject_send_failures++;
        }

        if (CONFIG_BURST_DELAY_US > 0) {
            esp_rom_delay_us(CONFIG_BURST_DELAY_US);
        }

        if ((i & 0x1F) == 0x1F) {
            vTaskDelay(1);
        }
    }
    if (burst_send_failures > 0) {
        ESP_LOGW(TAG, "Burst TX: %" PRIu32 "/%" PRIu32 " raw-inject sends failed "
                       "this burst (cumulative g_raw_inject_send_failures=%lu)",
                 burst_send_failures, count, (unsigned long)g_raw_inject_send_failures);
    }

    // Same conservative treatment as ESP-NOW's drain margin (see
    // transmit_burst_espnow()'s comment for the full reasoning) -- applies
    // here for an identical structural reason (process_trigger() adds this
    // node's own MAC to its recent-trigger-TX set before TX-vs-RX is
    // decided) and an ADDITIONAL one: esp_wifi_80211_tx()'s own doc
    // comment lists ESP_ERR_WIFI_NO_MEM as a possible failure, the same
    // shape of "queue can fill up" error ESP-NOW documents, with no
    // stronger synchronicity guarantee documented either. See
    // RAW_INJECT_TX_DRAIN_MARGIN_US's own comment for the sizing.
    esp_rom_delay_us(RAW_INJECT_TX_DRAIN_MARGIN_US);
    atomic_store(&is_transmitting, false);
    vTaskDelay(1);

    ESP_LOGD(TAG, "Burst TX complete: %" PRIu32 " packets (raw-inject broadcast)", count);
}

// =============================================================================
// v5.7.0/v5.8.0: TRANSMIT BURST dispatch
// =============================================================================
// Reads burst_transport (trigger reserved[2], 0=udp default) and selects
// the implementation: 0=UDP, 1=ESP-NOW broadcast, 2=raw-injection
// broadcast DATA frame (v5.8.0). UDP is the compiled default and what
// every field deployment runs until a server explicitly asks for one of
// the other two -- this dispatch is the ONLY new code on the hot path for
// a node that never receives that request, and it degrades to exactly
// the pre-v5.7.0 call (transmit_burst_udp) in that case.
static void transmit_burst(uint32_t count, uint32_t seq, uint8_t burst_transport,
                            uint8_t espnow_rate_index, uint16_t slot_ms)
{
    if (burst_transport == BURST_TRANSPORT_ESPNOW_BROADCAST) {
        // Lazy, one-shot init (directive-build attack a) -- see
        // g_espnow_init_attempted's own comment for why this isn't at boot.
        if (!g_espnow_ready && !g_espnow_init_attempted) {
            g_espnow_init_attempted = true;
            init_espnow_broadcast();
        }

        uint32_t effective_count = (count == 0) ? CONFIG_BURST_COUNT : count;

        if (g_espnow_ready
            && espnow_burst_fits_slot(effective_count, slot_ms)) {
            // Rate MUST be confirmed pinned before this path is trusted --
            // directive-build attack d: an unpinned send falls back to
            // ESP-NOW's own ~1 Mbps default, which this firmware never
            // knowingly transmits at (see the refusal below). Cheap when
            // already applied (apply_espnow_rate()'s own no-op check).
            apply_espnow_rate(espnow_rate_index);
            if (g_espnow_rate_pinned) {
                transmit_burst_espnow(effective_count, seq);
                return;
            }
        }
        // Refused: either ESP-NOW isn't ready, the requested burst would
        // overrun this trigger's own slot_ms at the worst pinned rate
        // (espnow_burst_fits_slot()), or the rate pin failed/never
        // succeeded (g_espnow_rate_pinned false) -- any of these means
        // "do not trust this path for this burst," not "degrade the rate
        // and send anyway." Graceful fallback, loudly logged, never silent.
        g_espnow_fallback_to_udp++;
        ESP_LOGW(TAG, "burst_transport=espnow_broadcast requested but not "
                       "usable this burst (ready=%d fits_slot=%d pinned=%d) "
                       "-- falling back to UDP burst (fallback count: %lu)",
                 g_espnow_ready,
                 espnow_burst_fits_slot(effective_count, slot_ms),
                 g_espnow_rate_pinned,
                 (unsigned long)g_espnow_fallback_to_udp);
    } else if (burst_transport == BURST_TRANSPORT_RAW_INJECT) {
        uint32_t effective_count = (count == 0) ? CONFIG_BURST_COUNT : count;

        // No init-readiness or rate-pin gate exists for this transport
        // (see the RAW-INJECTION section's own comments) -- the slot-fit
        // guard, using the wider, unconfirmed-rate-aware bound, IS this
        // transport's safety gate.
        if (raw_inject_burst_fits_slot(effective_count, slot_ms)) {
            transmit_burst_raw_inject(effective_count, seq);
            return;
        }
        g_raw_inject_fallback_to_udp++;
        ESP_LOGW(TAG, "burst_transport=raw_inject requested but the burst "
                       "would overrun this trigger's slot_ms at the "
                       "conservative per-frame bound -- falling back to UDP "
                       "burst (fallback count: %lu)",
                 (unsigned long)g_raw_inject_fallback_to_udp);
    }
    transmit_burst_udp(count, seq);
}

// =============================================================================
// HELPER: Validate binary packet CRC32
// =============================================================================
static bool validate_crc32(const uint8_t *data, size_t len)
{
    if (len < 4) return false;

    // CRC32 is last 4 bytes
    uint32_t received_crc;
    memcpy(&received_crc, data + len - 4, sizeof(uint32_t));

    // Calculate CRC over packet excluding CRC field
    uint32_t calculated_crc = esp_crc32_le(0, data, len - 4);

    return (received_crc == calculated_crc);
}

// =============================================================================
// HELPER: Convert MAC bytes to string for logging
// =============================================================================
static void mac_bytes_to_str(const uint8_t *mac_bytes, char *mac_str)
{
    snprintf(mac_str, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_bytes[0], mac_bytes[1], mac_bytes[2],
             mac_bytes[3], mac_bytes[4], mac_bytes[5]);
}

// =============================================================================
// HELPER: Process trigger and collect CSI
// =============================================================================
static void process_trigger(const uint8_t *tx_mac_bytes, uint32_t seq,
                           uint16_t burst_count, uint16_t slot_ms,
                           uint8_t slot_id, uint16_t attribution_margin_us,
                           uint8_t burst_transport, uint8_t espnow_rate_index)
{
    // Build MAC string for logging and CSI packet
    char tx_mac_str[18];
    mac_bytes_to_str(tx_mac_bytes, tx_mac_str);

    // Build new trigger state
    trigger_state_t new_state = {0};
    memcpy(new_state.tx_mac_bytes, tx_mac_bytes, 6);
    strncpy(new_state.tx_mac_str, tx_mac_str, sizeof(new_state.tx_mac_str) - 1);
    new_state.seq = seq;
    new_state.active = true;
    new_state.activated_at = xTaskGetTickCount();

    // Update global trigger state atomically. v5.6.0 round-3: also push
    // this trigger's TX into the recent-trigger-TX set, same critical
    // section -- see that set's own comment block (above csi_callback())
    // for why this replaces the exact-current-trigger capture filter.
    portENTER_CRITICAL(&trigger_spinlock);
    g_trigger_state = new_state;
    recent_trigger_tx_add_locked(tx_mac_bytes);
    portEXIT_CRITICAL(&trigger_spinlock);

    // Check if we are the transmitter
    if (mac_bytes_equal(tx_mac_bytes, node_mac_bytes)) {
        ESP_LOGD(TAG, "TRIGGER: TX mode, seq=%" PRIu32 ", bursts=%u", seq, burst_count);
        transmit_burst(burst_count, seq, burst_transport, espnow_rate_index, slot_ms);
        // v5.1.1 SAFETY: Feed watchdog after TX burst
        esp_task_wdt_reset();
        // v5.4.1 FIX: this early return used to skip clearing g_trigger_state.active,
        // leaving it stuck true with trigger_mac == our own MAC. csi_callback()'s
        // peer filter (mac_bytes_equal(info->mac, trigger_mac)) then discards every
        // subsequent CSI frame forever -- a node never sees CSI attributed to its
        // own MAC, so the filter matched nothing until the next trigger for this
        // node happened to arrive. Root-cause candidate for a node going fully CSI
        // silent (heartbeats/BFI unaffected) after being scheduled as TX and then
        // not retriggered as RX -- e.g. a server restart that drops it from the
        // rotation. is_transmitting (not trigger_active) already suppresses CSI
        // processing for the duration of the burst itself, so clearing here is safe.
        clear_trigger_state();
        return;
    }

    // We are receiver - collect CSI samples
    ESP_LOGD(TAG, "TRIGGER: RX mode, collecting CSI from %s", tx_mac_str);

    // v5.6.0: the ring buffer is NOT cleared here. It used to be
    // (window-scoped retention), which discarded frames from the correct,
    // already TX-filtered transmitter that happened to arrive before THIS
    // node got around to processing its copy of the broadcast trigger --
    // see the v5.6.0 changelog entry above main() for the full mechanism.
    // The ring keeps whatever csi_callback() already accepted; attribution
    // is decided explicitly below, at batch time, not by when an item
    // happened to arrive relative to this function running.
    size_t item_size;
    void *item;

    // Wait for CSI data
    vTaskDelay(pdMS_TO_TICKS(slot_ms));

    // v5.1.1 SAFETY: Feed watchdog after RX wait to prevent timeout
    // during rapid trigger processing
    esp_task_wdt_reset();

    // Batch all CSI samples into single packet
    start_new_batch_v5(slot_id);

    // v5.6.0 round-3 TX-MATCH ATTRIBUTION (2026-08-14, directive-build
    // round-2 review). An item earns a place in this trigger's batch if
    // EITHER of two disjuncts holds -- OR composition, not AND:
    //
    //   admitted = (pkt->trigger_seq == seq)
    //              || (pkt->src_mac == tx_mac_bytes && age <= max_age_us)
    //
    // WHY OR, NOT AND (this is the round-2 -> round-3 correction): the
    // round-2 design required ALL of {seq match, mac match, time bound},
    // which is SELF-CANCELLING for the exact population this fix exists to
    // recover -- an early-arriving frame from trigger N, captured before
    // this node's OWN g_trigger_state has rolled over to N (still reads
    // N-1's seq locally at capture time), gets pkt->trigger_seq == N-1 (or
    // whatever this node's local seq was at capture), never N -- AND-ing
    // that against seq==N always fails, regardless of how permissive the
    // capture-time filter or the time bound are. Round-2 hardened the
    // WRONG layer: the batch-time predicate was never the bottleneck: the
    // capture-time MAC filter (see recent_trigger_tx_contains_locked() and
    // its comment above csi_callback()) was rejecting the frame before it
    // ever reached this loop, and even the frames that DID reach the ring
    // were then rejected again here by the AND-composed seq requirement.
    //
    // DISJUNCT A, exact identity: pkt->trigger_seq == seq. Fires when this
    // node's own g_trigger_state had ALREADY rolled over to this trigger
    // by the time the frame was captured -- the common case once slot
    // timing has settled, and the same discriminator round-2 relied on for
    // separating back-to-back same-TX triggers (2-node focus mode can
    // repeat the same pair every MIN_SLOT_MS*2 ~ 12ms at the scheduler's
    // floor) -- src_mac alone cannot do this, since both triggers share
    // the same transmitter.
    // DISJUNCT B, mac-plus-tight-time fallback: pkt->src_mac equals THIS
    // trigger's specific tx_mac_bytes (not just "some recent TX" -- that
    // broader test already happened at capture time and is not repeated
    // here) AND the item's age is within max_age_us of this trigger's own
    // slot_ms window. This is what admits the early/gap-arrival frames
    // that disjunct A cannot: captured while this node's local seq was
    // still stale (or, right after boot, while it was 0), but genuinely
    // from the transmitter this trigger addresses, and recent enough to be
    // this trigger's own burst rather than a previous cycle's residue.
    //
    // SAFETY ARGUMENT for disjunct B admitting the wrong thing (a leftover
    // frame from an EARLIER same-TX cycle, not this one): at a k-node
    // focus/rotation set with k>=2, the same TX is not revisited sooner
    // than k*slot_ms (a full rotation -- k-1 OTHER slots plus this node's
    // own returning slot), so a previous same-TX cycle's leftover item is
    // AT LEAST k*slot_ms >= 2*slot_ms old by the time this trigger's batch
    // runs -- and max_age_us's margin term is derived specifically to stay
    // BELOW (k-1)*slot_ms (margin = min(5ms, slot_ms/2), see
    // ATTRIBUTION_TIME_MARGIN_US's comment for the full k-node derivation),
    // so max_age_us = slot_ms + margin < k*slot_ms <= the leftover's actual
    // age -- excluded by the time bound. For k=1 this argument does not
    // hold (the same TX is revisited every slot_ms, so a "previous cycle"
    // item is barely younger than max_age_us, not excluded) -- which is
    // exactly why a k=1 focus set is refused UPSTREAM, at the scheduler
    // (backend/src/scheduler/tdma_scheduler.py's _apply_mode_change,
    // 2026-08-14), not defended against here. Under OR composition that
    // guard is the ONLY protection against k=1, more load-bearing than it
    // was under round-2's AND composition, not less.
    //
    // Anything failing both disjuncts is dropped and counted
    // (g_csi_attribution_dropped), never requeued or guessed at --
    // misattribution is a worse failure than loss. Because
    // g_csi_attribution_dropped now only fires on the (rarer, since OR is
    // more permissive than AND) double-failure case, a low count here is
    // NOT proof the fix is working -- see the runbook's pilot watch
    // criteria, which use server-side delivered-sample-rate as the primary
    // signal instead, for exactly this reason.
    int64_t batch_now_us = esp_timer_get_time();
    // Derived default: min(cap, slot_ms_us / 2). slot_ms is uint16_t (max
    // 65535), so slot_ms*500 fits comfortably in uint32_t (max ~32.7M) --
    // no overflow. See ATTRIBUTION_TIME_MARGIN_US's comment for the
    // k-node invariant this derivation satisfies.
    uint32_t derived_default_margin_us = (uint32_t)slot_ms * 500;
    if (derived_default_margin_us > ATTRIBUTION_TIME_MARGIN_US) {
        derived_default_margin_us = ATTRIBUTION_TIME_MARGIN_US;
    }
    uint32_t effective_margin_us = attribution_margin_us
        ? (uint32_t)attribution_margin_us : derived_default_margin_us;
    int64_t max_age_us = ((int64_t)slot_ms * 1000) + (int64_t)effective_margin_us;

    uint8_t sample_count = 0;
    uint32_t attribution_dropped_this_batch = 0;
    while ((item = xRingbufferReceive(csi_ringbuf, &item_size, 0)) != NULL) {
        csi_packet_t *pkt = (csi_packet_t *)item;
        bool seq_match = (pkt->trigger_seq == seq);
        bool tx_match = mac_bytes_equal(pkt->src_mac, tx_mac_bytes);
        bool within_window = (batch_now_us - pkt->timestamp_us) <= max_age_us;
        bool admitted = seq_match || (tx_match && within_window);
        if (admitted) {
            add_sample_to_batch_v5(pkt);
            sample_count++;
        } else {
            g_csi_attribution_dropped++;
            attribution_dropped_this_batch++;
        }
        vRingbufferReturnItem(csi_ringbuf, item);
    }

    // Flush the batch
    finalize_and_send_batch_v5();

    if (sample_count > 0) {
        ESP_LOGD(TAG, "Batch sent: slot=%d, samples=%d", slot_id, sample_count);
    } else {
        ESP_LOGD(TAG, "No CSI data received");
    }
    if (attribution_dropped_this_batch > 0) {
        ESP_LOGD(TAG, "Attribution-dropped %lu item(s) this batch (src_mac/age "
                       "mismatch, expected TX %s)",
                 (unsigned long)attribution_dropped_this_batch, tx_mac_str);
    }

    // Clear trigger state
    clear_trigger_state();
}

// =============================================================================
// TRIGGER HANDLER TASK - Binary protocol with JSON fallback
// =============================================================================
// Adopt the sender of a CRC-valid server packet (DISCOVER or TRIGGER) as the
// server address. Only the server sends these, so their source IP is the
// server's address on this node's subnet. This makes mDNS and the Kconfig
// SERVER_IP optional: a node that booted before the server, or with the
// wrong fallback IP, or whose server changed address, corrects itself on the
// next packet instead of sending to a dead address until reboot. The
// single aligned 32-bit store is atomic on these cores, so the other tasks
// that read server_addr see either the old or the new address.
static void adopt_server_addr(const struct sockaddr_in *src)
{
    if (src->sin_family != AF_INET ||
        src->sin_addr.s_addr == server_addr.sin_addr.s_addr) {
        return;
    }
    char old_ip[INET_ADDRSTRLEN], new_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &server_addr.sin_addr, old_ip, sizeof(old_ip));
    inet_ntop(AF_INET, &src->sin_addr, new_ip, sizeof(new_ip));
    server_addr.sin_addr.s_addr = src->sin_addr.s_addr;
    burst_unicast_addr.sin_addr.s_addr = src->sin_addr.s_addr;
    ESP_LOGW(TAG, "Server address %s -> %s (learned from server packet)",
             old_ip, new_ip);
    send_hello();
}

static void trigger_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Trigger listener started on port %d (binary protocol)", CONFIG_TRIGGER_PORT);

    // Subscribe to task watchdog
    esp_task_wdt_add(NULL);

    uint8_t rx_buffer[512];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    uint32_t loop_count = 0;

    while (1) {
        // Feed watchdog every loop iteration
        esp_task_wdt_reset();

        // v5.4.1: trigger-active staleness watchdog. Runs every iteration
        // (bounded by the trigger socket's SOCKET_TIMEOUT_MS recv timeout, so
        // at least every ~100ms when idle) rather than the loop_count%50
        // cadence below, since trigger traffic rate (and therefore how often
        // this loop spins) varies with scheduler mode. Defense-in-depth for
        // the TX-branch fix in process_trigger(): if g_trigger_state.active
        // is ever left set longer than TRIGGER_ACTIVE_STALE_MS by any path,
        // force-clear it rather than let the CSI callback's peer filter
        // discard every frame indefinitely.
        portENTER_CRITICAL(&trigger_spinlock);
        bool trig_active = g_trigger_state.active;
        TickType_t trig_since = g_trigger_state.activated_at;
        portEXIT_CRITICAL(&trigger_spinlock);
        if (trig_active &&
            (xTaskGetTickCount() - trig_since) >= pdMS_TO_TICKS(TRIGGER_ACTIVE_STALE_MS)) {
            g_trigger_stale_recoveries++;
            ESP_LOGE(TAG, "trigger_active stuck for >%dms, force-clearing (recovery #%lu) -- "
                           "CSI capture was being silently discarded",
                     TRIGGER_ACTIVE_STALE_MS, (unsigned long)g_trigger_stale_recoveries);
            clear_trigger_state();
        }

        // Periodic heap check
        if ((++loop_count % 50) == 0) {
            uint32_t free_heap = esp_get_free_heap_size();
            uint32_t min_heap = esp_get_minimum_free_heap_size();

            if (free_heap < HEAP_CRITICAL_THRESHOLD) {
                ESP_LOGE(TAG, "CRITICAL: Heap exhausted (%lu bytes), rebooting!",
                         (unsigned long)free_heap);
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            } else if (free_heap < HEAP_WARNING_THRESHOLD) {
                ESP_LOGW(TAG, "WARNING: Low heap: %lu bytes (min: %lu)",
                         (unsigned long)free_heap, (unsigned long)min_heap);
            }

            // v5.2.0: Log CSI drops if any occurred
            if (g_csi_drops > 0) {
                ESP_LOGW(TAG, "CSI ring buffer drops: %lu", (unsigned long)g_csi_drops);
            }

            // v5.6.0: Log TX-match/time-bound attribution drops if any occurred
            if (g_csi_attribution_dropped > 0) {
                ESP_LOGW(TAG, "CSI attribution drops: %lu", (unsigned long)g_csi_attribution_dropped);
            }

            // v5.2.1: Log UDP errors for debugging
            if (g_udp_send_failures > 0 || g_udp_mutex_timeouts > 0) {
                ESP_LOGW(TAG, "UDP errors: send_fail=%lu, mutex_timeout=%lu",
                         (unsigned long)g_udp_send_failures, (unsigned long)g_udp_mutex_timeouts);
            }
        }

        int len = recvfrom(trigger_sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                          (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  // Normal timeout
            }
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Check for binary protocol (magic byte 0xC5)
        if (len >= 2 && rx_buffer[0] == BINARY_PROTO_MAGIC) {
            uint8_t msg_type = rx_buffer[1];

            switch (msg_type) {
                case MSG_TYPE_DISCOVER:
                    if (len >= BINARY_DISCOVER_SIZE && validate_crc32(rx_buffer, len)) {
                        ESP_LOGI(TAG, "DISCOVER received (binary)");
                        adopt_server_addr(&source_addr);
                        send_hello();
                    } else {
                        ESP_LOGW(TAG, "Invalid DISCOVER packet (len=%d, crc=%s)",
                                 len, validate_crc32(rx_buffer, len) ? "ok" : "fail");
                    }
                    break;

                case MSG_TYPE_TRIGGER:
                    if (len >= BINARY_TRIGGER_SIZE && validate_crc32(rx_buffer, len)) {
                        binary_trigger_t *trig = (binary_trigger_t *)rx_buffer;
                        adopt_server_addr(&source_addr);

                        // Rate limiting: ignore triggers that arrive too fast
                        // v5.2.1: Use esp_timer_get_time() (64-bit) to avoid tick count overflow
                        int64_t now_us = esp_timer_get_time();
                        int64_t elapsed_ms = (now_us - s_last_trigger_time_us) / 1000;

                        if (elapsed_ms < MIN_TRIGGER_INTERVAL_MS) {
                            ESP_LOGD(TAG, "Trigger skipped: only %" PRId64 "ms since last (min %dms)",
                                     elapsed_ms, MIN_TRIGGER_INTERVAL_MS);
                            break;  // Skip this trigger, wait for next
                        }

                        s_last_trigger_time_us = now_us;

                        // Simplified logging for production
                        ESP_LOGD(TAG, "TRIGGER seq=%" PRIu32, trig->seq);

                        // v5.6.0: attribution_margin_us is server-settable at
                        // runtime, same "0 = use compiled default" convention
                        // as burst_count/slot_ms above -- packed little-endian
                        // uint16 in the first 2 of the 9 reserved bytes (no
                        // BINARY_TRIGGER_SIZE change; those bytes already
                        // existed and were always sent as 0 by every server
                        // build to date, so this is backward compatible in
                        // both directions: an old server sending zeros here
                        // gets today's compiled-default behavior unchanged,
                        // and a new server can override per-trigger, cheaply,
                        // via the SAME packet already sent for every cycle --
                        // no new message type needed).
                        uint16_t attribution_margin_us;
                        memcpy(&attribution_margin_us, trig->reserved, sizeof(uint16_t));

                        // v5.7.0: burst_transport/espnow_rate_index are the
                        // next 2 of the 9 reserved bytes (reserved[2],
                        // reserved[3] -- attribution_margin_us already owns
                        // reserved[0:2]). Same "0 = compiled default,
                        // backward compatible in both directions" convention:
                        // every server build to date sends zeros here, which
                        // is BURST_TRANSPORT_UDP -- byte-identical to
                        // pre-v5.7.0 behavior until a server explicitly asks
                        // for espnow_broadcast. 5 bytes remain free after this.
                        uint8_t burst_transport = trig->reserved[2];
                        uint8_t espnow_rate_index = trig->reserved[3];
                        // v5.9.0: reserved[4] = burst_dest (0=broadcast
                        // default, 1=unicast-to-server). Airtime lever, does
                        // not touch illumination. Latched into the global the
                        // UDP send path reads.
                        // WARNING (review finding, do not remove): this is a
                        // GLOBAL latch, unlike burst_transport which is a
                        // per-trigger PARAMETER. Triggers are IP-broadcast
                        // fleet-wide, so every node latches every trigger. This
                        // is correct ONLY while burst_dest is fleet-uniform. If
                        // a per-MAC bench override is ever added (à la
                        // BURST_TRANSPORT_BENCH_MAC), convert burst_dest to a
                        // process_trigger PARAMETER first, or the bench node's
                        // value silently applies to all ten boards.
                        g_burst_unicast = trig->reserved[4];
                        process_trigger(
                            trig->tx_mac,
                            trig->seq,
                            trig->burst_count ? trig->burst_count : CONFIG_BURST_COUNT,
                            trig->slot_ms ? trig->slot_ms : 80,
                            trig->slot_id,
                            attribution_margin_us,
                            burst_transport,
                            espnow_rate_index
                        );
                    } else {
                        ESP_LOGW(TAG, "Invalid TRIGGER packet (len=%d, crc=%s)",
                                 len, validate_crc32(rx_buffer, len) ? "ok" : "fail");
                    }
                    break;

                default:
                    ESP_LOGD(TAG, "Unknown binary message type: 0x%02X", msg_type);
                    break;
            }
            continue;
        }

        // Non-binary packet - ignore silently (JSON support removed in v4.1)
    }
}

// =============================================================================
// HEARTBEAT TASK (with watchdog and WiFi monitoring)
// =============================================================================
static void heartbeat_task(void *pvParameters)
{
    // Subscribe to task watchdog
    esp_task_wdt_add(NULL);

    // Initial announcements with delay
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i < 3; i++) {
        esp_task_wdt_reset();
        send_hello();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    TickType_t last_hello = xTaskGetTickCount();
    TickType_t last_heartbeat = xTaskGetTickCount();
#ifdef CONFIG_ENABLE_BFI_CAPTURE
    uint32_t bfi_wd_last = g_bfi_captured;           // last observed capture count
    TickType_t bfi_wd_since = xTaskGetTickCount();   // when it last advanced
#endif
    // CSI delivery watchdog state (see CSI_DELIVERY_WATCHDOG_S)
    uint32_t dlv_wd_batches = g_csi_batches_sent;    // last observed dispatch count
    uint32_t dlv_wd_captured = g_csi_captured;       // capture count at window start
    TickType_t dlv_wd_since = xTaskGetTickCount();   // when dispatch last advanced
    uint8_t dlv_wd_strikes = 0;                      // consecutive expiries

    while (1) {
        // Feed watchdog
        esp_task_wdt_reset();

        TickType_t now = xTaskGetTickCount();

        // Check WiFi connection status
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        bool wifi_connected = (bits & WIFI_CONNECTED_BIT) != 0;

        // Send HELLO periodically for re-discovery (only if connected)
        if (wifi_connected && (now - last_hello) >= pdMS_TO_TICKS(ANNOUNCE_INTERVAL_MS)) {
            send_hello();
            last_hello = now;
        }

        // Send heartbeat (only if connected)
        if (wifi_connected && (now - last_heartbeat) >= pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS)) {
            send_heartbeat();
            last_heartbeat = now;
#ifdef CONFIG_ENABLE_BFI_CAPTURE
            if (g_bfi_captured || g_bfi_drops) {
                ESP_LOGI(TAG, "BFI: captured=%lu drops=%lu",
                         (unsigned long)g_bfi_captured, (unsigned long)g_bfi_drops);
            }
#endif
        }

#ifdef CONFIG_ENABLE_BFI_CAPTURE
        // Forward any queued beamforming reports (infrequent).
        if (wifi_connected) {
            drain_bfi();
        }

        // BFI watchdog: self-heal a stalled promiscuous RX path. If we are
        // connected but no BFI has been captured for BFI_WATCHDOG_STALL_S,
        // re-arm capture. Catches stalls that do not raise a re-arm event
        // (re-assoc keeping the IP, channel/scan events). Don't count stall
        // time while disconnected.
        if (wifi_connected) {
            uint32_t cur = g_bfi_captured;
            if (cur != bfi_wd_last) {
                bfi_wd_last = cur;
                bfi_wd_since = now;
            } else if ((now - bfi_wd_since) >= pdMS_TO_TICKS(BFI_WATCHDOG_STALL_S * 1000)) {
                ESP_LOGW(TAG, "BFI watchdog: no capture in %ds while connected, re-arming",
                         BFI_WATCHDOG_STALL_S);
                restart_capture();
                bfi_wd_since = now;  // reset window; don't re-arm every iteration
            }
        } else {
            bfi_wd_since = now;
        }
#endif

        // CSI delivery watchdog: capture advancing + no batch dispatched for
        // CSI_DELIVERY_WATCHDOG_S means the delivery path is wedged while the
        // node still looks alive to the server (heartbeats flow, capture
        // counter grows). The A5:02:40 incident sat in exactly this state for
        // 7 days. Escalation: strike 1 re-arms capture (cheap, idempotent);
        // strike 2 reboots. Any dispatch clears the strikes. Don't count
        // stall time while disconnected, and require capture to have
        // advanced within the window so an idle mesh never trips it.
        if (wifi_connected) {
            uint32_t sent_now = g_csi_batches_sent;
            uint32_t cap_now = g_csi_captured;
            if (sent_now != dlv_wd_batches) {
                dlv_wd_batches = sent_now;
                dlv_wd_captured = cap_now;
                dlv_wd_since = now;
                dlv_wd_strikes = 0;
            } else if ((now - dlv_wd_since) >= pdMS_TO_TICKS(CSI_DELIVERY_WATCHDOG_S * 1000)) {
                if (cap_now == dlv_wd_captured) {
                    // mesh genuinely quiet (nothing captured either): not a
                    // wedge, restart the observation window
                    dlv_wd_captured = cap_now;
                    dlv_wd_since = now;
                } else if (dlv_wd_strikes == 0) {
                    dlv_wd_strikes = 1;
                    ESP_LOGW(TAG, "CSI delivery watchdog: %lu frames captured but no "
                                   "batch dispatched in %ds, re-arming capture (strike 1)",
                             (unsigned long)(cap_now - dlv_wd_captured),
                             CSI_DELIVERY_WATCHDOG_S);
                    restart_capture();
                    dlv_wd_captured = cap_now;
                    dlv_wd_since = now;
                } else {
                    ESP_LOGE(TAG, "CSI delivery watchdog: still no batch dispatched "
                                   "%ds after re-arm while capture advances -- delivery "
                                   "path wedged, rebooting to recover",
                             CSI_DELIVERY_WATCHDOG_S);
                    vTaskDelay(pdMS_TO_TICKS(100));  // let the log line flush
                    esp_restart();
                }
            }
        } else {
            dlv_wd_since = now;
            dlv_wd_strikes = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =============================================================================
// DIAGNOSTIC FUNCTIONS
// =============================================================================
// Get chip name string for logging
static const char* get_chip_name(void) {
    esp_chip_info_t info;
    esp_chip_info(&info);
    switch (info.model) {
        case CHIP_ESP32:    return "ESP32";
        case CHIP_ESP32S2:  return "ESP32-S2";
        case CHIP_ESP32C3:  return "ESP32-C3";
        case CHIP_ESP32S3:  return "ESP32-S3";
        case CHIP_ESP32C2:  return "ESP32-C2";
        case CHIP_ESP32C6:  return "ESP32-C6";
        case CHIP_ESP32H2:  return "ESP32-H2";
        case CHIP_ESP32C5:  return "ESP32-C5";
        default:            return "Unknown";
    }
}

static void print_startup_config(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WiFi Tomography Node v%d.%d.%d-PURE",
             FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    ESP_LOGI(TAG, "  %s (%d core%s) - Batch CSI Protocol",
             get_chip_name(), chip_info.cores, chip_info.cores > 1 ? "s" : "");
    ESP_LOGI(TAG, "  Built: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  WiFi SSID:     %s", CONFIG_WIFI_SSID);
    ESP_LOGI(TAG, "  WiFi Channel:  %d %s", CONFIG_WIFI_CHANNEL,
             CONFIG_WIFI_CHANNEL > 0 ? "(forced)" : "(auto)");
    ESP_LOGI(TAG, "  Server IP:     %s", CONFIG_SERVER_IP);
    ESP_LOGI(TAG, "  Server Port:   %d", CONFIG_SERVER_PORT);
    ESP_LOGI(TAG, "  Trigger Port:  %d", CONFIG_TRIGGER_PORT);
    ESP_LOGI(TAG, "  Burst Port:    %d", CONFIG_BURST_PORT);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "CSI Settings:");
    ESP_LOGI(TAG, "  Burst count:   %d packets", CONFIG_BURST_COUNT);
    ESP_LOGI(TAG, "  Burst delay:   %d us", CONFIG_BURST_DELAY_US);
    ESP_LOGI(TAG, "  Protocol:      Batch (CRC32 validated)");
    ESP_LOGI(TAG, "========================================");
}

static void test_server_connectivity(void)
{
    ESP_LOGI(TAG, "Testing server connectivity...");

    // Create a simple test socket
    int test_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (test_sock < 0) {
        ESP_LOGE(TAG, "  FAIL: Cannot create test socket (errno %d)", errno);
        return;
    }

    // Set short timeout
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(test_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // Binary test packet (2 bytes: magic + type)
    uint8_t test_pkt[2] = { BINARY_PROTO_MAGIC, MSG_TYPE_TEST };
    int sent = sendto(test_sock, test_pkt, sizeof(test_pkt), 0,
                      (struct sockaddr *)&server_addr, sizeof(server_addr));

    // Log the actual IP being used (may differ from CONFIG_SERVER_IP if mDNS resolved)
    char actual_ip[16];
    inet_ntop(AF_INET, &server_addr.sin_addr, actual_ip, sizeof(actual_ip));
    if (sent > 0) {
        ESP_LOGI(TAG, "  OK: Sent %d bytes to %s:%d", sent,
                 actual_ip, CONFIG_SERVER_PORT);
    } else {
        ESP_LOGE(TAG, "  FAIL: sendto failed (errno %d)", errno);
    }

    close(test_sock);
}

// =============================================================================
// MAIN APPLICATION
// =============================================================================
void app_main(void)
{
    print_startup_config();

    boot_time_us = esp_timer_get_time();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erase required");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create CSI ring buffer
    csi_ringbuf = xRingbufferCreate(CSI_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (!csi_ringbuf) {
        ESP_LOGE(TAG, "Failed to create CSI ring buffer");
        return;
    }

#ifdef CONFIG_ENABLE_BFI_CAPTURE
    bfi_ringbuf = xRingbufferCreate(BFI_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (!bfi_ringbuf) {
        ESP_LOGE(TAG, "Failed to create BFI ring buffer");
        return;
    }
#endif

    // Create socket mutex for thread-safe UDP sends
    socket_mutex = xSemaphoreCreateMutex();
    if (!socket_mutex) {
        ESP_LOGE(TAG, "Failed to create socket mutex");
        return;
    }

    // Initialize on-die temperature sensor (non-fatal: heartbeat reports
    // "unavailable" via DIE_TEMP_READ_FAILED_X10 if this fails)
    init_temp_sensor();

    // Initialize WiFi with CSI
    if (wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed");
        return;
    }

    // Discover server via mDNS (falls back to Kconfig SERVER_IP on failure)
#ifdef CONFIG_MDNS_DISCOVERY
    resolve_server_via_mdns();
#endif

    // Initialize sockets
    if (init_sockets() != ESP_OK) {
        ESP_LOGE(TAG, "Socket initialization failed");
        return;
    }

    // Test server connectivity
    test_server_connectivity();

    // Initialize batch mode
    if (init_batch_mode_v5() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize batch mode");
        return;
    }
    ESP_LOGI(TAG, "Batch CSI mode enabled");

    // Initialize task watchdog for reliability
    // This will automatically reboot if tasks hang for >30 seconds
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_S * 1000,
        .idle_core_mask = 0,  // Don't watch idle tasks
        .trigger_panic = true,  // Reboot on timeout
    };
    esp_task_wdt_reconfigure(&wdt_config);
    ESP_LOGI(TAG, "Task watchdog configured: %d second timeout", WATCHDOG_TIMEOUT_S);

    // Start tasks with appropriate priorities and stack sizes
    // On multi-core chips (S3, ESP32), pin tasks to core 1 to leave core 0 for WiFi/CSI
    if (is_multicore()) {
        ESP_LOGI(TAG, "Multi-core detected: pinning tasks to core 1");
        xTaskCreatePinnedToCore(trigger_task, "trigger", 4096, NULL, 5, NULL, 1);
        xTaskCreatePinnedToCore(heartbeat_task, "heartbeat", 2048, NULL, 3, NULL, 1);
    } else {
        xTaskCreate(trigger_task, "trigger", 4096, NULL, 5, NULL);
        xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 3, NULL);
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  NODE READY");
    ESP_LOGI(TAG, "  MAC: %s", node_mac);
    ESP_LOGI(TAG, "  IP:  %s", node_ip);
    ESP_LOGI(TAG, "  Heap: %" PRIu32 " bytes free", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Sending initial HELLO messages...");
}
