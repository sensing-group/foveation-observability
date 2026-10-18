"""Presence hold and release by accumulated absence, as the deployed watcher ran it on
4 August 2026 (sensing repository commit 1cf4d069, scripts/watcher/filter.py).

The hold is a bistable ledger: once an entry is registered the room stays occupied until a
release path fires. This module carries the release path the case study used: an absence
score in nats. A bin adds to the score only when at least ABSENCE_MIN_LIVE_CHANNELS channels
report and every one of them is quiet, and only with evidence stillness cannot counterfeit (a
recent traffic dropout, or a probe that came back negative). A bin with movement evidence
subtracts a penalty scaled by the signed CSI statistic; the score never goes below zero. The
room is released when the score reaches ABSENCE_RELEASE_THRESHOLD_NATS.

Entry detection and the other release paths (exit signature at the door, half-life, the BFI
likelihood release) are not part of this extract; on the case-study evening none of them fired.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

BIN_S = 10.0
ABSENCE_QUIET_Z = 2.5
ABSENCE_NATS_PER_S = 0.005
ABSENCE_PROBE_BONUS_NATS_PER_S = 0.010
ABSENCE_RELEASE_THRESHOLD_NATS = 6.0
ABSENCE_POSTPROBE_CREDIT_WINDOW_BINS = 30
ABSENCE_ACTIVE_PENALTY_MODERATE_NATS = 1.85
ABSENCE_ACTIVE_STRONG_Z = 6.39
ABSENCE_ACTIVE_PENALTY_STRONG_NATS = 4.9
ABSENCE_DENSITY_FLOOR = 9.0
ABSENCE_DENSITY_DIP_WINDOW_BINS = 120
ABSENCE_MIN_LIVE_CHANNELS = 3
BFI_MIN_REPORTING_NODES = 2


def absence_evidence_nats(z_roll: Optional[float], z_mesh: Optional[float],
                          bfi_z: Optional[float], bfi_n_reporting: Optional[int],
                          illuminated: bool = False,
                          saw_density_dip: Optional[bool] = None,
                          probe_negative_recent: bool = False) -> Tuple[float, str]:
    """Evidence for absence from one bin, in nats, with the reason (deployed code, verbatim).

    A bin counts only when enough channels are reporting and all of them are quiet: the system
    asserts it looked and saw nothing, never that it failed to look.
    """
    live = 0
    noisy = []

    if z_roll is not None:
        live += 1
        if abs(z_roll) > ABSENCE_QUIET_Z:
            noisy.append("csi")
    if z_mesh is not None:
        live += 1
        if abs(z_mesh) > ABSENCE_QUIET_Z:
            noisy.append("mesh")
    # BFI counts as live only with a real reporting basis behind it.
    if bfi_z is not None and (bfi_n_reporting or 0) >= BFI_MIN_REPORTING_NODES:
        live += 1
        if abs(bfi_z) > ABSENCE_QUIET_Z:
            noisy.append("bfi")

    if live < ABSENCE_MIN_LIVE_CHANNELS:
        return 0.0, f"too_few_live_channels({live})"
    if noisy:
        return 0.0, "active:" + "+".join(noisy)

    quiet_credit = ABSENCE_NATS_PER_S * BIN_S

    # Quiet obtained under an active probe answers the dropout's question with a stronger
    # experiment, so it bypasses the traffic gate and earns a bonus.
    if illuminated:
        return (quiet_credit + ABSENCE_PROBE_BONUS_NATS_PER_S * BIN_S,
                f"quiet_{live}ch_probed")
    if probe_negative_recent and not saw_density_dip:
        return quiet_credit, f"quiet_{live}ch_postprobe"

    # Traffic dropout required: a motionless occupant silences every RF channel together, but
    # an occupied home holds a device-traffic floor that an empty one does not. None means the
    # caller did not supply density; an unsupplied discriminator is not a satisfied one.
    if not saw_density_dip:
        return 0.0, ("no_traffic_dropout" if saw_density_dip is not None
                     else "density_unavailable")
    return quiet_credit, f"quiet_{live}ch"


@dataclass
class Bin:
    ts: float
    z_roll: Optional[float]
    z_mesh: Optional[float]
    bfi_agitation_z: Optional[float]
    bfi_n_reporting: Optional[int]
    sounding_density: Optional[float]
    illuminated: bool = False


class AbsenceLedger:
    """The present-state branch of the deployed ledger, restricted to the absence release."""

    def __init__(self, bin_s: float = BIN_S):
        self.bin_s = bin_s
        self.present = False
        self.s_absence = 0.0
        self._last_density_dip_bin: Optional[int] = None
        self._last_probe_negative_bin: Optional[int] = None

    def enter(self) -> None:
        self.present = True
        self.s_absence = 0.0

    def step(self, obs: Bin) -> Tuple[Optional[str], str]:
        """Advance one bin; returns (event, reason). event is "absence_confirmed" on release."""
        why = ""
        event = None
        if self.present:
            bin_idx = int(obs.ts // self.bin_s)
            if obs.sounding_density is not None and obs.sounding_density < ABSENCE_DENSITY_FLOOR:
                self._last_density_dip_bin = bin_idx
            saw_dip = None
            if obs.sounding_density is not None:
                saw_dip = (self._last_density_dip_bin is not None
                           and (bin_idx - self._last_density_dip_bin)
                           <= ABSENCE_DENSITY_DIP_WINDOW_BINS)
            probe_neg_recent = (self._last_probe_negative_bin is not None
                                and (bin_idx - self._last_probe_negative_bin)
                                <= ABSENCE_POSTPROBE_CREDIT_WINDOW_BINS)
            nats, why = absence_evidence_nats(obs.z_roll, obs.z_mesh, obs.bfi_agitation_z,
                                              obs.bfi_n_reporting, illuminated=obs.illuminated,
                                              saw_density_dip=saw_dip,
                                              probe_negative_recent=probe_neg_recent)
            if why.endswith("_probed"):
                self._last_probe_negative_bin = bin_idx
            if nats > 0.0:
                self.s_absence += nats
            elif why.startswith("active:"):
                z_signed = obs.z_roll if obs.z_roll is not None else 0.0
                if z_signed >= ABSENCE_ACTIVE_STRONG_Z:
                    penalty = ABSENCE_ACTIVE_PENALTY_STRONG_NATS
                elif z_signed >= ABSENCE_QUIET_Z:
                    penalty = ABSENCE_ACTIVE_PENALTY_MODERATE_NATS
                elif ("bfi" in why and obs.bfi_agitation_z is not None
                      and obs.bfi_agitation_z > 0):
                    penalty = ABSENCE_ACTIVE_PENALTY_MODERATE_NATS
                else:
                    penalty = 0.0
                self.s_absence = max(0.0, self.s_absence - penalty)
            if self.s_absence >= ABSENCE_RELEASE_THRESHOLD_NATS:
                self.present = False
                event = "absence_confirmed"
                self.s_absence = 0.0
        if not self.present:
            self.s_absence = 0.0
        return event, why
