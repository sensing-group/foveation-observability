# Node firmware

The firmware that ran on the article's mesh nodes (v5.9.0), for ESP32-C5 and ESP32-C3, built with
ESP-IDF 5.4.1. One source file, `main/main.c`.

A node joins the Wi-Fi network, announces itself to the host with HELLO, and sends a HEARTBEAT
every 2 s. When the host broadcasts a TRIGGER naming this node, it transmits a burst. When a TRIGGER
names another node, it captures CSI from that node's burst and returns a batch tagged with the
trigger's slot. With BFI capture on (the default), it also overhears the compressed beamforming
reports that client stations send to the access point and forwards them unchanged; this is the
wide tier of the article. The wire format is in `../controller/protocol.py`.

## Build

```
docker run --rm -v $PWD:/project -w /project espressif/idf:v5.4.1 sh -c \
  "idf.py --preview set-target esp32c5 && idf.py build"
```

For the C3, use `idf.py set-target esp32c3` (no `--preview`). Without Docker, install ESP-IDF
v5.4.1 and run the same two commands. The only managed dependency, `espressif/mdns`, is fetched
by the component manager on the first build.

## Configure and flash

```
idf.py menuconfig          # WiFi Tomography Config: SSID, password, channel
idf.py -p /dev/ttyACM0 flash monitor
```

| Setting | Default | Meaning |
|---|---|---|
| `WIFI_SSID`, `WIFI_PASSWORD` | placeholders | the network the nodes join |
| `WIFI_CHANNEL` | 11 | the channel scanned first |
| `MDNS_DISCOVERY` | on | find the host as `_csi._udp.local` at boot |
| `SERVER_IP` | 0.0.0.0 | fallback host address when mDNS finds nothing |
| `SERVER_PORT`, `TRIGGER_PORT` | 5000, 5001 | host port for node reports, node port for triggers |
| `BURST_COUNT`, `BURST_DELAY_US` | 10, 200 | frames per transmit burst and their spacing |
| `ENABLE_BFI_CAPTURE` | on | forward overheard beamforming reports |

A node also adopts the source address of any valid DISCOVER or TRIGGER as its host, so a host
running `../controller/node_controller.py` on the same network is found without mDNS or a fixed
address.

ESP-IDF 5.4.1 lists the ESP32-C5 as a preview target, which is why its `set-target` needs
`--preview`.
