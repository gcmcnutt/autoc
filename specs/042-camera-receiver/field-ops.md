# Field ops — the hotspot link, VERIFIED by cold-boot rehearsal (2026-08-30)

Every number below is measured, not assumed. The rehearsal: ethernet pulled, power cycled cold, Pi
came back reachable over tailscale-through-hotspot in **51 s**, then ran the camera + tracker live over
that path (200 records, 0/200 deadline misses).

## The link, as it actually is

| | |
|---|---|
| Pi joins | `GregPhone` hotspot, autoconnect at boot (WPA2, 2.4 GHz; PSK stored in NetworkManager) |
| Pi address in the field | `pi@100.97.242.96` (tailscale) — the 10.42.0.153 wire address does not exist out there |
| path from home | **DERP relay via SFO** — the phone's CGNAT blocks direct; RTT ~364 ms, **~209 kB/s** |
| path from a laptop ON the hotspot | direct LAN (172.20.10.x) — full speed, no relay |

## What fits through 209 kB/s and what does not

- **YES**: `--emit json:-` (~5 kB/s), `ascii_scope` (10 Hz over ssh), truth/score CSVs, git pulls of source.
- **NO**: `live_view` in any configuration (minimum useful stream ~1 MB/s). Do not try; it will stall the link.
- **Clips never cross this link.** Record to NVMe (722 GB free), score ON the Pi (`pendulum_truth.py` /
  `pendulum_analyze.py` run there in ~40 s via `~/cv/bin/python`), pull only the JSON/CSV products.
  Raw `.bcnr` clips come home on the wire afterwards.

## Field session skeleton

```bash
# monitoring (either): 
ssh pi@100.97.242.96 'cd ~/autoc-beacon/firmware/beacon-receiver && \
  ../../build/firmware/beacon-receiver/beacon_trackd --config beacon-event.ini \
  --source live --emit json:- --record /data/field1.bcnr --record-mode continuous --duration 240' \
  | python3 firmware/beacon-receiver/tools/ascii_scope.py --source json:- --hz 5
# scoring, on the Pi:
ssh pi@100.97.242.96 'cd ~/autoc-beacon/firmware/beacon-receiver && \
  ~/cv/bin/python tools/pendulum_truth.py /data/field1.bcnr --arc --step 10'   # then --roi + --out
```

## Traps

- **The phone's hotspot can go dormant when its screen sleeps** (iPhone behaviour). Keep the hotspot
  settings screen open while the Pi is joining; once tailscale is up the keepalives hold it.
- The Pi's home wifi profile (`lyu-guest`) is autoconnect-DISABLED so it cannot fight the hotspot.
  Ethernet reconnects itself when the cable comes home — nothing to undo.
- `sync-to-pi.sh` works in the field via `PI=pi@100.97.242.96` but at 209 kB/s a full sync is minutes —
  push only deliberate changes.
