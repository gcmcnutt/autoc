# Pi install log — beacon receiver rig (Pi 3B → 3A+, same SD card)

Reproducible record of everything done to the card. Order matters only where noted.

## Base image
- Raspberry Pi OS **Bookworm Lite 32-bit** (Imager's "Legacy" slot delivered Bookworm, not Bullseye,
  2026-08-13). Kernel at time of writing: 6.12.93+rpt-rpi-v7.
- Headless setup via boot-partition files: `ssh` (empty), `userconf.txt` (user `pi`), Wi-Fi via
  `raspi-config nonint do_wifi_country US` + `do_wifi_ssid_passphrase` (the wpa_supplicant.conf
  drop-in did NOT take — rfkill-blocked until the country was set).
- `raspi-config nonint do_configure_keyboard us`, `do_change_timezone America/Los_Angeles`.

## /boot/firmware/config.txt (Bookworm path — /boot/config.txt is a decoy)
```
camera_auto_detect=0
dtoverlay=vc4-kms-v3d,cma-320     # KMS display REQUIRED for rpicam preview; cma-320 raises the fps ceiling
dtoverlay=ov9281
```
(Removing the vc4 line to add ov9281 kills the display — both must be present.)

## Packages
| date | package | why |
|---|---|---|
| 2026-08-13 | (rpicam-apps — present in image) | `rpicam-hello/-still/-raw/-vid` |
| 2026-08-13 | tailscale (`curl -fsSL https://tailscale.com/install.sh \| sh`; `tailscale up`) | reach the Pi from the DGX past Wi-Fi client isolation; node `raspberrypi` = 100.110.13.80 |
| 2026-08-16 | `python3-numpy` | frame analysis on-Pi |
| 2026-08-16 | `python3-picamera2` | Python capture API for the decoder prototype (A8-6) |

## Access
- DGX pubkey (`~/.ssh/id_ed25519.pub` on promaxgb10) appended to `pi:~/.ssh/authorized_keys` 2026-08-13.
- Camera FFC: standard 15-pin cable (InnoMaker shipped it + a Zero-style adapter).

## Not done / next
- InnoMaker Bullseye vendor driver (453 fps mode) → **second SD card**, don't disturb this one.
- The 3B is unplugged; this card runs the 3A+ under the same Tailscale identity.

## Card 2 — Trixie arm64 (2026-08-16/17; the high-fps + tracker card, node `beaconpi` 100.87.61.53)
- Image: Raspberry Pi OS **Trixie Lite arm64** (2026-06-18 build), flashed from the DGX (`dd`), pre-seeded
  headless via bootfs `ssh` + `userconf.txt` + cloud-init `user-data`/`network-config`/`meta-data`.
  Lessons: cloud-init's netplan Wi-Fi did NOT come up → NM profile written directly to rootfs
  (`/etc/NetworkManager/system-connections/lyu-guest.nmconnection`, must be 0600); the Pi's clock at
  first boot predates TLS certs → **wait for NTP before any HTTPS** (Tailscale install failed "cert not
  yet valid" until fixed); Tailscale static arm64 binaries pre-placed from the DGX, enrolled with a
  reusable auth key in `runcmd` (key scrubbed after; REVOKE it in the console).
- config.txt: `camera_auto_detect=0`, `dtoverlay=vc4-kms-v3d,cma-320`, `dtoverlay=ov9281`.
- Kernel 6.18.34+rpt-rpi-v8; **experimental `ov9282.ko` installed** (`pi/ov9282-experimental.c`, built
  on-Pi against `linux-headers-rpi-v8`; stock kept as `ov9282.ko.xz.stock`). Modes added: 640x200 (works,
  ~280 fps ceiling), 320x200 (does not stream), 640x392 (does not stream).
- Packages: `python3-numpy` (2026-08-17, for `beacon_track.py`), `i2c-tools`, `build-essential`.
- Scripts in `~pi/`: `fps_probe.py`, `beacon_track.py`; build dir `~pi/ov9282-build/`.
