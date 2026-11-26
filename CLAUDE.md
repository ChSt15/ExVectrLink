# ExVectrLink

Radio link firmware (PlatformIO, Arduino) for ESP32 / ESP32-S3 / ESP8285 boards with one or two SX1280 2.4 GHz radios. One code base builds both ends of the link:
- **TX side** (handset end): the TX module in a handset's external module bay (CRSF from EdgeTX, for example a Radiomaster Pocket).
- **RX side** (vehicle end): receivers that output CRSF to a flight controller.

The link library itself is the `lib/ExVectrLink` submodule. Packet formats are in `lib/ExVectrPackets`.

**ExVectrLink is shared between the user's FPV drone projects and the ExVectrStarship system.** Every change must keep FPV use working: lowest RC latency, maximum range, CRSF output to standard flight controllers. Starship-specific behaviour is selected at runtime (planned: link modes in the Lua menu, stored in EEPROM), never hard-coded into the default behaviour. See [docs/roadmap.md](docs/roadmap.md).

Part of the ExVectrStarship workspace (`../CLAUDE.md`). This file and `docs/` are enough to work on ExVectrLink on its own.

**Status:** network mode exists as separate firmwares (see [docs/network-mode.md](docs/network-mode.md)): `exvectrlink_tx_module_network` (node 2 router: USB, CRSF, FHSS air link) and `exvectrlink_rx_network_esp8285` / `_genericdiversity` (node 3 router: UART to the vehicle, FHSS air link), on the FHSS with air profile v1 (`feature/air-link-fhss`). FHSS parameters are runtime settings whose defaults are the FPV values; FPV is bit-for-bit unchanged (golden-trace test). Ground-station traffic over radio: only its heartbeat is dropped, REQUESTs pass; `feature/cmd-uplink` (GS-as-control-source): the Router forwards CMD_STATE (port 0x13, node 0 only, uplink only) over the air and the handset's RC_STATE no longer goes on the air at all. The link upload command (REQUEST action 10, target = module id) is always honoured by both modules (`feature/gs-requests-and-upload`). Open: the runtime mode switch on the TX module (Lua "Mode") and the senders of the upload command.

**Read [docs/README.md](docs/README.md) first.** It indexes how the firmware works: air protocol, CRSF, targets, known issues. Don't re-derive these from scratch.

## Rules

- **Branching** ([docs/development-workflow.md](docs/development-workflow.md)):
  - `master` holds the released state.
  - `Development` is for integration.
  - Work happens on `feature/<name>` branches cut from `Development`. Merge the latest `Development` in and re-test before merging back.
  - Never commit directly to `master` or `Development`.
- **Second working copy:** the user also keeps a copy at `~/Development/PlatformIO/ExVectrLink`. The two copies sync only through GitHub. Never edit that copy from here, and don't assume it is at the same commit.
- **Submodules** (`lib/ExVectr*`, `lib/SBUS`):
  - Library changes are committed inside the submodule, on a branch with the same name as the parent's feature branch.
  - Never leave a submodule dirty.
  - Never pin an unpushed submodule commit on `Development` or `master`.
  - `lib/SBUS` is pinned to a commit that may only exist locally (fetched by hand). Leave it alone.
- **Compatibility:**
  - Keep the air protocol backward compatible unless a break is agreed. The host serial protocol was removed (see [docs/serial-protocol.md](docs/serial-protocol.md)).
  - An air-protocol break bumps `FHSS::OTA_VERSION` (`lib/ExVectrLink/include/ExVectrLink/datalink/FHSS.hpp:184`).
  - Both ends must then be flashed together.
- **Lossy link:** safety-relevant information is sent as continuously repeated state, never as a one-shot event (`../docs/safety.md` section 2).
- **Docs are part of the change:** update the affected file in `docs/` in the same branch, including [docs/known-issues.md](docs/known-issues.md) when you find or fix a bug. Update `Last verified against commit:` when you re-verify a doc.

## Build

The PlatformIO binary on this machine is `~/.platformio/penv/bin/pio`. Run it from this folder:

```sh
~/.platformio/penv/bin/pio run -e exvectrlink_tx_module_diversity      # TX module (Pocket), 2 radios
~/.platformio/penv/bin/pio run -e exvectrlink_tx_module                # TX module, 1 radio
~/.platformio/penv/bin/pio run -e exvectrlink_tx_module_network        # TX module, ExVectrNetwork mode (separate firmware, no FPV code)
~/.platformio/penv/bin/pio run -e exvectrlink_rx_network_esp8285       # RX, ESP8285, ExVectrNetwork mode (UART 460800 to the vehicle)
~/.platformio/penv/bin/pio test -e native                              # host tests of src_network/
~/.platformio/penv/bin/pio test -e native_fhss                         # host tests of the real FHSS, incl. the FPV golden trace
~/.platformio/penv/bin/pio run -e exvectrlink_genericdiversity_lr_ota  # RX, ESP32 + 2 radios, CRSF out
~/.platformio/penv/bin/pio run -e exvectrlink_generic_esp8285_ota      # RX, ESP8285, CRSF out
```

- Every env is described in [docs/build-targets-and-hardware.md](docs/build-targets-and-hardware.md).
- **The Starship ship node board (ESP8285) has an external PA and LNA: always flash `exvectrlink_rx_network_esp8285_pa` (RX/TX enable pins 9 and 10), never the plain `exvectrlink_rx_network_esp8285`.** The plain env does not switch the PA/LNA: the ship's downlink then loses 40 to 70 % of its packets while the uplink looks fine (found 2026-11-26 after a night of wasted link experiments). Any experiment env for the ship needs a `_pa` variant. Ship OTA address: 192.168.178.57 (WiFi, via the `LINK_UPLOAD_MODE` request, target 3).
- Build the ESP32 TX env BEFORE the ESP8285 env: building an ESP8285 env replaces the `esptool` entry point in the shared PlatformIO penv (a wrapper `~/.platformio/penv/bin/esptool` calling `tool-esptoolpy` restores it).
- The network envs run air profile v2 (SF6, 220 B); `-DEXVECTRLINK_NET_PROFILE_V1` builds the v1 rollback ([docs/air-link-v2.md](docs/air-link-v2.md)).
- Build at least one RX and one TX env for every change. Check that there are no new warnings in `src/`.
- **`src/WifiCredentials.cpp` is gitignored and must exist.** It defines `const char *VCTR::WifiUpload::ssid` and `password`. Without it the link step fails with `undefined reference to VCTR::WifiUpload::ssid`.
- The `*_ota` envs upload over WiFi to a fixed IP (`upload_port`). Use `--upload-port` to override it.
- On this machine PlatformIO may stall on network checks (broken IPv6).
- Range, latency, throughput and sync can only be verified on hardware. Describe the bench test the user should run.

**Pin status (2026-11-23, `feature/cmd-uplink`):** ExVectrPackets moved to 49d89c6 (Development, pushed; schema hash 0xF1941A87; adds CMD_STATE 0x13, REQUEST.session, CMD_APPLIED, VEHICLE_STATUS.cmd_session). The routers need `Packets::CmdState::kPort` only. Earlier (2026-11-22, `feature/schema-t9b0`): ExVectrPackets 7b7285d (Development, pushed; schema hash 0x65E4E6E3; adds RequestAction.ZERO_ESTIMATORS, VEHICLE_STATUS T9b fields, RC_APPLIED 0x84). Earlier pin 29db156 (schema hash 0x414DE27A) appended ACTUATORS.roll_torque, MISSION.mission_mode/waypoint, VEHICLE_STATUS.calib_state, AIR_GNSS 0x24 and AIR_GNSS_INFO 0x25, AIR_POSE at 25 Hz. ExVectrNetwork stays at de708ca. No code change was needed: the hash comes from `Packets::kSchemaHash`, and the Router and ship node filter only by `portLimit = 0x80` plus explicit drop rules, so the new ports pass without a table entry. The air set is about 2.99 kB/s of the 6.15 kB/s downlink (see ExVectrPackets `docs/telemetry-sets.md`). Router and ship node built, `pio test -e native` run.

**Previous pin (2026-11-21, `feature/setpoint-attitude-pin`):** ExVectrPackets moved to 18b6884 (branch `feature/setpoint-attitude`, schema hash 0xBFF82392; appends `setpoint_q` to MISSION and AIR_MISSION). That commit exists only in the local ExVectrPackets repo and is not pushed: it must be pushed before anyone else can clone this pin. Router (`exvectrlink_tx_module_network`) and ship node (`exvectrlink_rx_network_esp8285_pa`) rebuilt, `pio test -e native` 85/85.
