# Serial Protocol (removed)

> The ExVectrLink host-to-module serial protocol (`SerialTelecoms`) no longer exists in this code base. This page is kept only so old links and references resolve.

Removed on branch `feature/remove-serial` (decided by the user: nothing uses it any more).

## What was removed

- **Library (`lib/ExVectrLink`):** `SerialTelecoms`, `SerialTelecomPackets` (all `SerialPacket_*` types), `SerialTelecomsDatalink`, the `ExVectrLinkVersion` constant, and the host-side `LinkManager`, `ExVectrLinkI` and `LinkInfo` struct that only existed to drive a module through that protocol.
- **Firmware (`src/`):**
  - all serial-host handlers in `DatalinkCommandManager.cpp` (`PacketData`, `SetModulationPreset`, `SetLinkChannel`, `StartFHSS`, `SetPowerParams`, serial `LinkInfo`, `Print`);
  - the `TempMonitor`, `SyncedMonitor` and `ErrorSender` tasks and the 1 Mbaud host UART in `main.cpp`;
  - the serial `UpdateMode` packet handler and the "serial host lost, start WiFi" rule in `WifiUpload.cpp`;
  - the "plain" build role (no `ENABLE_CRSF`/`ENABLE_CRSF_INPUT`/`ENABLE_SBUS`).
- **Build targets:** `exvectrlink_controller_usb`, `exvectrlink_controller_ota`, `TARGET_EXVECTRCONTROLLER` and `src/targets/ExVectrControllerLink.hpp`. They only served a serial host (the retired ExVectrController board).

## Why

The protocol had unfixed framing and buffering faults (a `0x7F` payload byte ended a frame, no checksum, a 100-byte send buffer), a serial-host-lost rule that could switch WiFi on next to a running link, and no env that used it as a vehicle receiver. Nothing uses it, so it was removed instead of repaired. These issues are recorded as resolved by removal in [known-issues.md](known-issues.md).

## What stays

CRSF in and out, the RC channel link, all over-the-air packet types (including `Data`), FHSS, diversity, dynamic power, settings/EEPROM, the Lua menu, link telemetry relay, and the WiFi/OTA upload mode with its remaining triggers (boot button, 5x `r` on `Serial`, held button, over-the-air `UploadMode`, Lua commands, 60 s sync failsafe). The air protocol (`OTA_VERSION`) is unchanged.

The `PacketTypes::Data` frame is still sent (RC channels, CRSF telemetry relay, decoded by `CRSFRcOutput`/`CRSFRcInput`). The command manager no longer forwards it anywhere.

## If a host link is needed again

The old design is in git history (`lib/ExVectrLink` before `feature/remove-serial`, and the previous version of this file). Do not restore it as it was. A new host interface should be specified from the requirements in [roadmap.md](roadmap.md), with safe framing, and with flow control and queueing that respect the RC channel priority.
