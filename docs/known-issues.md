# Known Issues

> Real problems found while reading the code for these docs. None of the entries in the table is fixed yet. Remove an entry when it is fixed, and add new ones as you find them.

Last verified against commit: 4830874 (submodules: ExVectrLink 37ab8e2, ExVectrPackets 46286bd, ExVectrNetwork 7cb0e79)

Severity: **High** = can lose the link or corrupt data in normal use, or blocks the Starship integration. **Medium** = wrong behaviour in some situations or misleading interfaces. **Low** = cosmetic, dead code or hygiene.

| Severity | Location | Issue | Impact |
|---|---|---|---|
| High | `src/main.cpp:258-293` | The sync failsafe enters full WiFi upload mode (link stopped) if FHSS doesn't sync within 60 s of boot. This applies on both sides, and in all builds. | Powering the vehicle 60 s before the TX module, or vice versa, leaves that end permanently linkless until power-cycled. Worth a pre-flight checklist item, or making it opt-in. |
| High | `src/DatalinkCommandManager.cpp:675-686`, `src/CRSFRcInput.cpp:503-567` | Lua commands "RX Upload Mode", "RX Block Test" (blocks the receiver for 1514 ms) and "RX Force Search" act on the receiver over the air. They have no confirmation step and no check of arm or flight state, and are carried by one-shot frames. | One menu click can stop or stall the vehicle's link in flight. That conflicts with safety.md (no accidental in-flight disruption). For FPV it is equally dangerous. |
| High | `src/DatalinkCommandManager.cpp:523`, `src/CRSFRcInput.cpp:1057`, `src/CRSFRcOutput.cpp:124` vs `lib/ExVectrLink/include/ExVectrLink/datalink/PacketTypes.hpp:37-88` | OTA type bytes are written and compared as raw values. The odd-parity encoding the header requires is not used in `src/`. | A single bit flip that survives the LoRa CRC and the trailer XOR can turn `Data` (0) into `UploadMode` (4) and drop the receiver into WiFi mode. The header documentation is misleading. |
| Medium | `.gitignore`, `src/WifiUpload.cpp:31-33` | `src/WifiCredentials.cpp` is required (it defines `ssid`/`password`) but gitignored, with no template. | A fresh clone fails at link with `undefined reference to VCTR::WifiUpload::ssid` for every env. |
| Medium | `lib/ExVectrLink/include/ExVectrLink/datalink/FHSS.hpp:226`, air protocol | There is no bind. All units use hop key 0 and the same checksum seed. | Any ExVectrLink TX links with any RX in range. Two systems flying together interfere and can control each other's receivers. |
| Medium | `lib/ExVectrLink/src/datalink/FHSS.cpp:47-61`, `279-282` | The trailer carries only 2 bits of `slotCounter`, but `setSlotsPerHop()` allows up to 16. `setRxSlotIndex()` caps at 4, while the default is 15. | Changing these parameters (needed for link modes) breaks counter sync unless the trailer is widened. |
| Medium | `src/DynamicPowerSetup.cpp:63-67` | Per-slot `incPower()` has no rate limit, while decreases are limited to 1/s. `maxRssi = -110` is below `minRssi = -100` (`DynamicPower.hpp:86-92`). | Power ratchets up quickly on short fades and comes down slowly. The decrease condition is effectively "RSSI ≥ -100". |
| Medium | `src/CRSF.cpp:206-213` | Half-duplex keeps one pending reply, which the 10 Hz link-stats frame and telemetry frames overwrite. | Lua menu replies are sometimes lost (the menu retries). Telemetry to the handset is throttled to the CRSF frame rate. |
| Medium | `lib/ExVectrPackets/include/ExVectrPackets/Packets.hpp:118-120` | The `PacketManager` size check is commented out. Every handler deserializes every payload. | Short payloads are over-read past the end of the buffer. A mismatched payload can decode as another type if the header bytes match. |
| Medium | `src/main.cpp:94-221` | The `ENABLE_SBUS` path decodes the old `RadioLinkPacket_A4D4`, not `RcLinkPacket`. | SBUS output would never see RC data. No env enables it. |
| Low | `platformio.ini` `[env:esp32-pico]` | Named for ESP32 but uses `esp8285`, has no `TARGET_*` and no `-std=gnu++2a`. | Doesn't compile (`'concept' does not name a type`). |
| Low | `lib/ExVectrLink/include/ExVectrLink/datalink/FHSS.hpp:186`, `src/DatalinkCommandManager.hpp:53-55`, `src/CRSFRcInput.hpp:114-117` | Stale comments: the trailer layout, and "blocks for 500 ms" (actually 1514 ms). | Misleading for maintainers. |
| Low | `src/DatalinkCommandManager.cpp` (`getRemote*RateHz`) | `getRemoteCrsfTxFailureRateHz()`/`getRemoteRcPacketRateHz()` always return 0. `RxState`, despite its name, is sent by both sides. | The debug print on the TX module shows remote rate fields that are always 0. |
| Low | `src/CRSFInput.*`, `src/CRSFOutput.*`, `lib/CRSFforArduino/`, `firmware.bin` | Dead legacy code and a vendored library that are compiled or committed but not used. A 1 MB build artefact is committed. | Build time, repo size, and confusion about which class is live. |
| Low | `lib/SBUS` | Pinned to 3dcd5b4 (`v8.1.4-3-g3dcd5b4`), which may only exist in a local fetch. | A fresh clone may fail `git submodule update` for `lib/SBUS`. |
| Low | `src/main.cpp:529`, `536`, `src/WifiUpload.cpp:138`, `204` | Existing warnings: `++` on `volatile` (deprecated in C++20), signed/unsigned compare. | Noise when checking for new warnings. |
| Low | `src/CRSFRcInput.cpp:665-682` | The temperature Lua entry has no name set. | The handset shows an empty or default label. |

## Resolved by removal

The serial host protocol (`SerialTelecoms`, `SerialTelecomsDatalink`, `LinkManager`, `ExVectrLinkI`, the `exvectrlink_controller_*` envs) was removed on branch `feature/remove-serial` because nothing uses it (see [serial-protocol.md](serial-protocol.md)). These entries no longer apply:

- Serial frame ended at the first `0x7F` data byte, with no `len` use, escaping or checksum.
- 100-byte serial send buffer with a 1024-byte overflow guard.
- Serial `PacketData` bypassed the OTA queue and silently dropped payloads over 8 bytes.
- "Serial host lost" turned WiFi upload mode on next to the running link after 10 s of uptime. The rule was compiled into every build without `ENABLE_SBUS`/`ENABLE_CRSF`, which includes the TX module (`ENABLE_CRSF_INPUT`), where the host-less serial port made it true. The TX module therefore no longer starts WiFi 10 s after boot on its own (to be confirmed on hardware, see the branch report).
- No env built a receiver in the plain `SerialTelecoms` role.
- `SerialTelecomsDatalink` needed a newer `DatalinkI`/`DataPacket` API than the flight software pins.
- Baud negotiation could strand the host (1 Mbaud open vs 115200 assumed).
- `SetModulationPreset` had no bounds check, `SetLinkChannel` was a no-op, `StartFHSS.enable` was ignored and truncated the key, `InitLink` had no handler.
- On ELRS-style boards a plain-serial build would have put two UARTs on GPIO 3/1.
- The umbrella header `ExVectrLink.hpp` included a nonexistent `ExVectrLink/SerialTelecoms.hpp`. It now includes the datalink headers.

Dead code left behind by the removal (not removed, because it is outside the serial path): `getCrsfRcPacketRateHz()`/`getCrsfTxFailureRateHz()` in `CRSFRcOutput` have no caller any more, and the `PacketTypes` parity helpers and `ModulationPresets` table are unused by `src/`.
