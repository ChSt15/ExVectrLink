# Roadmap / Planned Work

> User decisions about where ExVectrLink is going. **Nothing here is implemented yet.** These are design notes, recording which existing code each item would touch. Anything that turns out different during implementation must be agreed with the user first, and then recorded here.

Last verified against commit: 35fafb1 + feature/air-link-fhss (section 2 only; the rest is as of 4830874)

## Guiding principles

- **FPV keeps working by default.** The current behaviour (FHSS, RC channels, CRSF out, lowest latency, maximum range) stays the default link mode. Starship behaviour is selected at runtime and never hard-coded.
- **Lossy link principle:** safety-relevant information is carried as continuously repeated state, never as one-shot events (workspace `docs/safety.md` section 2). This applies to new link features too:
  - Mode changes, arm state and commands are repeated state.
  - One-shot frames, like today's `UploadMode`/`TestBlock`, are not used for anything that matters in flight.
- **The vehicle owns safety decisions.** The link relays and reports. It doesn't decide for the vehicle (safety.md section 1).

## 1. Link modes on three independent axes

Selectable in the handset Lua menu, stored in EEPROM, applied to **both** ends of the link.

| Axis | Options |
|---|---|
| **Protocol type** | **FPV:** radio-link RC channel packets for flight controllers, CRSF/SBUS out (today's behaviour). **Telemetry/telecommand:** ExVectrNetwork data frames both ways. The Starship uses it for commands and for full telemetry recorded on the ground. |
| **Link rate** | **Long range** · **High bandwidth** |
| **Hopping** | **FHSS:** today's behaviour. **Single band:** no hopping, one fixed channel, allowing tighter timing. **Dual band:** both radios of RX and TX used at the same time on different channels, doubling throughput. |

### Changing modes safely

- A mode change resyncs the link, so it must not be possible by accident in flight. For example, the TX module refuses a change while the vehicle reports armed.
  - That needs vehicle state in the downlink, as repeated state.
  - It also needs a confirmation step in the Lua command, which today has none (`src/CRSFRcInput.cpp:638-663`).
- Proposed handover:
  1. The TX side keeps announcing the target mode as repeated state for a while.
  2. The RX applies it at an agreed slot or hop boundary.
  3. On failure, both ends fall back to a known default mode, so a lost mode change never leaves the two ends permanently apart.
- How exactly the mode is agreed and fallen back from is still open.

### Code it would touch

- **Settings:** a new `LinkModeSettings` record with a fresh key in `src/Settings.hpp` (see settings.md). The RX also needs to store it, so it boots into the last agreed mode.
- **Lua menu:** new TEXT_SELECTION entries in `src/CRSFRcInput.hpp`/`.cpp`, with `kParametersTotal` bumped (`src/CRSFRcInput.hpp:249`).
- **FHSS** (`lib/ExVectrLink/src/datalink/FHSS.cpp`, `FHSS.hpp`):
  - The slot interval, slots per hop and role-reversal ratio become runtime parameters. Today they are fixed defaults (`FHSS.hpp:208-217`).
  - The trailer must carry more counter bits first (`FHSS.cpp:279-282`, and the known issue about 2-bit `slotCounter`).
  - "Single band" means no hops and a fixed channel. Sync currently relies on the RX hopping backwards while searching (`FHSS.cpp:636-641`), so that path needs a non-hopping variant.
  - Bump `OTA_VERSION` if the trailer changes.
- **Link rate:** make `ModulationPresets` (`lib/ExVectrLink/src/datalink/LinkSettings.cpp`) actually drive SF/BW/CR **and** the slot timing on both ends. Today they only change local radio parameters (`src/DatalinkCommandManager.cpp:528-540`). The fixed packet length in `src/main.cpp:669` and `FHSS::getMaxPacketSize()` (`FHSS.cpp:144`) would become per-rate values.
- **Dual band:** `Sx1280Diversity` (`lib/ExVectrLink/src/datalink/Sx1280Diversity.cpp`) today puts both radios on the same channel and picks one. Dual band would drive the radios independently, as two FHSS/link instances, or as a new combiner with two TX slots. It needs `HAS_DIVERSITY` hardware at both ends: the diversity TX module and a diversity RX.
- **Protocol type "telemetry/telecommand":**
  - Larger frames, fragmentation, and a downlink share well above today's 1 in 15 slots (`FHSS.hpp:217`).
  - A send queue for host data through `queueOtaPacket*()` in `src/DatalinkCommandManager.cpp` (the old direct `transmitDataframe()` path from the serial host was removed).
  - Flow control to the host.
  - A new, safely framed host interface. The old `SerialTelecoms` protocol was removed (see [serial-protocol.md](serial-protocol.md)) and is not to be restored as it was.
  - A vehicle RX env for that host interface (`platformio.ini`), with a UART that doesn't collide with `Serial`.
- **RC in Starship mode:** the Pocket's switch states must still reach the vehicle with every frame, as repeated state.

## 2. TX-module USB bridge

> **Status (feature/air-link-fhss): USB side and air link done, as separate firmwares.** The TX module firmware now runs the FHSS air link (air profile v1: SF5/BW1600/CR4/5, 255-byte frames, 20 ms slots, 1:1) and the vehicle receiver has its own network-mode firmware (`exvectrlink_rx_network_esp8285`, `_genericdiversity`: node 3 router, UART 460800 to the vehicle). FHSS slot interval, role ratio, frame length and link version are runtime parameters (`AirProfile`); FPV keeps its defaults bit for bit. Still open: the runtime FPV/network switch on the TX module (Lua "Mode", item 1, task T6c), the network upload command, a downlink-heavy slot ratio. Details: [network-mode.md](network-mode.md).
>
> Earlier status (feature/tx-network-router): USB side done, air side open. A separate firmware env `exvectrlink_tx_module_network` runs the ExVectrNetwork router (node 2) with the USB `StreamFrameLink`, the CRSF <-> `RC_STATE`/telemetry translation, the egress filters and an air stub. Details: [network-mode.md](network-mode.md). Still open: the FHSS air link behind the `AirAccessI` seam (including the decisions about WiFi upload, sync failsafe and test commands in that mode), the runtime FPV/network mode switch (Lua, reboot on change, item 1), and the vehicle-side receiver env. The text below is the original plan; where it says the host protocol must be designed, the design is now the ExVectrNetwork stream frame.

Forward ExVectrNetwork data frames both ways between the TX module's native USB (ESP32-S3 CDC) and the air link, next to the existing CRSF relay. The ground station connects here during flight tests.

The old `SerialTelecoms` protocol is gone, so the bridge's host protocol has to be designed (with safe framing and flow control) rather than reused.

USB was chosen over WiFi because WiFi shares the 2.4 GHz band with the link (see the workspace `docs/system-architecture.md`).

Code it would touch:
- **`src/main.cpp`:** on the TX module, `Serial` is USB CDC (`ARDUINO_USB_CDC_ON_BOOT=1`) and is used by `DebugMonitor` (`src/main.cpp:299-417`). The bridge would run its host protocol on `Serial` in `ENABLE_CRSF_INPUT` builds and move debug printing elsewhere or behind a flag.
- **`src/DatalinkCommandManager.cpp`:** the serial ↔ OTA data path was removed. A new one needs queueing and flow control, and must keep RC channels at their priority (`queueOtaPacketSecond`). The OTA `Data` type is still used for RC channels and CRSF telemetry relay, so a host path needs its own tag or sub-type.
- **`src/WifiUpload.cpp`:** the old "serial host lost → start WiFi" rule is gone and must not come back for the bridge or the vehicle RX.
- **Upload paths:** a USB-connected ground station could also replace WiFi for firmware updates later (not decided).

## 3. Related cleanup that the above depends on

These come from [known-issues.md](known-issues.md). They are prerequisites rather than separate features:
- Gate or remove the over-the-air test and upload commands for flight use, and revisit the 60 s sync failsafe.
- Use the parity-checked type byte on air, or an equivalent check, before adding more types.
- Bind or a per-system key, so two systems at one field can't interfere.
- Decide the vehicle-side host interface (the flight computer link) and, if it needs newer ExVectrNetwork/ExVectrCore APIs than the flight software pins, align those pins (a cross-project change). `SerialTelecomsDatalink` no longer exists.
