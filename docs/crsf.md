# CRSF

> CRSF on both ends of the link: the TX module talking to an EdgeTX handset, and the receiver talking to a flight controller.

Last verified against commit: 4830874 (submodule ExVectrPackets 46286bd)

## Generic CRSF class (`src/CRSF.hpp`, `src/CRSF.cpp`)

- **Frame format:** `[addr][len][type][payload][crc8 poly 0xD5]`, at most 64 bytes.
- **Receiving:**
  - The first byte of a frame must be one of 0xC8, 0x00, 0xEC, 0xEA or 0xEE (`CRSF.cpp:133-139`).
  - Frames with a valid CRC go into an 8-slot queue that drops the oldest entry when full (`CRSF.cpp:150-169`).
  - Callers then use `peekFrameType()`, `read<T>()` and `discardFrame()`.
- **Full duplex** (RX → FC): writes are appended to a 256-byte TX buffer and flushed. A write fails if the buffer is full.
- **Half duplex** (TX module ↔ handset, one shared wire):
  - Only **one** reply is ever pending. A new `write()` replaces the pending one (`CRSF.cpp:206-213`).
  - The reply goes out right after any valid frame is received from the handset (`CRSF.cpp:163-168`).
  - For the reply, the ESP32 GPIO matrix routes the pin from UART RX to UART TX and back (`switchToTx`/`switchToRx`). Inversion is handled there.

## TX module: `CRSFRcInput` (`ENABLE_CRSF_INPUT`)

Instantiated in `src/main.cpp:241-244`:
- `HardwareSerial(1)` at **400000 baud, inverted, half duplex** on the S.Port pin (the pin map's `serialRx == serialTx`).
- The handset's external-module baud rate must match.
- When RX and TX are the same pin, the port is opened as RX-only (`src/CRSFRcInput.cpp:160-185`). This avoids the ESP32 RS485 auto mode.

What it handles (`src/CRSFRcInput.cpp:201-252`):
- **RC input:**
  - Each `RC_CHANNELS_PACKED` (0x16) frame becomes one RC uplink packet (air-protocol.md, "RC channel packets").
  - Channels 1–4 are sticks. Channels 5–10 become aux1–aux6 as 3-position values.
  - CRSF channels 11–16 are not transmitted.
- **Link statistics to the handset** (0x14), queued every 100 ms (`src/CRSFRcInput.cpp:254-282`):
  - Uplink RSSI/LQ/SNR/antenna come from the far end's `LinkTelemetry`, and are 0 when that is stale.
  - Uplink power is the local TX power, as a CRSF power index.
  - Downlink values are the local radio A RSSI/SNR and the FHSS LQ.
  - `rfProfile` is fixed at 4.
- **Telemetry relay** (FC → handset, `src/CRSFRcInput.cpp:934-994`):
  - Every downlink `Data` payload is offered to a `PacketManager`.
  - `Telemetry_Battery` becomes CRSF battery (0x08), with capacity and percent left at 0.
  - `Telemetry_GNSS` and `Telemetry_GNSSPosition` update a cached CRSF GPS frame (0x02), which is re-sent on each update.
  - Adding a type means one `forwardTelemetry<>()` call here plus the matching sender on the RX side.
- **Lua parameter menu:** the standard ELRS/EdgeTX Lua script sees this module as an ELRS-like device (address 0xEE).
  - It answers Parameter Ping (0x28) with Device Info, `parametersTotal = 21` (`src/CRSFRcInput.cpp:284-299`).
  - Parameter Read (0x2C) returns entries. Parameter Write (0x2D) applies them.
  - A write to parameter 0 is the ELRS Lua link-stat poll. It is answered with 0x2E: good/bad derived from LQ, and the connected flag = remote stats valid (`src/CRSFRcInput.cpp:447-450`, `src/CRSFRcInput.cpp:598-622`).
  - Replies to the ELRS Lua's origin 0xEF are addressed to 0xEA (`src/CRSFRcInput.cpp:65-68`).

### Lua menu entries (`src/CRSFRcInput.hpp:93-249`)

| # | Name | Kind | Effect |
|---|---|---|---|
| 1 | (temperature) | info | module chip temperature (the entry name is not set in code) |
| 2 | RX Runtime | info | far end's uptime in minutes (`RxState`) |
| 3 | RX Desync | info | far end's desync count |
| 4 | FHSS Offset | info | far end's phase offset, µs |
| 5 | FHSS Interval | info | far end's clock-drift correction, ns |
| 6 | TX Missed | info | local missed-slot count |
| 7 | RX Missed | info | far end's missed-slot count (`SlotStats`) |
| 8 | Dyn Power | Off/On | enables the dynamic-power algorithm. **Saved to EEPROM.** |
| 9 | Dyn Min Pwr | dBm list | minimum dynamic power. **Saved.** |
| 10 | Dyn Max Pwr | dBm list | maximum dynamic power, and the fixed power when Dyn Power is off. **Saved.** |
| 11 | TX Upload Mode | command | this module → WiFi upload mode, link stops |
| 12 | RX Upload Mode | command | sends OTA `UploadMode`. **The receiver stops its link and enters WiFi mode.** |
| 13 | RX Block Test | command | after 10 s, sends OTA `TestBlock`. The receiver blocks for about 1.5 s. |
| 14 | TX Desync Test | command | after 10 s, shifts the local FHSS counters |
| 15 | RX Force Search | command | after 10 s, sends OTA `TestForceResync`. The receiver drops to `Searching`. |
| 16 | Sync Offset | info | far end's sync latency compensation (from `LinkTelemetry`) |
| 17 / 18 | Sync Offset +100us / −100us | command | sends OTA `SetSyncOffset` (current remote value ± 100 µs, clamped 0–2000). Not persisted on the far end. |
| 19 | RX RC Gap | info | far end's maximum gap between CRSF RC frames |
| 20 | RX Gap Reset | command | sends OTA `ResetRcGapStat` |
| 21 | TX Antenna | Diversity / Ant 1 / Ant 2 | pins TX to one antenna. **Saved.** |

- Commands run on click: there is no confirmation step, and there is no check of the link or vehicle state (see known-issues.md).
- Command status reverts to idle after 2 s (`src/CRSFRcInput.cpp:638-663`).
- Because half-duplex keeps only one pending reply, a Lua reply can be replaced by the 10 Hz link-stats frame or a telemetry frame before it is sent. The Lua script then retries.

## Receiver: `CRSFRcOutput` (`ENABLE_CRSF`)

Full duplex on `Serial` (UART0) at **420000 baud** (`src/CRSFRcOutput.hpp:60`, `src/main.cpp:225`).

- **RC output** (`src/CRSFRcOutput.cpp:277-312`):
  - One `RC_CHANNELS_PACKED` frame per received RC packet, written as soon as the packet arrives (the task deadline is pulled forward, `src/CRSFRcOutput.cpp:133-142`).
  - Channels 1–4 are sticks (the inverse of the TX mapping). Channels 5–10 are aux1–aux6 mapped 0/1/2 → 172/992/1811. Channels 11–16 are centre (992).
  - Analog jumps of more than 200 (out of 2000) are held for one packet by the glitch filter.
  - **On link loss no frames are written,** and the flight controller's RX-loss failsafe triggers. No failsafe channel values are sent.
- **Link statistics to the FC** (0x14), every 100 ms (`src/CRSFRcOutput.cpp:314-368`):
  - Uplink RSSI/SNR come from the local radio (the diversity aggregate on diversity boards).
  - Up and down LQ are both the local FHSS LQ.
  - Uplink power is the far end's reported TX power. The downlink fields are 0.
- **FC telemetry → downlink** (`src/CRSFRcOutput.cpp:236-268`):
  - CRSF battery (0x08) becomes `Telemetry_Battery` (key `CrsfBatterySensor`).
  - CRSF GPS (0x02) becomes `Telemetry_GNSS` plus `Telemetry_GNSSPosition` (two keys).
  - All are queued in the coalescing OTA queue, so only the newest value of each is sent, at the downlink rate.
- **`Telecommand_UpdateMode`** received in a `Data` frame enters WiFi upload mode (`src/CRSFRcOutput.cpp:144-148`).
- **Not ported:** bind command detection from the FC (the legacy `CRSFOutput.cpp:451-453` mapped it to WiFi upload mode).
- **Statistics** (`getCrsfRcPacketRateHz`, `getCrsfTxFailureRateHz`, `getCrsfMaxRcFrameGapUs`) feed `SlotStats` and the debug output.

## SBUS output (`ENABLE_SBUS`, stale)

`SBusTx` in `src/main.cpp:94-221`:
- 200000 baud 8E2, inverted, on `Serial`, every 5 ms. Failsafe flags are set after 200 ms without packets.
- It still decodes the old `RadioLinkPacket_A4D4` format, not the current `RcLinkPacket`, so it receives nothing from a current TX.
- No env enables it.

## Legacy classes

`src/CRSFInput.*` and `src/CRSFOutput.*` are the predecessors of `CRSFRcInput`/`CRSFRcOutput`. They are still compiled under the same flags but never instantiated, and they use the old A4D4 RC format. Their comments remain the reference for the byte-level diagnostics and the bind handling that haven't been ported yet.
