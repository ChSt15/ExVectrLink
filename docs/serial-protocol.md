# Serial Protocol (`SerialTelecoms`, host ↔ module)

> The UART protocol between an ExVectrLink module and a host computer: for example the Starship flight computer (Teensy 4.0) on the vehicle receiver, or a PC/ground station on a TX module. Both ends use the same library code (`lib/ExVectrLink/src/serial/SerialTelecoms.cpp`). Everything below is exactly what the code does today, including its weak points. Read [known-issues.md](known-issues.md) before relying on it in flight.

Last verified against commit: 4830874 (submodule ExVectrLink 37ab8e2, `ExVectrLinkVersion = 12`)

## When it is active

- `SerialTelecoms` runs only in **plain builds**, meaning no `ENABLE_CRSF`, `ENABLE_CRSF_INPUT` or `ENABLE_SBUS`. In CRSF/SBUS builds the task is removed and disabled (`src/main.cpp:695-698`).
- At this commit the only plain env is `exvectrlink_controller_*` (TX side). No RX env is plain yet (see build-targets-and-hardware.md).
- **Physical layer:** `HardwareSerial(1)` on the target's `serialRx`/`serialTx` pins, **1 000 000 baud, 8N1** (`src/main.cpp:41-46`, `src/main.cpp:583-584`). On ESP8266 the code would use `Serial`, but no plain ESP8266 env exists.

## Framing

```
+------+-----------------------+------+-----+--------------+------+
| 0x7E | 0x7C + Version (0x88) | type | len | data[0..len) | 0x7F |
+------+-----------------------+------+-----+--------------+------+
```

| Field | Value |
|---|---|
| Start A | `0x7E` (`SerialTelecoms.cpp:20`) |
| Start B | `0x7C + ExVectrLinkVersion`. With version 12 that is `0x88` (`SerialTelecoms.cpp:174-175`). A wrong byte here drops the frame, so incompatible versions don't talk at all. |
| type | `SerialPacketType`, one byte (table below) |
| len | Number of data bytes, 0–255 (`SerialTelecoms.cpp:177`) |
| data | `len` bytes, packet-specific, little-endian multi-byte values |
| End | `0x7F` |

**How the receiver actually decodes** (`SerialTelecoms.cpp:216-269`):
- After `len` it collects bytes until the first `0x7F` and treats that as the end of the frame. **`len` is stored but never used** (`SerialTelecoms.cpp:242`).
- There is **no escaping and no checksum or CRC**.
- **A payload byte equal to `0x7F` ends the frame early.** Typed packets then fail their size check and are dropped. Raw `PacketData` is delivered truncated, and the rest of the frame is parsed as garbage until the next `0x7E`.
- Any data, including binary telemetry, must therefore avoid `0x7F`, or the protocol has to be fixed first (see known-issues.md).
- More than 255 data bytes without an end byte resets the parser.
- Any complete frame (start A, start B, type, len, data, end) counts as "valid" for the connection state, whatever its type.

**How the sender works** (`SerialTelecoms.cpp:157-183`, `SerialTelecoms.cpp:66-144`):
- Frames are appended to a 100-byte ring buffer (`SerialTelecoms.hpp:114`). A frame that doesn't fit is partly written, because `placeBack` fails byte by byte, so the bytes that don't fit are lost. The 1024-byte guard at `SerialTelecoms.cpp:166` does not protect against this.
- The task alternates reading and writing, each limited to about 1 ms per run. It wakes immediately when bytes are readable or queued (`taskCheck`).

## Packet types (`lib/ExVectrLink/include/ExVectrLink/serial/SerialTelecomPackets.hpp:12-33`)

Dir: H→M = host to module, M→H = module to host. "Handled" refers to the module firmware at this commit.

| # | Type | Bytes | Dir | Layout | Module behaviour |
|---|---|---|---|---|---|
| 0 | `Ack` | 1 | none | `[success]` | not used |
| 1 | `PacketData` | 0–255 | both | raw | H→M: one air `Data` frame, **at most 8 bytes, or it is silently dropped** (`src/DatalinkCommandManager.cpp:517-526`). M→H: every OTA `Data` frame from the far end, type byte stripped (`src/DatalinkCommandManager.cpp:617-621`). |
| 2 | `ChannelState` | 2 | M→H | `[maxBytes, blocked]` | never sent (the code is commented out) |
| 3 | `SetModulationPreset` | 1 | H→M | `[presetIndex]` | changes local SF/BW/CR only, no bounds check (see air-protocol.md) |
| 4 | `SetLinkChannel` | 1 | H→M | `[channel 0–9]` | handler is empty (no-op) |
| 5 | `StartFHSS` | 5 | H→M | `[enable, key u32 LE]` | `setFhssKey(key)`, truncated to `uint8_t`. `enable` is ignored. Not persisted. |
| 6 | `SetPowerParams` | 3 | H→M | `[dyn<<7 \| txPower(7b), maxDyn, minDyn]` | applies to `DynamicPower` and saves to EEPROM (`src/DatalinkCommandManager.cpp:542-567`). 0 means maximum/minimum. On the RX side these values are overwritten by the fixed RX power only at the next boot. |
| 7 | `DeviceTemperature` | 1 | M→H | `[int8 °C]` | sent every 5 s on ESP32 (`src/main.cpp:48-64`) |
| 8 | `FhssSyncState` | 1 | M→H | `[synced]` | sent every 500 ms, 1 = `Synced` (`src/main.cpp:66-92`) |
| 9 | `Error` | n | M→H | NUL-terminated text | when radio init fails (retried every ~100 ms). With `ENABLE_DEBUG_OUTPUT`, a status line every ~100 ms (`src/main.cpp:419-509`). |
| 10 | `Print` | 1+n | M→H | `[len, text]` | not sent (log hook commented out, `src/DatalinkCommandManager.cpp:514-515`) |
| 11 | `Heartbeat` | 1 | both | `[isConnected]` | M→H every 100 ms. H→M sets `isOtherEndConnected` (see below). |
| 12 | `LinkInfo` | 27 | M→H | see below | sent every 500 ms (`src/DatalinkCommandManager.cpp:415-465`) |
| 13 | `SetBaudRate` | 4 | both | `[baud u32 LE]` | see below |
| 14 | `UpdateMode` | 9 | H→M | `[forward, duration i64 LE]` | contents ignored. Starts WiFi without stopping the link, and replies with an `UpdateMode` frame (`src/WifiUpload.cpp:149-153`). |
| 15 | `InitLink` | 1 | H→M | `[mak]` | **no handler**: the MAK is ignored |

### `LinkInfo` (27 bytes, `SerialTelecomPackets.hpp:299-379`)

Bytes 0–12 are the local side, 13–25 the remote side (same layout), and 26 is `remoteValid`.

| Offset | Field |
|---|---|
| +0 | RSSI, int8 dBm |
| +1 | SNR, int8 dB |
| +2 | bit 7 dynamic power, bits 6..0 TX power dBm |
| +3 | antenna index |
| +4 | link quality % (fraction of RX slots with a valid frame) |
| +5 | loss rate % (= 100 − LQ) |
| +6 | packet quality % (= LQ, kept for compatibility) |
| +7..8 | desync count u16 |
| +9..10 | RC packet rate u16 (0 in plain builds) |
| +11..12 | CRSF TX failures/s u16 (0 in plain builds) |

- Remote values come from the far end's OTA `LinkTelemetry`/`RxState`.
- The remote desync count and remote packet rate fields are either 0 or come from `RxState`.
- While the local FHSS is not `Synced`, remote LQ is reported as 0 and `remoteValid` as false (`src/DatalinkCommandManager.cpp:452-457`).

## Connection and heartbeat

- `isConnected()` (module hears the host) becomes true on any valid frame. It turns false 500 ms after the last one (`SerialTelecoms.cpp:130-136`).
- The module sends `Heartbeat{isConnected}` every 100 ms (`SerialTelecoms.cpp:138-141`), so the host can see whether the module hears it.
- `isOtherEndConnected()` is the flag from the last heartbeat the host sent.
- **The host must send at least one valid frame every 500 ms**, for example its own heartbeat every 100 ms. Otherwise the module counts it as disconnected. In plain builds, after 10 s uptime this also turns on WiFi upload mode alongside the link (`src/WifiUpload.cpp:188-193`). See known-issues.md.

## Baud rate switching

Handler at `SerialTelecoms.cpp:38-49`:
1. The host sends `SetBaudRate{baud}`.
2. If `baud` differs from the module's *internal* rate variable, the module echoes the same frame at the old rate, flushes it (blocking, up to 500 ms) and switches the port.
3. If no byte arrives for 1 s while the internal rate is not 115200, the module falls back to 115200 (`SerialTelecoms.cpp:121-128`).

**Mismatch to know about:** the internal variable starts at 115200 (`SerialTelecoms.hpp:24`, `SerialTelecoms.hpp:124`), but `main.cpp` opens the port at 1 000 000.
- Requesting 115200 from the 1 Mbaud start is a no-op, and the port stays at 1 Mbaud.
- After a switch to another rate, a 1 s silence drops the module to 115200, not back to 1 Mbaud.
- A host using baud switching must handle both of these, or simply stay at 1 Mbaud and keep traffic flowing.

## Data path and throughput (what a host can expect)

- **One H→M `PacketData` frame becomes one OTA frame,** with at most 8 data bytes. There is no fragmentation and no queueing. The handler writes straight into FHSS's single TX slot and overwrites whatever is pending (`src/DatalinkCommandManager.cpp:524`). There is no flow control: `ChannelState` is never sent.
- **Rates the air link carries** (see air-protocol.md):
  - Uplink (TX module → vehicle): about 93 frames/s, shared with RC channels and link telemetry.
  - Downlink (vehicle → TX module): **about 6.7 frames/s in total**. Link telemetry takes 2/s, RxState and SlotStats 1/s each. A vehicle host gets roughly 2–3 frames/s × 8 bytes ≈ 20 B/s if it sends no faster than that. Anything faster overwrites earlier frames.
- **Delivery:** frames arrive at the far host in order, without duplication, but with loss.
  - There is no ACK or retransmission at this layer.
  - Per safety.md section 2, commands must be sent as repeated state.

## Using it from a host: `SerialTelecomsDatalink` and `LinkManager`

- **`SerialTelecomsDatalink`** (`lib/ExVectrLink/include/ExVectrLink/serial/SerialTelecoms.hpp:136-189`, `SerialTelecoms.cpp:285-393`) implements `ExVectrLinkI`, which extends ExVectrNetwork's `network::datalink::DatalinkI` (`lib/ExVectrLink/include/ExVectrLink/ExVectrLinkI.hpp`).
  - The host builds a `SerialTelecoms` on its UART (`HAL::DigitalIO`), then a `SerialTelecomsDatalink` on top, and calls `initialize()`.
  - `transmitDataframe()` sends `PacketData`. `getMaxPacketSize()` returns 250, but the air link only carries 8 bytes. `isChannelBlocked()` always returns false.
  - Received `PacketData` goes to the `DatalinkI` receive handlers.
  - `LinkInfo` fills `getLinkInfo()`. `FhssSyncState` drives `isConnected()`.
  - The setters send `SetPowerParams`, `SetModulationPreset`, `StartFHSS`, `SetLinkChannel` and `InitLink`. Their module-side effects are in the table above.
- **`LinkManager`** (`lib/ExVectrLink/src/LinkManager.cpp`) wraps an `ExVectrLinkI`:
  - It appends `encodePacketType(Data)` (`0x80`, odd parity) to every outgoing payload (`LinkManager.cpp:99-107`).
  - On receive it accepts only payloads ending in a valid-parity `Data` byte, and strips it (`LinkManager.cpp:113-133`).
  - That costs one of the 8 OTA bytes, so **7 bytes of user data per frame** remain.
  - `isFailsafe()` = `!link.isConnected()`.
  - Binding and power control are stubs.
  - Both ends must use `LinkManager`, or neither.
- **The umbrella header `lib/ExVectrLink/include/ExVectrLink.hpp:4` includes a non-existent path** (`ExVectrLink/SerialTelecoms.hpp`). Include `ExVectrLink/serial/SerialTelecoms.hpp` directly.

### Starship flight computer integration notes

- **ExVectrNetwork:** the flight software pins cabddce, with the old API (`Datalink_Interface`, `Topic<List<uint8_t>>`, `NetworkNode`). This repo pins 7cb0e79 (29 commits later, `DatalinkI` with `DataPacket` handlers). Using `SerialTelecomsDatalink` on the Teensy requires moving the flight software to the newer ExVectrNetwork API.
- **ExVectrCore:** the pin used here (18bbb09) renamed `NOW()` to `NowNs()`. The flight software may need to follow.
- These are cross-project changes.
- **Frame size:** the existing flight-software radio packets (`PacketAttitude` etc. on ports 10–100) are much larger than 8 bytes, and the downlink carries only a few frames per second. The current protocol cannot carry the Starship telemetry set. This is the motivation for the "telemetry/telecommand" protocol mode in [roadmap.md](roadmap.md).
