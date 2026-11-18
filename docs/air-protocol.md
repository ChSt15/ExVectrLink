# Air Protocol

> How the two ends talk over 2.4 GHz: timing, hopping, framing, packet types, and the services built on top of them.

Last verified against commit: 4830874 (submodules: ExVectrLink 37ab8e2, ExVectrNetwork 7cb0e79, ExVectrPackets 46286bd)

## Layers

```
Application   src/DatalinkCommandManager.cpp   OTA type byte, OTA send queue, link telemetry, commands
              src/CRSFRcInput/CRSFRcOutput     RC channels, telemetry relay (as PacketTypes::Data)
Link          lib/ExVectrLink FHSS             slots, hops, role reversal, sync, 2-byte trailer, LQ
Diversity     lib/ExVectrLink Sx1280Diversity  1–2 radios behind one Sx1280_DirectI
PHY driver    lib/ExVectrNetwork Sx1280_Direct LoRa modem, channel table, DIO1 timestamps
```

## Radio settings actually in use

Set once in `setup()` for each radio (`src/main.cpp:664-674`):
- LoRa SF7, bandwidth 800 kHz, coding rate 4/8 (`LI_4_8`).
- Packet mode `Limited`: implicit header, fixed on-air size. A 1-byte length prefix is followed by 11 bytes, so 12 bytes are on air (`lib/ExVectrNetwork/include/ExVectrNetwork/datalink/sx1280/Sx1280_Settings.hpp:42`).
- 12-symbol preamble, hardware LoRa CRC on (`lib/ExVectrNetwork/src/sx1280/Sx1280_Direct.cpp:456-461`).
- Channel table: 20 channels evenly spaced over 2425–2475 MHz, about 2.63 MHz apart (`Sx1280_Direct.hpp:183-187`).

### Modulation presets (defined, not wired into the link)

`ModulationPresets` (`lib/ExVectrLink/include/ExVectrLink/datalink/LinkSettings.hpp:10-15`, table in `lib/ExVectrLink/src/datalink/LinkSettings.cpp:9-37`):

| Index | Preset | SF | BW | CR | `hopInterval` | Comment in code |
|---|---|---|---|---|---|---|
| 0 | Hyperspeed | 5 | 1600 kHz | 4/5 | 1/250 s | shortest range, "used mainly for dual end throughput" |
| 1 | Fast | 6 | 800 kHz | 4/8 | 1/250 s | short range |
| 2 | Medium | 7 | 800 kHz | 4/8 | 1/150 s | balanced |
| 3 | LongRange | 8 | 800 kHz | 4/8 | 1/50 s | long range |

Nothing in `src/` uses the table now: its only consumer, the serial `SetModulationPreset` handler, was removed with the serial protocol. `hopInterval` was never used, and the FHSS slot timing is not changed by any preset.

So the link always runs SF7/BW800/CR4/8. Those are the radio parameters of "Medium", but at 100 slots/s rather than 150. Making presets real is roadmap item "link rate".

## FHSS timing (`lib/ExVectrLink/src/datalink/FHSS.cpp`)

| Parameter | Value | Source | Set anywhere? |
|---|---|---|---|
| Slot interval | 10 ms | `FHSS.hpp:208` | No setter is called. |
| Slots per hop | 4 (40 ms per channel) | `FHSS.hpp:210` | No. The setter caps at 16, but the trailer only carries 2 bits. |
| Role-reversal period `numTxPacketsToRx` | 15 slots | `FHSS.hpp:217` | No. `setRxSlotIndex()` would cap it at 4 (`FHSS.cpp:54-61`). |
| Hop key | 0 | `FHSS.hpp:226` | No. The serial `StartFHSS` packet that used to set it was removed. |
| Sync latency compensation | 900 µs | `FHSS.hpp:213` | Lua "Sync Offset ±100us" → OTA `SetSyncOffset`, not persisted |

- **Slots.** Each slot is one TX or one RX operation. `roleReverseCounter` counts 0…14.
  - The TX side transmits in every slot where the counter is not 0, so 14 of 15 slots.
  - The RX side transmits only in the slot where it is 0 (`FHSS.cpp:628-629`).
  - So the link carries about **93 uplink frames/s and 6.7 downlink frames/s**.
  - The RX side may only transmit once it is `Synced` (`FHSS.cpp:648`).
- **Hopping.** A hop happens when `slotCounter` wraps to 0. The channel order is a deterministic Fisher–Yates shuffle of the 20 channels, seeded with `key * 2654435761` and stepped with xorshift (`FHSS.cpp:152-178`). The sequence repeats every 20 hops (800 ms).
- **TX timing:**
  - The task wakes 1 ms before the slot boundary (`FHSS.cpp:665-677`).
  - It loads the FIFO early (`push(true)`).
  - It busy-waits until the boundary, then calls `startTx()` (`FHSS.cpp:293-303`).
- **RX:** the radio is re-armed every RX slot with `startRx(0xFFFF)` (`FHSS.cpp:658-659`).
- **In-flight grace window:** if a reception is still in progress at wakeup, the task keeps polling until it completes, for at most 0.5 ms (`FHSS.cpp:458-481`).
  - If the next slot is RX, the limit is 0.5 ms past the boundary.
  - If the next slot is TX, the limit is 0.5 ms before the boundary.
  - Counters: `graceRescueCount`, `graceExpireCount`.
- **Missed-slot catch-up:** if the task wakes more than one slot late, it fast-forwards the grid, the counters and the channel index. Missed RX slots count as lost for LQ, and `missedSlotCounter` increases (`FHSS.cpp:577-614`).

### Sync states (RX side)

| State | Behaviour | Leaves when |
|---|---|---|
| `Searching` | Slots are stretched to 1.9× (19 ms, `FHSS.cpp:567-570`). Hops backwards through the sequence every slot (`FHSS.cpp:636-641`). Does not transmit. | First valid packet → `Syncing`. Phase is taken from the RX timestamp (`FHSS.cpp:208-224`) and counters from the trailer (`FHSS.cpp:346-349`). |
| `Syncing` | Phase error is corrected by 0.2 per packet (`FHSS.cpp:226-236`). Does not transmit. | → `Synced` when the phase error has stayed ≤ slot/10 for more than 500 ms **and** LQ > 0.75 (more precisely > max(1−1/slotsPerHop, 0.5)). → `Searching` after 0.5 s without a packet (`FHSS.cpp:506-514`). |
| `Synced` | Phase filter 0.98/0.02, offset correction 0.01 per packet. A clock-drift integrator (gain 1e-5, clamped to ±1 % of the slot) runs (`FHSS.cpp:238-254`). Transmits in its role-reversal slot. | → `Searching` after 3 s without a packet (`FHSS.cpp:498-505`). `desyncCounter` increases. |

- **The TX side doesn't sync.** It reports `Synced` if it received any packet in the last second, otherwise `Searching` (`FHSS.cpp:520-524`). Its slot grid free-runs.
- **Counter resync:** if a packet that passes all checks carries a different role counter, the RX re-snaps its counters as soon as two consecutive packets agree on the same shift. After more than 10 mismatches it drops to `Syncing` (`FHSS.cpp:350-382`).
- **Sync timestamp:** the RX_DONE time from the DIO1 ISR, minus the sync latency compensation (`FHSS.cpp:190`).
- **Link quality** (`getLinkQuality()`): the fraction of the last 100 RX slots that received a valid frame, with or without payload (`FHSS.cpp:530-545`, `FHSS.cpp:616-619`).

## OTA frame

The radio carries 11 bytes of FHSS payload:

```
 0 .. n-1   application payload (n ≤ 9 = FHSS::getMaxPacketSize(), FHSS.cpp:144)
            └─ last application byte = OTA packet type (see below)
 n          trailer byte 1: bits 7..2 = roleReverseCounter (6 bit), bits 1..0 = slotCounter (2 bit)
 n+1        trailer byte 2: XOR checksum
```

- **Checksum:** it starts at `key + OTA_VERSION(2) + (sender is RX side ? 0x40 : 0)` and XORs in every byte before it (`FHSS.cpp:279-291`). The receiver checks it with the opposite side bit (`FHSS.cpp:320-326`).
  - So a frame from a different key, a different `OTA_VERSION` or the same side (TX to TX) is rejected.
  - It is a 1-byte XOR on top of the LoRa CRC16, not a security measure.
  - The comment at `FHSS.hpp:186` describes an older trailer layout and is outdated.
- **Frame length:** fixed-length mode always sends 11 bytes. The driver's length prefix tells the receiver the real `n+2`.
- **No payload:** if the TX slot has no payload queued, a 2-byte trailer-only keep-alive goes out. It still counts for LQ and sync, but it isn't passed to the receive handlers (`FHSS.cpp:388-401`).
- **One TX slot only:** FHSS holds one pending frame (`packetToSend`). `transmitDataframe()` overwrites it without checking (`FHSS.cpp:129-142`). `isChannelBlocked()` is true while a frame is pending.

### OTA packet type byte

The **last application byte** of every frame is a `PacketTypes` value (`lib/ExVectrLink/include/ExVectrLink/datalink/PacketTypes.hpp:10-35`). Everything before it is that type's payload, up to 8 bytes.

| Value | Type | Payload | Sent by | Effect on receipt (`src/DatalinkCommandManager.cpp`, receive handler) |
|---|---|---|---|---|
| 0 | `Data` | ≤ 8 bytes | TX: RC channels. RX: CRSF telemetry relay. | Ignored by the command manager (it used to forward them to the serial host). `CRSFRcOutput` and `CRSFRcInput` decode ExVectrPackets out of it. |
| 1 | `Heartbeat` | none | nobody | ignored |
| 2 | `LinkTelemetry` | 8 bytes (5 used) | both, 2 Hz plus filler | updates `remoteStats` and drives the far end's dynamic power |
| 3 | `RxState` | 8 bytes | both (despite the name), 1 Hz | desync count, runtime, FHSS offset, interval correction (shown in the Lua menu) |
| 4 | `UploadMode` | none | TX: Lua "RX Upload Mode" | **receiver enters WiFi upload mode and stops all tasks, including the link** |
| 5 | `SlotStats` | 8 bytes | both, 1 Hz | far end's missed-slot count and maximum CRSF RC gap |
| 6 | `TestBlock` | none | TX: Lua "RX Block Test" (10 s delay) | **receiver blocks for 1514 ms** (`delay(1514)`, line 684) |
| 7 | `TestForceResync` | none | TX: Lua "RX Force Search" (10 s delay) | receiver desyncs and goes straight to `Searching` |
| 8 | `SetSyncOffset` | 2 bytes LE, µs | TX: Lua "Sync Offset ±100us" | sets sync latency compensation, clamped to 0–2000 µs |
| 9 | `ResetRcGapStat` | none | TX: Lua "RX Gap Reset" | resets the CRSF RC-gap statistic |

**About the parity bit:** `PacketTypes.hpp:37-88` defines an odd-parity encoding, with the type in bits 6..0 and parity in bit 7, via `encodePacketType()` / `packetTypeIsValid()`. **The firmware in `src/` does not use it.** It writes and compares the raw enum value (for example `src/DatalinkCommandManager.cpp:523`, `src/CRSFRcInput.cpp:1057`). The host-side `LinkManager` that used it inside `Data` payloads was removed. The header comment ("everything … must go through encodePacketType()") does not describe current behaviour. See known-issues.md.

### OTA send queue

Every state-like sender queues through `OtaPacketQueue<8>` (`src/OtaPacketQueue.hpp`, instance at `src/DatalinkCommandManager.cpp:105`). The queue is keyed by `OtaSendKey` (`src/DatalinkCommandManager.hpp:37-65`):
- A key that is already queued has its packet replaced in place, so stale duplicates never pile up.
- New keys go to the back (`queueOtaPacket`), the front (`queueOtaPacketFront`) or second place (`queueOtaPacketSecond`, used by RC channels and link telemetry).
- `OtaQueuePump` (2 ms) hands the next entry to FHSS whenever the TX slot is free. When the queue is empty it sends a fresh `LinkTelemetry` (`src/DatalinkCommandManager.cpp:139-157`).

**Downlink budget:** the RX side has about 6.7 TX slots/s. Of those, link telemetry takes 2/s, RxState 1/s and SlotStats 1/s. Battery, GPS stats and GPS position share the rest. Every idle slot is filled with link telemetry.

## RC channel packets

Defined in `src/RcLinkChannels.hpp` as `RadioLinkMuxPacket<4, 1, 0, 5>` (`lib/ExVectrPackets/include/ExVectrPackets/radiolink/packets/RadioLinkMuxPackets.hpp`). These template parameters are part of the wire format, and both ends must use the aliases.

| Field | Bits | Content |
|---|---|---|
| Priority analog ×4 | 4×10 | roll, pitch, yaw, throttle, −1000…1000 mapped to 0…1023 |
| Priority digital ×1 | 2 | aux1 (arm), 0…2 (3-position) |
| Mux index | 3 | which of aux2…aux6 this packet carries |
| Mux value | 10 | that channel's raw 0…3 value |
| Type byte | 8 | `(PacketType::RadioLinkMux=4) << 2 \| datatype 0` = 0x10 |

- That is 55 payload bits → 7 bytes + 1 ExVectrPackets type byte = 8 bytes. With the OTA `Data` type byte it fills the 9-byte frame exactly.
- Each aux2–aux6 value refreshes every 5th RC packet.
- **TX side:**
  - `CRSFRcInput::forwardRcChannels()` (`src/CRSFRcInput.cpp:1020-1060`) maps CRSF channels 0–3 to roll, pitch, throttle and yaw into analog 0, 1, 3 and 2.
  - CRSF channels 4–9 become digital 0–5, each snapped to 0/1/2.
  - One RC packet is built per CRSF frame from the handset and queued second (key `RcChannels`).
- **RX side:**
  - `CRSFRcOutput` accumulates packets (`RcLinkAccumulator`).
  - A glitch filter rejects an analog jump of more than 200 once, then accepts it (`RadioLinkPackets.hpp:85-108` in ExVectrPackets).
  - One CRSF frame is written per received RC packet. See crsf.md.
- **No RC failsafe values are sent.** When packets stop, the RX stops writing RC frames, and the flight controller's own RX-loss failsafe takes over.

## Data packets (ExVectrPackets inside `Data`)

ExVectrPackets `Telemetry_*` / `Telecommand_*` start with a 2-byte header `[PacketType, dataType]` (`lib/ExVectrPackets/include/ExVectrPackets/telecoms/packets/Telemetry.cpp`).

| Packet | Bytes | Encoding |
|---|---|---|
| `Telemetry_Battery` | 6 | u16 mV, u16 cA |
| `Telemetry_GNSS` | 7 | u16 km/h, u8 satellites, i16 altitude m |
| `Telemetry_GNSSPosition` | 8 | 24-bit big-endian lat/lon, 40000 units/degree (~2.8 m) |
| `Telecommand_UpdateMode` | 2 | received by `CRSFRcOutput`: enters WiFi upload mode (`src/CRSFRcOutput.cpp:144-148`) |

Receivers try every registered type on every `Data` payload (`PacketManager::receivePacketData`). The size check there is commented out (`lib/ExVectrPackets/include/ExVectrPackets/Packets.hpp:118-120`).

## Link telemetry

- **`LinkTelemetry`** (`encodeLinkInfo`, `src/DatalinkCommandManager.cpp:239-257`):

| Byte | Bits | Field |
|---|---|---|
| 0 | 7..5 | TX antenna index (0–7) |
| 0–1 | 0[4..0], 1[7..6] | LQ, 0–100 % (7 bit) |
| 1 | 5..1 | TX power − 1 dBm (1–32 dBm) |
| 2 | 7..0 | \|RSSI\| dBm (0–127) |
| 3 | 7..3 | SNR + 15 (−15…+15 dB) |
| 4 | 0 | dynamic power enabled |
| 4 | 6..1 | sync latency compensation / 50 µs (0–40) |
| 5–7 | | unused (0) |

- **`RxState`:** u16 desync count, u16 runtime in minutes, i16 FHSS offset in µs, i16 interval correction in ns, all little-endian (`src/DatalinkCommandManager.cpp:292-303`).
- **`SlotStats`:** u32 missed slots, u32 maximum CRSF RC gap in µs (`src/DatalinkCommandManager.cpp:320-330`).
- **Remote stats validity:** `getRemoteLinkStats().valid` turns false 1 s after the last frame of any kind from the far end (`src/DatalinkCommandManager.cpp:59-75`, `src/DatalinkCommandManager.cpp:587-593`).

## Diversity (`lib/ExVectrLink/src/datalink/Sx1280Diversity.cpp`)

Diversity is used on targets with `HAS_DIVERSITY`: the diversity TX module and the GenericDiversityELRS RX. It handles up to 2 radios.
- **RX:** both radios listen on the same channel. The winner of a slot is the one with the highest SNR. On a tie the earlier timestamp wins. Only the winner's FIFO is read (`Sx1280Diversity.cpp:331-451`). The reported SNR is the median of the last 10 winning SNRs.
- **Demotion:** a radio that didn't receive in a slot where the other did gets worst-case RSSI/SNR floors, so a deaf radio can't keep the TX role (`Sx1280Diversity.cpp:394-412`).
- **TX:** transmits on the best "fresh" radio, meaning one that received within the last 1 s (`Sx1280Diversity.cpp:453-505`). Alternatively, transmissions can be pinned to one antenna with the Lua "TX Antenna" setting (stored in EEPROM, `src/RFSetup.cpp:47-54`).
- **Self-echo:** after our own TX, the passive radio's echo of it is drained without updating its stats (`Sx1280Diversity.cpp:113-132`).
- **Single-radio builds** go through the same class with one link (`src/DiversitySetup.cpp`).

## Dynamic power

- **Power levels** are {10, 14, 17, 20, 24, 27, 30, 33} dBm (`lib/ExVectrLink/include/ExVectrLink/datalink/DynamicPower.hpp:11`).
  - The hardware maximum is the board's `MaxPower` snapped down to a level.
  - The hardware minimum is 10 dBm (`src/main.cpp:683-685`).
- **TX side:**
  - Dynamic min/max and the enable flag come from EEPROM. Defaults: enabled, 10–20 dBm.
  - Startup is at the dynamic maximum (`src/RFSetup.cpp:61-71`).
  - On each received remote `LinkTelemetry`, `update()` runs (`lib/ExVectrLink/src/datalink/DynamicPower.cpp:160-190`):
    - one step up if remote RSSI < −100, SNR < 1 or LQ < 85;
    - one step down if RSSI > −110, SNR > 8 and LQ > 95, at most once per second.
  - Separately, at the end of every RX slot, power goes up one step if 2 or more consecutive RX slots were lost or the remote stats are stale. This step has no rate limit (`src/DynamicPowerSetup.cpp:39-68`).
  - With dynamic power off, the radio runs at the dynamic maximum.
- **RX side:** fixed 20 dBm, algorithm off, whatever is stored (`src/RFSetup.cpp:18`, `src/RFSetup.cpp:72-79`).

## Bind

**There is no bind procedure.**
- All units use hop key 0 by default, so they share one hop sequence and one trailer-checksum seed. Any TX links with any RX of the same `OTA_VERSION`.
- Nothing changes the key at runtime now (the serial `StartFHSS` packet was removed), and it is not persisted.
- The legacy `CRSFOutput` mapped the Betaflight bind command to WiFi upload mode. That isn't ported to `CRSFRcOutput`.

## WiFi / OTA upload mode (`src/WifiUpload.cpp`)

Upload mode connects to the WiFi network in `src/WifiCredentials.cpp`, starts mDNS `exvectr_link.local` and ArduinoOTA (port and upload IPs are in `platformio.ini`), and enables modem sleep.

How it is entered:

| Trigger | Stops the link? | Where |
|---|---|---|
| Boot button held at power-up | yes (link never starts) | `src/main.cpp:589-594` |
| 5× `r` on `Serial` in the first second | yes (link never starts) | `src/main.cpp:601-615` |
| Button held 3 s while running | **no**, WiFi runs next to the link | `src/WifiUpload.cpp:177-185` |
| `ENABLE_FORCEWIFI` | no | `src/WifiUpload.cpp:155-157` |
| OTA `UploadMode`, `Telecommand_UpdateMode`, Lua "TX Upload Mode", sync failsafe | **yes**: `startWiFiUploadMode()` removes every scheduler task except WiFi | `src/WifiUpload.cpp:199-210` |

The only exit is a reset.

## Sync failsafe

`SyncFailsafeMonitor` (`src/main.cpp:247-293`) runs on every build and both sides:
- If FHSS never leaves `Searching` within 60 s of boot, it calls `startWiFiUploadMode()`. The link is stopped until reset.
- Reaching `Syncing` once cancels it permanently.
- On the TX side, `Searching` means no packet from an RX in the last second. So a TX module, or a vehicle RX, powered for 60 s without its partner also ends up in upload mode.

This is meant as a recovery path for bad firmware or configuration. Its operational impact is listed in known-issues.md.
