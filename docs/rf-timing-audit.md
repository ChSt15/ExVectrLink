# RF timing audit: network-mode firmwares

> Code-reading audit of everything that can run between two FHSS slot events in `exvectrlink_tx_module_network` (ESP32-S3, node 2, sync master) and `exvectrlink_rx_network_esp8285` (ESP8285 @160 MHz, node 3, sync follower). Written for the bench symptom of 2026-11-20: both ends Synced, RSSI -62..-70, SNR 10, no bad air packets, **uplink LQ 88-96 %, downlink LQ (TX module hears RX) 20-40 %**, air profile v1, 10 dBm, sync latency compensation at the 900 µs default.

Last verified against commit: 101b886 plus the uncommitted working tree of `feature/network-power-cap` as of 2026-11-20 (including its LINK_REBOOT command and the FHSS counters in the temporary diagnostic) (submodules: ExVectrLink `feature/air-link-fhss`, ExVectrNetwork `feature/gs-requests-and-upload`). Frameworks as installed: Arduino-ESP32 3.3.9 (pioarduino), ESP8266 core 3.1.2.

**Status of the numbers:** nothing here was measured. Execution times are estimates from the code structure (loop counts, bytes, SPI traffic, heap allocations) at 240 MHz (S3) and 160 MHz from flash cache (ESP8285). They show where the time can go and which sections matter. Section 8 gives the measurement plan that replaces them with real numbers. The sync latency compensation is deliberately not determined here (it is being swept on the bench). It only appears as a parameter in the margin model.

## 1. Timing model: when a slot is lost

Air profile v1 (`lib/ExVectrLink/include/ExVectrLink/datalink/AirProfile.hpp:40-42`): 20 ms slots, 1:1 roles, 2 slots per hop, wakeup lead 2.5 ms, 255 bytes per frame, time on air 10.56 ms. The LoRa preamble is 12 symbols. At SF5/BW1625 a symbol lasts 19.7 µs, so the whole preamble plus sync is only **about 0.32 ms**. In FPV (SF7/BW800) it is about 1.9 ms, which is what the grace-window comment in `FHSS.cpp:466-468` assumes.

The FHSS task (`lib/ExVectrLink/src/datalink/FHSS.cpp:449-701`) is a Core scheduler task. It is released at `boundary - wakeupLead` (`FHSS.cpp:693-698`) and runs only when the main loop reaches `Core::getSystemScheduler().tick()` (`src_network/target/NetRadio.cpp:139`). In each wake it:
- polls the radios and, if a packet arrived in the slot that is ending, reads the FIFO and runs the receive handler (`FHSS.cpp:451-506`);
- advances the counters, hops, and then either
  - **RX slot next:** arms RX at once (`push()` + `startRx()`, `FHSS.cpp:686-687`): the RX window opens at the wake, about 2.5 ms before the boundary, or later if the wake is late;
  - **TX slot next:** builds the 253-byte payload (`fillTxPayload`, `FHSS.cpp:664-679`), loads the FIFO and busy-waits until the boundary (`FHSS.cpp:311-320`).

Symbols used below:

| Symbol | Meaning |
|---|---|
| `lead` | wakeup lead, 2.5 ms |
| `L_wake` | how late the FHSS task starts after its release. Set by the main loop: the longest stretch between two `tick()` calls. |
| `T_arm` | SPI work from the wake until `setRx` has reached the radio(s) |
| `T_det` | how much of the preamble may be missed while detection still works (a few symbols, roughly 0.05-0.15 ms at SF5) |
| `C` | sync latency compensation (900 µs) |
| `ε` | offset of the follower's slot grid against the master's true grid: `ε = T_txstart + L_rx - C`. `T_txstart` is setTx → preamble on air. `L_rx` is the true preamble start → (DIO1 RX_DONE − computed ToA). `C` too large means ε < 0: the RX module runs early. |

The follower's sync estimate is the RX_DONE timestamp minus the computed time on air minus `C` (`Sx1280_Direct.cpp:559-566`, `FHSS.cpp:226`). The ToA formula is the datasheet one (`SX128XLT.cpp:4982-5011`, 5212-5228). The DIO1 time comes from the ISR (`NetRadio.cpp:34-55`), so main-loop latency does **not** corrupt the timestamp. Both drain paths (`NetRadio.cpp:128-138` and `Sx1280_Direct.cpp:363-372`) use the ISR capture.

**Downlink (TX module receives, RX-slot arming on the S3):** the RX module's preamble starts at `B + ε + T_txstart_rx`. The TX module listens from `B - lead + L_wake_tx + T_arm_tx`. The slot is lost if

```
L_wake_tx  >  M_down = lead - T_arm_tx + ε + T_txstart_rx + T_det
```

**Uplink (RX module receives):** the TX module's preamble starts at `B + T_txstart_tx`. The RX module listens from `B + ε - lead + L_wake_rx + T_arm_rx`. The slot is lost if

```
L_wake_rx  >  M_up = lead - T_arm_rx - ε + T_txstart_tx + T_det
```

Consequences:
- `M_down + M_up` is fixed by `lead`. **`C` only moves margin from one direction to the other. Main-loop latency eats each side's own margin.** With `C` too large (ε < 0), the RX module transmits early and the TX module's margin shrinks, while the uplink margin grows. That is the observed direction of the asymmetry.
- Example with an assumed `T_txstart + L_rx ≈ 0.1-0.3 ms` (not measured): at `C = 0.9 ms`, ε ≈ -0.6..-0.8 ms, so `M_down ≈ 1.6-1.9 ms` and `M_up ≈ 2.9-3.2 ms`.
- **Late packets are harmless** in this profile, unlike FPV. A late downlink packet only has to finish before the TX module's next wake plus grace, `B + 17.5 ms` up to `B + 19.5 ms` (`FHSS.cpp:475-497`). That leaves about 8 ms of slack. A late RX-module transmission (its own TX-slot work overrunning the lead) therefore does not lose downlink slots. It only adds jitter.
- Missed-slot catch-up (`FHSS.cpp:593-629`) needs a wake more than one slot (20 ms) late. That is not expected in steady state.

## 2. Inventory: TX module (`src_network/target_tx/network_main.cpp`, ESP32-S3)

`loop()` runs every section in sequence on every pass (`network_main.cpp:186-275`). `tick()` is called once per pass, at the top. **The FHSS wake latency is therefore up to the sum of all sections in one pass**, plus whatever was in progress when the release passed.

| # | Section | Lines | Work | Estimate (S3) | When |
|---|---|---|---|---|---|
| T1 | `WifiUpload::loopWifiUpload()` | 187; `src/WifiUpload.cpp:147-175` | `digitalRead` of the button. WiFi is not started (the task is paused, `WifiUpload.cpp:64-68`). | ~1 µs | every pass |
| T2 | `netRadioLoop()` minus FHSS | 193; `NetRadio.cpp:128-140` | 2 DIO1 drains, scheduler tick over 3 tasks (FHSS, WiFi, SettingsSync) | 3-10 µs | every pass |
| T3 | FHSS **RX-slot** wake (arming) | `FHSS.cpp:449-701` | pull of the TX radio (TX done), slot handler (`network_main.cpp:172-180`, no SPI), hop on the hop slot (`setMode` + `setRfFrequency` per radio), echo drain of the passive radio (`Sx1280Diversity.cpp:113-131`), `setRx` per radio | 0.1-0.25 ms (= `T_arm_tx`) | every 40 ms |
| T4 | FHSS **TX-slot** wake | `FHSS.cpp:449-681`, `311-320` | downlink packet: pull of 2 radios, **2 by-value copies of a 255-byte `ListArray`** for a timestamp (`Sx1280Diversity.cpp:384`), FIFO read of 255 B over SPI at 8 MHz (~0.3 ms), 2 more copies (`Sx1280Diversity.cpp:449`), copy (`FHSS.cpp:505`), copy (`FHSS.cpp:406`), mailbox copy (`NetAirAccess.hpp:126-136`). Payload: aggregator purge/pack + bitwise CRC-32 over 249 B (`Crc.cpp:17-24`, ~40-50 µs), 253-byte element copy (`FHSS.cpp:676-678`), copy (`FHSS.cpp:289`), trailer XOR over 254 B, FIFO write of 257 B (~0.3 ms), `setTxParams`. Then **busy-wait to the boundary**. | 0.9-1.3 ms work, then busy-wait; the task returns at the boundary | every 40 ms |
| T5 | `usbLink.poll()` #1 | 195; `lib/ExVectrNetwork/src/link/StreamFrameLink.cpp:23-45` | RX: **unbounded** loop until `readable()==0`, 64-byte chunks, per byte `Serial.available()` + `Serial.read()` = 2 FreeRTOS queue calls (`HWCDC.cpp:683-710`); per frame COBS + bitwise CRC-16 + `Node::onFrame` → route → aggregator or USB ring. TX: one `Serial.write` of ≤ free ring (256 B), non-blocking (see 2.1). | idle 5 µs. RX ~2-3 µs/byte + 10-30 µs/frame → a 256-byte burst ≈ 0.6-0.9 ms. TX 10-30 µs. | every pass |
| T6 | host-attach edge | 198-200 | `Serial` bool → SOF check (`HWCDC.cpp:268-280`); one LOG_TEXT on the edge | ~1 µs | every pass |
| T7 | CRSF drain + frame loop | 202-234; `src/CRSF.cpp:174-184`, `133-172` | `uart` mutex + 16-byte `readBytes` chunks. Per RC frame: CRC-8, `onCrsfRcFrame` = 2 × `Node::send` (air aggregator replace + USB COBS/CRC-16, `NetworkRouter.cpp:75-87`), `nextHandsetReply` (`localAirStats`, 64-entry LQ loop, `NetworkRouter.cpp:93-105`), queue reply. **The half-duplex reply is sent synchronously inside `drainInput()` right after the next handset frame, with a busy-wait `flush()`** (`CRSF.cpp:163-168`, `283-299`; `esp32-hal-uart.c:1474-1486`): LINK_STATISTICS 14 B ≈ **0.35 ms**, DEVICE_INFO 31 B ≈ **0.78 ms**, GPS/ATTITUDE/BATTERY 10-17 B ≈ 0.25-0.45 ms. | RC frame only: 30-60 µs. With a reply: +0.25-0.8 ms. | handset frames every ~4 ms (period not measured); replies: LINK_STATISTICS 10/s, telemetry up to 10/s per kind while HANDSET_TELEM is fresh, one DEVICE_INFO per ping |
| T8 | temporary CRSF/FHSS diagnostic | 236-252 | 2 × `snprintf` + 2 LOG_TEXT (CRSF counts; FHSS missed/grace/resync counters and FHSS LQ) | ~60 µs | every 5 s |
| T9 | `airAccess.process()` | 253; `NetAirAccess.hpp:139-155`, `AirFrameLink.hpp:71-89` | bitwise CRC-32 over 249 B, entry walk, per frame `Node::onFrame` → USB COBS + CRC-16 into the 1024-byte ring | 0.05-0.1 ms + 10-20 µs/frame (≤ ~0.6 ms for a packet full of small frames) | 25/s |
| T10 | `setStatus` / `netRadioStatus` | 254; `NetRadio.cpp:154-172` | getters | ~2 µs | every pass |
| T11 | dynamic power | 257-262 | `DynamicPower::update`, `setTxPower` stores a value only (`Sx1280_Direct.cpp:157`) | ~2 µs | 5/s |
| T12 | `router.poll()` | 264; `NetworkRouter.cpp:56-73` | heartbeat 1 Hz, LINK_STATS 2 Hz (`localAirStats`, double maths in software, encode, 2 links) | 2 µs; 20-50 µs when due | every pass |
| T13 | `usbLink.poll()` #2 | 265 | as T5, RX usually empty; flushes what T7/T9/T12 queued | 10-30 µs | every pass |
| T14 | reboot / upload triggers | 267-274 | compares; `rebootNow()` (`delay(20)` + restart, `network_main.cpp:137-143`) only on a LINK_REBOOT command | ~1 µs | every pass |

Typical pass: 20-100 µs. With an air packet: 0.1-0.3 ms. A plausible worst-case coincidence (USB burst + CRSF DEVICE_INFO reply + full air packet + LINK_STATS due): **about 1.5-2 ms**, the same order as `M_down`.

### 2.1 Serial semantics relied on (S3)

| Port | Behaviour | Where |
|---|---|---|
| USB `Serial` = HWCDC (`ARDUINO_USB_MODE=1` from the board JSON, `ARDUINO_USB_CDC_ON_BOOT=1`) | `availableForWrite()` = free bytes in a 256-byte TX ring. `write()` with TX timeout 0 (`network_main.cpp:151`): `xRingbufferSend(..., 0)`, at most 20 failed tries, then it returns short. With no host it uses the FIFO-replacement policy. **Never blocks** when bounded by `availableForWrite()`. RX: one byte per `xQueueReceive`. | `HWCDC.cpp:420-422, 528-621, 683-722` |
| CRSF UART1 | TX software buffer 0, so `write()` goes into the 128-byte HW FIFO (a reply always fits). `flush()` busy-waits until the line is idle, holding the UART mutex. | `HardwareSerial.cpp:143`, `esp32-hal-uart.c:1474-1486` |
| Cores | `loopTask` on core 1 (`ARDUINO_RUNNING_CORE=1`), WiFi never started. On core 1 only the USB-Serial-JTAG and UART ISRs run (µs each). No millisecond-scale preemption is expected. | board JSON |

## 3. Inventory: RX module (`src_network/target_rx/rx_network_main.cpp:117-140`, ESP8285 @160 MHz)

| # | Section | Lines | Work | Estimate (8285) | When |
|---|---|---|---|---|---|
| R1 | `loopWifiUpload()` | 118 | button read | ~2 µs | every pass |
| R2 | `netRadioLoop()` minus FHSS | 124 | 1 DIO1 drain + tick | 5-15 µs | every pass |
| R3 | FHSS **RX-slot** wake | `FHSS.cpp` | pull (TX done), hop, `setRx` | 0.1-0.2 ms (= `T_arm_rx`) | every 40 ms |
| R4 | FHSS **TX-slot** wake | `FHSS.cpp` | as T4 with one radio: 1 copy (`Sx1280Diversity.cpp:384`), 255 B SPI read, 2 copies (`:449`), 2 copies (`FHSS.cpp:505, 406`), mailbox, aggregator + CRC-32 (~60-120 µs), copies (`FHSS.cpp:676-678, 289`), 257 B SPI write. Each 255-byte `ListArray` copy = 9 `new[]`/`delete[]` (doubling from 1, `list_array.hpp:47-51, 210-238, 254-265`) + ~770 byte moves ≈ **60-150 µs** under umm_malloc. | **1.5-2.5 ms**: close to or over the 2.5 ms lead even with `L_wake_rx = 0`. An overrun makes the downlink TX late (harmless for loss, see section 1, but jitter). | every 40 ms |
| R5 | `uartLink.poll()` #1 | 126; `StreamFrameLink.cpp:23-45` | RX: **unbounded**, per byte `Serial.available()` + `Serial.read()` (UART IRQ off/on, FIFO → buffer), COBS + CRC-16 (~0.5 µs/byte), per frame `Node::onFrame` + aggregator (30-60 µs). TX: ≤ free HW FIFO (128 B), non-blocking. | 2-4 µs/byte. Typical backlog per pass 0-150 B (≈115 B accumulate during a 2.5 ms FHSS TX wake at 460800) → 0-0.5 ms. A full 1024-byte RX buffer (after a ≥22 ms stall or a vehicle burst) → **2.5-4 ms**. | every pass |
| R6 | `airAccess.process()` | 127 | CRC-32 (60-120 µs) + per frame UART COBS/CRC-16 (30-60 µs/frame) | 0.1-0.3 ms | 25/s |
| R7 | `setStatus` | 128 | getters | ~3 µs | every pass |
| R8 | `router.poll()` | 129; `RxRouter.cpp` | heartbeat 1 Hz, LINK_STATS 5 Hz (64-entry LQ loop with software `%`, double maths without FPU, 2 links) | 5 µs; 0.1-0.2 ms when due | every pass |
| R9 | `uartLink.poll()` #2 | 130 | flush ≤128 B | 20-50 µs | every pass |
| R10 | reboot / upload triggers | 132-139 | compares | ~2 µs | every pass |
| R11 | SDK between passes | ESP8266 core `loop_wrapper` | WiFi is off at boot in core 3.1.2 (`core_esp8266_main.cpp:406-409`), so SDK housekeeping only | not quantified; small | every pass |

### 3.1 Serial semantics relied on (ESP8266)

- `Serial.availableForWrite()` = free bytes in the 128-byte hardware FIFO. There is no software TX buffer (`uart.cpp:555-561`). `write()` busy-waits per byte only when the FIFO is full (`uart.cpp:511-516, 533-551`), so the bounded writes in `UartStream` never block. The 1024-byte "TX ring" in `network-mode.md` is `StreamFrameLink`'s own ring.
- Interrupts are briefly disabled in umm_malloc and in UART reads. This shifts DIO1 ISR timestamps by microseconds only.
- If WiFi upload is started while running (button held 3 s), the SDK WiFi stack runs next to the link with ms-scale stalls. This is outside the bench symptom.

## 4. Can these sections delay the FHSS slot task, the DIO1 handling or the RX window on the TX module?

- **FHSS slot task / RX-window opening: yes.** Every TX-module section T5-T14 runs between two `tick()` calls. The release of the RX-slot wake passes at a random phase relative to the loop, so `L_wake_tx` is up to one full pass, plus a CRSF reply if one is in progress. The relevant tail contributors are T7 (blocking half-duplex `flush()`, 0.25-0.8 ms, asynchronous to FHSS), T5/T13 (unbounded USB receive), and T9 (frame fan-out to USB). Their **sum** can reach `M_down` (about 1.6-1.9 ms at `C = 900 µs` under the assumption in section 1). Their individual values cannot.
- **DIO1 timestamps: no.** They are taken in the ISR. The TX module does not use them for sync at all (`FHSS.cpp:237`, only `isRxSide` filters the phase).
- **TX-slot punctuality on the TX module: no measurable risk.** T4 has 0.9-1.3 ms of work inside a 2.5 ms lead on the S3.
- **RX module:** R4 alone can overrun the lead on the ESP8285, and R5 can add milliseconds after a backlog. This makes the downlink TX late (not lost) and the uplink RX window late (`M_up` is the larger margin at `C = 900 µs`).

**Assessment, stated plainly:** from these estimates, the TX module's typical pass is well under 1 ms. Main-loop latency alone would produce occasional downlink losses, not 60-80 % of them, unless one of the following holds:
1. the loop is much slower on hardware than estimated (measure: section 8, M1/M2);
2. ε is more negative than assumed, so `M_down` is small and even short loop passes lose packets (the `C` sweep shows this; section 8, M4 links the two);
3. the losses are not timing at all: LoRa CRC errors, trailer mismatches, false-preamble locks (implicit-header SF5 demodulates a full 10.56 ms frame after a false detect and is deaf for that time), or diversity accounting. **None of these are counted today.** "No bad packets" only covers `AirFrameLink` CRC-32 rejects (`AirFrameLink.hpp:71-77`). LoRa CRC errors are dropped silently in `Sx1280_Direct.cpp:343-350`, and trailer mismatches in `FHSS.cpp:341-343`.

The measurement plan separates these three cases quickly.

## 5. Comparison with the FPV firmware

| Aspect | FPV (`src/`) | Network mode (`src_network/`) |
|---|---|---|
| Loop structure | `loop()` = DIO1 drain + **one scheduler task per pass** (`src/main.cpp:565-572`; `scheduler2.cpp:167-201` runs exactly one task). All work is in tasks. FHSS has priority 5000 (`FHSS.cpp:22`) and pseudo-priority = misses × 10 + priority (`scheduler2.cpp:90`), so it wins as soon as it is released. **Worst-case FHSS latency = the longest single task.** | ~10 sections run outside the scheduler, in sequence, every pass. `tick()` once per pass. **Worst-case latency = sum of the sections.** |
| CRSF input | `CRSFRcInput`: 2 ms periodic task, `taskCheck()` pulls the deadline forward when bytes arrive (`src/CRSFRcInput.cpp:144-151, 190-199`). Same blocking `flush()` (shared `CRSF.cpp:297`). | Same CRSF class, polled in the loop every pass (`network_main.cpp:202-234`) plus `nextHandsetReply()` per RC frame. |
| Lateness tolerance of RX arming | Preamble ≈ 1.9 ms (SF7/BW800); a late arm still catches most of it. The TX module has 1 RX slot in 15 (role period 15). | Preamble ≈ 0.32 ms: almost no late-arm tolerance. The TX module has 1 RX slot in 2. |
| TX payload | Pre-queued outside the slot by the `OtaQueuePump` task (`src/DatalinkCommandManager.cpp:122-151`). FHSS copies ≤ 9 bytes inside a 1 ms lead. | Built inside the 2.5 ms lead (pull mode, `FHSS.cpp:664-679`, `NetAirAccess.hpp:111-122`): aggregator + bitwise CRC-32 + 253-byte copies. |
| Packet size in shared code | 12 bytes: the by-value `DataPacket` copies in FHSS, diversity and driver are cheap. | 255 bytes: each by-value copy costs 9 heap allocations + ~770 byte moves (6 copies per received packet on the TX module, 5 on the RX module, 1 more per transmitted packet). |
| Host stream I/O | none (serial protocol removed) | USB CDC (TX) and UART 460800 (RX) through `StreamFrameLink::poll()` with an unbounded receive loop (`StreamFrameLink.cpp:26-32`), per-byte `available()`/`read()` calls |
| Timing instrumentation | Missed-slot counter and maximum RC gap sent over the air (`SlotStats`, `docs/air-protocol.md:118,188`); `DebugMonitor` with DIO1 edge rates (`src/main.cpp:265-366`); OTA sync-offset tuning (`DatalinkCommandManager.cpp:517-525`) | the temporary bench diagnostic (FHSS missed/grace/resync every 5 s) and the compile-time compensation overrides were removed again; no wake-lateness or loop-time numbers, no counters in LINK_STATS |
| Sync compensation | 900 µs, calibrated for SF7/BW800 and FPV latencies, where late packets are the danger (8 ms ToA in a 10 ms slot) | same 900 µs; in this profile early packets are the danger (section 1) |
| WiFi | task paused unless triggered | same (`WifiUpload.cpp:64-68`) |

Differences that matter for slot timing, in order: (1) loop structure (sum vs max); (2) RX-arm tolerance (0.32 ms vs 1.9 ms preamble); (3) no instrumentation; (4) payload building and 255-byte copies inside the lead; (5) unbounded stream receive; (6) CRSF reply busy-wait at a random phase with 2× more RX slots exposed.

## 6. Ranked fixes

All of these are untested proposals. Fixes 1-3 touch only `src_network/` (FPV unaffected). Fixes 4-6 touch shared libraries and need the FPV golden trace (`pio test -e native_fhss`).

| Rank | Fix | Where | Expected effect | Risk |
|---|---|---|---|---|
| 1 | **Guard window plus interleaved ticks.** Add `int64_t netRadioNsUntilWake()` = `netFhss().getRelease() - Core::NowNs()` (`getRelease()` is public, `scheduler2.hpp:132`). In both loops, call `netRadioLoop()` between sections, and skip a non-RF section (USB/UART poll, `process()`, `router.poll()`, diagnostic) while the next wake is closer than a guard (start at 1.5 ms; at most ~16 ms of every 20 ms remain for the rest). | `network_main.cpp:193-274`, `rx_network_main.cpp:124-139`, `NetRadio.cpp/.hpp` | Brings back FPV's property: `L_wake` ≤ the longest single section instead of the sum. On the TX module that is ≈ 0.1-0.3 ms outside CRSF replies, so most of `M_down` is recovered. | Low. Frames are delayed by ≤ the guard. The mailbox holds one packet and the next one comes 40 ms later, so no overrun. No protocol change. |
| 2 | **Gate the CRSF handset reply.** Call `nextHandsetReply()` / queue DEVICE_INFO only while the next FHSS wake is more than (handset period + reply time + margin) away, for example 6 ms. Otherwise skip: the telemetry scheduler does not advance, so the frame goes out one handset frame later. | `network_main.cpp:211-215, 218-231` | Removes the 0.25-0.8 ms busy-wait (`CRSF.cpp:297`) from the window before every FHSS wake. LINK_STATISTICS stays at 10 Hz (≈ 5 handset frames per 20 ms, ~3 of them outside the gate). | Low. Somewhat lower telemetry rate to the Pocket. Needs the handset frame period (from the existing diagnostic). Alternative: a non-blocking half-duplex turnaround in shared `CRSF.cpp` (medium risk, FPV affected). |
| 3 | **Bound stream receive work per pass.** Make `UsbStream::readable()` / `UartStream::readable()` return `min(available, budget)` with a per-pass budget (e.g. 128 B), and use bulk reads (`HWCDC::read(uint8_t*, size_t)`, `HWCDC.cpp:712-722`; the ESP8266 `HardwareSerial` bulk read) instead of per-byte `available()`+`read()`. Library alternative: a byte limit in `StreamFrameLinkBase::poll()`. | `network_main.cpp:51-60`, `rx_network_main.cpp:44-53` (or `StreamFrameLink.cpp:26-32`) | Caps T5/T13/R5 at ≈ 0.2-0.4 ms per call, including the 2.5-4 ms full-buffer case on the ESP8285. | Low. 1024 B of UART RX buffer covers 22 ms at 460800; 128 B per pass is far above line rate. |
| 4 | **Remove the 255-byte by-value copies from the slot path.** `Sx1280Diversity.cpp:384`: a full packet copy just to read `.timestamp` (add a timestamp getter). `:449` (by-value + assignment). `FHSS.cpp:505` (`getRxPacket()` by value; return a const reference in `Sx1280_DirectI`, `Sx1280_Direct.cpp:68`). `FHSS.cpp:406` (copy to drop the trailer; pass pointer/length). `FHSS.cpp:289` (append the trailer in place: `packetToSend` is cleared at `:322` anyway). | ExVectrLink + ExVectrNetwork submodules | ≈ 0.1-0.2 ms (S3) and **0.4-0.8 ms (ESP8285)** less work inside the 2.5 ms lead; less heap churn. On the RX module this makes the downlink TX start punctual. On the TX module it does not touch RX arming, so the direct effect on downlink loss is small. | Medium: interface change in shared driver headers; FPV must stay bit-identical (golden trace). |
| 5 | **Faster CRC-32** (nibble table, 64 B of flash) instead of bitwise. | `lib/ExVectrNetwork/src/wire/Crc.cpp:17-24` | ≈ 4× faster: saves ≈ 50-90 µs (ESP8285) inside the lead and in `process()`. | Low (host test vectors). |
| 6 | **Open the RX window independently of the loop** for the network profile only: arm RX for the next RX slot as soon as the own TX is done (the channel is known), or use a larger lead for RX slots only. A runtime profile parameter, so FPV is unchanged. | `FHSS.cpp:661-698`, `AirProfile.hpp` | Makes `M_down`/`M_up` nearly independent of main-loop latency (several ms). | Medium-high: a longer listening window means more false-preamble locks (implicit SF5 is then deaf for up to 10.56 ms). Changes FHSS. Only after M3 shows the false-lock rate. |
| 7 | Build the TX air packet in the main loop shortly before the wake (expiry still evaluated against the slot start), not inside the lead. | `NetAirAccess.hpp:111-122`, `FHSS.cpp:664-679` | ≈ 0.1-0.2 ms less lead work. | Medium (design statement in `network-mode.md` changes). Low value. |
| 8 | Remove the temporary CRSF diagnostic once it has served its purpose. | `network_main.cpp:236-252` | negligible (5 s period) | none |

Before applying 1-3: run M1/M2. If the TX module's RX-arm lateness never comes near `M_down` while the downlink stays at 20-40 %, the cause is not main-loop timing (case 2 or 3 in section 4). Fixes 1-3 are still cheap hygiene that restores the FPV discipline.

## 7. What already avoids trouble (no action)

- USB and UART writes are bounded by `availableForWrite()` and cannot block (sections 2.1, 3.1). `UsbGate` drops frames while the ring holds more than 256 bytes (`network_main.cpp:89-92`).
- No `delay()`, `yield()` or Serial prints in either loop. WiFi is off unless upload mode is triggered.
- The receive handler inside the FHSS task only copies into the mailbox (`NetRadio.cpp:120-125`). Router and stream work happens in `process()`.
- `startRx()`/`push()` skip redundant SPI (`Sx1280_Direct.cpp:40-62, 226-282`). The FIFO is loaded before the busy-wait, so only `setTx` remains at the boundary (`FHSS.cpp:311-320`).

## 8. Measurement plan (hardware only, quick)

All counters are reported as one LOG_TEXT every 2 s (text ≤ 233 bytes, `lib/ExVectrPackets/gen/ExVectrPackets/gen/common.hpp:191`). On the TX module it goes to USB. On the RX module, `router.log()` goes to node 0 over the air (small, fine). Reset the max values after each report.

**M1 — FHSS wake lateness per slot role (both firmwares, the key number).** In `netRadioLoop()` before `tick()`:

```cpp
const int64_t rel = netFhss().getRelease();   // read BEFORE tick: Task_Periodic::taskRun overwrites it (task_types.cpp:29-41)
const int64_t now = Core::NowNs();
if (now >= rel) gPendingLateNs = now - rel;    // FHSS wins the tick whenever it is released
const int64_t t0 = now;
Core::getSystemScheduler().tick();
if (now >= rel) gLastFhssRunNs = Core::NowNs() - t0, gLastRel = rel;
```

Register a slot handler in `netRadioSetup()`. It runs inside the wake (`FHSS.cpp:579`). `isRxSlot == false` means a TX slot just ended, so this wake **arms RX**: put `gPendingLateNs` into the RX-arm statistics. Otherwise it goes into the TX-prep statistics. Report the maximum and the counts above 0.5 / 1.0 / 1.5 / 2.0 ms. For TX-prep wakes also report the overrun `max(0, (t0 + gLastFhssRunNs) - (gLastRel + 2.5 ms))`: the busy-wait ends at the boundary, so a task that returns later than the boundary shows how far `startTx` slipped past it (approximately; the return includes the few µs after `startTx`).

**M2 — per-section loop time.** Wrap each loop section (T5, T7, T9, T12, T13; R5, R6, R8, R9) and the whole pass in `micros()`. Keep the maximum and the count above 500 µs per window. On the TX module also count CRSF replies sent (frames with a queued reply) and pings.

**M3 — where downlink slots die (TX module).** Without library changes:
- per-radio DIO1 edge counters in the ISRs (as FPV `src/main.cpp:377-395`). DIO1 is masked to RX_DONE (`Sx1280_Direct.cpp:28`), which also fires on LoRa CRC errors. The passive radio also counts the module's own TX echo (about 25/s);
- `getRxPacketCount()` per radio (CRC-ok only), `netAirAccess.counters.rxFrames` (trailer OK), LINK_STATS `air_rx_ok`;
- FHSS getters: `getMissedSlotCounter()`, `getGraceRescueCount()`, `getGraceExpireCount()`, `getCounterResyncCount()`, `getDesyncCounter()` (`FHSS.hpp:142-179`). The TX module's temporary diagnostic already logs the first four (`network_main.cpp:243-249`); add the same on the RX module.

Interpretation: DIO1 edges ≈ 25/s above the echo rate but CRC-ok counts much lower → CRC errors (RF, partial reception). Edges far below 25/s → the radio never finished a reception (window opened too late or packet too early, or no signal). Trailer-OK below CRC-ok → trailer/counter rejects. Optional temporary library patch for direct counts: increment counters at `Sx1280_Direct.cpp:343-350` (CRC/header error, timeout) and `FHSS.cpp:341-343` (trailer mismatch).

**M4 — A/B runs (minutes each), with M1-M3 logged:**
1. baseline as on the bench;
2. ground station disconnected (USB gate closed, T5/T13 nearly idle);
3. Pocket CRSF unplugged (no T7; the TX module still transmits heartbeat/LINK_STATS, so downlink LQ is still measurable from the TX module's LINK_STATS);
4. a build with the TX loop reduced to `netRadioLoop()` + `airAccess.process()` + `router.poll()` + `usbLink.poll()` once;
5. repeat 1 for two or three values of the parallel `C` sweep.

As `C` decreases, `M_down` grows by the same amount (section 1). If the downlink improves with `C` while RX-arm lateness (M1) stays far below `M_down`, the loop is not the limit. If downlink losses track RX-arm lateness counts above 1-1.5 ms, apply fixes 1-3.

**M5 — RX module check.** Report the M1 TX-prep overrun and the RX-arm lateness on the RX module. Expect TX-prep overruns from R4 on the ESP8285 (harmless for downlink loss, but they confirm the budget). Expect uplink losses to correlate with RX-arm lateness above `M_up`.

## 9. Open points

- `T_txstart + L_rx` (and so ε at a given `C`) is unknown. The bench sweep gives it indirectly. It should then go into `network-mode.md` "Sync" and `known-issues.md`.
- Handset CRSF frame period and ping rate on the Pocket in network mode (the diagnostic counters give both). They set the gate in fix 2.
- Whether false-preamble locks happen at a relevant rate (M3) decides whether fix 6 is viable.
