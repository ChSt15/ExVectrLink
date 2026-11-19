# Network mode (ExVectrNetwork firmwares)

> Separately compiled firmwares that run the ExVectrNetwork stack instead of the FPV link: the **TX module** (Radiomaster Pocket, diversity board) as node 2 and the **vehicle receiver** as node 3. Both are routers. They talk over the existing FHSS with a network-mode air profile and carry ExVectrNetwork frames end to end: USB (ground station) ↔ TX module ↔ air ↔ RX module ↔ UART (vehicle). The FPV firmwares are unchanged.

Last verified against commit: ce2b498 + `feature/gs-requests-and-upload` (submodules: ExVectrLink `feature/gs-requests-and-upload`, ExVectrNetwork `feature/gs-requests-and-upload` de708ca, ExVectrPackets 4ca8fb1 local, schema hash 0x3AFA3CCE)

Spec: workspace `docs/network-design.md`. The **Revision 1 box** at its top supersedes the text below it: FHSS (not a new TDMA) carries the air link, air profile v1, fixed full-length packets, 1:1 slot ratio, compile-time separation on the ESP8285, over-the-air upload and the 60 s sync failsafe kept. Also sections 3.2 (datalinks), 3.3 (network, filters), 3.3.5 (timeouts), 4 (wire format), 5.3. Stack docs: `lib/ExVectrNetwork/docs/` (`node.md`, `wire.md`, `aggregator.md`, `sx1280-frame-length.md`). Packets: `lib/ExVectrPackets/docs/`.

## What it is, and what it is not

| Env | Board | Node | Contents |
|---|---|---|---|
| `exvectrlink_tx_module_network` | ESP32-S3, TX module diversity pin map | 2, router, **sync master** | USB stream, CRSF relay to the Pocket, air link with diversity and dynamic power, settings store (read only), WiFi upload |
| `exvectrlink_rx_network_esp8285` | ESP8285, GenericEsp8266 pin map, 1 radio | 3, router, **sync follower** | UART stream to the vehicle, air link, WiFi upload. No CRSF, no FPV application layer |
| `exvectrlink_rx_network_genericdiversity` | ESP32, GenericDiversityELRS pin map, 2 radios | 3, as above | as above, with RX diversity |

- **Compile-time separation.** The network envs build only `src_network/` plus the few `src/` files they need (`SX1280.cpp`, `DatalinkSetup.cpp`, `DiversitySetup.cpp`, `FhssSetup.cpp`, `WifiUpload.cpp`, `WifiCredentials.cpp`; the TX module also `CRSF.cpp` and `Settings.cpp`). The FPV envs build only `src/` (default filter), so no network code is linked into them (checked with `nm`, see "Build and sizes").
- The runtime FPV/network switch on the TX module (Lua "Mode", reboot on change) is a later task (T6c). Until then, flash the firmware you need.
- **No Lua menu, no `DebugMonitor`, no `Serial` prints** in either network firmware. Logs are `LOG_TEXT` packets.

## Layout

```
src_network/
  NetCrsf.hpp/.cpp        pure: CRSF <-> RC_STATE mapping, CRSF telemetry builders, handset reply scheduler (TX)
  NetworkRouter.hpp/.cpp  pure: node 2 router (TX module)
  RxRouter.hpp/.cpp       pure: node 3 router (vehicle receiver)
  LinkUploadCommand.hpp   pure: the link-upload command (REQUEST action 10), ACKs and delay (both routers)
  NetAirAccess.hpp        pure: the air access on the FHSS slots (AirAccessI + FHSS pull hook), receive mailbox,
                          LQ, local air stats, LINK_STATS builder, TX power policy
  UploadTriggers.hpp      pure: sync failsafe, stale UART, command trigger for WiFi upload
  LinkGuard.hpp           pure: USB gate (refuse frames while no host is attached or the stream is backed up)
  target/NetRadio.*       Arduino: radios, air profile, FHSS hook-up, DIO1 ISRs (both firmwares)
  target_tx/network_main.cpp   Arduino: TX module setup()/loop()
  target_rx/rx_network_main.cpp Arduino: receiver setup()/loop()
test/test_network_mode/   host tests, TX router (pio test -e native)
test/test_air_link/       host tests, air access, RX router, end to end, maths (pio test -e native)
test/test_fhss/           host tests of the real FHSS class on fake radios (pio test -e native_fhss)
```

The "pure" files have no Arduino or ExVectrCore dependency and no heap use after construction (tested with an allocation hook).

## Air link

### Air profile v1

`kNetworkProfileV1` in `lib/ExVectrLink/include/ExVectrLink/datalink/AirProfile.hpp` (the FPV values are `kFpvProfile`, which are also the FHSS defaults):

| Parameter | Network v1 | FPV (unchanged) |
|---|---|---|
| Modulation | LoRa SF5, BW 1600 (1625 kHz), CR 4/5 | SF7, BW 800, LI 4/8 |
| Header, preamble, CRC | implicit (Fixed mode, no length prefix), 12 symbols, hardware CRC on | implicit (Limited: 1-byte length prefix), 12, on |
| Bytes on air | **255** (every frame) | 12 |
| Time on air | **10.56 ms** (datasheet formula, same as the driver's) | about 8 ms |
| FHSS slot | **20 ms** | 10 ms |
| Role period (`numTxPacketsToRx`) | **2: 1:1**, 25 packets/s each way | 15: 14 up, 1 down |
| Slots per hop | **2**: one uplink and one downlink slot per channel, a **40 ms superframe** | 4 |
| Wakeup lead | 2.5 ms (the TX slot builds and loads a 255-byte packet inside it) | 1 ms |
| Link version / hop key | 0x11 / 0x37 | 2 (`OTA_VERSION`) / 0 |
| Max payload (`getMaxPacketSize()`) | 253 | 9 |

FPV and network units can never accept each other's frames: different modulation and length (they cannot even demodulate each other), and different trailer checksum seeds (FPV 0x02/0x42, network 0x48/0x88). The host test `test_fpv_and_network_never_accept_each_other` checks the seed part.

### Frame on air

```
 0 .. 252   air packet (ExVectrNetwork wire format, design 4.6): AHDR, SFC, NETID, {LEN, frame}..., zero padding, CRC-32
 253        FHSS trailer byte 1: role counter (bits 7..2) | slot counter (bits 1..0)
 254        FHSS trailer byte 2: XOR checksum, seeded with hop key + link version + sender side bit
```

- The aggregator builds a 253-byte air packet (`maxLen = getMaxPacketSize()`), padded to full length. FHSS appends its 2-byte trailer, so every frame is exactly 255 bytes on air (constant airtime).
- The air packet carries its own CRC-32; the trailer is only for FHSS counter sync and side/version separation, exactly as in FPV.
- **Capacity:** 253 − 7 = 246 bytes of `{LEN, frame}` entries per packet, 25 packets/s: about **6.1 kB/s each way** before frame headers. A host test fills the downlink from 40 telemetry streams and gets 6000 frame bytes/s.
- If FHSS ever has nothing to send in an own slot (it always asks the access, so only if the access returns 0), it sends zero padding plus the trailer. The receiving access recognises the zero first byte (air version 0 does not exist) and counts it as a keep-alive, not as a rejected packet.

### Schedule and slot work

- The **master (TX module)** free-runs its slot grid and transmits in every slot with role counter 1. The **follower (RX module)** transmits only in role-0 slots and only while FHSS is `Synced` (never blind).
- **TX slot (both ends):** FHSS wakes 2.5 ms before the boundary and calls `NetAirAccess::fillTxPayload()` (pull mode, `FHSS::setTxPayloadProvider()`). That builds one air packet from the aggregator: class 0 (`RC_STATE`) first, then link, event, telemetry; latest-wins in place; **expiry is evaluated at the slot start**, so an `RC_STATE` that is 100 ms old when the slot starts is never sent, and an expired frame is never released later. Then the FIFO is loaded and FHSS busy-waits to the boundary.
- **Master superframe counter (SFC)** increments in every master TX slot; the follower sends the last SFC it received.
- **RX slot:** a frame that passes the FHSS trailer check is copied by the FHSS receive handler into a one-packet mailbox (`onRadioPacket()`), nothing else. The main loop calls `process()`, which hands it to `AirFrameLink::onAirPacket()` (CRC-32, version, direction, NETID, entry checks) and delivers the frames to the router. So no router or UART work happens inside the FHSS slot deadline. A second packet before `process()` replaces the first (counted, cannot happen at 25 packets/s with a running loop).
- Air packets the master sends carry DIR = master → slave, the follower's DIR = slave → master; each end rejects its own direction (a TX module never accepts another TX module).

### Sync

The FHSS sync engine is reused unchanged (air-protocol.md "Sync states"): the follower searches (hops backwards, slots stretched 1.9×), takes the phase from the first valid packet (RX_DONE timestamp minus time on air minus the sync latency compensation), filters it in `Syncing`, locks when the phase error has stayed within slot/10 for 500 ms and LQ > 0.5 (with 2 slots per hop), and drops back to `Searching` after 3 s without packets (0.5 s while `Syncing`). The drift integrator corrects crystal mismatch.

- Simulated with the real FHSS class on fake radios (`test/test_fhss`): lock from `Searching` in about 2.2 s; sync held for 30 s with a 20 ppm clock mismatch and no packet loss; after a 5 s RF block the follower is `Searching` (stopped transmitting after about 3 s) and is `Synced` again about 2.3 s after the block ends.
- The **sync latency compensation** stays at the FHSS default (900 µs). It was calibrated for the FPV modulation on FPV hardware. It is not adjustable in network mode (no Lua). If the bench shows marginal downlink or uplink LQ, it has to be recalibrated for this profile (see "Bench plan").

### Diversity and power

- **Diversity:** both radios of the TX module (and of the ESP32 diversity receiver) go through `Sx1280Diversity` as in FPV: receive on both, take the best, transmit on the best fresh radio or on the antenna pinned by the stored "TX Antenna" setting (TX module).
- **TX module power:** `DynamicPower` with the FPV rules, fed by the vehicle receiver's `LINK_STATS` (node 3: uplink RSSI, SNR, LQ as the receiver sees them) instead of the FPV `LinkTelemetry`: one step up/down per new `LINK_STATS` by `DynamicPower::update()`; one step up at the end of an RX slot after 2 lost RX slots in a row, or while node 3's `LINK_STATS` is older than 1 s (as FPV). Limits and the enable flag come from the stored FPV settings (read only). Starts at the dynamic maximum.
- **Receiver power:** fixed 20 dBm, clamped to the board maximum (12 dBm on the plain ESP8285 board), as the FPV receiver.

### Local air stats and LINK_STATS

`NetAirAccess::localAirStats()` feeds the module's own `LINK_STATS` and, on the TX module, the downlink fields of the CRSF `LINK_STATISTICS` frame:

| Field | Source |
|---|---|
| air state | FHSS: Searching → 1, Syncing → 2, Synced → 3 (the TX side reports Synced while it received anything in the last 1 s) |
| rx LQ | valid air packets (accepted by `AirFrameLink`) in the last 1 s, in % of the expected 25. **0 while Searching.** |
| RSSI, SNR | of the last valid air packet (diversity winner) |
| TX power | current power |
| antenna | bits 3-0 diversity RX winner, bits 7-4 TX radio |
| air rx ok / bad | `AirFrameLink` accepted / rejected packets |
| frames dropped | aggregator rejected + evicted + invalid |
| frames expired | aggregator expired |
| queue peak | peak aggregator fill (of 24 slots) over the current and previous 1 s window |

- TX module: `LINK_STATS` 2 Hz broadcast (USB and air).
- RX module: `LINK_STATS` **5 Hz** broadcast (UART and air), also while the air link is down (LQ 0, Searching).
- The Pocket's CRSF `LINK_STATISTICS`: uplink fields from node 3's `LINK_STATS`, downlink fields from the TX module's own stats (see "Network to CRSF").

## Routers and filters

### Nodes and links

```
 ground station (0) --USB-- [ node 2, TX module ] ~~ air (FHSS, profile v1) ~~ [ node 3, RX module ] --UART 460800-- vehicle (1)
                                   |
                       CRSF half duplex, 400 kBaud
                                   |
                              Radiomaster Pocket
```

- Node 2: role `LinkTx`, link 0 USB (`StreamFrameLink<1024>` behind the USB gate), link 1 air. HEARTBEAT 1 Hz, `LINK_STATS` 2 Hz, `LOG_TEXT`.
- Node 3: role `LinkRx`, link 0 UART (`StreamFrameLink<1024>` on `Serial`, 460800 8N1), link 1 air. HEARTBEAT 1 Hz (on both links), `LINK_STATS` 5 Hz.
- Both use one clock for the stack, FHSS and the radio timestamps: `Core::NowNs()`.

### Egress filters (ExVectrNetwork `EgressFilter`)

User decision 2026-11-19: **only the ground station's HEARTBEAT is dropped over radio** (and its RC_STATE). Every other ground-station frame passes, in particular `REQUEST` (calibration commands, link upload) towards the vehicle and the link modules, and `REQUEST_ACK` back.

| Router | Link | Rule | Why |
|---|---|---|---|
| TX (2) | air | `addHeartbeatDrop(0)` | the ground station's heartbeat never goes on the air (the vehicle must not see the ground station as a peer; it is also not a failure condition) |
| TX (2) | air | `addRule(usb, 0x10)` | RC_STATE that arrived over USB never goes to air, whatever source it claims. The Pocket (originated locally, ingress `kIngressLocal`) is the only RC source on the radio path. |
| TX (2) | air | `addDstDrop(0)` | Nothing addressed to the ground station belongs on the air (it is on USB). `Node::send` floods a unicast frame while it has no route: the dst-0 copy of the Pocket's RC_STATE would otherwise go on the air whenever no ground station is attached. |
| TX (2) | air | `portLimit 0x80` | USB-only debug ports |
| TX (2) | USB | `addDstDrop(1)` | the dst-1 copy of the Pocket's RC_STATE stays off USB (the ground station gets its own dst-0 copy) |
| TX (2) | USB | `LinkGuard` gate | refuse while no host is attached or more than 256 bytes wait in the TX ring; nothing is held back |
| RX (3) | UART | `addHeartbeatDrop(0)` | no ground-station heartbeat reaches the vehicle |
| RX (3) | UART | `addSrcPortDrop(0, 0x10)` | **no RC_STATE from node 0 ever reaches the vehicle** (second layer behind the TX module). RC_STATE from node 2 (the Pocket) passes. Other node 0 frames (REQUEST, ...) pass. |
| RX (3) | UART | `addDstDrop(0)`, `portLimit 0x80` | the vehicle is an end node and never forwards, frames for the ground station are useless there; USB-only ports |
| RX (3) | air | `addDstDrop(1)`, `portLimit 0x80` | nothing addressed to the vehicle goes back on the air; USB-only ports |

- Frames from the vehicle to node 0 (REQUEST_ACK, telemetry) pass both routers: RX air egress (only dst 1 is dropped), then the TX module's pinned route to USB (host tests `test_e2e_request_ack_from_vehicle_reaches_usb`, `test_vehicle_request_ack_reaches_usb`).
- `addDstDrop`, `addSrcDrop` and `addSrcPortDrop(src, port)` are ExVectrNetwork additions (`feature/air-link-fhss`, `feature/gs-requests-and-upload`). `LinkGuard` only implements the USB gate. `addSrcDrop` is no longer used by the firmwares.

### Pinned routes

The topology is fixed, so both routers pin their unicast routes (`Node::pinRoute`, new in ExVectrNetwork): TX module: node 1 and node 3 → air, node 0 → USB. RX module: node 1 → UART, node 0 and node 2 → air. Without pins a single frame with a forged source (for example the bench injection of `HANDSET_TELEM` with src 1 over USB) made the TX module learn "node 1 is on USB" and send the Pocket's RC_STATE there for the 3 s route lifetime, where the dst-1 drop discarded it. Pinning keeps RC_STATE on the air link whatever arrives on USB (host test `test_e2e_gs_heartbeat_and_rc_never_reach_uart_but_request_does`).

### RC state

- The TX module originates `RC_STATE` only from CRSF frames (one per frame, no timer path). If the Pocket stops, nothing is originated; the queued state expires at 100 ms and is never released later (host-tested at the slot level and end to end).
- The RX module never creates RC state. Silence on the air is silence on the UART, and the vehicle's control timeout runs. No retransmission of expired data.

## CRSF (TX module)

Unchanged from the USB-side step (feature/tx-network-router):

### CRSF to network: `RC_STATE`

`0x16 RC_CHANNELS_PACKED` becomes `RC_STATE` (port 0x10, QoS control, LATEST), design section 3.6:

- channel k = `clamp(round((crsf_k - 992) * 1000 / 820), -1000, 1000)`, all 16 channels, integer arithmetic, half away from zero. Raw 172 gives -1000, 992 gives 0, 1811 gives 999, 1812 and above give 1000.
- `flags = SOURCE_OK` (bit 0), reserved 0.
- Each CRSF frame **replaces** the queued state (latest-wins, key `(src, dst, port)`).
- Offered once per CRSF frame: dst 1 (towards the vehicle, the air side) and dst 0 (USB, so the ground station shows the Pocket as handset origin, read-only).
- The module is generic. It knows no meaning of any channel. Non-RC CRSF frames (Lua parameter traffic) are discarded.

### Network to CRSF

One telemetry frame is queued as the handset reply after each RC frame (`CrsfTelemetryScheduler`):

| CRSF frame | From | Mapping |
|---|---|---|
| `0x14 LINK_STATISTICS` | `LINK_STATS` of **node 3** + local air stats | up RSSI = -rssi, up LQ, up SNR, antenna = RX winner, RF profile 4, power index from local TX power, downlink RSSI/LQ/SNR from the local air stats. **All fields zero while node 3 is absent** (no valid frame from node 3 for 3 s). 10 Hz, takes priority. |
| `0x21 FLIGHT_MODE` | `HANDSET_TELEM.mode_text` | NUL-terminated ASCII, at most 8 characters, non-printable bytes become `?` |
| `0x02 GPS` | `HANDSET_TELEM` | lat/lon deg x 1e7, groundspeed km/h x 10, heading centi-degrees, altitude m + 1000, satellites. Big endian. |
| `0x1E ATTITUDE` | `HANDSET_TELEM` | pitch, roll, yaw as rad x 10000 |
| `0x07 VARIO` | `HANDSET_TELEM.vertical_speed` | cm/s |
| `0x08 BATTERY_SENSOR` | `HANDSET_TELEM.battery` | voltage in 100 mV, only when the wire value is not 0 |

- Round robin, one frame per handset frame, each kind at most 10 Hz. Sources are fixed: `LINK_STATS` from node 3 only, `HANDSET_TELEM` from node 1 only. Telemetry frames stop when the last `HANDSET_TELEM` is 3 s old.

## WiFi upload and failsafes

Over-the-air (WiFi, ArduinoOTA) upload stays possible on both modules (Revision 1: the receiver cannot be flashed over UART without soldering). Upload mode connects to the WiFi network in `src/WifiCredentials.cpp`, as in FPV (air-protocol.md "WiFi / OTA upload mode").

| Trigger | TX module | Receiver | Link |
|---|---|---|---|
| Boot button held at power-up | yes | yes | never starts |
| Button held 3 s while running | yes | yes | keeps running, WiFi next to it (as FPV) |
| **60 s sync failsafe**: FHSS never left `Searching` within 60 s of boot (reaching `Syncing` once cancels it for good) | yes | yes | stopped until reset |
| **Stale UART**: no valid stream frame from the UART host within 60 s of boot (one valid frame cancels it for good) | – | yes | stopped until reset |
| **Network upload command** (REQUEST action 10, target = module id, from any link) | yes | yes | stopped until reset, 400 ms after the ACK was queued |

- The sync failsafe on the TX side means "no packet from a receiver in the last second", as in FPV: a TX module powered for 60 s without its receiver goes to upload mode. Same operational caveat as FPV (known-issues.md).
- The stale-UART trigger only fires if the host was **never** seen since boot, so a vehicle that goes quiet later (in flight) never stops the receiver's link. On the bench, connect the UART host (or send any valid stream frame) within 60 s of powering the receiver.
- See "Link upload command" below for the network upload command.
- The FPV over-the-air test commands (block, force resync, sync offset) do not exist in network mode.
- The TX module keeps USB flashing (the CDC handles the 1200 baud reset; otherwise hold the boot button while plugging in).
- On entering upload mode the module sends one `LOG_TEXT` with the reason (it may not get out before the link stops).
- In upload mode the ArduinoOTA progress callbacks print on `Serial`: on the receiver that is the vehicle UART (the link is stopped then; the host resynchronises later).

## Link upload command

`REQUEST` (port 0x11, QoS event) with `action = LINK_UPLOAD_MODE (10)` and `target` = the module's own node id (TX module 2, receiver 3), implemented in `src_network/LinkUploadCommand.hpp` (subscribed on the router's port mux). User decision 2026-11-19, an explicit exception to safety.md 2.5:

- **Accepted from any link** (USB, UART, air) and **always honoured**: no armed or on-ground condition, no other gating. The module cannot know the vehicle state. The senders guard against accidents (ground station: only behind a deliberate confirmation; TX module: only from the Lua menu entry; the flight software ignores action 10).
- **Idempotent:** the first accepted request fixes the deadline (`rxTime + kUploadDelayNs`). A retry (same `req_id`, or any later request while pending) does not move it. **Every copy is answered** with `REQUEST_ACK` (`req_id` echoed, status `Done`, detail 0), addressed to the requester (dst = request src, src = module), sent from `poll()` (not inside the receive callback). The ACK queue holds 4; a full queue drops the newest (the sender retries).
- **Delay:** `LinkUploadCommand::kUploadDelayNs` = **400 ms**, the one place for the constant. Then `uploadCommandDue()` becomes true, the main loop calls `UploadTriggers::requestCommand()` and enters the existing WiFi upload mode (stops the air link until reset). On the receiver the ACK crosses the air first (next own slot, at most 40 ms plus the TX module's USB forward); if it is lost the sender retries into a silent link, which is acceptable.
- **Never triggers on anything else:** a request for another target (it is forwarded by the router as usual: a request for node 3 from the ground station crosses the TX module's air link), another action, a payload shorter than the 12-byte `Request`, or `req_id` 0 (the schema says non-zero) is ignored and not acknowledged. Frames that fail the link CRC never reach the handler. `PortMux` additionally drops a copy with a repeated `(src, seq)` within 1 s, so a retry needs a new frame (sequence number), as every sender produces.
- The ACK is a normal frame: from the receiver to node 0 it takes the air (pinned route) and the TX module forwards it to USB; to node 1 (UART host) it goes on the UART.
- The sender is not in this repo. `req_id`s should be unique per operator action and reused on retry.

## Receiver UART (vehicle side)

- `Serial` (UART0, GPIO 3 RX / GPIO 1 TX on both receiver pin maps) at **460800 8N1**, COBS + CRC-16 stream frames (design 4.2), the same `StreamFrameLink` as on USB. RX buffer 1024 bytes (ESP8285), TX ring 1024 bytes, writes never block (only what the UART accepts is written).
- On the ESP8285 UART0 is the only full UART and is shared with debug output, so this firmware prints nothing. The boot ROM prints at 74880 baud after reset; the host sees garbage bytes and resynchronises at the next 0x00 delimiter.
- Flow control: none. The aggregator's latest-wins queues absorb a vehicle that offers more than the air carries (6 kB/s).
- The ESP8285 runs at 160 MHz in this env (margin for the TX slot work inside the 2.5 ms lead).

## Build and sizes

```sh
~/.platformio/penv/bin/pio run -e exvectrlink_tx_module_network
~/.platformio/penv/bin/pio run -e exvectrlink_rx_network_esp8285
~/.platformio/penv/bin/pio run -e exvectrlink_rx_network_genericdiversity
~/.platformio/penv/bin/pio test -e native        # test_network_mode + test_air_link
~/.platformio/penv/bin/pio test -e native_fhss   # test_fhss: the real FHSS on fake radios, incl. the FPV golden trace
```

- `pio run` without `-e` also visits the test-only `native` envs: always pass `-e`.
- Sizes are in [build-targets-and-hardware.md](build-targets-and-hardware.md).

## Host tests

- `test/test_fhss` (env `native_fhss`, C++20, compiles the real `FHSS.cpp` with ExVectrCore and a fake clock and fake radios):
  - **FPV golden trace:** everything both FPV ends transmit in 3 s (bytes, channels, slot timing in µs) hashes to the value recorded from the unmodified FHSS. Any change to the FPV air format or timing fails it.
  - Network profile: parameters and trailer limits; lock from Searching; full 255-byte frames; the follower never transmits before Synced; 25 packets/s each way, own TX slot every 40 ms; sync held with 20 ppm mismatch; recovery after a 5 s block; FPV and network never accept each other.
- `test/test_air_link` (env `native`): time on air (10.56 ms, and 10.38 ms with 8 symbols/no CRC), slot budget, expected rates, slot roles, trailer seeds and counters (FPV golden values), phase wrap and filter steps; LQ counter; RC_STATE expiry evaluated at the slot start; priority order; downlink fill (≥ 5 kB/s); mailbox, keep-alive, corrupted packet, LQ, LINK_STATS fields, SFC; direction; RX router filters both ways (node 0 heartbeat and RC_STATE dropped at the UART egress, node 0 REQUEST passes, REQUEST_ACK to node 0 goes on the air), link upload command on the RX module (UART and air, ACK content and addressing, delay, repeated id, wrong target/action/short/id 0), LINK_STATS 5 Hz with LQ 0 / Searching, no RC state without air; upload triggers; power policy; no heap use; end to end (TX router + air + RX router over simulated slots): RC_STATE at 25 Hz on the RX UART, no ground-station heartbeat or RC_STATE on the UART (including a forged source) while its REQUESTs do reach it, REQUEST_ACK from the vehicle reaches USB, upload command to the RX module over the air (ACK reaches USB before the module is due) and to the TX module, Pocket off → silence, no stale release after a 5 s block.
- `test/test_network_mode` (env `native`): the TX router (CRSF mapping, filters, heartbeat, telemetry conversions, round robin), now against the real air access. Filters: node 0 heartbeat and RC_STATE (from USB) never on air; node 0 REQUEST, broadcast data pass; dst 0 never on air. Link upload command on the TX module (USB and air, ACK content and addressing, delay, idempotence, wrong target/action/short/id 0, a request for node 3 is forwarded and not executed).

## Bench plan (user, hardware)

Hardware: the TX module (Pocket) flashed with `exvectrlink_tx_module_network`, a receiver flashed with `exvectrlink_rx_network_esp8285` (or the ESP32 diversity env), a UART host on the receiver (a USB-serial adapter at 460800 8N1 into the ground station or a test script that decodes stream frames), the ground station on the TX module's USB. Connect the UART host before or within 60 s of powering the receiver (stale-UART trigger), and power both modules within 60 s of each other (sync failsafe).

1. **Sync:** power both. The receiver's `LINK_STATS` (on the UART, 5 Hz) goes from air state 1 (Searching) to 3 (Synced) within a few seconds. The TX module's `LINK_STATS` (on USB, 2 Hz) shows Synced. Leave it running for 10 minutes: no return to Searching, air rx ok counting at 25/s on both ends.
2. **LQ on both ends:** with the modules a few metres apart, rx LQ ≥ 95 % in both `LINK_STATS`; RSSI/SNR plausible. The Pocket's telemetry screen shows uplink LQ/RSSI (from node 3) and downlink LQ/RSSI (TX module).
3. **Uplink RC_STATE at 25 Hz:** count `RC_STATE` frames (port 0x10, src 2, dst 1) on the receiver UART over 10 s: about 250. Move the Pocket sticks: the values follow (-1000..1000, centre 0).
4. **Downlink throughput:** have the UART host send telemetry above the air capacity (for example 40 latest-wins streams of 40-byte frames, port 0x20..0x47, dst 0, every 40 ms). Measure the frame bytes arriving at the ground station: at least 5 kB/s (about 6 kB/s expected). Receiver `LINK_STATS` queue peak high, frames expired/dropped counting, sync unaffected.
5. **RF block of 5 s:** shield one end or unplug the antennas for 5 s. After about 3 s the receiver reports Searching and LQ 0 (still at 5 Hz on the UART); RC_STATE stops on the UART within about 140 ms of the block. After the block the link is Synced again within a few seconds and RC_STATE resumes **without a burst of old frames** (check sequence numbers and arrival times).
6. **Ground-station traffic over radio:** with the ground station connected on the TX module's USB: (a) a ground-station `REQUEST` to the vehicle (for example a calibration start) **reaches the receiver UART** (src 0, dst 1, port 0x11), and the vehicle's `REQUEST_ACK` comes back to the ground station; (b) the ground station's **heartbeat does not** appear on the UART; (c) **RC_STATE from the ground station does not** appear on the UART (enable the ground station's virtual radio on the TX-module source if the GUI allows it; only the Pocket's RC_STATE, src 2, shows); (d) the same with RC_STATE injected over USB with src 1.
7. **Pocket off:** switch the Pocket off (or stop CRSF): RC_STATE on the UART stops within about 140 ms; nothing is repeated.
8. **Injection over USB** (unchanged bench step from the USB-side task): inject `HANDSET_TELEM` (src 1) and `LINK_STATS` (src 3) from the ground-station side. The Pocket shows them, and the Pocket's RC_STATE keeps reaching the receiver UART throughout (pinned routes).
9. **Power control:** with dynamic power enabled in the stored FPV settings, the TX module's `LINK_STATS` tx power steps down to the dynamic minimum at close range and up when the receiver's LQ/RSSI drop.
10a. **Upload command:** from the ground station (behind its deliberate confirmation) send `REQUEST` action 10 with target 2: the ground station shows the `REQUEST_ACK` (src 2, status 0, detail 0), and about 400 ms later the TX module is in WiFi upload mode (reachable by espota, the link is stopped). Power-cycle. Then target 3: the ACK arrives over the air, about 400 ms after the request the receiver is in WiFi upload mode (UART host sees LINK_STATS stop). With the receiver out of range, the request gets no ACK and nothing happens. Repeat the same `req_id` several times: every copy is acknowledged, the module still enters upload mode once. A request with another target or action (for example target 1) does nothing on the modules. Do this only on the bench, never on a vehicle that is armed or flying.
10. **Upload paths:** (a) receiver powered without a TX module: after 60 s it is in WiFi upload mode and reachable by espota; (b) receiver powered with a TX module but no UART host: after 60 s in WiFi upload mode; (c) boot button held at power-up: WiFi only.
11. **Sync latency compensation check:** if uplink or downlink LQ is consistently below the other, the 900 µs default is off for this profile: report it (the value is a constant in `FHSS.hpp`, calibrated per target and modulation).
12. **FPV regression:** flash the FPV envs (`exvectrlink_tx_module_diversity` + an FPV receiver) and check RC link, CRSF telemetry and FHSS lock as before.

## Open points

- **Network upload command:** done (see "Link upload command"); only the senders are open (ground-station confirmation, Lua menu entry on the TX module).
- **Stale-UART semantics:** implemented as "no valid frame from the host within 60 s of boot, cancelled for good by the first frame". Confirm, or define a different timeout.
- **Ground-station traffic over radio:** decided 2026-11-19: only its heartbeat (and RC_STATE) is dropped, REQUESTs and everything else pass (see "Egress filters").
- Sync latency compensation for this profile (bench item 11).
- Downlink-heavy slot ratio (Revision 1, later): FHSS still has one role-reversal slot per period; a generalisation is needed.
