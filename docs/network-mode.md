# Network mode (ExVectrNetwork firmwares)

> Separately compiled firmwares that run the ExVectrNetwork stack instead of the FPV link: the **TX module** (Radiomaster Pocket, diversity board) as node 2 and the **vehicle receiver** as node 3. Both are routers. They talk over the existing FHSS with a network-mode air profile and carry ExVectrNetwork frames end to end: USB (ground station) ↔ TX module ↔ air ↔ RX module ↔ UART (vehicle). The FPV firmwares are unchanged.

Last verified against commit: ce2b498 + [2026-11-26: ExVectrPackets 3bdd417, schema hash 0xF49ACD01 (`mission_elapsed` in MISSION/AIR_MISSION), pointer-only, `feature/pin-mission-elapsed`] + [2026-11-26: ExVectrPackets a93a66d, schema hash 0x93EF9C5F, pointer-only, `feature/pin-ground-distance`] + `feature/gs-requests-and-upload` (submodules: ExVectrLink `feature/gs-requests-and-upload`, ExVectrNetwork `feature/gs-requests-and-upload` de708ca, ExVectrPackets 49d89c6 (Development, pushed), schema hash 0xF1941A87 (link nodes only forward the hash via `Packets::kSchemaHash`, no constant in this repo); `feature/cmd-uplink` (CMD_STATE uplink, RC_STATE off the air); src_network changes of `feature/network-power-cap`: Router Lua menu, one manual power setting, LINK_CONFIG)

Spec: workspace `docs/network-design.md`. The **Revision 1 box** at its top supersedes the text below it: FHSS (not a new TDMA) carries the air link, air profile v1, fixed full-length packets, 1:1 slot ratio, compile-time separation on the ESP8285, over-the-air upload and the 60 s sync failsafe kept. Also sections 3.2 (datalinks), 3.3 (network, filters), 3.3.5 (timeouts), 4 (wire format), 5.3. Stack docs: `lib/ExVectrNetwork/docs/` (`node.md`, `wire.md`, `aggregator.md`, `sx1280-frame-length.md`). Packets: `lib/ExVectrPackets/docs/`.

## Terms (naming)

| Term | Means |
|---|---|
| **Router** | The TX module in the Radiomaster Pocket = node 2 = env `exvectrlink_tx_module_network`. Sync master and **configuration master** of the link. |
| **ship radio node** (short: **ship node**) | The receiver on the vehicle = RX module = node 3 = envs `exvectrlink_rx_network_*`. Sync follower. |

User-facing text (Lua menu, `LOG_TEXT`, docs) says Router and ship node. Code identifiers, env names, file names and packet type names keep the older "TX module" / "RX module" wording; they are not renamed.

## What it is, and what it is not

| Env | Board | Node | Contents |
|---|---|---|---|
| `exvectrlink_tx_module_network` | ESP32-S3, TX module diversity pin map | 2, router, **sync master** | USB stream, CRSF relay to the Pocket, Lua parameter menu, air link with diversity and one manual power setting, settings store (read only, initial TX antenna), WiFi upload |
| `exvectrlink_rx_network_esp8285` | ESP8285, GenericEsp8266 pin map, 1 radio | 3, router, **sync follower** | UART stream to the vehicle, air link, WiFi upload. No CRSF, no FPV application layer |
| `exvectrlink_rx_network_genericdiversity` | ESP32, GenericDiversityELRS pin map, 2 radios | 3, as above | as above, with RX diversity |

- **Compile-time separation.** The network envs build only `src_network/` plus the few `src/` files they need (`SX1280.cpp`, `DatalinkSetup.cpp`, `DiversitySetup.cpp`, `FhssSetup.cpp`, `WifiUpload.cpp`, `WifiCredentials.cpp`; the TX module also `CRSF.cpp` and `Settings.cpp`). The FPV envs build only `src/` (default filter), so no network code is linked into them (checked with `nm`, see "Build and sizes").
- The runtime FPV/network switch on the TX module (Lua "Mode", reboot on change) is a later task (T6c). Until then, flash the firmware you need.
- **No `DebugMonitor`, no `Serial` prints** in either network firmware. Logs are `LOG_TEXT` packets. The Router has a Lua parameter menu (see "Lua menu (Router)"); the ship node has none.

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

### Air profile v2 (default) and v1 (rollback)

The network firmwares run **v2** (`kNetworkProfileV2`, `kNetworkProfileActive`): LoRa **SF6**, BW1600, CR 4/5, **220 bytes on air**, **15.409 ms** on air, 20 ms slots (strict margin about 1.09 ms after the 1 ms slot margin), 2.5 ms wakeup lead, hop key 0x37, link version **0x12**. Short note, bench results and rollback: [air-link-v2.md](air-link-v2.md). v1 (below) stays as `kNetworkProfileV1` and is built with `-DEXVECTRLINK_NET_PROFILE_V1`. A v1 and a v2 end never accept each other (different length and trailer seed 0x48/0x88 against 0x49/0x89). The v1 table and frame layout below use 255 bytes; for v2 read 220 bytes on air, an air packet of 218 bytes, a net frame of at most 210 bytes and `LOG_TEXT` of at most 203 bytes (the routers truncate it, `kMaxLogTextActive`).

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
- The **sync latency compensation** stays at the FHSS default (900 µs). It was calibrated for the FPV modulation on FPV hardware. It is not adjustable in network mode (the Lua menu has no entry for it). If the bench shows marginal downlink or uplink LQ, it has to be recalibrated for this profile (see "Bench plan").

### Diversity and power

- **Diversity:** both radios of the TX module (and of the ESP32 diversity receiver) go through `Sx1280Diversity` as in FPV: receive on both, take the best, transmit on the best fresh radio or on the antenna pinned by the stored "TX Antenna" setting (TX module).
- **Receiver power:** see "Power policy" below.

### Power policy

**One manual power setting for the whole network. Dynamic power is OFF in network mode** (decision 2026-11-20). The old policy (`DynamicPower` fed by node 3's `LINK_STATS`, one step up after 2 lost RX slots) is removed from the Router firmware; `NetPowerPolicy` stays in `NetAirAccess.hpp` as an unused, host-tested class. The stored FPV dynamic-power settings are ignored.

- **Legal limit: 10 mW = 10 dBm.** `kNetTxPowerDefaultDbm = 10` (`src_network/NetPower.hpp`; `kNetTxPowerCapDbm` in `NetRadio.hpp` is an alias, now only the DEFAULT, no longer a hard cap) is the power **at every boot, on both ends**.
- **Runtime ceiling = the board maximum** (`HardwareConfig::sx1280AMaxPower`: Router 27 dBm, ship node 12 dBm on the ESP8285 board). The Router's Lua menu may raise the power temporarily for testing up to it. The raise is **RAM only, never stored**: a reboot returns to 10 dBm. Requests are clamped to 0..board maximum (`clampNetPower`, `NetTxPowerSetting`).
- **The Router applies the setting to both radios** and broadcasts the very same variable as `LINK_CONFIG` (below). Its `LINK_STATS` `tx_power` field and the Pocket's CRSF LINK_STATISTICS power index are derived from that variable, so they always show what the Router really uses.
- **Ship node:** follows `LINK_CONFIG.tx_power_dbm` clamped to 0..its board maximum, applied in the main loop on the first pass after reception (a variable write; the radio uses it at its next own TX slot, within 1 slot). **Fail-safe:** if no `LINK_CONFIG` arrived for 10 s (or never), it falls back to 10 dBm, so an elevated power never outlives the Router commanding it. Its `LINK_STATS` `tx_power` is the value actually in use, so the ground station and the Router's Lua "Ship power" line can verify it.
- Unauthenticated on air: see known-issues.md.

### LINK_CONFIG (port 0x04)

Generated common packet (ExVectrPackets `feature/link-config`, schema hash 0x8D56E26D): `tx_power_dbm` (i8, dBm), `flags` (u8, reserved, sent 0, ignored). Link QoS, latest-wins, 2 bytes.

- **Sent by the Router only**, continuously at about 2 Hz (and at the next `poll()` after a change), class link, unicast to node 3, **never a one-shot event**. Latest-wins: a newer value replaces a queued one on the air side.
- **Router -> air only.** The Router's air egress filter drops `LINK_CONFIG` that came in over USB (`addRule(usb, 0x04)`, like `RC_STATE`, whatever its source), so the ground station cannot inject it; it never goes on USB. The ship node accepts it only from node 2 and only if it arrived over the **air** link (a frame on the UART is rejected, whatever source it claims) and does not forward it to the vehicle. The ground station and the flight software ignore it (the ground station gets a mirror later).
- Host tests: encode/decode bytes, 2 Hz rate and unicast, USB drop (with a forged source), accept rules, 10 s fail-safe, end to end including a link loss.

## Lua menu (Router)

The Router implements the EdgeTX/ELRS Lua parameter server (`src_network/NetLuaMenu.hpp`, pure and host-tested; wired in `src_network/target_tx/network_main.cpp`). The handset pings (~250/s) and the module answers `ParameterPing` with a DeviceInfo (name "ExVectr Router", `parametersTotal` = 7, ELRS magic serial number as FPV), answers `ParameterRead` with one 0x2B entry frame and applies `ParameterWrite`. Parameter 0 write (the ELRS status request, once a second) is answered with the 0x2E status (LQ as good/bad, connected = a fresh ship node `LINK_STATS`). The entries use their own encoder (not the 16/32 byte legacy packet classes), so labels can be longer; every entry is one chunk of at most 64 bytes. The FPV menu in `src/CRSFRcInput.cpp` is unchanged and not shared: its entry builders are member functions of `CRSFRcInput` bound to the FPV globals.

| # | Label | Type | Options / content |
|---|---|---|---|
| 1 | `Power (10mW legal)` | selection, unit dBm | `0;5;10;13;17;20;23;27` (levels above the board maximum are not offered). RAM only, boots at 10 dBm. Default marker: 10. |
| 2 | `Power (mW)` | info | the selected level in mW, for example `10 mW`, `501 mW` (the mW value of the label) |
| 3 | `TX antenna` | selection | `Auto;A;B`, RAM only (the initial value is the stored FPV setting, read only) |
| 4 | `Link` | info | air state and LQ, for example `Synced LQ 97%` |
| 5 | `Ship power` | info | the ship node's reported `tx_power` from its `LINK_STATS`, `10 dBm`, or `no data` when stale (> 1 s) |
| 6 | `Reboot router` | command | click -> confirmation `Reboot router?` -> confirm restarts the Router 500 ms later (clean software restart, LOG_TEXT first) |
| 7 | `Reboot ship node` | command | click -> confirmation `Reboot ship node?` -> confirm queues `REQUEST LINK_REBOOT` target 3 in the Router like the ground station would, repeated every 1 s until `REQUEST_ACK`, 5 tries at most (`ShipRebootRequester`). **A reboot of the ship node in flight stops RC for a few seconds.** |

- **Command protocol** (CRSF command steps): Start -> reply `ConfirmationNeeded` with the question as info; Confirm -> execute once, reply `Progress` (shown for 2 s, then Ready); Cancel or an unconfirmed click (lapses after 10 s) -> Ready; a Confirm without a click, or a repeated Confirm, executes nothing.
- Info entries (2, 4, 5) are read when the Lua script loads the field; they are not pushed. Re-open the menu to refresh them.
- **Timing:** at most one parameter frame (ping, read or write) is handled per main-loop pass, each builds at most one frame from a few `snprintf()`s and never blocks; the rest of the CRSF queue (8 frames) waits for the next pass. No parameter work runs inside the FHSS slot task. This follows the main finding of [rf-timing-audit.md](rf-timing-audit.md). Half-duplex keeps one pending reply, so a parameter reply replaces the pending telemetry frame (same as FPV).
- Not in this menu: upload mode, dynamic power, persistence, the runtime mode switch.

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

User decision 2026-11-19: **only the ground station's HEARTBEAT is dropped over radio.** Every other ground-station frame passes, in particular `REQUEST` (calibration commands, link upload) towards the vehicle and the link modules, and `REQUEST_ACK` back.

| Router | Link | Rule | Why |
|---|---|---|---|
| TX (2) | air | `addHeartbeatDrop(0)` | the ground station's heartbeat never goes on the air (the vehicle must not see the ground station as a peer; it is also not a failure condition) |
| TX (2) | air | `addRule(any, 0x10)` | **RC_STATE (port 0x10) never goes on the air**, from any ingress (GS-control cutover, `feature/cmd-uplink`). The Pocket's RC_STATE is sent to the ground station on USB only. |
| TX (2) | air | `addSrcPortDrop(1 / 2 / 3, 0x13)` | **CMD_STATE (port 0x13) crosses to the air only from node 0** (its only possible ingress is USB: the air link never egresses to itself and node 2 originates none). A forged source 1, 2 or 3 on USB is dropped (src 2 is this node: dropped as self-echo at ingress). Latest-wins in the aggregator (CONTROL class, key src/dst/port, 100 ms expiry), uplink only. |
| TX (2) | air | `addRule(usb, 0x04)` | `LINK_CONFIG` that arrived over USB never goes to air (see below). |
| TX (2) | air | `addDstDrop(0)` | Nothing addressed to the ground station belongs on the air (it is on USB). `Node::send` floods a unicast frame while it has no route. |
| TX (2) | air | `portLimit 0x80` | USB-only debug ports |
| TX (2) | USB | `addDstDrop(1)` | frames addressed to the vehicle stay off USB (`Node::send` floods a unicast frame while it has no route) |
| TX (2) | USB | `addRule(any, 0x13)` | CMD_STATE is never sent back to the ground station (uplink only; also covers a CMD_STATE from the air side) |
| TX (2) | USB | `LinkGuard` gate | refuse while no host is attached or more than 256 bytes wait in the TX ring; nothing is held back |
| RX (3) | UART | `addHeartbeatDrop(0)` | no ground-station heartbeat reaches the vehicle |
| RX (3) | UART | `addSrcPortDrop(0, 0x10)`, `addRule(any, 0x10)` | **no RC_STATE (port 0x10) reaches the vehicle from any source** (the vehicle takes no handset channels over the radio; second layer behind the TX module). Other node 0 frames (REQUEST, ...) pass. |
| RX (3) | UART | `addSrcPortDrop(1 / 2 / 3, 0x13)` | **CMD_STATE reaches the vehicle only from node 0** (src 0 preserved). A forged source on the air is dropped. |
| RX (3) | air | `addRule(uart, 0x13)` | CMD_STATE coming from the vehicle side is never put on the air (uplink only, anti-spoof, whatever source it claims) |
| RX (3) | UART | `addDstDrop(0)`, `portLimit 0x80` | the vehicle is an end node and never forwards, frames for the ground station are useless there; USB-only ports |
| RX (3) | air | `addDstDrop(1)`, `portLimit 0x80` | nothing addressed to the vehicle goes back on the air; USB-only ports |

- Frames from the vehicle to node 0 (REQUEST_ACK, telemetry) pass both routers: RX air egress (only dst 1 is dropped), then the TX module's pinned route to USB (host tests `test_e2e_request_ack_from_vehicle_reaches_usb`, `test_vehicle_request_ack_reaches_usb`).
- `addDstDrop`, `addSrcDrop` and `addSrcPortDrop(src, port)` are ExVectrNetwork additions (`feature/air-link-fhss`, `feature/gs-requests-and-upload`). `LinkGuard` only implements the USB gate. `addSrcDrop` is no longer used by the firmwares.

### Pinned routes

The topology is fixed, so both routers pin their unicast routes (`Node::pinRoute`, new in ExVectrNetwork): TX module: node 1 and node 3 → air, node 0 → USB. RX module: node 1 → UART, node 0 and node 2 → air. Without pins a single frame with a forged source (for example the bench injection of `HANDSET_TELEM` with src 1 over USB) made the TX module learn "node 1 is on USB" and send the Pocket's RC_STATE there for the 3 s route lifetime, where the dst-1 drop discarded it. Pinning keeps RC_STATE on the air link whatever arrives on USB (host test `test_e2e_gs_heartbeat_and_rc_never_reach_uart_but_request_does`).

### RC state and CMD_STATE (GS-as-control-source, `feature/cmd-uplink`)

- The TX module originates `RC_STATE` only from CRSF frames (one per frame, no timer path) and sends it to the ground station on USB (dst 0) **only**. It is never on the air and never on the vehicle UART. No `pocket_rc_to_air` flag exists: the ship node drops port 0x10 toward the UART unconditionally, so such a flag would have no effect; add it together with that drop if a profile ever needs the handset on the air.
- `CMD_STATE` (port 0x13, 14 B, node 0 -> vehicle node 1, design `docs/gs-control-design.md`) arrives on the Router's USB at 25 Hz (plus immediate on-change frames) from the ground station and is forwarded over the air as received: qos/latest flags and sequence unchanged, src 0 preserved, no parsing. The sender must use QoS control and LATEST: the aggregator classes and keys the frame by those header bits (key src/dst/port, so one slot, newest wins, expiry 100 ms evaluated at the slot start). The ship node puts it on the UART with src 0. Uplink only: both routers drop 0x13 in the other direction. Air cost: 21 B x 25/s = 525 B/s; uplink total about 611 B/s = 9.9 % of the 6.15 kB/s entry capacity (`tools/network_budget.py`).
- Not implemented (the brief asked for latest-wins only): the design's USB-ingress token bucket (50/s, burst 2). A sender faster than 25 Hz cannot raise the air load (latest-wins replaces in place), it only wastes USB and CPU.
- Silence on the air is silence on the UART, and the vehicle's command timeout runs. No retransmission of expired data. The RX module never creates command state.

## CRSF (TX module)

Unchanged from the USB-side step (feature/tx-network-router):

### CRSF to network: `RC_STATE`

`0x16 RC_CHANNELS_PACKED` becomes `RC_STATE` (port 0x10, QoS control, LATEST), design section 3.6:

- channel k = `clamp(round((crsf_k - 992) * 1000 / 820), -1000, 1000)`, all 16 channels, integer arithmetic, half away from zero. Raw 172 gives -1000, 992 gives 0, 1811 gives 999, 1812 and above give 1000.
- `flags = SOURCE_OK` (bit 0), reserved 0.
- Each CRSF frame **replaces** the queued state (latest-wins, key `(src, dst, port)`).
- Offered once per CRSF frame: dst 1 (towards the vehicle, the air side) and dst 0 (USB, so the ground station shows the Pocket as handset origin, read-only).
- The module is generic. It knows no meaning of any channel. Non-RC CRSF frames (Lua parameter traffic is handled by the Lua menu, everything else is discarded).

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

### RC stream diagnostics and CRSF parser hardening (`feature/rc-stream-instrumentation`)

The shared CRSF framing is `src/CrsfParser.hpp` (pure class, host-tested in `test/test_crsf_parser`). **Legacy mode is the default and is exactly the old `decodeByte()`** (what every FPV firmware runs; a random-stream test compares it with a copy of the original). The network TX module calls `crsf.setHardenedParser(true)`:

- 0x00 is not accepted as a frame start (FPV starts: 0xC8, 0x00, 0xEC, 0xEA, 0xEE; hardened: without 0x00). A 0x00 is what a break or the `switchToTx` constant-low RX routing looks like.
- After a bad length or CRC failure the bytes behind the first one are rescanned for a real start instead of being discarded (one corrupt byte costs one frame, not the next 2 to 64 bytes).
- A half-received frame with no new byte for 20 ms is dropped (`staleResets`). This is a poll-to-poll timeout, not the 1 ms inter-byte gap: bytes are read from the UART FIFO in chunks without per-byte timestamps, so a 1 ms gap cannot be told from a loop pause, and the rescan already covers the desync.
- The main loop uses `crsf.queuedFrames()` instead of `peekFrameType() != 0`, so a CRC-valid type-0 frame at the queue front is dropped (not a loop end), and a failed RC `read()` (frame already removed) no longer causes the next frame to be discarded unseen.

Counters leave as **LOG_TEXT** lines to the ground station at 1 Hz (no ExVectrPackets field has spare room; `LinkStats` has none and the schema is being changed elsewhere). They are dropped while no USB host is attached. Read them in the ground-station log view / recording (severity Info for `RCDIAG`, Warning for `RCFROZEN`):

`RCDIAG b= rc= lua= oth= crc= len= drop= z0= resc= stale= rcAge= anyAge= off= sf= rep=sent/replaced`

| Field | Meaning |
|---|---|
| `b` | UART bytes received (cumulative) |
| `rc` / `lua` / `oth` | CRC-valid frames: RC 0x16 / Lua 0x28, 0x2C, 0x2D / anything else |
| `crc` / `len` | CRC failures / bad-length resets |
| `drop` | bytes discarded while hunting for a frame start |
| `z0` | 0x00 seen as a tentative frame start (always 0 in hardened mode, which no longer accepts it) |
| `resc` / `stale` | rescans after a failure / stale partial frames dropped |
| `rcAge` / `anyAge` | ms since the last RC frame / last valid frame of any kind (-1 = none yet; loop granularity) |
| `off` / `sf` | `rcFramesOffered` / `rcSendFailed` of the router |
| `rep` | half-duplex replies sent / queued replies overwritten before they could go out (`write()` replacing DeviceInfo with telemetry) |

`RCFROZEN atMs= bytesIn= last16: xx ...` appears every second once RC has been silent for 200 ms and stays until RC returns; the 16 bytes are the last raw UART bytes at that moment (frozen copy).

How to read a stall: `b` still rising and `lua`/`oth` rising but `rc` flat = the handset sends no 0x16 (hypothesis 1; check `rep` replaced > 0 and the frozen bytes for a repeating 0x28 ping pattern). `b` rising, nothing valid, `crc`/`len`/`drop` rising = parser desync or line noise (frozen bytes show what is on the wire). `b` flat = nothing arrives at the UART (handset silent, or RX wedged). `rc` rising but `off` flat is impossible (same call); `off` rising with `sf` rising = the air side refuses RC_STATE, not a CRSF problem.

Cost: FPV TX-module env flash +620 bytes, RAM +80 bytes (counters, 16-byte ring, parser object); behaviour unchanged.

## WiFi upload and failsafes

Over-the-air (WiFi, ArduinoOTA) upload stays possible on both modules (Revision 1: the receiver cannot be flashed over UART without soldering). Upload mode connects to the WiFi network in `src/WifiCredentials.cpp`, as in FPV (air-protocol.md "WiFi / OTA upload mode").

| Trigger | TX module | Receiver | Link |
|---|---|---|---|
| Boot button held at power-up | yes | yes | never starts |
| Button held 3 s while running | yes | yes | keeps running, WiFi next to it (as FPV) |
| **60 s sync failsafe**: FHSS never left `Searching` within 60 s of boot (reaching `Syncing` once cancels it for good) | yes | yes | stopped until reset |
| **Stale UART**: no valid stream frame from the UART host within 60 s of boot (one valid frame cancels it for good) | – | yes | stopped until reset |
| **Network upload command** (REQUEST action 10, target = module id, from any link) | yes | yes | stopped until reset, 400 ms after the ACK was queued |
| **Network reboot command** (REQUEST action 11, target = module id, from any link) | yes | yes | clean software restart (`ESP.restart()`), 400 ms after the first accepted copy; the link restarts and re-syncs |

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
- **Idempotent:** the first accepted request fixes the deadline (`rxTime + kDelayNs`). A retry (same `req_id`, or any later request while pending) does not move it. **Every copy is answered** with `REQUEST_ACK` (`req_id` echoed, status `Done`, detail 0), addressed to the requester (dst = request src, src = module), sent from `poll()` (not inside the receive callback). The ACK queue holds 4; a full queue drops the newest (the sender retries).
- **Delay:** `LinkCommand::kDelayNs` (alias `kUploadDelayNs`) = **400 ms**, the one place for the constant. Then `uploadCommandDue()` becomes true, the main loop calls `UploadTriggers::requestCommand()` and enters the existing WiFi upload mode (stops the air link until reset). On the receiver the ACK crosses the air first (next own slot, at most 40 ms plus the TX module's USB forward); if it is lost the sender retries into a silent link, which is acceptable.
- **Never triggers on anything else:** a request for another target (it is forwarded by the router as usual: a request for node 3 from the ground station crosses the TX module's air link), another action, a payload shorter than the 12-byte `Request`, or `req_id` 0 (the schema says non-zero) is ignored and not acknowledged. Frames that fail the link CRC never reach the handler. `PortMux` additionally drops a copy with a repeated `(src, seq)` within 1 s, so a retry needs a new frame (sequence number), as every sender produces.
- The ACK is a normal frame: from the receiver to node 0 it takes the air (pinned route) and the TX module forwards it to USB; to node 1 (UART host) it goes on the UART.
- The sender is not in this repo. `req_id`s should be unique per operator action and reused on retry.

### LINK_REBOOT (action 11)

Decision 2026-11-20. `REQUEST` with `action = LINK_REBOOT (11)` has exactly the semantics above (class `LinkCommand`, one state slot per action, one shared ACK queue): target = the module's node id (TX 2, RX 3), accepted from any link, always honoured with no gating, `REQUEST_ACK` (status `Done`, detail 0, `req_id` echoed) queued first and every copy acknowledged, idempotent by `req_id` (0 invalid), executed once 400 ms after the first accepted copy. The difference: the main loop (`rebootCommandDue()`, checked right next to the upload command in `network_main.cpp` and `rx_network_main.cpp`) does a clean software restart, `ESP.restart()`, instead of entering WiFi upload mode (the TX module logs one `LOG_TEXT` first). The module boots normally and the link re-syncs; no settings are changed. The two actions are independent: a pending upload and a pending reboot do not affect each other (upload mode, once entered, wins because the link is stopped and the reboot check is skipped).

- **Unauthenticated on air**, like action 10 (known-issues.md). Wrong target or other action: ignored, not acknowledged.
- **Every sender must guard it.** A reboot of the receiver in flight stops RC to the flight controller for a few seconds (the flight controller goes into its failsafe until the link has re-synced). Senders: only behind a deliberate confirmation (ground station: danger confirm, only while the vehicle is on the ground and disarmed; TX module: only from a deliberate menu entry). The flight software ignores action 11, like action 10: it has no handler for these actions and the modules consume them before the vehicle sees them.
- Host tests: `test_network_mode` (TX: ACK, delay, idempotence, ignored cases, from air) and `test_air_link` (RX: UART and air, idempotence, ignored cases), the action 10 tests are unchanged.

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
- `test/test_air_link` (env `native`): also the power policy (default 10 dBm, ceiling clamp, `LinkConfigFollower` apply and 10 s fail-safe), `LINK_CONFIG` (bytes, 2 Hz unicast from the Router, USB drop with forged source, ship accept rules, end-to-end mirroring and link-loss fall-back), `ShipRebootRequester` (retry, ack, end to end) and `NetLuaMenu` (levels/options, every entry parsed by an independent decoder, writes, command confirmation flow). Time on air (10.56 ms, and 10.38 ms with 8 symbols/no CRC), slot budget, expected rates, slot roles, trailer seeds and counters (FPV golden values), phase wrap and filter steps; LQ counter; RC_STATE expiry evaluated at the slot start; priority order; downlink fill (≥ 5 kB/s); mailbox, keep-alive, corrupted packet, LQ, LINK_STATS fields, SFC; direction; RX router filters both ways (node 0 heartbeat dropped and no RC_STATE from any source at the UART egress, CMD_STATE from node 0 passes and from 1/2/3 is dropped, CMD_STATE from the UART never on air, downlink unaffected, node 0 REQUEST passes, REQUEST_ACK to node 0 goes on the air), link upload command on the RX module (UART and air, ACK content and addressing, delay, repeated id, wrong target/action/short/id 0), LINK_STATS 5 Hz with LQ 0 / Searching, no RC state without air; upload triggers; power policy; no heap use; end to end (TX router + air + RX router over simulated slots): CMD_STATE at 25 Hz on the RX UART, the Pocket's RC_STATE at 250 Hz never on the UART (it reaches USB), forged CMD_STATE sources 1/2/3 never on the UART, no ground-station heartbeat or RC_STATE on the UART while its REQUESTs do reach it, REQUEST_ACK from the vehicle reaches USB, upload command to the RX module over the air (ACK reaches USB before the module is due) and to the TX module, GS stream off → silence, no stale CMD_STATE release after a 5 s block.
- `test/test_network_mode` (env `native`): the TX router (CRSF mapping, filters, heartbeat, telemetry conversions, round robin), now against the real air access. Filters: node 0 heartbeat never on air; RC_STATE (any source or origin, including the Pocket) never on air and still on USB; CMD_STATE from node 0 on USB forwarded (bytes unchanged), from nodes 1/2/3 dropped, never to USB; latest-wins (one per slot, newest, no resend); 25 Hz stream one per slot; node 0 REQUEST, broadcast data pass; dst 0 never on air. Link upload command on the TX module (USB and air, ACK content and addressing, delay, idempotence, wrong target/action/short/id 0, a request for node 3 is forwarded and not executed).

## Bench plan (user, hardware)

Hardware: the TX module (Pocket) flashed with `exvectrlink_tx_module_network`, a receiver flashed with `exvectrlink_rx_network_esp8285` (or the ESP32 diversity env), a UART host on the receiver (a USB-serial adapter at 460800 8N1 into the ground station or a test script that decodes stream frames), the ground station on the TX module's USB. Connect the UART host before or within 60 s of powering the receiver (stale-UART trigger), and power both modules within 60 s of each other (sync failsafe).

1. **Sync:** power both. The receiver's `LINK_STATS` (on the UART, 5 Hz) goes from air state 1 (Searching) to 3 (Synced) within a few seconds. The TX module's `LINK_STATS` (on USB, 2 Hz) shows Synced. Leave it running for 10 minutes: no return to Searching, air rx ok counting at 25/s on both ends.
2. **LQ on both ends:** with the modules a few metres apart, rx LQ ≥ 95 % in both `LINK_STATS`; RSSI/SNR plausible. The Pocket's telemetry screen shows uplink LQ/RSSI (from node 3) and downlink LQ/RSSI (TX module).
3. **Uplink CMD_STATE at 25 Hz (GS-control):** count `CMD_STATE` frames (port 0x13, src 0, dst 1) on the receiver UART over 10 s: about 250. **No port 0x10 frame** on the UART with the Pocket on. The Pocket's `RC_STATE` still reaches the ground station on the Router USB (about 200/s).
4. **Downlink throughput:** have the UART host send telemetry above the air capacity (for example 40 latest-wins streams of 40-byte frames, port 0x20..0x47, dst 0, every 40 ms). Measure the frame bytes arriving at the ground station: at least 5 kB/s (about 6 kB/s expected). Receiver `LINK_STATS` queue peak high, frames expired/dropped counting, sync unaffected.
5. **RF block of 5 s:** shield one end or unplug the antennas for 5 s. After about 3 s the receiver reports Searching and LQ 0 (still at 5 Hz on the UART); CMD_STATE stops on the UART within about 140 ms of the block. After the block the link is Synced again within a few seconds and CMD_STATE resumes **without a burst of old frames** (check sequence numbers and arrival times).
6. **Ground-station traffic over radio:** with the ground station connected on the TX module's USB: (a) a ground-station `REQUEST` to the vehicle (for example a calibration start) **reaches the receiver UART** (src 0, dst 1, port 0x11), and the vehicle's `REQUEST_ACK` comes back to the ground station; (b) the ground station's **heartbeat does not** appear on the UART; (c) **no RC_STATE** (port 0x10) appears on the UART whatever its source; (d) `CMD_STATE` injected over USB with src 1, 2 or 3 does **not** appear on the UART, only src 0 does.
7. **GS stream stops:** stop the ground station's command stream: CMD_STATE on the UART stops within about 140 ms; nothing is repeated. Switching the Pocket off changes nothing on the UART.
8. **Injection over USB** (unchanged bench step from the USB-side task): inject `HANDSET_TELEM` (src 1) and `LINK_STATS` (src 3) from the ground-station side. The Pocket shows them, and the ground station's CMD_STATE keeps reaching the receiver UART throughout (pinned routes).
9. **Power and Lua menu:**
   a. Power both ends: both `LINK_STATS` (Router on USB, ship node over the UART) show `tx_power` 10 and the Pocket shows 10 mW. Open the Lua menu on the Pocket (System > Tools > ExpressLRS script, or the external-module script): the entries 1-7 of "Lua menu (Router)" appear, `Power (10mW legal)` shows 10, `Link` shows Synced, `Ship power` shows `10 dBm`.
   b. Raise the power to 20 dBm: the Router's `LINK_STATS` `tx_power` becomes 20 at once, the ship node's becomes 12 (board maximum) within about 0.5 s, the Pocket's power index follows. Lower to 5: both follow. RSSI at the other end moves with it (spectrum or RSSI check, power meter if available).
   c. With power raised, switch the Router off or block it (unplug its antennas or the Pocket module bay) for 12 s: the ship node's `tx_power` returns to 10 within 10 s of the last `LINK_CONFIG`. Reboot the Router: it comes back at 10 dBm, the Lua menu shows 10.
   d. `TX antenna`: Auto, A, B: `LINK_STATS` antenna field (bits 7-4) follows; B and A each still link at close range.
   e. `Reboot ship node` with confirmation: cancel does nothing; confirm shows the Ack on USB (`REQUEST_ACK` src 3) and the ship node restarts about 400 ms later and re-syncs, back at 10 dBm. `Reboot router` likewise.
   f. While the menu is open and being scrolled, watch `LINK_STATS` rx LQ on both ends: the Lua traffic must not cost packets (compare with the menu closed).
   g. Inject a `LINK_CONFIG` frame (port 0x04, src 0 and src 2, dst 3) from the ground-station USB side: the ship node's `tx_power` does not change.
10a. **Upload command:** from the ground station (behind its deliberate confirmation) send `REQUEST` action 10 with target 2: the ground station shows the `REQUEST_ACK` (src 2, status 0, detail 0), and about 400 ms later the TX module is in WiFi upload mode (reachable by espota, the link is stopped). Power-cycle. Then target 3: the ACK arrives over the air, about 400 ms after the request the receiver is in WiFi upload mode (UART host sees LINK_STATS stop). With the receiver out of range, the request gets no ACK and nothing happens. Repeat the same `req_id` several times: every copy is acknowledged, the module still enters upload mode once. A request with another target or action (for example target 1) does nothing on the modules. Do this only on the bench, never on a vehicle that is armed or flying.
10. **Upload paths:** (a) receiver powered without a TX module: after 60 s it is in WiFi upload mode and reachable by espota; (b) receiver powered with a TX module but no UART host: after 60 s in WiFi upload mode; (c) boot button held at power-up: WiFi only.
11. **Sync latency compensation check:** if uplink or downlink LQ is consistently below the other, the 900 µs default is off for this profile: report it (the value is a constant in `FHSS.hpp`, calibrated per target and modulation).
12. **FPV regression:** flash the FPV envs (`exvectrlink_tx_module_diversity` + an FPV receiver) and check RC link, CRSF telemetry and FHSS lock as before.

10b. **Reboot command:** from the ground station (behind its confirmation) send `REQUEST` action 11 with target 2: the `REQUEST_ACK` (src 2, status 0, detail 0) arrives, about 400 ms later the TX module restarts (USB re-enumerates, the Pocket shows no link for a moment) and the link re-syncs without any other action. Then target 3 (receiver on the bench, no flight controller needed, or one that is not armed): the ACK arrives over the air, about 400 ms later the receiver restarts (UART host sees LINK_STATS stop for a few seconds, then resume after re-sync). Repeat the same `req_id`: every copy is acknowledged, one restart. Out of range: no ACK, nothing happens. Other target or action: nothing happens. Never on an armed or flying vehicle (RC stops for a few seconds).

## Open points

- **Network upload command:** done (see "Link upload command"); only the ground-station sender (with its confirmation) is open. The Lua menu has no upload entry (reboot entries only).
- **Stale-UART semantics:** implemented as "no valid frame from the host within 60 s of boot, cancelled for good by the first frame". Confirm, or define a different timeout.
- **Ground-station traffic over radio:** decided 2026-11-19: only its heartbeat (and RC_STATE) is dropped, REQUESTs and everything else pass (see "Egress filters").
- Sync latency compensation for this profile (bench item 11).
- Downlink-heavy slot ratio (Revision 1, later): FHSS still has one role-reversal slot per period; a generalisation is needed.
