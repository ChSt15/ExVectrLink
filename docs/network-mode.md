# Network mode (TX module, USB side)

> A separately compiled firmware for the TX module (Radiomaster Pocket, diversity board) that acts as ExVectrNetwork **node 2, router**: USB CDC towards the ground station, the CRSF relay to the Pocket, and a stub where the air link will plug in. The FPV firmware is unchanged.

Last verified against commit: fb04cb8 + `feature/tx-network-router` (submodules: ExVectrNetwork 4090dce, ExVectrPackets 8170849)

Spec: workspace `docs/network-design.md` sections 3.3, 3.3.5, 3.6, 5.3 and 4 (wire format). Stack docs: `lib/ExVectrNetwork/docs/` (`node.md`, `wire.md`, `aggregator.md`), packets: `lib/ExVectrPackets/docs/`.

## What it is, and what it is not

- Env `exvectrlink_tx_module_network` (`platformio.ini`). It builds only `src_network/` plus `src/CRSF.cpp`. The FPV envs build only `src/`. So the network code is not compiled into any FPV firmware (the FPV TX env has the same size with and without it, see "Build and sizes").
- It is a **compile-time** separate firmware. Switching between FPV and network mode at runtime (Lua "Mode", reboot on change) is a later task.
- **Not part of this step:** the FHSS air link. The air side is `AirSide` (`src_network/AirStub.hpp`), a real `AirFrameLink` with a stub radio access. No radio is initialised or used (the two SX1280s are only parked: NSS high, RF switches off).
- **No WiFi, no FHSS, no sync failsafe, no Lua menu, no `DebugMonitor`, no `Serial` prints.** Nothing in the new env's code path depends on or triggers the WiFi upload code, the 60 s sync failsafe or the over-the-air test commands. Reflash over USB (the CDC handles the 1200 baud reset; otherwise hold the boot button while plugging in).

## Layout

```
src_network/
  NetCrsf.hpp/.cpp        pure: CRSF <-> RC_STATE mapping, CRSF telemetry frame builders, handset reply scheduler
  NetworkRouter.hpp/.cpp  pure: node 2 router (Node + PortMux + egress filters), RC offer, LOG_TEXT, own LINK_STATS
  LinkGuard.hpp           pure: per-link wrapper (drop by destination, refuse while the link is gated)
  AirStub.hpp             pure: AirFrameLink + stub radio access + stub local stats (the air seam)
  target/network_main.cpp ESP32-S3 glue: USB byte stream, CRSF UART, loop (the only file that includes Arduino)
test/test_network_mode/   Unity host tests (pio test -e native)
```

The "pure" files have no Arduino or ExVectrCore dependency and no heap use after construction, so they run on the host.

### Nodes and links

```
 ground station (node 0) --USB CDC, COBS+CRC16-- [ node 2, router ] --air stub (AirFrameLink)-- (later: FHSS) -- node 3 -- node 1
                                                       |
                                              CRSF half duplex, 400 kBaud, one shared pin
                                                       |
                                                  Radiomaster Pocket
```

- Node 2, role `LinkTx`, `forwarding = true`, `schemaHash = Packets::kSchemaHash`, time source injected (`esp_timer`).
- Link 0 is USB (`StreamFrameLink<1024>` on `Serial`), link 1 is the air side. Both are wrapped in a `LinkGuard`.
- The CRSF port is not a network link. `NetworkRouter::onCrsfRcFrame()` and `nextHandsetReply()` are the two calls the loop makes per handset frame.
- Own frames: HEARTBEAT 1 Hz (role 2, network version 1, schema hash, uptime; broadcast, TTL 3, so it goes out on both links), `LINK_STATS` 2 Hz broadcast (air state Off and zeros until the air link exists), `LOG_TEXT` (see below).

## CRSF to network: `RC_STATE`

`0x16 RC_CHANNELS_PACKED` becomes `RC_STATE` (port 0x10, QoS control, LATEST), exactly as in design section 3.6:

- channel k = `clamp(round((crsf_k - 992) * 1000 / 820), -1000, 1000)`, all 16 channels, integer arithmetic, half away from zero. Raw 172 gives -1000, 992 gives 0, 1811 gives 999, 1812 and above give 1000, below 172 gives -1000.
- `flags = SOURCE_OK` (bit 0), reserved 0.
- Each CRSF frame **replaces** the queued state: the air side keeps one RC_STATE slot per key `(src, dst, port)` and overwrites it in place.
- Offered at the CRSF rate, once per CRSF frame: dst 1 (towards the vehicle, the air side) and dst 0 (USB, so the ground station shows the Pocket as handset origin, read-only).
- **If no CRSF RC frame arrives for 100 ms, nothing is originated.** There is no timer path that creates or repeats RC state. The queued state expires on the air side at 100 ms (aggregator class limit) and is never released late (tested). `rcSourceFresh()` reports the SOURCE_OK condition (last frame younger than 100 ms).
- The module is generic. It knows no meaning of any channel.
- Non-RC CRSF frames (Lua parameter traffic and so on) are discarded. There is no Lua menu in this firmware.

## Network to CRSF

The handset reply is one pending frame (half-duplex, `src/CRSF.cpp`), sent right after the handset's next frame. After each RC frame the loop asks `nextHandsetReply()` for one telemetry frame (`CrsfTelemetryScheduler`):

| CRSF frame | From | Mapping |
|---|---|---|
| `0x14 LINK_STATISTICS` | `LINK_STATS` of **node 3** (uplink as the vehicle receiver sees it) + local radio | up RSSI = -rssi, up LQ, up SNR, antenna = RX winner (bits 3-0), RF profile 4, power index from local TX power, downlink RSSI/LQ/SNR from the local radio (zeros in the stub). **All fields zero while node 3 is absent** (no valid frame from node 3 for 3 s). 10 Hz, takes priority. |
| `0x21 FLIGHT_MODE` | `HANDSET_TELEM.mode_text` | NUL-terminated ASCII, at most 8 characters, non-printable bytes become `?` |
| `0x02 GPS` | `HANDSET_TELEM` | lat/lon deg x 1e7 (i32), groundspeed km/h x 10 (u16), heading centi-degrees (u16), altitude m + 1000 (u16), satellites. Big endian. |
| `0x1E ATTITUDE` | `HANDSET_TELEM` | pitch, roll, yaw as rad x 10000 (i16) |
| `0x07 VARIO` | `HANDSET_TELEM.vertical_speed` | cm/s (i16) |
| `0x08 BATTERY_SENSOR` | `HANDSET_TELEM.battery` | voltage in 100 mV, current/capacity/percent 0. **Only when the wire value is not 0** (the packet decodes 0 as NaN = unknown). |

- Round robin, one frame per handset frame: LINK_STATISTICS when due (every 100 ms), otherwise the next of FLIGHT_MODE, GPS, ATTITUDE, VARIO, BATTERY whose last send is at least 100 ms ago. Each kind is therefore sent at most 10 Hz.
- **Sources are fixed:** `LINK_STATS` is accepted from node 3 only, `HANDSET_TELEM` from node 1 only (the Pocket shows vehicle-reported state, safety.md section 1.4). Frames from other nodes on these ports are counted and ignored. A bench test that injects them over USB must set src 3 and src 1.
- **Stale data is not shown:** the HANDSET_TELEM frames stop when the last `HANDSET_TELEM` is 3 s old (the presence timeout). LINK_STATISTICS keeps going (zeroed if node 3 is gone).
- Decoded values are SI (the generated packets do the wire scaling), the CRSF scaling is in `src_network/NetCrsf.cpp`. A NaN (invalid) value converts to 0.

## Filters

Per design 3.3.3 and the Revision 1 heartbeat rule. Library egress filters (`EgressFilter`) on the air link, plus two destination drops the library cannot express (`LinkGuard`):

| Where | Rule | Why |
|---|---|---|
| Air egress | drop **node 0 heartbeats** (`addHeartbeatDrop(0)`) | The vehicle must never see the ground station over radio. Heartbeats of nodes 1, 2, 3 pass. |
| Air egress | drop **`RC_STATE` arriving from USB** (`addRule(usbLink, 0x10)`) | The ground station's virtual radio never reaches the vehicle over the radio path. The module's own RC_STATE (locally originated) is not affected. |
| Air egress | drop ports >= 0x80 | USB-only debug/telemetry range. |
| Air egress | `LinkGuard`: drop frames with **DST 0** | `Node::send` floods a unicast frame to every link when it has no fresh route. Without this the dst-0 copy of RC_STATE would go on the air whenever no ground station is attached (the normal flight case). Nothing addressed to node 0 belongs on the radio side. |
| USB egress | `LinkGuard`: drop frames with **DST 1** | Same reason: the dst-1 copy of RC_STATE would otherwise also go to USB (the ground station would see two copies). |
| USB egress | `LinkGuard` gate: refuse while no host is attached or more than 256 bytes wait in the TX ring | Nothing is held back for later: stale frames are dropped, never queued. |

Everything else is forwarded by the router rules (TTL, split horizon, no echo to the ingress link). An RC_STATE from USB reaches only the module itself (nobody subscribes to port 0x10 on node 2).

### Behaviour on USB host loss

Only forwarding to USB stops. The CRSF relay, the air side, the heartbeat on the air side and everything else keep running. There is no WiFi fallback and no mode change. When the host returns, nothing from the outage is released. A single `LOG_TEXT` "tx network router up" is sent on the first host attach. Residual: whatever the ESP32's USB hardware buffer already holds when the host vanishes is not under this firmware's control.

## Logging

No debug text on the USB port: `DebugMonitor`, `Core::initialise()` (which hooks the print functions to `Serial`) and `Serial.print*` are absent, and `CORE_DEBUG_LEVEL=0` silences the Arduino core's own logging. Logs are `LOG_TEXT` packets (port 0x02, to the ground station) through `NetworkRouter::log()`. Only the boot ROM may still print before the firmware starts: depending on the chip's console settings that text can appear on the CDC and cost the first stream frame (the receiver resynchronises at the next delimiter).

## Air stub and the seam

`AirSide` (`src_network/AirStub.hpp`) holds an `AirFrameLink<16, 8>` (the library's aggregator with latest-wins, priority classes and expiry) and a `StubAirAccess` that, every 20 ms, builds the next air packet and throws it away. It therefore behaves like a perfect-but-silent radio: queues drain, expiry and counters work, nothing leaves the module.

To plug in the air link (a later task):
1. Replace `StubAirAccess` by the FHSS adapter implementing `network::AirAccessI`: `setSource(link)`, then `buildAirPacket()` before each own slot and `onAirPacket()` for each received radio packet (see `lib/ExVectrNetwork/docs/aggregator.md`).
2. Replace `StubAirStats` by an `AirStatsSourceI` that reports the local radio (air state, LQ, RSSI, SNR, TX power). It feeds the node-2 `LINK_STATS` and the CRSF downlink fields.
3. `NetworkRouter` itself does not change: it only sees a `FrameLinkI`.
4. The decisions about WiFi upload, the sync failsafe and the over-the-air test commands belong to that task.

## Build and sizes

```sh
~/.platformio/penv/bin/pio run -e exvectrlink_tx_module_network
~/.platformio/penv/bin/pio test -e native        # host tests, 33 cases
```

- The network env needs the ExVectrNetwork library explicitly (`lib_deps = ExVectrNetwork`): the library finder does not find it from `src_network/` alone.
- `pio run` without `-e` also tries the `native` env, which is a test-only env: always pass `-e`.
- Sizes are in [build-targets-and-hardware.md](build-targets-and-hardware.md).

## Bench test with the ground station

Hardware: the new env flashed on the TX module in the Pocket, the module's USB connected to the ground-station host.

1. Flash `exvectrlink_tx_module_network`. Configure the Pocket's external module as CRSF, 400 kBaud (as for FPV).
2. In the ground station (`config/gs.example.toml`) use the `link_module` source with the `exvectrnet-1` profile and the module's CDC port.
3. Expect:
   - heartbeats from **node 2** (role link TX module) at 1 Hz, with no schema-hash mismatch flag;
   - `LINK_STATS` from node 2 at 2 Hz with air state Off and zeros;
   - a `LOG_TEXT` "tx network router up" at connect.
4. Move the Pocket's sticks and switches: the virtual radio must show them **read-only as handset origin** (RC_STATE src 2, dst 0), all 16 channels in -1000..1000, SOURCE_OK set. Centre is 0, full stick is about -1000 and +1000 (CRSF 172 and 1811 map to -1000 and 999).
5. On the Pocket (telemetry screen) the link statistics are all zero.
6. Switch the Pocket off or unplug the module bay, or stop CRSF: RC_STATE stops on USB within about 100 ms (no further frames, nothing repeated).
7. Unplug the USB cable while the Pocket is on: the CRSF relay is unaffected. Plug it back in: the ground station reconnects, receives heartbeats again, and no old RC_STATE appears.
8. Telemetry to the Pocket without an air link: inject `HANDSET_TELEM` (port 0x03, **src 1**, dst 0xFF or 2) and `LINK_STATS` (port 0x01, **src 3**) from the ground-station side over USB. Mode text, GPS, attitude, vario and battery then show on the Pocket, and the link statistics take the injected values. They fall back to zero 3 s after the last injected frame.
9. The ground station's own heartbeats and any RC_STATE it sends must not appear on the air side. Without an air link this is verified by the host tests only; check again when the air link exists.

What cannot be checked without the air link: that node 0 heartbeats and USB RC_STATE stay off the radio, real expiry on the air, and the real downlink fields. The host tests cover these against the stub with the real aggregator.
