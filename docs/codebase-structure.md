# Codebase Structure

> What lives where, and how the pieces run together.

Last verified against commit: ce2b498 + `feature/gs-requests-and-upload` (submodules: ExVectrNetwork de708ca, ExVectrPackets 4ca8fb1 local) (pins, library table, `src_network/` and test entries only; the rest of this doc is as of 4830874)

## Repository layout

```
ExVectrLink/                 (github.com/ExVeCtr/ExVectrLink parent firmware repo)
├── platformio.ini           One [env:...] per board/role, see build-targets-and-hardware.md
├── CLAUDE.md, docs/         Developer documentation (this folder)
├── src/                     The firmware application: everything board- and role-specific
│   ├── main.cpp             Global objects, debug/monitor tasks, DIO1 ISRs, setup()/loop()
│   ├── Hardware.hpp         Picks the pin map from TARGET_*, defines IS_TX_SIDE
│   ├── targets/*.hpp        Pin maps per board (namespace VCTR::HardwareConfig)
│   ├── SX1280.*             Pins, SPI and SX128XLT driver objects, reset/init of the radios
│   ├── DatalinkSetup.*      Sx1280_Direct datalink per radio (getDatalinkRadioA/B)
│   ├── DiversitySetup.*     Sx1280Diversity over the one or two radios
│   ├── FhssSetup.*          The FHSS instance (getFhss), built on the diversity object
│   ├── DynamicPowerSetup.*  DynamicPower instance and its per-slot hook
│   ├── RFSetup.*            Applies/saves RF settings (power limits, TX antenna)
│   ├── Settings.*           EEPROM-backed settings store (see settings.md)
│   ├── DatalinkCommandManager.*  OTA application layer: packet types, OTA send queue,
│   │                        link telemetry, test commands
│   ├── OtaPacketQueue.hpp   Keyed, coalescing OTA send queue
│   ├── RcLinkChannels.hpp   RC channel packet configuration (wire format!)
│   ├── CRSF.*               Generic CRSF framing (full/half duplex, ESP32 GPIO-matrix pin switching)
│   ├── CRSFRcInput.*        TX side: CRSF from handset → RC uplink, Lua menu, telemetry to handset
│   ├── CRSFRcOutput.*       RX side: RC → CRSF to flight controller, FC telemetry → downlink
│   ├── CRSFInput.*, CRSFOutput.*  Legacy predecessors (compiled, not instantiated; old A4D4 format)
│   ├── WifiUpload.*         WiFi + ArduinoOTA firmware upload mode
│   ├── WifiCredentials.cpp  (gitignored, must exist locally) ssid/password
│   ├── Config.hpp, tests/   Radio bench tests, enabled by #defines in Config.hpp (all off)
├── src_network/             ExVectrNetwork-mode firmwares (envs exvectrlink_tx_module_network, exvectrlink_rx_network_*; never built
│                            by the FPV envs): routers (node 2, node 3), air access on FHSS, CRSF <-> network mapping, upload
│                            triggers and the link upload command (LinkUploadCommand.hpp), power policy and LINK_CONFIG follower (NetPower.hpp), Router Lua menu (NetLuaMenu.hpp), ship reboot requester (ShipRebootRequester.hpp); target/ (shared radio glue), target_tx/, target_rx/ are the Arduino files (network-mode.md)
├── test/test_network_mode/  Host Unity tests: TX router, link upload command (pio test -e native)
├── test/test_air_link/      Host Unity tests: air access, RX router, end to end, FHSS maths (pio test -e native)
├── test/test_fhss/          Host Unity tests of the real FHSS on fake radios, FPV golden trace (pio test -e native_fhss)
├── test/stubs/Arduino.h     Empty Arduino.h for the FHSS host build
├── lib/                     Submodules (below) plus lib/CRSFforArduino (vendored, unused)
├── firmware.bin             A committed build artefact (see known-issues.md)
├── restore_submodules.sh    Re-adds the submodules listed in .gitmodules
└── include/, test/          PlatformIO defaults, unused
```

## `src/` vs the `lib/ExVectrLink` library

The library holds the reusable, board-independent link pieces. `src/` wires them to a board and adds the application layer.

| Library file (`lib/ExVectrLink/...`) | What it is | Used by |
|---|---|---|
| `include/ExVectrLink/datalink/FHSS.hpp`, `src/datalink/FHSS.cpp` | Slot-timed frequency hopping, sync, trailer, link quality. A `DatalinkI` and a `Task_Periodic`. Runtime parameters (`applyProfile()`, pull-mode TX hook) with the FPV values as defaults. | `src/FhssSetup.cpp` (FPV and network mode) |
| `datalink/FhssMath.hpp` | Pure schedule, trailer, sync-filter and LoRa time-on-air maths (host-tested) | `FHSS.cpp`, `src_network/` |
| `datalink/AirProfile.hpp` | `AirProfile` parameter sets (`kFpvProfile`, `kNetworkProfileV1`), `FhssTxProviderI` | network mode |
| `datalink/Sx1280Diversity.*` | Combines up to 2 `Sx1280_DirectI` radios into one (RX on all, TX on best) | `src/DiversitySetup.cpp` |
| `datalink/DynamicPower.*` | Power-level stepping logic (no radio access) | `src/DynamicPowerSetup.cpp` |
| `datalink/LinkSettings.*` | `ModulationPresets` table (currently unused by `src/`) | nothing at present |
| `datalink/PacketTypes.hpp` | OTA packet type enum and the odd-parity type-byte helpers | `src/DatalinkCommandManager.cpp` (raw values; the parity helpers are unused) |

Rule of thumb: code that both ends of the link need goes in the library. Board, pin and target logic, CRSF, WiFi and the Lua menu stay in `src/`. The library must not include from `src/`.

## Submodules

| Submodule | Pin | Used here for |
|---|---|---|
| `lib/ExVectrCore` | 18bbb09 | Scheduler (`scheduler2.hpp`), `Task_Periodic`, time (`Core::NowNs()`), containers (`ListArray`, `ListBuffer`), `HandlerGroup`, printing |
| `lib/ExVectrHAL` | 46b2460 | `HAL::DigitalIO` and GPIO interfaces |
| `lib/ExVectrArduinoPlatform` | 6fb1434 (branch ESP32-Branch) | `Platform::BusUART`, `PinGPIO`, `BusSPIDevice`, `Memory_ArduinoEEPROM` |
| `lib/ExVectrNetwork` | `feature/gs-requests-and-upload` de708ca (on `feature/air-link-fhss`: adds `EgressFilter::addDstDrop/addSrcDrop`, `Node::pinRoute`; now also `addSrcPortDrop`) | SX1280 driver (`datalink/sx1280/Sx1280_Direct`, vendored `sx12xxAL`), `DataPacket`, `DatalinkI` (FPV). New stack (network mode only): `StreamFrameLink`, `Node`, `PortMux`, `AirFrameLink`; the old network/transport/`Datalink` classes are gone. Channel plan: 20 channels, 2425–2475 MHz (`lib/ExVectrNetwork/include/ExVectrNetwork/datalink/sx1280/Sx1280_Direct.hpp:183`) |
| `lib/ExVectrPackets` | a96bd7b (local commit on `feature/link-config`: adds `LINK_CONFIG` port 0x04, schema hash 0x8D56E26D; on top of) 5f4226f (local commit on `feature/link-reboot-action`, on top of 4ca8fb1: adds `RequestAction::kLinkReboot` = 11, schema hash 0xEBD672E6; 4ca8fb1 added `kLinkUploadMode`, hash 0x3AFA3CCE; pinned on this feature branch only; Development has 8170849) | Schema-generated packets `gen/ExVectrPackets/gen/*.hpp` (network mode: `RcState`, `LinkStats`, `HandsetTelem`, `LogText`, `kSchemaHash`); legacy hand-written: RC mux packets (`radiolink/packets/RadioLinkMuxPackets.hpp`), `Telemetry_*`/`Telecommand_*`, CRSF packet classes, `PacketManager` |
| `lib/ExVectrData` | 283596f | `Memory_Manager` / `Memory_Internal` behind the settings store |
| `lib/ExVectrLink` | `feature/air-link-fhss` (on f3204bb: FHSS runtime parameters, `FhssMath.hpp`, `AirProfile.hpp`) | The link library (table above) |
| `lib/SBUS` | 3dcd5b4 | bolderflight SBUS, only for the `ENABLE_SBUS` path (no env enables it) |

`lib/CRSFforArduino` is a plain committed folder. Nothing in `src/` includes it.

## Runtime model

- **One cooperative scheduler, no RTOS tasks of our own.** `loop()` runs three steps (`src/main.cpp:724`):
  1. `WifiUpload::loopWifiUpload()`
  2. `processDio1Interrupts()`
  3. `Core::getSystemScheduler().tick()`
- **How `tick()` picks a task.** It runs at most one due task, the one with the highest `misses*10 + priority` (`lib/ExVectrCore/src/scheduler2.cpp:90`). A task that blocks stalls everything, including FHSS slot timing.
- **Tasks** derive from `Core::Task_Periodic` and add themselves to the system scheduler in their constructor. Most are global statics or function-local statics.
  - `taskCheck()` runs on every tick. Tasks use it to pull their deadline forward when work arrives, for example when CRSF bytes are readable.

| Task | Period | Where | Notes |
|---|---|---|---|
| `FHSS` | self-scheduled per slot | `lib/ExVectrLink/src/datalink/FHSS.cpp:19` | Priority 5000. Wakes 1 ms before each slot boundary (`FHSS.cpp:665-677`) and busy-waits to the exact boundary before TX (`FHSS.cpp:300`). |
| `OtaQueuePump` | 2 ms | `src/DatalinkCommandManager.cpp:125` | Moves the next queued OTA packet into FHSS's single TX slot |
| `LinkInfoSender` | 50 ms | `src/DatalinkCommandManager.cpp:370` | Link telemetry (2 Hz), RxState and SlotStats (1 Hz) |
| `CRSFRcInput` | 2 ms, woken by data | `src/CRSFRcInput.cpp:146` | `ENABLE_CRSF_INPUT` (TX module) |
| `CRSFRcOutput` | 10 ms, woken by RC packet | `src/CRSFRcOutput.cpp:104` | `ENABLE_CRSF` (RX to FC) |
| `SettingsSync` | 250 ms | `src/Settings.cpp:60` | Writes EEPROM 1 s after the last change |
| `WiFi Connection` | 1 s | `src/WifiUpload.cpp:41` | Idle until upload mode starts |
| `SyncFailsafeMonitor` | 1 s | `src/main.cpp:258` | All builds, both sides (see air-protocol.md) |
| `DebugMonitor` | 1 s | `src/main.cpp:315` | Builds without `ENABLE_SBUS`/`ENABLE_CRSF`: prints link stats on USB `Serial` |

- **Interrupts:** the only ISRs are the DIO1 rising-edge handlers (`src/main.cpp:526-538`). They only record a `micros()` timestamp.
  - `processDio1Interrupts()` converts the timestamp to `Core::NowNs()` and hands it to the radio driver (`src/main.cpp:701`).
  - The drivers can also pull it themselves (`setDio1TimestampSource`, `src/main.cpp:677`).
  - The timestamp of RX_DONE is what the RX side syncs its slot grid to.
- **Startup order** (`src/main.cpp:569-699`):
  1. Boot button held → WiFi upload mode, and nothing else starts.
  2. Five `r` bytes on `Serial` within the first second → WiFi upload mode.
  3. DIO1 interrupts attached.
  4. FHSS side set from `IS_TX_SIDE`.
  5. `Core::initialise()` (not in CRSF/SBUS output builds, because those take over `Serial`).
  6. Radio reset and init.
  7. Command manager (`initCommandManager()`: OTA receive handler and slot handler).
  8. Radio modulation (SF7 / BW 800 kHz / CR 4/8, fixed length 11).
  9. Power limits.
  10. EEPROM load and `applyRfSettings()`.
  11. Dynamic power hook.
- **Time** is `int64_t` nanoseconds (`Core::NowNs()`, `Core::MILLISECONDS`, …), as everywhere in the ExVectr framework.

## Where to put new code

| If the code is… | Put it in |
|---|---|
| Link timing, framing, diversity, power logic usable by every board | `lib/ExVectrLink` |
| A wire packet format shared with other projects | `lib/ExVectrPackets` (a cross-project change) |
| A board's pins | `src/targets/<Board>.hpp` + `Hardware.hpp` + an env in `platformio.ini` |
| CRSF, Lua menu, WiFi, application glue | `src/` |
| A persisted setting | a struct in `src/Settings.hpp` (see settings.md) |
