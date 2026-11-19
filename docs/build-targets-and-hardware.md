# Build Targets and Hardware

> Every `[env:...]` in `platformio.ini`: which board it builds for, which role it plays, and the pin map it uses.

Last verified against commit: ce2b498 + `feature/gs-requests-and-upload` (env table and sizes; submodules ExVectrNetwork de708ca, ExVectrPackets 4ca8fb1 local)

## How a build is configured

- **`TARGET_*`** picks the pin map. `src/Hardware.hpp:4-26` includes exactly one `src/targets/*.hpp`, which defines `namespace VCTR::HardwareConfig`. A build without a `TARGET_*` has no pin map and does not compile.
- **`IS_TX_SIDE`** is defined for `TARGET_EXVECTRLINKTXMODULE` and `TARGET_EXVECTRLINKTXMODULEDIVERSITY` (`src/Hardware.hpp`). It selects:
  - the FHSS role (`src/main.cpp:634-638`);
  - dynamic power from settings vs. a fixed 20 dBm on the RX (`src/RFSetup.cpp:61-79`).
  - A new handset-side target must be added to that list. Otherwise it comes up as a receiver and the link never forms.
- **`HAS_DIVERSITY`** is defined by the pin map. It means two radios are used together through `Sx1280Diversity`.
- **`HAS_RADIO_B_HARDWARE`** means a second radio is populated but only held in a safe pin state, not used (`src/SX1280.cpp:26-33`). No remaining target sets it (the removed ExVectrController board was the only one).
- **Output/input role flags:**

| Flag | Effect |
|---|---|
| `ENABLE_CRSF` | RX side: `CRSFRcOutput` drives CRSF at 420000 baud on `Serial` (UART0) to a flight controller (`src/main.cpp:222-225`). |
| `ENABLE_CRSF_INPUT` | TX module: `CRSFRcInput` reads CRSF from the handset at 400000 baud, inverted, half-duplex on one pin, on `HardwareSerial(1)` (`src/main.cpp`). |
| `ENABLE_SBUS` | RX side: `SBusTx` on `Serial`. Not set by any env, and stale (see known-issues.md). |
| none of the above | No longer a supported role. The old "plain" host-serial build was removed together with the serial protocol (see [serial-protocol.md](serial-protocol.md)). Every env sets one of the flags above. |
| `ENABLE_FORCEWIFI` | Starts WiFi upload mode at boot, while the link keeps running (`src/WifiUpload.cpp:155-157`) |
| `ENABLE_FORCEONLYWIFI` | WiFi upload mode only, link not started (`src/main.cpp:596-599`). Not set by any env. |
| `ARDUINO_USB_CDC_ON_BOOT=1` | ESP32-S2/S3: `Serial` is native USB CDC. That is where `DebugMonitor` prints on the TX module. |

## Environments

| Env | Board / MCU | Role | Flags | Pin map | Build status after serial removal |
|---|---|---|---|---|---|
| `esp32-pico` | `esp8285` (despite the name) | none | none | none | **Fails:** no `TARGET_*`, and the ESP8266 toolchain without `-std=gnu++2a` rejects `concept` |
| `exvectrlink_tx_module` | `esp32-s3-devkitc-1`, 16 MB flash, OPI PSRAM enabled | TX module, 1 radio | `TARGET_EXVECTRLINKTXMODULE`, `ENABLE_CRSF_INPUT`, `BOARD_HAS_PSRAM` | `ExVectrLinkTxModule.hpp` | Builds (RAM 18.2 %, flash 35.3 %, 59632 / 1180535 bytes) |
| `exvectrlink_tx_module_diversity` | `esp32-s3-devkitc-1`, 16 MB flash, **PSRAM disabled** | TX module for the Radiomaster Pocket, true diversity (2 radios) | `TARGET_EXVECTRLINKTXMODULEDIVERSITY`, `ENABLE_CRSF_INPUT` | `ExVectrLinkTxModuleDiversity.hpp` | Builds (RAM 18.3 %, flash 35.2 %, 59892 / 1176445 bytes) |
| `exvectrlink_tx_module_network` | `esp32-s3-devkitc-1`, 16 MB flash, PSRAM disabled (as the diversity env) | **ExVectrNetwork mode** TX module: node 2 router, sync master, USB network stream, CRSF relay, FHSS air link (profile v1) with diversity and dynamic power, WiFi upload. Separate firmware: `src_network/` (without the receiver files) + `src/` `CRSF`, `SX1280`, `DatalinkSetup`, `DiversitySetup`, `FhssSetup`, `Settings`, `WifiUpload`, `WifiCredentials` (see [network-mode.md](network-mode.md)). USB flashing. | `TARGET_EXVECTRLINKTXMODULEDIVERSITY`, `ARDUINO_USB_CDC_ON_BOOT=1`, `CORE_DEBUG_LEVEL=0` (no `ENABLE_CRSF_INPUT`: `main.cpp` is not part of it) | `ExVectrLinkTxModuleDiversity.hpp` | Builds (RAM 20.8 %, flash 31.7 %, 68204 / 1059129 bytes) |
| `exvectrlink_rx_network_esp8285` | `esp8285`, 160 MHz | **ExVectrNetwork mode** vehicle receiver: node 3 router, sync follower, UART0 460800 to the vehicle, FHSS air link, WiFi upload (espota to 192.168.178.22). Only the network stack: `src_network/RxRouter`, `target/`, `target_rx/` + `src/` `SX1280`, `DatalinkSetup`, `DiversitySetup`, `FhssSetup`, `WifiUpload`, `WifiCredentials`. No debug output. | `TARGET_GENERICESP8285`, `-std=gnu++2a`, `NON32XFER_HANDLER` | `GenericEsp8266.hpp` | Builds (RAM 52.1 %, flash 47.9 %, 42680 / 365119 bytes) |
| `exvectrlink_rx_network_genericdiversity` | `pico32` (ESP32) | As `exvectrlink_rx_network_esp8285`, two radios with RX diversity, espota to 192.168.178.144 | `TARGET_GENERICDIVERSITY_ELRS`, `CORE_DEBUG_LEVEL=0` | `GenericDiversityELRS.hpp` | Builds (RAM 20.3 %, flash 80.7 %, 66400 / 1057299 bytes) |
| `native` | host | Test-only: Unity tests of `src_network/` (`test_network_mode`, `test_air_link`; `pio test -e native`). Not a firmware: always pass `-e` to `pio run`. | none | none | 68 tests pass |
| `native_fhss` | host | Test-only: the real FHSS class on fake radios, C++20 (`test_fhss`; `pio test -e native_fhss`), including the FPV golden trace | none | none | 7 tests pass |
| `exvectrlink_genericdiversity_lr_ota` | `pico32` (ESP32) | RX, diversity, CRSF to FC, upload to 192.168.178.144 | `TARGET_GENERICDIVERSITY_ELRS`, `ENABLE_CRSF` | `GenericDiversityELRS.hpp` | Builds (RAM 18.1 %, flash 89.6 %, 59408 / 1173883 bytes) |
| `exvectrlink_genericdiversity_ripper_ota` | `pico32` | same as above, different upload IP (192.168.178.184) | same | same | Builds (same size as `_lr_ota`) |
| `exvectrlink_generic_esp8285_ota` | `esp8285` | RX, single radio, no PA, CRSF to FC | `TARGET_GENERICESP8285`, `ENABLE_CRSF`, `-std=gnu++2a`, `NON32XFER_HANDLER` | `GenericEsp8266.hpp` | Builds (RAM 43.1 %, flash 48.8 %, 35288 / 371547 bytes) |
| `exvectrlink_generic_esp8285_pa_ota` | `esp8285` | RX, single radio with PA/LNA, CRSF to FC, WiFi on at boot | `TARGET_GENERICESP8285WITHPA`, `ENABLE_FORCEWIFI`, `ENABLE_CRSF`, … | `GenericEsp8266WithPA.hpp` | Builds (RAM 43.1 %, flash 48.8 %, 35288 / 371563 bytes) |

"Builds" means it compiled and linked with a placeholder `src/WifiCredentials.cpp`. Without that gitignored file every env fails at link (see known-issues.md).

The `exvectrlink_controller_usb` and `exvectrlink_controller_ota` envs (old ExVectrController handset, `TARGET_EXVECTRCONTROLLER`) were removed with the serial protocol: they only existed for a serial host. Every FPV RX env sets `ENABLE_CRSF`. The Starship receiver envs are `exvectrlink_rx_network_*` (network mode, UART stream to the vehicle).

### Sizes before and after `feature/gs-requests-and-upload`

Release builds with the local `src/WifiCredentials.cpp`. "Before" is Development `c2a4d30` (ExVectrNetwork `feature/air-link-fhss`, ExVectrPackets 8170849), measured by building it on this machine; "after" is the feature branch (ExVectrNetwork de708ca, ExVectrPackets 4ca8fb1). Nothing in `src/` changed. FPV envs are byte-identical (RAM and flash), as expected: they build none of `src_network/` and the new packet enum value is not used there.

| Env | RAM before | RAM after | Flash before | Flash after |
|---|---|---|---|---|
| `exvectrlink_tx_module_diversity` | 59908 | 59908 (0) | 1176705 | 1176705 (0) |
| `exvectrlink_tx_module` | 59648 | 59648 (0) | 1180795 | 1180795 (0) |
| `exvectrlink_genericdiversity_lr_ota` (and `_ripper_ota`) | 59424 | 59424 (0) | 1174183 | 1174183 (0) |
| `exvectrlink_generic_esp8285_ota` | 35304 | 35304 (0) | 371803 | 371803 (0) |
| `exvectrlink_tx_module_network` | 68100 | 68204 (+104) | 1058489 | 1059129 (+640) |
| `exvectrlink_rx_network_esp8285` | 42264 | 42680 (+416) | 364191 | 365119 (+928) |
| `exvectrlink_rx_network_genericdiversity` | 65984 | 66400 (+416) | 1056175 | 1057299 (+1124) |

The receiver's RAM grows by the new port mux (sequence state per node and port) and the upload command state; the TX module already had a mux.

### Sizes before and after `feature/air-link-fhss`

Release builds with the local `src/WifiCredentials.cpp`, all clean builds. "Before" is Development `35fafb1`. Nothing in `src/` changed. The FPV difference comes from the FHSS library: four new members (+16 bytes RAM: link version, fixed frame length, wakeup lead, provider pointer) and the network-only branches in the slot code (pull hook, padding), which the FPV path skips. `nm` finds no network symbols (`network::Node`, `NetAirAccess`, `RxRouter`, `NetworkRouter`, `AirFrameLink`, `StreamFrameLink`, `FrameAggregator`) in any FPV ELF, and the new FHSS setters (`applyProfile`, `setFixedFrameLength`) are not linked there either. FPV behaviour is checked by the golden-trace host test.

| Env | RAM before | RAM after | Flash before | Flash after |
|---|---|---|---|---|
| `exvectrlink_tx_module_diversity` | 59892 | 59908 (+16) | 1176445 | 1176705 (+260) |
| `exvectrlink_tx_module` | 59632 | 59648 (+16) | 1180535 | 1180795 (+260) |
| `exvectrlink_genericdiversity_lr_ota` (and `_ripper_ota`) | 59408 | 59424 (+16) | 1173883 | 1174183 (+300) |
| `exvectrlink_generic_esp8285_ota` | 35288 | 35304 (+16) | 371547 | 371803 (+256) |
| `exvectrlink_generic_esp8285_pa_ota` | 35288 | 35304 (+16) | 371563 | 371819 (+256) |
| `exvectrlink_tx_module_network` | 35392 (air stub) | 68100 | 377008 | 1058489 (radios, FHSS, WiFi upload, settings) |
| `exvectrlink_rx_network_esp8285` (new) | - | 42264 | - | 364191 |
| `exvectrlink_rx_network_genericdiversity` (new) | - | 65984 | - | 1056175 |

The ESP8285 network receiver uses about 7 KB more RAM than the FPV receiver (the aggregator holds 24 frames of 240 bytes) and about 7 KB less flash. 81920 bytes of RAM leave about 39 KB.

### Sizes before and after `feature/tx-network-router`

Release builds with a placeholder `src/WifiCredentials.cpp`. "Before" is Development `fb04cb8` with ExVectrNetwork 7cb0e79 and ExVectrPackets 46286bd. "After" has the library pins moved to ExVectrNetwork 4090dce and ExVectrPackets 8170849. Nothing in `src/` changed, and no network-mode code is linked into the FPV envs (`nm` finds no `network::Node`, and the size is identical with the new env present).

| Env | RAM before | RAM after | Flash before | Flash after |
|---|---|---|---|---|
| `exvectrlink_tx_module_diversity` | 59636 | 59892 (+256) | 1178185 | 1176445 (-1740) |
| `exvectrlink_tx_module` | 59504 | 59632 (+128) | 1182275 | 1180535 (-1740) |
| `exvectrlink_genericdiversity_lr_ota` (and `_ripper_ota`) | 59152 | 59408 (+256) | 1175631 | 1173883 (-1748) |
| `exvectrlink_generic_esp8285_ota` | 35160 | 35288 (+128) | 371403 | 371547 (+144) |
| `exvectrlink_generic_esp8285_pa_ota` | 35160 | 35288 (+128) | 371419 | 371563 (+144) |
| `exvectrlink_tx_module_network` (new) | - | 35392 | - | 377008 |

The small RAM increase in every FPV env comes from the library update (not investigated in detail; the `Sx1280_Direct` driver changed, its maximum frame length is now configurable with default 128). See known-issues.md. The ESP8285 has 81920 bytes of RAM, so +128 bytes matters there.

### PSRAM on the diversity TX module

`exvectrlink_tx_module_diversity` deliberately leaves PSRAM unconfigured: no `psram_type`, `memory_type = qio_qspi`, no `BOARD_HAS_PSRAM` (`platformio.ini`, comment in that env).
- Octal PSRAM claims GPIO 33–37. This board wires radio A's DIO1 to GPIO 36 and its RX_EN to GPIO 35.
- With OPI PSRAM enabled, the DIO1 interrupt never fires and the RF switch never toggles.
- The firmware doesn't allocate from PSRAM, so nothing is lost.
- The single-radio `exvectrlink_tx_module` still enables OPI PSRAM. Its pins avoid GPIO 33–37.

## Pin maps (`src/targets/`)

All values are GPIO numbers. `-1` means not connected. `PAdbm` is the external PA gain in dB. `MaxPower` is the board's maximum output in dBm: it is passed to `DynamicPower::setMaxPower()` (`src/main.cpp:683`) and the radios' `setTxMaxPower()`.

| Signal | TxModuleDiversity | TxModule | GenericDiversityELRS | GenericEsp8266 | GenericEsp8266WithPA |
|---|---|---|---|---|---|
| SPI MISO/MOSI/SCK | 10/9/3 | 11/10/9 | 33/32/25 | 12/13/14 | 12/13/14 |
| A: BUSY/DIO1/NSS/RST | 48/36/14/47 | 4/5/6/7 | 39/34/13/21 | 5/4/15/2 | 5/4/15/2 |
| A: RX_EN/TX_EN | 35/21 | 15/16 | 9/15 | -1/-1 | 9/10 |
| B: BUSY/DIO1/NSS/RST | 8/11/13/17 | none | 36/37/27/26 | none | none |
| B: RX_EN/TX_EN | 12/18 | none | 10/14 | none | none |
| PAdbm / MaxPower | 26 / 27 | 26 / 27 | 19 / 20 | 0 / 12 | 19 / 20 |
| serialRx / serialTx | 16 / 16 (S.Port wire) | 12 / 12 (S.Port wire) | 3 / 1 | 3 / 1 | 3 / 1 |
| Button | 0 | 0 | 0 | 0 | 0 |
| Diversity | `HAS_DIVERSITY` | no | `HAS_DIVERSITY` | no | no |

Notes:
- The TX module pin maps use one shared pin for CRSF (the Pocket's nano bay exposes CRSF as a single half-duplex wire). See crsf.md.
- Both radios share one reset pulse (`src/SX1280.cpp:97-109`), because some boards appear to couple their reset lines.
- `GenericEsp8266WithPA.hpp` reuses the include guard of `GenericEsp8266.hpp`. This is harmless while only one is included.
- Generic ELRS-style boards put `serialRx`/`serialTx` on GPIO 3/1. Those are UART0 pins, also used by `Serial`. That works for CRSF output.
