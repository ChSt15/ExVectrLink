# Build Targets and Hardware

> Every `[env:...]` in `platformio.ini`: which board it builds for, which role it plays, and the pin map it uses.

Last verified against commit: 4830874

## How a build is configured

- **`TARGET_*`** picks the pin map. `src/Hardware.hpp:4-26` includes exactly one `src/targets/*.hpp`, which defines `namespace VCTR::HardwareConfig`. A build without a `TARGET_*` has no pin map and does not compile.
- **`IS_TX_SIDE`** is defined for `TARGET_EXVECTRCONTROLLER`, `TARGET_EXVECTRLINKTXMODULE` and `TARGET_EXVECTRLINKTXMODULEDIVERSITY` (`src/Hardware.hpp:32-36`). It selects:
  - the FHSS role (`src/main.cpp:634-638`);
  - dynamic power from settings vs. a fixed 20 dBm on the RX (`src/RFSetup.cpp:61-79`).
  - A new handset-side target must be added to that list. Otherwise it comes up as a receiver and the link never forms.
- **`HAS_DIVERSITY`** is defined by the pin map. It means two radios are used together through `Sx1280Diversity`.
- **`HAS_RADIO_B_HARDWARE`** means a second radio is populated but only held in a safe pin state, not used (`src/SX1280.cpp:26-33`).
- **Output/input role flags:**

| Flag | Effect |
|---|---|
| `ENABLE_CRSF` | RX side: `CRSFRcOutput` drives CRSF at 420000 baud on `Serial` (UART0) to a flight controller (`src/main.cpp:222-225`). `SerialTelecoms` is removed. |
| `ENABLE_CRSF_INPUT` | TX module: `CRSFRcInput` reads CRSF from the handset at 400000 baud, inverted, half-duplex on one pin, on `HardwareSerial(1)` (`src/main.cpp:226-244`). `SerialTelecoms` is removed. |
| `ENABLE_SBUS` | RX side: `SBusTx` on `Serial`. Not set by any env, and stale (see known-issues.md). |
| none of the above | "Plain" build: `SerialTelecoms` on `HardwareSerial(1)` at 1 Mbaud 8N1 on `serialRx`/`serialTx` (`src/main.cpp:581-585`). The `SyncedMonitor` and `ErrorSender` tasks run. This is the host-module role that the Starship flight computer would use. |
| `ENABLE_FORCEWIFI` | Starts WiFi upload mode at boot, while the link keeps running (`src/WifiUpload.cpp:155-157`) |
| `ENABLE_FORCEONLYWIFI` | WiFi upload mode only, link not started (`src/main.cpp:596-599`). Not set by any env. |
| `ARDUINO_USB_CDC_ON_BOOT=1` | ESP32-S2/S3: `Serial` is native USB CDC. That is where `DebugMonitor` prints on the TX module. |

## Environments

| Env | Board / MCU | Role | Flags | Pin map | Build status at 4830874 |
|---|---|---|---|---|---|
| `esp32-pico` | `esp8285` (despite the name) | none | none | none | **Fails:** no `TARGET_*`, and the ESP8266 toolchain without `-std=gnu++2a` rejects `concept` |
| `exvectrlink_controller_usb` | `lolin_s2_mini` (ESP32-S2) | TX (old ExVectrController handset), plain serial | `TARGET_EXVECTRCONTROLLER`, `ENABLE_FORCEWIFI` | `ExVectrControllerLink.hpp` | Builds (flash 89 %) |
| `exvectrlink_controller_ota` | `lolin_s2_mini` | same, upload over WiFi to 192.168.178.29 | `TARGET_EXVECTRCONTROLLER` | `ExVectrControllerLink.hpp` | not built here (same sources as `_usb`) |
| `exvectrlink_tx_module` | `esp32-s3-devkitc-1`, 16 MB flash, OPI PSRAM enabled | TX module, 1 radio | `TARGET_EXVECTRLINKTXMODULE`, `ENABLE_CRSF_INPUT`, `BOARD_HAS_PSRAM` | `ExVectrLinkTxModule.hpp` | Builds |
| `exvectrlink_tx_module_diversity` | `esp32-s3-devkitc-1`, 16 MB flash, **PSRAM disabled** | TX module for the Radiomaster Pocket, true diversity (2 radios) | `TARGET_EXVECTRLINKTXMODULEDIVERSITY`, `ENABLE_CRSF_INPUT` | `ExVectrLinkTxModuleDiversity.hpp` | Builds (RAM 18 %, flash 36 %) |
| `exvectrlink_genericdiversity_lr_ota` | `pico32` (ESP32) | RX, diversity, CRSF to FC, upload to 192.168.178.144 | `TARGET_GENERICDIVERSITY_ELRS`, `ENABLE_CRSF` | `GenericDiversityELRS.hpp` | Builds (flash 91 %) |
| `exvectrlink_genericdiversity_ripper_ota` | `pico32` | same as above, different upload IP (192.168.178.184) | same | same | not built here (identical flags) |
| `exvectrlink_generic_esp8285_ota` | `esp8285` | RX, single radio, no PA, CRSF to FC | `TARGET_GENERICESP8285`, `ENABLE_CRSF`, `-std=gnu++2a`, `NON32XFER_HANDLER` | `GenericEsp8266.hpp` | Builds (RAM 44 %, flash 50 %) |
| `exvectrlink_generic_esp8285_pa_ota` | `esp8285` | RX, single radio with PA/LNA, CRSF to FC, WiFi on at boot | `TARGET_GENERICESP8285WITHPA`, `ENABLE_FORCEWIFI`, `ENABLE_CRSF`, … | `GenericEsp8266WithPA.hpp` | not built here |

"Builds" means it compiled and linked with a placeholder `src/WifiCredentials.cpp`. Without that gitignored file every env fails at link (see known-issues.md).

**There is currently no env for a vehicle receiver in the plain `SerialTelecoms` role.** Every RX env sets `ENABLE_CRSF`. The Starship receiver needs a new env (see roadmap.md).

### PSRAM on the diversity TX module

`exvectrlink_tx_module_diversity` deliberately leaves PSRAM unconfigured: no `psram_type`, `memory_type = qio_qspi`, no `BOARD_HAS_PSRAM` (`platformio.ini`, comment in that env).
- Octal PSRAM claims GPIO 33–37. This board wires radio A's DIO1 to GPIO 36 and its RX_EN to GPIO 35.
- With OPI PSRAM enabled, the DIO1 interrupt never fires and the RF switch never toggles.
- The firmware doesn't allocate from PSRAM, so nothing is lost.
- The single-radio `exvectrlink_tx_module` still enables OPI PSRAM. Its pins avoid GPIO 33–37.

## Pin maps (`src/targets/`)

All values are GPIO numbers. `-1` means not connected. `PAdbm` is the external PA gain in dB. `MaxPower` is the board's maximum output in dBm: it is passed to `DynamicPower::setMaxPower()` (`src/main.cpp:683`) and the radios' `setTxMaxPower()`.

| Signal | TxModuleDiversity | TxModule | ControllerLink | GenericDiversityELRS | GenericEsp8266 | GenericEsp8266WithPA |
|---|---|---|---|---|---|---|
| SPI MISO/MOSI/SCK | 10/9/3 | 11/10/9 | 12/11/9 | 33/32/25 | 12/13/14 | 12/13/14 |
| A: BUSY/DIO1/NSS/RST | 48/36/14/47 | 4/5/6/7 | 8/14/13/10 | 39/34/13/21 | 5/4/15/2 | 5/4/15/2 |
| A: RX_EN/TX_EN | 35/21 | 15/16 | 7/6 | 9/15 | -1/-1 | 9/10 |
| B: BUSY/DIO1/NSS/RST | 8/11/13/17 | none | 21/5/4/17 (held safe) | 36/37/27/26 | none | none |
| B: RX_EN/TX_EN | 12/18 | none | 3/2 | 10/14 | none | none |
| PAdbm / MaxPower | 26 / 27 | 26 / 27 | 26 / 27 | 19 / 20 | 0 / 12 | 19 / 20 |
| serialRx / serialTx | 16 / 16 (S.Port wire) | 12 / 12 (S.Port wire) | 18 / 16 | 3 / 1 | 3 / 1 | 3 / 1 |
| Button | 0 | 0 | 0 | 0 | 0 | 0 |
| Diversity | `HAS_DIVERSITY` | no | `HAS_RADIO_B_HARDWARE` only | `HAS_DIVERSITY` | no | no |

Notes:
- The TX module pin maps use one shared pin for CRSF (the Pocket's nano bay exposes CRSF as a single half-duplex wire). See crsf.md.
- Both radios share one reset pulse (`src/SX1280.cpp:97-109`), because some boards appear to couple their reset lines.
- `GenericEsp8266WithPA.hpp` reuses the include guard of `GenericEsp8266.hpp`. This is harmless while only one is included.
- Generic ELRS-style boards put `serialRx`/`serialTx` on GPIO 3/1. Those are UART0 pins, also used by `Serial`. That works for CRSF output, but a plain-serial build on that hardware would have two UARTs on the same pins (see known-issues.md).
