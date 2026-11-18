# Settings / EEPROM

> What the firmware keeps across power cycles, and how to add to it.

Last verified against commit: 4830874 (submodule ExVectrData 283596f)

## What is persisted

| Record | Key | Field | Default | Changed by | Applied |
|---|---|---|---|---|---|
| `RFSettings` (`src/Settings.hpp:28-59`) | `0x0102` | `dynamicPowerEnabled` | true | Lua "Dyn Power", serial `SetPowerParams` | TX side only (`src/RFSetup.cpp:61-71`) |
| | | `dynMinPowerDbm` | 10 | Lua "Dyn Min Pwr", serial | TX side only |
| | | `dynMaxPowerDbm` | 20 | Lua "Dyn Max Pwr", serial | TX side only. Also the fixed power when dynamic power is off. |
| | | `txAntenna` | 255 = diversity | Lua "TX Antenna" | both sides. An index the build has no radio for falls back to diversity (`src/RFSetup.cpp:81-88`). |

On the RX side the power fields are stored but ignored: the receiver always runs at a fixed 20 dBm (`src/RFSetup.cpp:72-79`).

**Not persisted**, so these reset at every boot:
- FHSS hop key (always 0 unless a serial host sets it)
- sync latency compensation (900 µs)
- modulation preset
- serial baud rate

The planned link-mode settings (roadmap.md) would be the second record.

## How it works

- **Storage:** `Settings` keeps one RAM image of 128 bytes (`kSettingsMemorySize`, `src/Settings.hpp:73`), managed by ExVectrData's `Memory_Manager` (20-byte header + 10 bytes per record). On ESP32/ESP8266 the image is backed by the Arduino flash-emulated `EEPROM` (`src/Settings.cpp:23-37`).
- **Startup:** `setupSettings()` (`src/main.cpp:691`) loads the image.
  - A blank image, or one written by another manager version, is cleared.
  - Missing records are created from their defaults (`src/Settings.cpp:123-155`).
- **Changes:** `getSettings().set(record)` changes RAM only and marks the store dirty. `SettingsSync` (every 250 ms) writes to flash once nothing has changed for 1 s (`src/Settings.cpp:17-21`, `src/Settings.cpp:60-69`). That way scrolling through a menu produces one flash write.
- **Byte order:** records are stored in native little-endian layout. The image never leaves the device.

## Adding a setting

1. Write a struct with default member values and a fresh, never-reused `static constexpr uint16_t kStorageKey`.
2. Add a `static_assert` on its size.
3. Add it to `Settings::Records` (`src/Settings.hpp:106`). Keys are checked at compile time for being non-zero and unique.
4. If you change an existing struct's layout, give it a **new key**. The old item is then ignored, and the device comes up at the defaults.
5. Keep within 128 bytes in total, or grow `kSettingsMemorySize`. Growing is safe, shrinking is not.
6. Document the record in the table above.
