#ifndef EXVECRLINK_GENERICESP8285_HPP
#define EXVECRLINK_GENERICESP8285_HPP

namespace VCTR::HardwareConfig {

constexpr int button = 0; // GPIO pin for the button

// ESP8285 uses the default SPI pins (GPIO 12-14)
constexpr int spiMiso = 12;
constexpr int spiMosi = 13;
constexpr int spiSck = 14;

constexpr int sx1280ABusy = 5;
constexpr int sx1280ADio1 = 4;
constexpr int sx1280ANss = 15;
constexpr int sx1280ARst = 2;
constexpr int sx1280ARxEn = -1;
constexpr int sx1280ATxEn = -1;
constexpr int sx1280APAdbm = 0;
constexpr int sx1280AMaxPower = 12;

constexpr int ledRgb = -1; // Not available or set to -1

constexpr int serialRx = 3;
constexpr int serialTx = 1;

// #define HAS_DIVERSITY

} // namespace VCTR::HardwareConfig

#endif // EXVECRLINK_GENERICESP8285_HPP