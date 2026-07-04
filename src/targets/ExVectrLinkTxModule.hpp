#ifndef EXVECRLINK_EXVECTRLINKTXMODULE_HPP
#define EXVECRLINK_EXVECTRLINKTXMODULE_HPP

namespace VCTR::HardwareConfig {

constexpr int button = 0; // GPIO pin for the button

constexpr int spiMiso = 11;
constexpr int spiMosi = 10;
constexpr int spiSck = 9;

// Radio A is Horizontal antenna
constexpr int sx1280ABusy = 4;
constexpr int sx1280ADio1 = 5;
constexpr int sx1280ANss = 6;
constexpr int sx1280ARst = 7;
constexpr int sx1280ARxEn = 15;
constexpr int sx1280ATxEn = 16;
constexpr int sx1280APAdbm = 26;
constexpr int sx1280AMaxPower = 27;

// Radio B is still physically populated on the controller board. Keep its
// pins in a safe state so it does not interfere with Radio A, but do not use
// it for datalink diversity.
// constexpr int sx1280BBusy = 21;
// constexpr int sx1280BDio1 = 5;
// constexpr int sx1280BNss = 4;
// constexpr int sx1280BRst = 17;
// constexpr int sx1280BRxEn = 3;
// constexpr int sx1280BTxEn = 2;
// constexpr int sx1280BPAdbm = 26;
// constexpr int sx1280BMaxPower = 27;

// #define HAS_RADIO_B_HARDWARE

// constexpr int ledRgb = -1; // Not used, set to -1

constexpr int i2cSda = 38;
constexpr int i2cScl = 37;

constexpr int serialRx = 36;
constexpr int serialTx = 35;

} // namespace VCTR::HardwareConfig

#endif // EXVECRLINK_EXVECTRLINKTXMODULE_HPP