#ifndef EXVECRLINK_EXVECTRCONTROLLERLINK_HPP
#define EXVECRLINK_EXVECTRCONTROLLERLINK_HPP

namespace VCTR::HardwareConfig {

constexpr int button = 0; // GPIO pin for the button

constexpr int spiMiso = 12;
constexpr int spiMosi = 11;
constexpr int spiSck = 9;

// Radio A is Horizontal antenna
constexpr int sx1280ABusy = 8;
constexpr int sx1280ADio1 = 14;
constexpr int sx1280ANss = 13;
constexpr int sx1280ARst = 10;
constexpr int sx1280ARxEn = 7;
constexpr int sx1280ATxEn = 6;
// constexpr int sx1280ARxEn = 6;
// constexpr int sx1280ATxEn = 7;
constexpr int sx1280APAdbm = 26;
constexpr int sx1280AMaxPower = 27;

// Radio B is Vertical antenna
constexpr int sx1280BBusy = 21;
constexpr int sx1280BDio1 = 5;
constexpr int sx1280BNss = 4;
constexpr int sx1280BRst = 17;
constexpr int sx1280BRxEn = 3;
constexpr int sx1280BTxEn = 2;
// constexpr int sx1280BRxEn = 2;
// constexpr int sx1280BTxEn = 3;
constexpr int sx1280BPAdbm = 26;
constexpr int sx1280BMaxPower = 27;

constexpr int ledRgb = -1; // Not used, set to -1

constexpr int serialRx = 18;
constexpr int serialTx = 16;

#define HAS_DIVERSITY

} // namespace VCTR::HardwareConfig

#endif // EXVECRLINK_EXVECTRCONTROLLERLINK_HPP