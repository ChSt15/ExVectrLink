#ifndef EXVECRLINK_EXVECTRCONTROLLERLINK_HPP
#define EXVECRLINK_EXVECTRCONTROLLERLINK_HPP

namespace VCTR::HardwareConfig {

constexpr int button = 0; // GPIO pin for the button

constexpr int spiMiso = 33;
constexpr int spiMosi = 32;
constexpr int spiSck = 25;

constexpr int sx1280ABusy = 36;
constexpr int sx1280ADio1 = 37;
constexpr int sx1280ANss = 27;
constexpr int sx1280ARst = 26;
constexpr int sx1280ARxEn = 10;
constexpr int sx1280ATxEn = 14;
constexpr int sx1280APAdbm = 20;

constexpr int sx1280BBusy = 39;
constexpr int sx1280BDio1 = 34;
constexpr int sx1280BNss = 13;
constexpr int sx1280BRst = 21;
constexpr int sx1280BRxEn = 9;
constexpr int sx1280BTxEn = 15;
constexpr int sx1280BPAdbm = 20;

constexpr int ledRgb = 22;

constexpr int serialRx = 3;
constexpr int serialTx = 1;

} // namespace VCTR::HardwareConfig

#endif // EXVECRLINK_EXVECTRCONTROLLERLINK_HPP