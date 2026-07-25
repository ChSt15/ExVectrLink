#ifndef EXVECRLINK_GENERICDIVERSITYELRS_HPP
#define EXVECRLINK_GENERICDIVERSITYELRS_HPP

namespace VCTR::HardwareConfig {

constexpr int button = 0; // GPIO pin for the button

constexpr int spiMiso = 33;
constexpr int spiMosi = 32;
constexpr int spiSck = 25;

constexpr int sx1280ABusy = 39;
constexpr int sx1280ADio1 = 34;
constexpr int sx1280ANss = 13;
constexpr int sx1280ARst = 21;
constexpr int sx1280ARxEn = 9;
constexpr int sx1280ATxEn = 15;
constexpr int sx1280APAdbm = 19;
constexpr int sx1280AMaxPower = 20;

constexpr int sx1280BBusy = 36;
constexpr int sx1280BDio1 = 37;
constexpr int sx1280BNss = 27;
constexpr int sx1280BRst = 26;
constexpr int sx1280BRxEn = 10;
constexpr int sx1280BTxEn = 14;
constexpr int sx1280BPAdbm = sx1280APAdbm;
constexpr int sx1280BMaxPower = sx1280AMaxPower;

constexpr int ledRgb = 22;

constexpr int serialRx = 3;
constexpr int serialTx = 1;

#define HAS_DIVERSITY

} // namespace VCTR::HardwareConfig

#endif // EXVECRLINK_GENERICDIVERSITYELRS_HPP