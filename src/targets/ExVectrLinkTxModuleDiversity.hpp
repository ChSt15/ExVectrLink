#ifndef EXVECRLINK_EXVECTRLINKTXMODULEDIVERSITY_HPP
#define EXVECRLINK_EXVECTRLINKTXMODULEDIVERSITY_HPP

namespace VCTR::HardwareConfig {

constexpr int button = 0; // GPIO pin for the button

constexpr int spiMiso = 10;
constexpr int spiMosi = 9;
constexpr int spiSck = 3;

// Radio A is Horizontal antenna
constexpr int sx1280ABusy = 48;
constexpr int sx1280ADio1 = 36;
constexpr int sx1280ANss = 14;
constexpr int sx1280ARst = 47;
constexpr int sx1280ARxEn = 35;
constexpr int sx1280ATxEn = 21;
constexpr int sx1280APAdbm = 26;
constexpr int sx1280AMaxPower = 27;

// Radio B is still physically populated on the controller board. Keep its
// pins in a safe state so it does not interfere with Radio A, but do not use
// it for datalink diversity.
constexpr int sx1280BBusy = 8;
constexpr int sx1280BDio1 = 11;
constexpr int sx1280BNss = 13;
constexpr int sx1280BRst = 17;
constexpr int sx1280BRxEn = 12;
constexpr int sx1280BTxEn = 18;
constexpr int sx1280BPAdbm = sx1280APAdbm;
constexpr int sx1280BMaxPower = sx1280AMaxPower;

#define HAS_DIVERSITY
#define HAS_RADIO_B_HARDWARE

// constexpr int ledRgb = -1; // Not used, set to -1

// constexpr int i2cSda = 38;
// constexpr int i2cScl = 37;

// The RadioMaster Pocket's "nano" external module bay exposes CRSF as a
// single, half-duplex signal wire (S.Port style) rather than separate RX/TX
// lines. Both must therefore point at the SAME physical pin -- CRSFRcInput
// binds the pad as UART RX only, and CRSF reroutes it to UART TX via the
// GPIO matrix for the duration of each reply frame (see
// CRSF::switchToRx()/switchToTx()), so it only drives the bus while
// actually transmitting telemetry and otherwise stays high-impedance so it
// does not fight the handset's driver.
constexpr int sport = 16;
constexpr int serialRx = sport;
constexpr int serialTx = sport;

} // namespace VCTR::HardwareConfig

#endif // EXVECRLINK_EXVECTRLINKTXMODULEDIVERSITY_HPP