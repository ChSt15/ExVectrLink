#pragma once
// Pure CRSF <-> ExVectrNetwork packet mapping for the TX module's network mode (docs/network-mode.md,
// workspace docs/network-design.md section 3.6). No Arduino, Core or heap: host-testable.
//
// The module is generic: it knows no vehicle-specific meaning of any channel.
#include <stddef.h>
#include <stdint.h>

#include "ExVectrPackets/gen/all.hpp"

namespace VCTR::ExVectrLink::netmode {

constexpr uint8_t kCrsfSync = 0xC8;
constexpr size_t kCrsfMaxFrame = 64;
constexpr size_t kCrsfRcPayloadLen = 22;

enum CrsfType : uint8_t {
  kCrsfGps = 0x02,
  kCrsfVario = 0x07,
  kCrsfBattery = 0x08,
  kCrsfLinkStatistics = 0x14,
  kCrsfRcChannelsPacked = 0x16,
  kCrsfAttitude = 0x1E,
  kCrsfFlightMode = 0x21,
};

/// One complete CRSF frame: [sync][len][type][payload][crc8 0xD5].
struct CrsfFrame {
  uint8_t bytes[kCrsfMaxFrame] = {};
  uint8_t len = 0;
};

/// CRC-8 poly 0xD5 over type + payload.
uint8_t crsfCrc8(const uint8_t *data, size_t len);

/// Builds a frame from type and payload. Returns false if the payload does not fit.
bool crsfBuildFrame(CrsfFrame &out, uint8_t type, const uint8_t *payload,
                    size_t payloadLen);

// ---- Pocket -> network ------------------------------------------------------

/// Unpacks the 22-byte 0x16 payload into 16 raw 11-bit CRSF channel values.
void crsfUnpackRcChannels(const uint8_t payload[kCrsfRcPayloadLen],
                          uint16_t channels[16]);

/// Packs 16 raw values into the 22-byte 0x16 payload (used by tests and bench tools).
void crsfPackRcChannels(const uint16_t channels[16],
                        uint8_t payload[kCrsfRcPayloadLen]);

/// Design 3.6: clamp(round((crsf - 992) * 1000 / 820), -1000, 1000). Rounds half away from zero.
int16_t crsfToRcChannel(uint16_t crsf);

/// All 16 channels, SOURCE_OK = 1, reserved = 0.
void rcStateFromCrsf(const uint16_t channels[16], Packets::RcState &out);

// ---- network -> Pocket ------------------------------------------------------

/// Local air-side measurements (the module's own radio). The air stub reports zeros until the air link exists.
struct LocalAirStats {
  Packets::AirState airState = Packets::AirState::kOff;
  uint8_t rxLq = 0;   ///< %, downlink quality as seen by this module
  int8_t rssi = 0;    ///< dBm of the last valid downlink packet
  int8_t snr = 0;     ///< dB
  int8_t txPower = 0; ///< dBm
};

/// CRSF power index (0, 10, 25, 50, 100, 250, 500, 1000, 2000 mW) for a dBm value, as in CRSFRcInput.
uint8_t crsfPowerIndex(int8_t dbm);

/// 0x14 LINK_STATISTICS. `remote` is node 3's LINK_STATS (uplink as the vehicle receiver sees it), or nullptr if
/// node 3 is absent: then every field is zero.
void crsfBuildLinkStatistics(CrsfFrame &out, const Packets::LinkStats *remote,
                             const LocalAirStats &local);

/// 0x21 FLIGHT_MODE: mode_text as a NUL-terminated ASCII string (at most 8 characters).
void crsfBuildFlightMode(CrsfFrame &out, const Packets::HandsetTelem &t);
/// 0x02 GPS: lat/lon deg*1e7, groundspeed km/h*10, heading cdeg, altitude m+1000, satellites. Big endian.
void crsfBuildGps(CrsfFrame &out, const Packets::HandsetTelem &t);
/// 0x1E ATTITUDE: pitch, roll, yaw as rad*10000 (i16, big endian).
void crsfBuildAttitude(CrsfFrame &out, const Packets::HandsetTelem &t);
/// 0x07 VARIO: vertical speed in cm/s (i16, big endian).
void crsfBuildVario(CrsfFrame &out, const Packets::HandsetTelem &t);
/// 0x08 BATTERY_SENSOR: voltage in 100 mV units, current/capacity/percent 0. Caller sends it only if battery != 0.
void crsfBuildBattery(CrsfFrame &out, const Packets::HandsetTelem &t);

/// Telemetry frame kinds sent in round robin, one per handset frame (the half-duplex port keeps one pending reply).
enum class TelemKind : uint8_t {
  LinkStats,
  FlightMode,
  Gps,
  Attitude,
  Vario,
  Battery,
  Count
};

/// Chooses the next telemetry frame to hand to the handset. Pure logic: the caller supplies time and data.
///
/// Rules:
///  - LINK_STATISTICS has priority and is due every 100 ms (10 Hz). It is sent even when node 3 is absent (zeroed).
///  - The HANDSET_TELEM frames (FLIGHT_MODE, GPS, ATTITUDE, VARIO, BATTERY) go round robin, each at most every
///    100 ms, and only while the HANDSET_TELEM packet is fresh (younger than `staleNs`). Stale vehicle data is
///    not shown to the pilot. BATTERY is skipped while battery == 0.
///  - At most one frame per call: call once per received handset frame.
class CrsfTelemetryScheduler {
public:
  static constexpr int64_t kLinkStatsPeriodNs = 100000000LL; // 10 Hz
  static constexpr int64_t kTelemMinPeriodNs = 100000000LL;  // per frame kind
  static constexpr int64_t kStaleNs = 3000000000LL;          // as node presence

  /// `remote`: node 3's LINK_STATS if node 3 is present, else nullptr. `telem`: latest HANDSET_TELEM from the
  /// vehicle, or nullptr if none was ever received; `telemRxNs` is when it arrived.
  /// Returns true and fills `out` if a frame is to be sent now.
  bool next(int64_t nowNs, const Packets::LinkStats *remote,
            const LocalAirStats &local, const Packets::HandsetTelem *telem,
            int64_t telemRxNs, CrsfFrame &out);

private:
  int64_t lastSent_[static_cast<size_t>(TelemKind::Count)] = {};
  bool everSent_[static_cast<size_t>(TelemKind::Count)] = {};
  uint8_t rr_ = 1; // next HANDSET_TELEM kind to try (index into TelemKind, 1..Count-1)
};

} // namespace VCTR::ExVectrLink::netmode
