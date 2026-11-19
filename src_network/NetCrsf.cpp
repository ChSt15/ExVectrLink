#include "NetCrsf.hpp"

#include <math.h>
#include <string.h>

namespace VCTR::ExVectrLink::netmode {

uint8_t crsfCrc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                         : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

bool crsfBuildFrame(CrsfFrame &out, uint8_t type, const uint8_t *payload,
                    size_t payloadLen) {
  // [sync][len][type][payload][crc], len = type + payload + crc
  if (payloadLen + 4 > kCrsfMaxFrame) return false;
  out.bytes[0] = kCrsfSync;
  out.bytes[1] = static_cast<uint8_t>(payloadLen + 2);
  out.bytes[2] = type;
  if (payloadLen) memcpy(out.bytes + 3, payload, payloadLen);
  out.bytes[3 + payloadLen] = crsfCrc8(out.bytes + 2, payloadLen + 1);
  out.len = static_cast<uint8_t>(payloadLen + 4);
  return true;
}

void crsfUnpackRcChannels(const uint8_t *payload, uint16_t *channels) {
  uint32_t bitOffset = 0;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    const uint32_t byteIdx = bitOffset / 8;
    const uint8_t bitIdx = bitOffset % 8;
    uint32_t val = static_cast<uint32_t>(payload[byteIdx]) >> bitIdx;
    val |= static_cast<uint32_t>(payload[byteIdx + 1]) << (8 - bitIdx);
    if (bitIdx > 5) val |= static_cast<uint32_t>(payload[byteIdx + 2]) << (16 - bitIdx);
    channels[ch] = static_cast<uint16_t>(val & 0x7FF);
    bitOffset += 11;
  }
}

void crsfPackRcChannels(const uint16_t *channels, uint8_t *payload) {
  memset(payload, 0, kCrsfRcPayloadLen);
  uint32_t bitOffset = 0;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    const uint32_t val = channels[ch] & 0x7FF;
    const uint32_t byteIdx = bitOffset / 8;
    const uint8_t bitIdx = bitOffset % 8;
    payload[byteIdx] |= static_cast<uint8_t>(val << bitIdx);
    payload[byteIdx + 1] |= static_cast<uint8_t>(val >> (8 - bitIdx));
    if (bitIdx > 5) payload[byteIdx + 2] |= static_cast<uint8_t>(val >> (16 - bitIdx));
    bitOffset += 11;
  }
}

int16_t crsfToRcChannel(uint16_t crsf) {
  // Integer arithmetic, round half away from zero.
  const int32_t num = (static_cast<int32_t>(crsf) - 992) * 1000;
  const int32_t q = num >= 0 ? (num + 410) / 820 : -((-num + 410) / 820);
  if (q > 1000) return 1000;
  if (q < -1000) return -1000;
  return static_cast<int16_t>(q);
}

void rcStateFromCrsf(const uint16_t *channels, Packets::RcState &out) {
  out.flags = Packets::RcState::kFlagsSourceOk;
  out.reserved = 0;
  for (size_t i = 0; i < 16; i++) out.ch[i] = crsfToRcChannel(channels[i]);
}

uint8_t crsfPowerIndex(int8_t dbm) {
  // 0 mW, 10 mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000 mW, 250 mW, 50 mW (CRSF enum order)
  if (dbm <= 0) return 0;
  if (dbm <= 10) return 1;
  if (dbm <= 14) return 2;
  if (dbm <= 17) return 8;
  if (dbm <= 20) return 3;
  if (dbm <= 24) return 7;
  if (dbm <= 27) return 4;
  if (dbm <= 30) return 5;
  if (dbm <= 33) return 6;
  return 0;
}

namespace {

void putBe16(uint8_t *p, uint16_t u) { // negative i16 values convert modulo 2^16
  p[0] = static_cast<uint8_t>(u >> 8);
  p[1] = static_cast<uint8_t>(u);
}
void putBe32(uint8_t *p, int32_t v) {
  const uint32_t u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u >> 24);
  p[1] = static_cast<uint8_t>(u >> 16);
  p[2] = static_cast<uint8_t>(u >> 8);
  p[3] = static_cast<uint8_t>(u);
}

int32_t roundToI32(double v, int32_t lo, int32_t hi) {
  if (!(v == v)) return 0; // NaN
  if (v >= static_cast<double>(hi)) return hi;
  if (v <= static_cast<double>(lo)) return lo;
  return static_cast<int32_t>(llround(v));
}

constexpr double kRadToDeg = 57.29577951308232;

uint8_t clampDbmMagnitude(int8_t rssi) {
  return rssi < 0 ? static_cast<uint8_t>(-static_cast<int16_t>(rssi)) : 0;
}

} // namespace

void crsfBuildLinkStatistics(CrsfFrame &out, const Packets::LinkStats *remote,
                             const LocalAirStats &local) {
  uint8_t p[10] = {};
  if (remote != nullptr) {
    p[0] = clampDbmMagnitude(remote->rssi);       // uplink RSSI antenna 1
    p[1] = 0;                                     // antenna 2 not reported
    p[2] = remote->rx_lq > 100 ? 100 : remote->rx_lq;
    p[3] = static_cast<uint8_t>(remote->snr);
    p[4] = remote->antenna & 0x0F;                // RX winner
    p[5] = 4;                                     // RF profile, as in FPV mode
    p[6] = crsfPowerIndex(local.txPower);
    p[7] = clampDbmMagnitude(local.rssi);         // downlink RSSI
    p[8] = local.rxLq > 100 ? 100 : local.rxLq;
    p[9] = static_cast<uint8_t>(local.snr);
  }
  crsfBuildFrame(out, kCrsfLinkStatistics, p, sizeof p);
}

void crsfBuildFlightMode(CrsfFrame &out, const Packets::HandsetTelem &t) {
  uint8_t p[9];
  size_t n = 0;
  while (n < sizeof t.mode_text && t.mode_text[n] != '\0') {
    const char c = t.mode_text[n];
    p[n] = (c >= 0x20 && c < 0x7F) ? static_cast<uint8_t>(c) : '?';
    n++;
  }
  p[n++] = 0;
  crsfBuildFrame(out, kCrsfFlightMode, p, n);
}

void crsfBuildGps(CrsfFrame &out, const Packets::HandsetTelem &t) {
  uint8_t p[15];
  putBe32(p + 0, roundToI32(t.latitude * kRadToDeg * 1e7, INT32_MIN, INT32_MAX));
  putBe32(p + 4, roundToI32(t.longitude * kRadToDeg * 1e7, INT32_MIN, INT32_MAX));
  putBe16(p + 8, static_cast<uint16_t>(roundToI32(t.ground_speed * 3.6 * 10.0, 0, 65535)));
  putBe16(p + 10, static_cast<uint16_t>(roundToI32(t.heading * kRadToDeg * 100.0, 0, 65535)));
  putBe16(p + 12, static_cast<uint16_t>(roundToI32(t.altitude + 1000.0, 0, 65535)));
  p[14] = t.satellites;
  crsfBuildFrame(out, kCrsfGps, p, sizeof p);
}

void crsfBuildAttitude(CrsfFrame &out, const Packets::HandsetTelem &t) {
  uint8_t p[6];
  putBe16(p + 0, static_cast<uint16_t>(roundToI32(t.pitch * 10000.0, -32768, 32767)));
  putBe16(p + 2, static_cast<uint16_t>(roundToI32(t.roll * 10000.0, -32768, 32767)));
  putBe16(p + 4, static_cast<uint16_t>(roundToI32(t.yaw * 10000.0, -32768, 32767)));
  crsfBuildFrame(out, kCrsfAttitude, p, sizeof p);
}

void crsfBuildVario(CrsfFrame &out, const Packets::HandsetTelem &t) {
  uint8_t p[2];
  putBe16(p, static_cast<uint16_t>(roundToI32(t.vertical_speed * 100.0, -32768, 32767)));
  crsfBuildFrame(out, kCrsfVario, p, sizeof p);
}

void crsfBuildBattery(CrsfFrame &out, const Packets::HandsetTelem &t) {
  uint8_t p[8] = {};
  putBe16(p + 0, static_cast<uint16_t>(roundToI32(t.battery * 10.0, 0, 32767)));
  // current, capacity used and remaining percent are unknown: 0
  crsfBuildFrame(out, kCrsfBattery, p, sizeof p);
}

bool CrsfTelemetryScheduler::next(int64_t nowNs, const Packets::LinkStats *remote,
                                  const LocalAirStats &local,
                                  const Packets::HandsetTelem *telem,
                                  int64_t telemRxNs, CrsfFrame &out) {
  constexpr size_t kLs = static_cast<size_t>(TelemKind::LinkStats);
  if (!everSent_[kLs] || nowNs - lastSent_[kLs] >= kLinkStatsPeriodNs) {
    everSent_[kLs] = true;
    lastSent_[kLs] = nowNs;
    crsfBuildLinkStatistics(out, remote, local);
    return true;
  }
  if (telem == nullptr || nowNs - telemRxNs >= kStaleNs) return false;

  constexpr uint8_t kFirst = static_cast<uint8_t>(TelemKind::FlightMode);
  constexpr uint8_t kCount = static_cast<uint8_t>(TelemKind::Count);
  for (uint8_t n = 0; n < kCount - kFirst; n++) {
    const uint8_t k = rr_;
    rr_ = (rr_ + 1 >= kCount) ? kFirst : static_cast<uint8_t>(rr_ + 1);
    if (everSent_[k] && nowNs - lastSent_[k] < kTelemMinPeriodNs) continue;
    switch (static_cast<TelemKind>(k)) {
    case TelemKind::FlightMode: crsfBuildFlightMode(out, *telem); break;
    case TelemKind::Gps: crsfBuildGps(out, *telem); break;
    case TelemKind::Attitude: crsfBuildAttitude(out, *telem); break;
    case TelemKind::Vario: crsfBuildVario(out, *telem); break;
    case TelemKind::Battery:
      if (!(telem->battery > 0.0f)) continue; // wire 0 (unknown) decodes to NaN: no frame
      crsfBuildBattery(out, *telem);
      break;
    default: continue;
    }
    everSent_[k] = true;
    lastSent_[k] = nowNs;
    return true;
  }
  return false;
}

} // namespace VCTR::ExVectrLink::netmode
