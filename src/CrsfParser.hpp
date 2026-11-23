#pragma once
// CRSF byte-stream framing as a pure class (no Arduino, no heap), so it is tested on the host (test/test_network_mode).
//
// Two modes:
//  - Legacy (default, what every FPV firmware uses): exactly the original CRSF::decodeByte() behaviour. A frame starts
//    at 0xC8, 0x00, 0xEC, 0xEA or 0xEE; a bad length or a bad CRC discards the whole buffer without rescanning.
//  - Hardened (network-mode TX module only, CRSF::setHardenedParser(true)): 0x00 is not a start byte (it is what the
//    half-duplex direction switch and a break look like), and after a bad length or CRC failure the bytes behind the
//    first one are rescanned for a real frame start instead of being thrown away, so one corrupt byte or one false
//    start costs one frame at most instead of the next 2..64 bytes.
//
// Counters are always kept (a few increments per byte, no behaviour effect).
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace VCTR::ExVectrLink {

class CrsfParser {
public:
  static constexpr uint8_t kMaxFrameLen = 64;
  static constexpr uint8_t kRecentBytes = 16;

  struct Stats {
    uint32_t bytesIn = 0;       ///< every byte fed
    uint32_t framesRc = 0;      ///< CRC-valid frames of type 0x16 (RC_CHANNELS_PACKED)
    uint32_t framesLua = 0;     ///< CRC-valid frames of type 0x28 / 0x2C / 0x2D (ping, parameter read, write)
    uint32_t framesOther = 0;   ///< any other CRC-valid frame
    uint32_t crcFail = 0;       ///< complete frames whose CRC did not match
    uint32_t badLen = 0;        ///< length byte out of range (resync)
    uint32_t droppedHunting = 0; ///< bytes discarded while waiting for a frame start
    uint32_t zeroStarts = 0;    ///< 0x00 seen as the first byte of a (tentative) frame
    uint32_t rescans = 0;       ///< hardened: failures after which the buffer tail was rescanned
    uint32_t staleResets = 0;   ///< partial frame discarded because no byte followed it for too long
  };

  void setHardened(bool on) { hardened_ = on; }
  bool hardened() const { return hardened_; }
  const Stats &stats() const { return stats_; }
  bool hasPartial() const { return pos_ > 0; }

  /// Drops a half-received frame (the caller decides by time; see CRSF::drainInput()).
  void resetPartial() {
    if (pos_ > 0) stats_.staleResets++;
    pos_ = 0;
  }

  /// Copies the last kRecentBytes raw bytes fed, oldest first (zero padded at the front before 16 bytes were seen).
  void copyRecent(uint8_t out[kRecentBytes]) const {
    for (uint8_t i = 0; i < kRecentBytes; i++) out[i] = recent_[(recentHead_ + i) % kRecentBytes];
  }

  /// Feeds one byte. `onFrame(const uint8_t *bytes, size_t len)` is called for each CRC-valid frame (sync, length,
  /// type, payload, CRC), possibly more than once in hardened mode (replay after a rescan).
  template <class F> void feed(uint8_t b, F &&onFrame) {
    stats_.bytesIn++;
    recent_[recentHead_] = b;
    recentHead_ = (recentHead_ + 1) % kRecentBytes;
    if (!hardened_) {
      step(b, onFrame, nullptr);
      return;
    }
    // Replay queue: at most kMaxFrameLen bytes (buffer + the new byte); every failure drops one byte of it.
    uint8_t q[kMaxFrameLen + 2];
    size_t qn = 0, qi = 0;
    q[qn++] = b;
    while (qi < qn) {
      bool failed = false;
      step(q[qi++], onFrame, &failed);
      if (failed && pos_ > 1) {
        // Buffer = [bad start, rest...]: replay rest + not yet processed bytes.
        stats_.rescans++;
        uint8_t nq[kMaxFrameLen + 2];
        size_t nn = 0;
        for (size_t i = 1; i < pos_; i++) nq[nn++] = buf_[i];
        for (size_t i = qi; i < qn; i++) nq[nn++] = q[i];
        pos_ = 0;
        memcpy(q, nq, nn);
        qn = nn;
        qi = 0;
      } else if (failed) {
        pos_ = 0;
      }
    }
  }

  /// Plain function-pointer-free convenience for tests: counts nothing, ignores frames.
  static uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
      crc ^= data[i];
      for (uint8_t k = 0; k < 8; k++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
  }

private:
  bool isStart(uint8_t b) const {
    return b == 0xC8 || b == 0xEC || b == 0xEA || b == 0xEE || (!hardened_ && b == 0x00);
  }

  /// One state-machine step. `failed` (hardened only) is set when buf_ holds a rejected frame prefix that the
  /// caller must rescan; in legacy mode the buffer is simply discarded here, as before.
  template <class F> void step(uint8_t b, F &onFrame, bool *failed) {
    if (pos_ == 0) {
      if (isStart(b)) {
        if (b == 0x00) stats_.zeroStarts++;
        buf_[pos_++] = b;
      } else {
        stats_.droppedHunting++;
      }
      return;
    }
    buf_[pos_++] = b;
    if (pos_ == 2) {
      if (buf_[1] < 2 || buf_[1] > kMaxFrameLen - 2) {
        stats_.badLen++;
        if (failed) *failed = true; else pos_ = 0;
      }
      return;
    }
    const uint8_t total = buf_[1] + 2;
    if (pos_ >= total) {
      if (crc8(&buf_[2], buf_[1] - 1) == buf_[total - 1]) {
        const uint8_t t = buf_[2];
        if (t == 0x16) stats_.framesRc++;
        else if (t == 0x28 || t == 0x2C || t == 0x2D) stats_.framesLua++;
        else stats_.framesOther++;
        pos_ = 0; // before the callback: it may do slow work, nothing here depends on it
        onFrame(static_cast<const uint8_t *>(buf_), static_cast<size_t>(total));
      } else {
        stats_.crcFail++;
        if (failed) *failed = true; else pos_ = 0;
      }
    }
  }

  uint8_t buf_[kMaxFrameLen];
  uint8_t pos_ = 0;
  bool hardened_ = false;
  uint8_t recent_[kRecentBytes] = {0};
  uint8_t recentHead_ = 0;
  Stats stats_;
};

} // namespace VCTR::ExVectrLink
