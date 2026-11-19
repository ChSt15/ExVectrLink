#pragma once
// The ExVectrNetwork air access on top of the FHSS slots (docs/network-mode.md "Air link"). Pure: no Arduino, no
// ExVectrCore, no FHSS include, no heap. The target glue (src_network/target/NetRadio.cpp) connects it to FHSS:
//   - FHSS calls fillTxPayload() in every own TX slot (FhssTxProviderI, pull mode). It builds one air packet from
//     the aggregator (priority classes, latest-wins, expiry evaluated at the slot start, so an RC_STATE that is
//     100 ms old when the slot starts is never sent).
//   - The FHSS receive handler calls onRadioPacket() with the 253-byte payload of every frame that passed the
//     trailer check. It only copies into a one-packet mailbox. process() (main loop, outside the slot deadline)
//     hands it to AirFrameLink::onAirPacket(), which validates CRC-32/version/direction/NETID and delivers the frames.
//   - setStatus() reports the FHSS state, power and antennas; localAirStats() derives the module's own LINK_STATS
//     and CRSF downlink fields from them.
// Threading: all calls come from the same cooperative loop (FHSS runs as a scheduler task in it), so the aggregator
// is never entered concurrently.
#include <stddef.h>
#include <stdint.h>

#include "ExVectrLink/datalink/AirProfile.hpp"
#include "ExVectrNetwork/TimeSourceI.hpp"
#include "ExVectrNetwork/link/AirAccessI.hpp"
#include "ExVectrNetwork/link/AirFrameLink.hpp"

#include "NetCrsf.hpp"

namespace VCTR::ExVectrLink::netmode {

/// Where a router reads the module's own radio measurements (its LINK_STATS and the CRSF downlink fields).
class AirStatsSourceI {
public:
  virtual ~AirStatsSourceI() = default;
  virtual LocalAirStats localAirStats() = 0;
};

/// "Radio off": zeros, air state Off. Used by host tests.
class StubAirStats : public AirStatsSourceI {
public:
  LocalAirStats localAirStats() override { return LocalAirStats(); }
};

/// Link quality as LINK_STATS defines it: valid air packets received in the last 1 s, in percent of the expected
/// count. Fixed storage for up to 64 packets per second.
class LqCounter {
public:
  static constexpr size_t kCap = 64;
  static constexpr int64_t kWindowNs = 1000000000LL;

  void onValid(int64_t nowNs) {
    t_[head_] = nowNs;
    head_ = (head_ + 1) % kCap;
    if (n_ < kCap) n_++;
  }
  void clear() { n_ = 0; }
  /// Count of valid packets with a time in (now - 1 s, now].
  uint32_t countLastSecond(int64_t nowNs) const {
    uint32_t c = 0;
    for (size_t i = 0; i < n_; i++) {
      const int64_t t = t_[(head_ + kCap - 1 - i) % kCap];
      if (t <= nowNs && nowNs - t < kWindowNs) c++;
    }
    return c;
  }
  uint8_t percent(int64_t nowNs, uint32_t expectedPerSecond) const {
    if (expectedPerSecond == 0) return 0;
    const uint32_t p = countLastSecond(nowNs) * 100u / expectedPerSecond;
    return static_cast<uint8_t>(p > 100 ? 100 : p);
  }

private:
  int64_t t_[kCap] = {};
  size_t head_ = 0, n_ = 0;
};

/// FHSS state, power and antennas as the glue reads them from the radio objects.
struct AirLinkStatus {
  Packets::AirState state = Packets::AirState::kOff;
  int8_t txPowerDbm = 0;
  uint8_t rxAntenna = 0; ///< diversity winner of the last packet
  uint8_t txAntenna = 0;
};

class NetAirAccess : public network::AirAccessI,
                     public ExVectrLink::datalink::FhssTxProviderI,
                     public AirStatsSourceI {
public:
  using Link = network::AirFrameLink<16, 8>;
  static constexpr size_t kMaxAir = 255;
  static constexpr size_t kQueueCapacity = 16 + 8;

  struct Counters {
    uint32_t txSlots = 0;        ///< fillTxPayload() calls (own TX slots that were sent)
    uint32_t rxFrames = 0;       ///< frames handed over by FHSS (trailer valid)
    uint32_t keepAlives = 0;     ///< FHSS keep-alives without an air packet (zero padding), not passed on
    uint32_t mailboxOverrun = 0; ///< a packet replaced one that process() had not taken yet
  } counters;

  /// `isMaster`: TX module (sync master, sends master -> slave and increments the superframe counter).
  /// `time` must be the same clock as the router's (the aggregator compares enqueue times with it).
  NetAirAccess(Link &link, network::TimeSourceI &time, bool isMaster, uint16_t expectedRxPerSecond)
      : link_(link), time_(time), master_(isMaster), expected_(expectedRxPerSecond) {
    network::AirLinkConfig cfg = link_.config();
    cfg.txIsSlaveToMaster = !isMaster;
    cfg.padToMaxLen = true;
    link_.setConfig(cfg);
    setSource(link_);
  }

  // ---- AirAccessI
  void setSource(network::AirPacketSourceI &src) override { src_ = &src; }
  uint16_t expectedRxPacketsPerSecond() const override { return expected_; }

  // ---- FhssTxProviderI: own TX slot, called inside the FHSS wakeup lead
  size_t fillTxPayload(uint8_t *buf, size_t maxLen, int64_t slotStartNs) override {
    if (!src_) return 0;
    if (master_) sfc_++;
    link_.setSuperframeCounter(master_ ? sfc_ : link_.airStats().lastRxSfc);
    // Expiry is evaluated at the slot start (the moment the packet leaves), never earlier.
    const int64_t now = time_.nowNs();
    const int64_t at = slotStartNs > now ? slotStartNs : now;
    sampleQueue(at);
    const size_t n = src_->buildAirPacket(buf, maxLen > kMaxAir ? kMaxAir : maxLen, at);
    counters.txSlots++;
    return n;
  }

  // ---- receive path
  /// From the FHSS receive handler: the frame payload without the FHSS trailer. Copies only.
  void onRadioPacket(const uint8_t *data, size_t len, int64_t rxTimeNs, int16_t rssi, int16_t snr) {
    counters.rxFrames++;
    if (pending_) counters.mailboxOverrun++;
    if (len > kMaxAir) len = kMaxAir;
    for (size_t i = 0; i < len; i++) box_[i] = data[i];
    boxLen_ = len;
    boxTime_ = rxTimeNs;
    boxRssi_ = rssi;
    boxSnr_ = snr;
    pending_ = true;
  }

  /// Main loop: validates and delivers a pending packet. Returns true if a packet was accepted.
  bool process() {
    if (!pending_) return false;
    pending_ = false;
    // A zero first byte is no air packet (air version 0 does not exist): an FHSS keep-alive (padding only).
    if (boxLen_ == 0 || box_[0] == 0) {
      counters.keepAlives++;
      return false;
    }
    const uint32_t before = link_.airStats().packetsRx;
    if (src_) src_->onAirPacket(box_, boxLen_, boxTime_, boxRssi_, boxSnr_);
    if (link_.airStats().packetsRx == before) return false;
    lq_.onValid(boxTime_);
    lastRssi_ = boxRssi_;
    lastSnr_ = boxSnr_;
    return true;
  }

  void setStatus(const AirLinkStatus &s) {
    if (s.state == Packets::AirState::kSearching && status_.state != Packets::AirState::kSearching) lq_.clear();
    status_ = s;
  }
  const AirLinkStatus &status() const { return status_; }

  // ---- AirStatsSourceI
  LocalAirStats localAirStats() override {
    const int64_t now = time_.nowNs();
    LocalAirStats s;
    s.airState = status_.state;
    s.rxLq = (status_.state == Packets::AirState::kSearching || status_.state == Packets::AirState::kOff)
                 ? 0
                 : lq_.percent(now, expected_);
    s.rssi = clampI8(lastRssi_);
    s.snr = clampI8(lastSnr_);
    s.txPower = status_.txPowerDbm;
    s.antenna = static_cast<uint8_t>(((status_.txAntenna & 0x0F) << 4) | (status_.rxAntenna & 0x0F));
    s.airRxOk = link_.airStats().packetsRx;
    s.airRxBad = link_.airStats().packetsRejected;
    const auto &a = link_.aggregator().stats();
    uint32_t dropped = a.invalid, expired = 0;
    for (uint8_t c = 0; c < network::kAggClasses; c++) {
      dropped += a.rejected[c] + a.evicted[c];
      expired += a.expired[c];
    }
    s.framesDropped = dropped;
    s.framesExpired = expired;
    sampleQueue(now);
    s.queuePeak = curPeak_ > prevPeak_ ? curPeak_ : prevPeak_;
    return s;
  }

  uint8_t superframeCounter() const { return sfc_; }

private:
  static int8_t clampI8(int16_t v) { return static_cast<int8_t>(v < -128 ? -128 : (v > 127 ? 127 : v)); }

  /// Peak queue fill in percent, over the current and the previous 1 s window.
  void sampleQueue(int64_t now) {
    if (now - windowStart_ >= LqCounter::kWindowNs) {
      prevPeak_ = (now - windowStart_ >= 2 * LqCounter::kWindowNs) ? 0 : curPeak_;
      curPeak_ = 0;
      windowStart_ = now;
    }
    const size_t q = link_.aggregator().queuedTotal();
    const uint8_t p = static_cast<uint8_t>(q * 100 / kQueueCapacity);
    if (p > curPeak_) curPeak_ = p;
  }

  Link &link_;
  network::TimeSourceI &time_;
  network::AirPacketSourceI *src_ = nullptr;
  bool master_;
  uint16_t expected_;
  uint8_t sfc_ = 0;

  uint8_t box_[kMaxAir] = {};
  size_t boxLen_ = 0;
  int64_t boxTime_ = 0;
  int16_t boxRssi_ = 0, boxSnr_ = 0;
  bool pending_ = false;

  LqCounter lq_;
  int16_t lastRssi_ = 0, lastSnr_ = 0;
  AirLinkStatus status_;
  int64_t windowStart_ = 0;
  uint8_t curPeak_ = 0, prevPeak_ = 0;
};

/// LINK_STATS (port 0x01) of a link module from its local air stats. Counters wrap at 16 bits on the wire.
inline Packets::LinkStats makeLinkStats(const LocalAirStats &l, double uptimeS) {
  Packets::LinkStats ls;
  ls.uptime = uptimeS;
  ls.air_state = l.airState;
  ls.rx_lq = l.rxLq;
  ls.rssi = l.rssi;
  ls.snr = l.snr;
  ls.tx_power = l.txPower;
  ls.antenna = l.antenna;
  ls.air_rx_ok = static_cast<uint16_t>(l.airRxOk);
  ls.air_rx_bad = static_cast<uint16_t>(l.airRxBad);
  ls.frames_dropped = static_cast<uint16_t>(l.framesDropped);
  ls.frames_expired = static_cast<uint16_t>(l.framesExpired);
  ls.queue_peak = l.queuePeak;
  return ls;
}

/// TX-module power steps from the vehicle receiver's LINK_STATS (node 3), the same rules as the FPV firmware
/// (src/DynamicPowerSetup.cpp): one step up at the end of an RX slot after 2 or more consecutive lost RX slots, or
/// while node 3's LINK_STATS is stale (> 1 s); the RSSI/SNR/LQ-based up/down steps are DynamicPower::update(),
/// fed with every new node-3 LINK_STATS.
class NetPowerPolicy {
public:
  static constexpr int64_t kRemoteStaleNs = 1000000000LL;

  /// End of an own RX slot. Returns true if the power should go one step up.
  bool onRxSlotEnd(bool receivedPacket, bool remoteFresh) {
    if (receivedPacket) lost_ = 0;
    else if (lost_ < 255) lost_++;
    return lost_ >= 2 || !remoteFresh;
  }
  /// A node-3 LINK_STATS with this receive time was seen. Returns true once per new packet.
  bool newRemoteStats(int64_t rxNs) {
    if (rxNs == lastRemoteNs_) return false;
    lastRemoteNs_ = rxNs;
    return true;
  }
  static bool remoteFresh(bool hasValue, int64_t rxNs, int64_t nowNs) {
    return hasValue && nowNs - rxNs < kRemoteStaleNs;
  }

private:
  uint8_t lost_ = 0;
  int64_t lastRemoteNs_ = 0;
};

} // namespace VCTR::ExVectrLink::netmode
