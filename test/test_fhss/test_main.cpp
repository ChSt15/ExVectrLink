// Host tests of the real FHSS class (lib/ExVectrLink/src/datalink/FHSS.cpp) on fake radios and a fake clock:
//  - FPV defaults: a golden trace of everything both ends transmit over 3 s of simulated time. FPV must stay
//    bit-for-bit identical (same bytes, same channels, same slot timing). The hash below was recorded from the
//    unmodified FHSS before the network-mode parameters were added.
//  - Network profile v1 (20 ms slots, 1:1 roles, 2 slots per hop, fixed 255-byte frames, own link version and key):
//    lock from Searching, holding sync with a crystal mismatch, recovery after a 5 s RF block, the TX payload provider,
//    FPV/network frames never accepted by each other.
// Run: ~/.platformio/penv/bin/pio test -e native_fhss
#include <unity.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <stdint.h>
#include <vector>

#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/time_base.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrLink/datalink/FHSS.hpp"
#include "ExVectrLink/datalink/FhssMath.hpp"

using namespace VCTR;
using namespace VCTR::ExVectrLink::datalink;
namespace dl = VCTR::network::datalink;

// ------------------------------------------------------------------------------------------------ fake clock
// Core::NowNs() is provided here instead of by a platform. The simulation loop advances time in 20 us steps. Code
// that spins on the clock (FHSS busy-waits to the slot boundary, the in-flight grace window) advances it by 1 us per
// 50 reads, so loops terminate and the result does not depend on how often normal code reads the clock.
static int64_t g_now = 1000000000LL;
static int g_spin = 0;
int64_t VCTR::Core::NowNs() {
  if (++g_spin > 50) {
    g_spin = 0;
    g_now += 1000;
  }
  return g_now;
}
double VCTR::Core::NowS() { return (double)g_now * 1e-9; }
void VCTR::Core::delay(int64_t d) { g_now += d; }
static void advance(int64_t ns) {
  g_now += ns;
  g_spin = 0;
}

// ------------------------------------------------------------------------------------------------ fake air
struct Transmission {
  int src;
  size_t channel;
  int64_t start, end;
  std::vector<uint8_t> bytes;
};

struct Air {
  std::vector<Transmission> inFlight;
  int64_t blockStart = 0, blockEnd = 0; ///< every transmission starting in [blockStart, blockEnd) is lost
  uint64_t hash = 1469598103934665603ULL;
  uint32_t txCount = 0;
  bool recordTrace = false;
  void mix(uint64_t v) {
    for (int i = 0; i < 8; i++) {
      hash ^= (v >> (8 * i)) & 0xFF;
      hash *= 1099511628211ULL;
    }
  }
};

class FakeRadio : public dl::Sx1280_DirectI {
public:
  /// `preambleGraceNs`: a reception is still caught if RX is armed (on the right channel) up to this long after the
  /// packet started, as the real radio only needs part of the preamble. The FPV golden run uses 0 (it was recorded
  /// that way); the network runs use 150 us (8 of the 12 SF5/BW1600 preamble symbols). Needed because this
  /// simulation runs both ends on one thread: one end's busy-wait before its TX slot delays the other end's wakeup.
  FakeRadio(Air &air, int id, int64_t toaNs, int64_t hwLatencyNs, int64_t preambleGraceNs = 0)
      : air_(air), id_(id), toa_(toaNs), latency_(hwLatencyNs), grace_(preambleGraceNs) {}

  // HasChannels
  size_t getNumChannels() const override { return 20; }
  size_t getCurrentChannel() const override { return channel_; }
  void setChannel(size_t c) override {
    if (c != channel_) truncateIfTx();
    if (c != channel_) abortRx();
    channel_ = c;
  }

  bool configureRadio() override { return true; }
  void startRx(int64_t) override {
    if (mode_ == Mode::Tx) truncateIfTx();
    if (mode_ != Mode::Rx) {
      mode_ = Mode::Rx;
      rxArmedAt_ = g_now;
    }
  }
  int16_t getPacketRSSI() const override { return -60; }
  int16_t getPacketSNR() const override { return 9; }
  network::DataPacket getRxPacket() const override { return rxPacket_; }
  uint32_t getRxPacketCount() const override { return rxCount_; }
  void fetchRxPayload() override {
    if (!pendingPayload_) return;
    pendingPayload_ = false;
    rxPacket_.payload.clear();
    for (uint8_t b : pendingBytes_) rxPacket_.payload.append(b);
    rxPacket_.timestamp = pendingTs_;
  }
  bool setupTxPacket(const network::DataPacket &p) override {
    if (fixedLen_ && p.payload.size() != fixedLen_ && mode_pkt_ == dl::SX1280_PacketMode::Fixed) return false;
    txBytes_.clear();
    for (size_t i = 0; i < p.payload.size(); i++) txBytes_.push_back(p.payload[i]);
    return true;
  }
  bool silentStartTx = false; ///< test hook: startTx() does nothing, like Sx1280_Direct::startTx() without a loaded packet
  void startTx() override {
    if (silentStartTx) return;
    abortRx();
    mode_ = Mode::Tx;
    Transmission t{id_, channel_, g_now, g_now + toa_, txBytes_};
    air_.txCount++;
    if (air_.recordTrace) {
      air_.mix((uint64_t)id_);
      air_.mix((uint64_t)channel_);
      air_.mix((uint64_t)(g_now / 1000)); // us
      air_.mix((uint64_t)txBytes_.size());
      for (uint8_t b : txBytes_) air_.mix(b);
    }
    lastTx = t;
    txLog.push_back(t);
    if (!(t.start >= air_.blockStart && t.start < air_.blockEnd)) air_.inFlight.push_back(t);
    txEnd_ = t.end;
  }
  void setFrequency(uint32_t) override {}
  void setSpreadingFactor(dl::SX1280_SF) override {}
  void setBandwidth(dl::SX1280_BW) override {}
  void setCodingRate(dl::SX1280_CR) override {}
  void setTxPower(int8_t p) override { power_ = p; }
  uint8_t getTxPower() const override { return (uint8_t)power_; }
  void setTxMaxPower(int8_t) override {}
  void setPacketMode(dl::SX1280_PacketMode m) override { mode_pkt_ = m; }
  void setFixedPacketLength(uint8_t l) override { fixedLen_ = l; }
  void setPAdbm(uint8_t) override {}
  void setAutoFS(bool) override {}
  void setIdle() override {
    abortRx();
    mode_ = Mode::Idle;
  }
  void push(bool) override {
    if (mode_ == Mode::Tx && g_now >= txEnd_) mode_ = Mode::Idle;
    if (mode_ == Mode::Rx) return; // the real driver also keeps listening when re-armed with the same timeout
    abortRx();
    mode_ = Mode::Idle;
  }
  void pull() override {
    if (mode_ == Mode::Tx && g_now >= txEnd_) mode_ = Mode::Idle;
    if (mode_ != Mode::Rx) return;
    for (auto it = air_.inFlight.begin(); it != air_.inFlight.end();) {
      const Transmission &t = *it;
      if (t.src != id_ && t.channel == channel_ && t.end <= g_now && t.start + grace_ >= rxArmedAt_ &&
          t.start + grace_ >= lastChannelChange_) {
        rxCount_++;
        pendingPayload_ = true;
        pendingBytes_ = t.bytes;
        pendingTs_ = t.start + latency_; // driver: RX_DONE minus time on air; latency = TX ramp etc.
        it = air_.inFlight.erase(it);
        continue;
      }
      ++it;
    }
    // forget transmissions that are long over
    for (auto it = air_.inFlight.begin(); it != air_.inFlight.end();) {
      if (it->end + 50000000LL < g_now) it = air_.inFlight.erase(it);
      else ++it;
    }
  }
  bool isReceivingPacket() const override {
    if (mode_ != Mode::Rx) return false;
    for (const auto &t : air_.inFlight)
      if (t.src != id_ && t.channel == channel_ && t.start <= g_now && g_now < t.end && t.start + grace_ >= rxArmedAt_)
        return true;
    return false;
  }

  Transmission lastTx;
  std::vector<Transmission> txLog;
  uint32_t txTruncated = 0; ///< own transmissions cut short by a channel change or RX arm while still on air

private:
  enum class Mode { Idle, Rx, Tx };
  /// The real radio aborts a running TX on setRx / channel change: the receiver then sees a broken frame (lost).
  void truncateIfTx() {
    if (mode_ != Mode::Tx || g_now >= txEnd_) return;
    for (auto it = air_.inFlight.begin(); it != air_.inFlight.end(); ++it) {
      if (it->src == id_ && it->start == lastTx.start) {
        air_.inFlight.erase(it);
        break;
      }
    }
    txTruncated++;
  }
  void abortRx() {
    lastChannelChange_ = g_now;
    rxArmedAt_ = g_now;
  }
  Air &air_;
  int id_;
  int64_t toa_, latency_, grace_;
  size_t channel_ = 0;
  Mode mode_ = Mode::Idle;
  int64_t rxArmedAt_ = 0, lastChannelChange_ = 0, txEnd_ = 0;
  std::vector<uint8_t> txBytes_;
  uint32_t rxCount_ = 0;
  bool pendingPayload_ = false;
  std::vector<uint8_t> pendingBytes_;
  int64_t pendingTs_ = 0;
  network::DataPacket rxPacket_;
  int8_t power_ = 0;
  uint8_t fixedLen_ = 0;
  dl::SX1280_PacketMode mode_pkt_ = dl::SX1280_PacketMode::Limited;
};

// FHSS adds itself to the global Core scheduler in its constructor. The tests never tick that scheduler (FHSS
// objects are leaked so it never holds a dangling pointer); they run the FHSS tasks with this loop instead, which
// does what Scheduler::tick() does for a single due task: init once, then taskRun() once the release has passed.
static void runDue(FHSS &a, FHSS &b) {
  FHSS *tasks[2] = {&a, &b};
  for (FHSS *t : tasks) {
    if (Core::NowNs() <= t->getRelease()) continue;
    if (!t->getInitialised()) {
      t->setInitialised(true);
      t->taskInit();
    }
    t->taskRun();
  }
}

// ------------------------------------------------------------------------------------------------ FPV golden
// Recorded from the unmodified FHSS (lib/ExVectrLink f3204bb) with this harness.
static constexpr uint64_t kFpvGoldenHash = 0x823F0C9ACAA90BE4ULL;
static constexpr uint32_t kFpvGoldenTxCount = 292;

struct FpvRun {
  uint64_t hash;
  uint32_t txCount;
  uint32_t rxPayloads[2];
  FHSSState rxState;
};

static FpvRun runFpv() {
  g_now = 1000000000LL;
  Air air;
  air.recordTrace = true;
  // FPV: SF7/BW800/LI4/8, 12 bytes on air -> about 7.3 ms. Hardware latency = the default compensation (900 us).
  FakeRadio &radioTx = *new FakeRadio(air, 0, 7300000LL, 900000LL);
  FakeRadio &radioRx = *new FakeRadio(air, 1, 7300000LL, 900000LL);
  FHSS &tx = *new FHSS(radioTx), &rx = *new FHSS(radioRx);
  tx.setIsRxSide(false);
  rx.setIsRxSide(true);
  FpvRun r{};
  tx.addReceiveHandler([&](const network::DataPacket &p) { r.rxPayloads[0] += p.payload.size(); });
  rx.addReceiveHandler([&](const network::DataPacket &p) { r.rxPayloads[1] += p.payload.size(); });
  uint8_t seqTx = 0, seqRx = 0;
  const int64_t end = g_now + 3000000000LL;
  while (g_now < end) {
    // The OTA queue pump: hand FHSS a fresh payload whenever its single slot is free (9 bytes uplink, 8 downlink).
    if (!tx.isChannelBlocked()) {
      network::DataPacket p;
      for (int i = 0; i < 9; i++) p.payload.append((uint8_t)(seqTx + i));
      seqTx++;
      tx.transmitDataframe(p);
    }
    if (!rx.isChannelBlocked()) {
      network::DataPacket p;
      for (int i = 0; i < 8; i++) p.payload.append((uint8_t)(0x80 + seqRx + i));
      seqRx++;
      rx.transmitDataframe(p);
    }
    runDue(tx, rx);
    advance(20000);
  }
  r.hash = air.hash;
  r.txCount = air.txCount;
  r.rxState = rx.getFhssState();
  return r;
}

void test_fpv_trace_is_unchanged() {
  FpvRun r = runFpv();
  char msg[160];
  snprintf(msg, sizeof msg, "FPV trace hash 0x%016llX tx %u rxPayloads %u/%u state %d",
           (unsigned long long)r.hash, (unsigned)r.txCount, (unsigned)r.rxPayloads[0], (unsigned)r.rxPayloads[1],
           (int)r.rxState);
  TEST_MESSAGE(msg);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)r.rxState); // the harness itself links up
  TEST_ASSERT_EQUAL_HEX64(kFpvGoldenHash, r.hash);
  TEST_ASSERT_EQUAL_UINT32(kFpvGoldenTxCount, r.txCount);
}


// ------------------------------------------------------------------------------------------------ network profile
struct CountingProvider : FhssTxProviderI {
  uint8_t tag;
  uint32_t calls = 0;
  size_t lastMaxLen = 0;
  int64_t lastSlotStart = 0, minGap = INT64_MAX;
  explicit CountingProvider(uint8_t t) : tag(t) {}
  size_t fillTxPayload(uint8_t *buf, size_t maxLen, int64_t slotStartNs) override {
    if (calls > 0 && slotStartNs - lastSlotStart < minGap) minGap = slotStartNs - lastSlotStart;
    calls++;
    lastMaxLen = maxLen;
    lastSlotStart = slotStartNs;
    buf[0] = tag;
    buf[1] = (uint8_t)calls;
    return 2; // FHSS pads to the fixed length
  }
};

struct NetPair {
  Air air;
  FakeRadio *rTx, *rRx;
  FHSS *tx, *rx;
  CountingProvider pTx{0x10}, pRx{0x18};
  uint32_t rxGot[2] = {0, 0}; ///< payloads delivered: [0] at TX side, [1] at RX side
  size_t rxLen[2] = {0, 0};
  int64_t lastRxAtRx = 0;
  int64_t firstSyncedNs = 0;

  explicit NetPair(int64_t txIntervalExtraNs = 0, bool fpvTx = false,
                   const AirProfile &prof = kNetworkProfileV1, int64_t toaNs = 10560000LL) {
    g_now = 1000000000LL;
    rTx = new FakeRadio(air, 0, toaNs, 900000LL, 150000LL);
    rRx = new FakeRadio(air, 1, toaNs, 900000LL, 150000LL);
    tx = new FHSS(*rTx);
    rx = new FHSS(*rRx);
    if (!fpvTx) {
      tx->applyProfile(prof);
      tx->setSlotInterval(prof.slotIntervalNs + txIntervalExtraNs); // crystal mismatch
      tx->setTxPayloadProvider(&pTx);
    }
    rx->applyProfile(prof);
    rx->setTxPayloadProvider(&pRx);
    tx->setIsRxSide(false);
    rx->setIsRxSide(true);
    tx->addReceiveHandler([this](const network::DataPacket &p) {
      rxGot[0]++;
      rxLen[0] = p.payload.size();
    });
    rx->addReceiveHandler([this](const network::DataPacket &p) {
      rxGot[1]++;
      rxLen[1] = p.payload.size();
      lastRxAtRx = g_now;
    });
  }
  void runFor(int64_t ns) {
    const int64_t end = g_now + ns;
    while (g_now < end) {
      runDue(*tx, *rx);
      if (firstSyncedNs == 0 && rx->getFhssState() == FHSSState::Synced) firstSyncedNs = g_now;
      advance(20000);
    }
  }
};

void test_net_profile_parameters() {
  FakeRadio r(*new Air, 0, 1, 1);
  FHSS &f = *new FHSS(r);
  TEST_ASSERT_EQUAL_UINT32(9, f.getMaxPacketSize()); // FPV default
  TEST_ASSERT_EQUAL_UINT8(15, f.getRxSlotIndex());
  TEST_ASSERT_EQUAL_UINT8(4, f.getSlotsPerHop());
  TEST_ASSERT_EQUAL_UINT8(2, f.getLinkVersion());
  f.applyProfile(kNetworkProfileV1);
  TEST_ASSERT_EQUAL_UINT32(253, f.getMaxPacketSize());
  TEST_ASSERT_EQUAL_INT64(20000000LL, f.getSlotInterval());
  TEST_ASSERT_EQUAL_UINT8(2, f.getRxSlotIndex());
  TEST_ASSERT_EQUAL_UINT8(2, f.getSlotsPerHop());
  TEST_ASSERT_EQUAL_UINT8(0x37, f.getFhssKey());
  // trailer limits
  f.setSlotsPerHop(16);
  TEST_ASSERT_EQUAL_UINT8(4, f.getSlotsPerHop());
  f.setRxSlotIndex(200);
  TEST_ASSERT_EQUAL_UINT8(64, f.getRxSlotIndex());
  f.setRxSlotIndex(1);
  TEST_ASSERT_EQUAL_UINT8(2, f.getRxSlotIndex());
}

void test_net_locks_from_searching_and_sends_full_frames() {
  NetPair n;
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Searching, (int)n.rx->getFhssState());
  n.runFor(10000000000LL); // 10 s
  char msg[120];
  snprintf(msg, sizeof msg, "network lock after %.2f s", (double)(n.firstSyncedNs - 1000000000LL) * 1e-9);
  TEST_MESSAGE(msg);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  TEST_ASSERT_TRUE(n.firstSyncedNs > 0 && n.firstSyncedNs - 1000000000LL < 6000000000LL);
  TEST_ASSERT_EQUAL_UINT32(255, n.rTx->lastTx.bytes.size());
  TEST_ASSERT_EQUAL_UINT32(255, n.rRx->lastTx.bytes.size());
  TEST_ASSERT_EQUAL_UINT32(253, n.rxLen[0]); // trailer removed
  TEST_ASSERT_EQUAL_UINT32(253, n.rxLen[1]);
  TEST_ASSERT_EQUAL_UINT32(253, n.pTx.lastMaxLen);
  // The RX side never transmits before it is Synced.
  for (const auto &t : n.rRx->txLog) TEST_ASSERT_TRUE(t.start >= n.firstSyncedNs - 20000000LL);
}

void test_net_one_to_one_rates_when_synced() {
  NetPair n;
  n.runFor(8000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  const uint32_t tx0 = n.pTx.calls, rx0 = n.pRx.calls, g0 = n.rxGot[0], g1 = n.rxGot[1];
  n.runFor(4000000000LL); // 4 s
  const uint32_t up = n.pTx.calls - tx0, down = n.pRx.calls - rx0;
  char msg[160];
  snprintf(msg, sizeof msg, "4 s synced: uplink tx %u rx %u, downlink tx %u rx %u", (unsigned)up,
           (unsigned)(n.rxGot[1] - g1), (unsigned)down, (unsigned)(n.rxGot[0] - g0));
  TEST_MESSAGE(msg);
  TEST_ASSERT_UINT32_WITHIN(2, 100, up);   // 25/s
  TEST_ASSERT_UINT32_WITHIN(2, 100, down); // 25/s
  TEST_ASSERT_TRUE(n.rxGot[1] - g1 >= 98);
  TEST_ASSERT_TRUE(n.rxGot[0] - g0 >= 98);
  TEST_ASSERT_EQUAL_INT64(40000000LL, n.pTx.minGap); // own TX slot every 40 ms, never more often
}

void test_net_holds_sync_with_crystal_mismatch() {
  NetPair n(400); // TX clock 20 ppm slow relative to RX
  n.runFor(8000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  const uint32_t desync0 = n.rx->getDesyncCounter(), g1 = n.rxGot[1], g0 = n.rxGot[0];
  n.runFor(30000000000LL); // 30 s
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  TEST_ASSERT_EQUAL_UINT32(desync0, n.rx->getDesyncCounter());
  TEST_ASSERT_TRUE(n.rxGot[1] - g1 >= 740); // of 750
  TEST_ASSERT_TRUE(n.rxGot[0] - g0 >= 740);
}

void test_net_recovers_after_5s_rf_block() {
  NetPair n(200);
  n.runFor(8000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  n.air.blockStart = g_now;
  n.air.blockEnd = g_now + 5000000000LL;
  const uint32_t rxTx0 = (uint32_t)n.rRx->txLog.size();
  n.runFor(5000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Searching, (int)n.rx->getFhssState()); // dropped after 3 s
  // The RX side stopped transmitting once it left Synced (about 3 s into the block): no blind TX.
  TEST_ASSERT_TRUE(n.rRx->txLog.size() - rxTx0 <= 3 * 25 + 2);
  const int64_t blockEnd = g_now;
  n.firstSyncedNs = 0;
  n.runFor(10000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  char msg[100];
  snprintf(msg, sizeof msg, "resynced %.2f s after the block", (double)(n.firstSyncedNs - blockEnd) * 1e-9);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE(n.firstSyncedNs - blockEnd < 6000000000LL);
}

static void v2TrialHoldsSync(const AirProfile &prof, uint32_t expectMaxLen, const char *label) {
  const auto timing = netSlotTiming(prof);
  NetPair n(200, false, prof, timing.toaNs); // airtime from the repo formula
  n.runFor(10000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  TEST_ASSERT_EQUAL_UINT32(expectMaxLen, n.pTx.lastMaxLen);
  const uint32_t desync0 = n.rx->getDesyncCounter(), g1 = n.rxGot[1], g0 = n.rxGot[0];
  n.runFor(20000000000LL); // 20 s
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  TEST_ASSERT_EQUAL_UINT32(desync0, n.rx->getDesyncCounter());
  char msg[120];
  snprintf(msg, sizeof msg, "%s 20 s: uplink %u, downlink %u of 500 each", label, (unsigned)(n.rxGot[1] - g1),
           (unsigned)(n.rxGot[0] - g0));
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE(n.rxGot[1] - g1 >= 490);
  TEST_ASSERT_TRUE(n.rxGot[0] - g0 >= 490);
  TEST_ASSERT_INT64_WITHIN(1000, 40000000LL, n.pTx.minGap);
  TEST_ASSERT_EQUAL_UINT32(0, n.rTx->txTruncated); // the RX-slot wakeup never cuts an own frame
  TEST_ASSERT_EQUAL_UINT32(0, n.rRx->txTruncated);
}

// 220 B frames, 15.41 ms on air: the shipped v2-trial profile.
void test_net_v2_trial_locks_and_holds_with_220b_frames() {
  v2TrialHoldsSync(kNetworkProfileV2Trial, 218, "v2-trial 220 B");
}

// The earlier 255 B variant (17.77 ms on air) is still supported by the code: it relies on the TX grace and the short
// RX-slot lead. Kept as a local fixture so those paths stay covered.
static AirProfile long255Profile() {
  AirProfile p = kNetworkProfileV2Trial;
  p.fixedFrameLength = 255;
  p.onAirBytes = 255;
  p.wakeupLeadRxNs = 1000000LL; // short RX-slot lead: the frame is still on air at the 2.5 ms wakeup
  return p;
}

void test_net_long_255b_frames_with_short_rx_lead_hold_sync() {
  static AirProfile p = long255Profile();
  v2TrialHoldsSync(p, 253, "255 B fixture");
}

void test_net_long_255b_frames_with_single_lead_cut_own_frames() {
  // Negative control: the 255 B / 17.77 ms frames with the usual 2.5 ms lead before RX slots. The wakeup at 17.5 ms hops
  // and arms RX while the own frame is still on air; the simulation models that as a lost frame.
  static AirProfile oneLead = long255Profile();
  oneLead.wakeupLeadRxNs = 0;
  NetPair n(200, false, oneLead, netSlotTiming(oneLead).toaNs);
  n.runFor(10000000000LL);
  TEST_ASSERT_TRUE(n.rTx->txTruncated > 50);
}

void test_net_v2_trial_single_lead_clean_at_220b() {
  // The shipped 220 B profile uses the single 2.5 ms lead (wakeupLeadRxNs = 0): the own frame ends 2.09 ms before the
  // wakeup, so nothing is cut.
  TEST_ASSERT_EQUAL_INT64(0, kNetworkProfileV2Trial.wakeupLeadRxNs);
  NetPair n(200, false, kNetworkProfileV2Trial, netSlotTiming(kNetworkProfileV2Trial).toaNs);
  n.runFor(10000000000LL);
  TEST_ASSERT_EQUAL_UINT32(0, n.rTx->txTruncated);
}

// Build-flag experiment (-DEXVECTRLINK_NET_WAKEUP_LEAD_NS=3500000), emulated through a profile copy: v1 with a 3.5 ms
// wakeup lead (before both TX and RX slots) locks, holds sync and never cuts an own frame.
void test_net_v1_with_3p5ms_wakeup_lead_holds_sync() {
  static AirProfile lead35 = kNetworkProfileV1;
  lead35.wakeupLeadNs = 3500000LL;
  const auto t = netSlotTiming(lead35);
  TEST_ASSERT_TRUE(t.fitsStrict);
  v2TrialHoldsSync(lead35, 253, "v1 lead 3.5 ms");
}

void test_fpv_and_network_never_accept_each_other() {
  NetPair n(0, /*fpvTx=*/true); // FPV-default TX against a network-profile RX
  network::DataPacket p;
  for (int i = 0; i < 9; i++) p.payload.append((uint8_t)i);
  const int64_t end = g_now + 5000000000LL;
  while (g_now < end) {
    if (!n.tx->isChannelBlocked()) n.tx->transmitDataframe(p);
    runDue(*n.tx, *n.rx);
    advance(20000);
  }
  TEST_ASSERT_TRUE(n.air.txCount > 100);
  TEST_ASSERT_EQUAL_UINT32(0, n.rxGot[1]);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Searching, (int)n.rx->getFhssState());
}

// ------------------------------------------------------------------------------------------------ AIRDIAG counters
// SlotDiag counts must agree with what the fake radios really did, and must not change behaviour (the FPV golden
// above covers that: it runs the same FHSS code with the counters compiled in).
static void checkDiagAgainstRadios(NetPair &n) {
  const SlotDiag &t = n.tx->getSlotDiag();
  const SlotDiag &r = n.rx->getSlotDiag();
  // TX started == transmissions that really left the fake radio.
  TEST_ASSERT_EQUAL_UINT32(n.rTx->txLog.size(), t.txStarted);
  TEST_ASSERT_EQUAL_UINT32(n.rRx->txLog.size(), r.txStarted);
  // Every own TX slot is either started, skipped (not synced / setup) or lost in a catch-up; the latest may be pending.
  TEST_ASSERT_UINT32_WITHIN(1, t.txSlots, t.txStarted + t.txSkipNotSynced + t.txSkipSetup + t.txMissed);
  TEST_ASSERT_UINT32_WITHIN(1, r.txSlots, r.txStarted + r.txSkipNotSynced + r.txSkipSetup + r.txMissed);
  // RX slots split exactly into valid and none.
  TEST_ASSERT_EQUAL_UINT32(t.rxSlots, t.rxValid + t.rxNone);
  TEST_ASSERT_EQUAL_UINT32(r.rxSlots, r.rxValid + r.rxNone);
  // Valid slots == frames delivered to the handler (every network frame carries a payload).
  TEST_ASSERT_UINT32_WITHIN(1, n.rxGot[0], t.rxValid); // the newest one is counted at its slot end
  TEST_ASSERT_UINT32_WITHIN(1, n.rxGot[1], r.rxValid);
  TEST_ASSERT_TRUE(t.wakeups > 100 && r.wakeups > 100);
}

static void diagCleanRun(const AirProfile &prof, int64_t toaNs) {
  NetPair n(0, false, prof, toaNs);
  n.runFor(10000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  checkDiagAgainstRadios(n);
  const SlotDiag &t = n.tx->getSlotDiag();
  const SlotDiag &r = n.rx->getSlotDiag();
  char msg[200];
  snprintf(msg, sizeof msg, "tx: slots %u started %u rx %u/%u radio %u | rx side: slots %u started %u nsy %u rx %u/%u",
           (unsigned)t.txSlots, (unsigned)t.txStarted, (unsigned)t.rxValid, (unsigned)t.rxSlots,
           (unsigned)t.rxRadioPkts, (unsigned)r.txSlots, (unsigned)r.txStarted, (unsigned)r.txSkipNotSynced,
           (unsigned)r.rxValid, (unsigned)r.rxSlots);
  TEST_MESSAGE(msg);
  snprintf(msg, sizeof msg, "wakeups tx %u rx %u, wl1 tx %u rx %u, wakeLateMax tx %lld rx %lld us, prepMax %lld us",
           (unsigned)t.wakeups, (unsigned)r.wakeups, (unsigned)t.wakeLate1ms, (unsigned)r.wakeLate1ms,
           (long long)(t.wakeLateMaxNs / 1000), (long long)(r.wakeLateMaxNs / 1000), (long long)(t.prepMaxNs / 1000));
  TEST_MESSAGE(msg);
  // The RX side skipped its first TX slots while Searching/Syncing, the TX side (always synced) never.
  TEST_ASSERT_TRUE(r.txSkipNotSynced > 0);
  TEST_ASSERT_EQUAL_UINT32(0, t.txSkipNotSynced);
  TEST_ASSERT_EQUAL_UINT32(0, t.txSkipSetup);
  // The harness runs both ends on one thread, so one end's busy-wait delays the other's wakeup (up to ~2.5 ms): the
  // lateness counters are only checked for consistency here, and exactly in the stall test below.
  TEST_ASSERT_TRUE(t.wakeLate1ms <= t.wakeups && t.wakeLateMaxNs >= 0);
  // Nothing fast-forwards in a clean run.
  TEST_ASSERT_EQUAL_UINT32(0, t.missEvents);
  TEST_ASSERT_EQUAL_UINT32(0, t.txMissed);
  TEST_ASSERT_EQUAL_UINT32(0, t.txLateStart);
  // Radio packets seen == frames passing the trailer check (nothing bad on this fake air).
  TEST_ASSERT_EQUAL_UINT32(0, t.rxTrailerBad);
  TEST_ASSERT_EQUAL_UINT32(0, r.rxTrailerBad);
  // Prep time is wakeup -> prep done: positive and inside the wakeup lead.
  TEST_ASSERT_TRUE(t.prepMaxNs >= 0 && t.prepMaxNs < prof.wakeupLeadNs);
  TEST_ASSERT_TRUE(t.prepSumNs / (int64_t)t.txStarted <= t.prepMaxNs);
  // Window reset zeroes only the maxima.
  n.tx->resetSlotDiagWindow();
  TEST_ASSERT_EQUAL_INT64(0, n.tx->getSlotDiag().prepMaxNs);
  TEST_ASSERT_EQUAL_UINT32(t.txStarted, n.tx->getSlotDiag().txStarted);
}

void test_diag_counters_v1() { diagCleanRun(kNetworkProfileV1, 10560000LL); }
void test_diag_counters_v2_trial() { diagCleanRun(kNetworkProfileV2Trial, netSlotTiming(kNetworkProfileV2Trial).toaNs); }

void test_diag_late_wakeup_and_rf_block() {
  NetPair n;
  n.runFor(8000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  n.tx->resetSlotDiagWindow();
  const SlotDiag before = n.tx->getSlotDiag();
  // A 12 ms scheduler stall on both ends: the next wakeup is ~12 ms late (far above the ~2.5 ms harness baseline).
  advance(12000000LL);
  n.runFor(1000000000LL);
  const SlotDiag &t = n.tx->getSlotDiag();
  TEST_ASSERT_TRUE(t.wakeLate1ms > before.wakeLate1ms);
  TEST_ASSERT_TRUE(t.wakeLateMaxNs >= 9000000LL);
  // RF blocked for 1 s: the TX side keeps transmitting, receives nothing valid.
  n.air.blockStart = g_now;
  n.air.blockEnd = g_now + 1000000000LL;
  const SlotDiag mid = n.tx->getSlotDiag();
  n.runFor(1000000000LL);
  const SlotDiag &after = n.tx->getSlotDiag();
  TEST_ASSERT_TRUE(after.txStarted - mid.txStarted >= 24);
  TEST_ASSERT_TRUE(after.rxValid - mid.rxValid <= 1); // one frame in flight at the block start
  TEST_ASSERT_TRUE(after.rxNone - mid.rxNone >= 20);
}

// The silent-skip hypothesis: startTx() issued but the radio never transmits. txStarted rises, the radio's TX-done
// count (here: the fake's txLog) does not.
void test_diag_shows_silent_startTx() {
  NetPair n;
  n.runFor(8000000000LL);
  TEST_ASSERT_EQUAL_INT((int)FHSSState::Synced, (int)n.rx->getFhssState());
  const uint32_t started0 = n.rx->getSlotDiag().txStarted, done0 = (uint32_t)n.rRx->txLog.size();
  n.rRx->silentStartTx = true;
  n.runFor(2000000000LL);
  const uint32_t started = n.rx->getSlotDiag().txStarted - started0;
  const uint32_t done = (uint32_t)n.rRx->txLog.size() - done0;
  TEST_ASSERT_TRUE(started >= 45);
  TEST_ASSERT_EQUAL_UINT32(0, done);
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_fpv_trace_is_unchanged);
  RUN_TEST(test_net_profile_parameters);
  RUN_TEST(test_net_locks_from_searching_and_sends_full_frames);
  RUN_TEST(test_net_one_to_one_rates_when_synced);
  RUN_TEST(test_net_holds_sync_with_crystal_mismatch);
  RUN_TEST(test_net_recovers_after_5s_rf_block);
  RUN_TEST(test_net_v2_trial_locks_and_holds_with_220b_frames);
  RUN_TEST(test_net_long_255b_frames_with_short_rx_lead_hold_sync);
  RUN_TEST(test_net_long_255b_frames_with_single_lead_cut_own_frames);
  RUN_TEST(test_net_v2_trial_single_lead_clean_at_220b);
  RUN_TEST(test_net_v1_with_3p5ms_wakeup_lead_holds_sync);
  RUN_TEST(test_fpv_and_network_never_accept_each_other);
  RUN_TEST(test_diag_counters_v1);
  RUN_TEST(test_diag_counters_v2_trial);
  RUN_TEST(test_diag_late_wakeup_and_rf_block);
  RUN_TEST(test_diag_shows_silent_startTx);
  return UNITY_END();
}
