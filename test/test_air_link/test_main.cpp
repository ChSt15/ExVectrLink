// Host tests of the network-mode air link pieces that do not need FHSS itself (that is test_fhss):
//  - FhssMath / AirProfile: time on air, slot budget, expected packet rates, trailer seeds and counters.
//  - NetAirAccess with the real AirFrameLink: RC_STATE expiry at the slot start, priority order, downlink fill,
//    receive mailbox, keep-alives, LQ, LINK_STATS derivation, superframe counter, direction.
//  - RxRouter (node 3): egress filters, LINK_STATS 5 Hz, LQ 0 / Searching without air, no synthesised RC state.
//  - Link upload command (REQUEST action 10) on the RX module and end to end; UploadTriggers and NetPowerPolicy.
//  - End to end: TX router (node 2) + air access <-> air access + RX router (node 3) over simulated 20 ms slots.
// Run: ~/.platformio/penv/bin/pio test -e native
#include <unity.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <atomic>
#include <new>
#include <vector>

#include "ExVectrLink/datalink/AirProfile.hpp"
#include "ExVectrLink/datalink/FhssMath.hpp"
#include "LinkUploadCommand.hpp"
#include "NetAirAccess.hpp"
#include "NetLuaMenu.hpp"
#include "NetPower.hpp"
#include "NetworkRouter.hpp"
#include "RxRouter.hpp"

using namespace VCTR;
using namespace VCTR::ExVectrLink::netmode;
namespace dl = VCTR::ExVectrLink::datalink;
namespace fm = VCTR::ExVectrLink::datalink::fhss;
namespace net = VCTR::network;

// ---- allocation hook ----
static std::atomic<unsigned long> g_allocs{0};
void *operator new(size_t n) {
  g_allocs++;
  void *p = malloc(n ? n : 1);
  if (!p) abort();
  return p;
}
void *operator new[](size_t n) { return operator new(n); }
void operator delete(void *p) noexcept { free(p); }
void operator delete[](void *p) noexcept { free(p); }
void operator delete(void *p, size_t) noexcept { free(p); }
void operator delete[](void *p, size_t) noexcept { free(p); }

// ---- doubles ----
struct FakeClock : net::TimeSourceI {
  int64_t t = 1000000000LL;
  int64_t nowNs() override { return t; }
  void advanceMs(int ms) { t += static_cast<int64_t>(ms) * 1000000LL; }
};

struct Frame {
  uint8_t d[net::kMaxNetFrame];
  uint8_t len = 0;
};

struct RecordLink : net::FrameLinkI {
  std::vector<Frame> sent;
  FakeClock *clk;
  explicit RecordLink(FakeClock &c) : clk(&c) { sent.reserve(8192); }
  bool send(const uint8_t *f, uint8_t len, int64_t) override {
    Frame it;
    it.len = len;
    memcpy(it.d, f, len);
    sent.push_back(it);
    stats.txOk++;
    return true;
  }
  uint8_t mtu() const override { return net::kMaxNetFrame; }
  void inject(const Frame &f) {
    stats.rxOk++; // as a stream link counts a good frame
    net::FrameView v{f.d, f.len, clk->nowNs()};
    deliver(v);
  }
};

struct CollectSink : net::FrameSinkI {
  std::vector<Frame> got;
  void onFrame(const net::FrameView &f, uint8_t) override {
    Frame x;
    x.len = f.len;
    memcpy(x.d, f.data, f.len);
    got.push_back(x);
  }
};

static Frame makeData(uint8_t src, uint8_t dst, uint8_t port, uint8_t seq, const uint8_t *pl, size_t n,
                      net::wire::Qos qos = net::wire::Qos::Telemetry, bool latest = true) {
  Frame f;
  net::wire::NetHeader h;
  h.type = 0;
  h.qos = static_cast<uint8_t>(qos);
  h.latest = latest;
  h.ttl = 3;
  h.dst = dst;
  h.src = src;
  net::wire::encodeNetHeader(h, f.d, sizeof f.d);
  net::wire::DataHeader dh;
  dh.port = port;
  dh.seq = seq;
  net::wire::encodeDataHeader(dh, f.d + 4, sizeof f.d - 4);
  if (n) memcpy(f.d + 6, pl, n);
  f.len = static_cast<uint8_t>(6 + n);
  return f;
}

static Frame makeHeartbeat(uint8_t src, uint8_t role) {
  Frame f;
  net::wire::NetHeader h;
  h.type = static_cast<uint8_t>(net::wire::FrameType::Heartbeat);
  h.qos = static_cast<uint8_t>(net::wire::Qos::Link);
  h.latest = true;
  h.dst = 0xFF;
  h.src = src;
  h.ttl = 3;
  net::wire::encodeNetHeader(h, f.d, sizeof f.d);
  net::wire::Heartbeat hb;
  hb.role = role;
  hb.schemaHash = Packets::kSchemaHash;
  net::wire::encodeHeartbeat(hb, f.d + 4, sizeof f.d - 4);
  f.len = 15;
  return f;
}

static Frame rcStateFrame(uint8_t src, uint8_t dst, uint8_t seq) {
  Packets::RcState rc;
  rc.flags = 1;
  for (int i = 0; i < 16; i++) rc.ch[i] = 100;
  uint8_t buf[Packets::RcState::kMaxSize];
  size_t n = rc.encode(buf, sizeof buf);
  return makeData(src, dst, Packets::RcState::kPort, seq, buf, n, net::wire::Qos::Control, true);
}

/// CMD_STATE (port 0x13, 14 B). The routers never parse it; the payload carries `tag` in byte 0 to tell frames apart.
static Frame cmdStateFrame(uint8_t src, uint8_t dst, uint8_t seq, uint8_t tag = 0) {
  uint8_t pl[Packets::CmdState::kMaxSize] = {};
  pl[0] = tag;
  return makeData(src, dst, Packets::CmdState::kPort, seq, pl, sizeof pl, net::wire::Qos::Control, true);
}

static bool isData(const Frame &f, uint8_t port) { return f.len >= 6 && (f.d[0] & 0x30) == 0 && f.d[4] == port; }
static bool isHb(const Frame &f) { return f.len >= 4 && ((f.d[0] >> 4) & 3) == 1; }
static int countData(const std::vector<Frame> &v, uint8_t port, int src = -1, int dst = -1) {
  int n = 0;
  for (auto &f : v)
    if (isData(f, port) && (src < 0 || f.d[3] == src) && (dst < 0 || f.d[2] == dst)) n++;
  return n;
}
static int countHb(const std::vector<Frame> &v, int src) {
  int n = 0;
  for (auto &f : v)
    if (isHb(f) && f.d[3] == src) n++;
  return n;
}

/// Parses an air packet into frames.
static std::vector<Frame> parseAir(const uint8_t *buf, size_t n) {
  std::vector<Frame> out;
  net::wire::AirPacketReader r;
  if (!r.parse(buf, n)) return out;
  const uint8_t *f;
  size_t fl;
  while (r.next(f, fl)) {
    Frame x;
    memcpy(x.d, f, fl);
    x.len = static_cast<uint8_t>(fl);
    out.push_back(x);
  }
  return out;
}

// =========================================================================================== FhssMath, profiles

void test_time_on_air_profile_v1() {
  const float us = fm::loraTimeOnAirUs(dl::kNetworkProfileV1.lora, 255);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 10560.0f, us); // the user's figure (Revision 1): 10.56 ms
  // 8 symbols and no CRC (the alternative in the Revision box): 10.38 ms
  fm::LoRaParams p = dl::kNetworkProfileV1.lora;
  p.preamble = 8;
  p.crcOn = false;
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 10380.0f, fm::loraTimeOnAirUs(p, 255));
  // Design table 2.3 cross-check: SF5/BW800/CR4/6, explicit header, 255 B = 25.4 ms
  fm::LoRaParams d{5, 812500, 2, 12, true, true};
  TEST_ASSERT_FLOAT_WITHIN(100.0f, 25400.0f, fm::loraTimeOnAirUs(d, 255));
}

void test_slot_budget() {
  // network: 10.56 ms in 20 ms with 2 ms guard fits; it would not fit a 10 ms slot
  TEST_ASSERT_TRUE(fm::slotFits(dl::kNetworkProfileV1.lora, 255, dl::kNetworkProfileV1.slotIntervalNs, 2000000));
  TEST_ASSERT_FALSE(fm::slotFits(dl::kNetworkProfileV1.lora, 255, 10000000, 0));
  // FPV profile fits its 10 ms slot
  TEST_ASSERT_TRUE(fm::slotFits(dl::kFpvProfile.lora, dl::kFpvProfile.onAirBytes, dl::kFpvProfile.slotIntervalNs, 1000000));
  // the network wakeup lead leaves the busy-wait inside the slot
  TEST_ASSERT_TRUE(dl::kNetworkProfileV1.wakeupLeadNs < dl::kNetworkProfileV1.slotIntervalNs / 4);
}

void test_expected_rates() {
  const auto &n = dl::kNetworkProfileV1;
  TEST_ASSERT_EQUAL_UINT32(25, fm::expectedRxPacketsPerSecond(false, n.rolePeriod, n.slotIntervalNs));
  TEST_ASSERT_EQUAL_UINT32(25, fm::expectedRxPacketsPerSecond(true, n.rolePeriod, n.slotIntervalNs));
  const auto &f = dl::kFpvProfile;
  TEST_ASSERT_EQUAL_UINT32(6, fm::expectedRxPacketsPerSecond(false, f.rolePeriod, f.slotIntervalNs));  // 6.7
  TEST_ASSERT_EQUAL_UINT32(93, fm::expectedRxPacketsPerSecond(true, f.rolePeriod, f.slotIntervalNs));  // 93.3
}

void test_slot_roles_one_to_one() {
  // rolePeriod 2: the role counter alternates 0,1; master transmits on 1, follower on 0: exactly one TX per slot.
  for (size_t c = 0; c < 2; c++) TEST_ASSERT_TRUE(fm::slotIsTx(false, c) != fm::slotIsTx(true, c));
  int masterTx = 0;
  for (size_t c = 0; c < 15; c++) masterTx += fm::slotIsTx(false, c);
  TEST_ASSERT_EQUAL_INT(14, masterTx); // FPV: 14 of 15
}

void test_trailer_counters_and_seeds() {
  // FPV golden values (trailer byte 1 and the checksum seeds as FHSS always produced them)
  TEST_ASSERT_EQUAL_HEX8((14 << 2) | 3, fm::trailerCounterByte(14, 15, 3, 4));
  TEST_ASSERT_EQUAL_HEX8((1 << 2) | 0, fm::trailerCounterByte(16, 15, 4, 4));
  TEST_ASSERT_EQUAL_HEX8(0x02, fm::trailerSeed(0, 2, false));
  TEST_ASSERT_EQUAL_HEX8(0x42, fm::trailerSeed(0, 2, true));
  // network seeds differ from both FPV seeds in both directions
  const uint8_t nTx = fm::trailerSeed(dl::kNetworkProfileV1.hopKey, dl::kNetworkProfileV1.linkVersion, false);
  const uint8_t nRx = fm::trailerSeed(dl::kNetworkProfileV1.hopKey, dl::kNetworkProfileV1.linkVersion, true);
  const uint8_t all[4] = {0x02, 0x42, nTx, nRx};
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 4; j++) TEST_ASSERT_NOT_EQUAL(all[i], all[j]);
  const uint8_t d[3] = {0xAA, 0x0F, 0x11};
  TEST_ASSERT_EQUAL_HEX8(0x02 ^ 0xAA ^ 0x0F ^ 0x11, fm::trailerChecksum(0x02, d, 3));
  TEST_ASSERT_EQUAL_UINT8(3, fm::trailerSlotCounter((5 << 2) | 3));
  TEST_ASSERT_EQUAL_UINT8(5, fm::trailerRoleCounter((5 << 2) | 3));
}

void test_phase_wrap_and_filters() {
  TEST_ASSERT_EQUAL_INT64(-5000000, fm::wrapPhaseError(15000000, 20000000));
  TEST_ASSERT_EQUAL_INT64(3000000, fm::wrapPhaseError(43000000, 20000000));
  TEST_ASSERT_EQUAL_INT64(-3000000, fm::wrapPhaseError(-43000000, 20000000));
  int64_t off = 0;
  fm::syncingStep(off, 1000);
  TEST_ASSERT_EQUAL_INT64(200, off);
  int64_t to = 0, ot = 0;
  float ic = 0;
  for (int i = 0; i < 100000; i++) fm::syncedStep(to, ot, ic, 1000000, 20000000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 200000.0f, ic); // clamped to 1 % of the slot
  TEST_ASSERT_FALSE(fm::syncLinkQualityOk(0.5f, 2));
  TEST_ASSERT_TRUE(fm::syncLinkQualityOk(0.51f, 2));
  TEST_ASSERT_FALSE(fm::syncLinkQualityOk(0.75f, 4));
}

// =========================================================================================== LqCounter

void test_lq_counter() {
  LqCounter lq;
  int64_t t = 5000000000LL;
  for (int i = 0; i < 25; i++) lq.onValid(t + i * 40000000LL);
  const int64_t end = t + 24 * 40000000LL;
  TEST_ASSERT_EQUAL_UINT8(100, lq.percent(end, 25));
  TEST_ASSERT_EQUAL_UINT8(100, lq.percent(end, 20)); // clamped
  TEST_ASSERT_EQUAL_UINT8(52, lq.percent(end + 500000000LL, 25)); // packets in the last 1 s: 13 of 25
  TEST_ASSERT_EQUAL_UINT8(0, lq.percent(end + 1000000000LL, 25));
  TEST_ASSERT_EQUAL_UINT8(0, lq.percent(end, 0));
}

// =========================================================================================== NetAirAccess

struct AccessRig {
  FakeClock clk;
  NetAirAccess::Link link;
  NetAirAccess acc;
  CollectSink sink;
  explicit AccessRig(bool master) : acc(link, clk, master, 25) { link.setSink(sink, 0); }
};

void test_rc_state_expiry_is_evaluated_at_slot_start() {
  AccessRig r(true);
  Frame rc = rcStateFrame(2, 1, 1);
  uint8_t buf[253];
  // queued at t0, slot starts at t0 + 99 ms: sent
  TEST_ASSERT_TRUE(r.link.send(rc.d, rc.len, r.clk.nowNs()));
  int64_t slot = r.clk.nowNs() + 99000000LL;
  size_t n = r.acc.fillTxPayload(buf, sizeof buf, slot);
  TEST_ASSERT_EQUAL_UINT32(253, n);
  TEST_ASSERT_EQUAL_INT(1, countData(parseAir(buf, n), 0x10));
  // queued at t0; built 97.5 ms later (inside the 2.5 ms wakeup lead) for a slot starting at t0 + 100 ms: NOT sent
  r.clk.advanceMs(200);
  TEST_ASSERT_TRUE(r.link.send(rc.d, rc.len, r.clk.nowNs()));
  const int64_t t0 = r.clk.nowNs();
  r.clk.t = t0 + 97500000LL;
  n = r.acc.fillTxPayload(buf, sizeof buf, t0 + 100000000LL);
  TEST_ASSERT_EQUAL_INT(0, countData(parseAir(buf, n), 0x10));
  // and it is not released later either
  r.clk.advanceMs(40);
  n = r.acc.fillTxPayload(buf, sizeof buf, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, countData(parseAir(buf, n), 0x10));
  TEST_ASSERT_TRUE(r.acc.localAirStats().framesExpired >= 1);
}

void test_priority_order_in_air_packet() {
  AccessRig r(true);
  const uint8_t pl[8] = {0};
  Frame tel = makeData(2, 1, 0x30, 1, pl, 8, net::wire::Qos::Telemetry);
  Frame ev = makeData(2, 1, 0x11, 1, pl, 8, net::wire::Qos::Event, false);
  Frame lnk = makeData(2, 0xFF, 0x01, 1, pl, 8, net::wire::Qos::Link);
  Frame rc = rcStateFrame(2, 1, 1);
  const int64_t now = r.clk.nowNs();
  r.link.send(tel.d, tel.len, now);
  r.link.send(ev.d, ev.len, now);
  r.link.send(lnk.d, lnk.len, now);
  r.link.send(rc.d, rc.len, now);
  uint8_t buf[253];
  size_t n = r.acc.fillTxPayload(buf, sizeof buf, now);
  auto f = parseAir(buf, n);
  TEST_ASSERT_EQUAL_INT(4, (int)f.size());
  TEST_ASSERT_EQUAL_UINT8(0x10, f[0].d[4]);
  TEST_ASSERT_EQUAL_UINT8(0x01, f[1].d[4]);
  TEST_ASSERT_EQUAL_UINT8(0x11, f[2].d[4]);
  TEST_ASSERT_EQUAL_UINT8(0x30, f[3].d[4]);
}

void test_downlink_fill_throughput() {
  // The RX module with a vehicle producing more telemetry than the air carries: 40 streams of 40-byte frames,
  // refreshed every 40 ms. Each own TX slot (25 per second) must carry a full packet.
  AccessRig r(false);
  uint8_t pl[34];
  memset(pl, 0x5A, sizeof pl);
  uint32_t frameBytes = 0;
  int64_t slot = r.clk.nowNs();
  for (int s = 0; s < 25; s++) {
    for (uint8_t k = 0; k < 40; k++) {
      Frame f = makeData(1, 0, (uint8_t)(0x20 + k), (uint8_t)s, pl, sizeof pl, net::wire::Qos::Telemetry);
      r.link.send(f.d, f.len, r.clk.nowNs());
    }
    uint8_t buf[253];
    size_t n = r.acc.fillTxPayload(buf, sizeof buf, slot);
    TEST_ASSERT_EQUAL_UINT32(253, n);
    for (auto &f : parseAir(buf, n)) frameBytes += f.len;
    r.clk.advanceMs(40);
    slot = r.clk.nowNs();
  }
  char msg[80];
  snprintf(msg, sizeof msg, "downlink frame bytes in 1 s: %u", (unsigned)frameBytes);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE(frameBytes >= 5000); // design / bench target: at least 5 kB/s of frames
  TEST_ASSERT_TRUE(r.acc.localAirStats().queuePeak > 50);
}

void test_receive_mailbox_keepalive_and_lq() {
  AccessRig master(true), slave(false);
  slave.clk.t = master.clk.t;
  Frame rc = rcStateFrame(2, 1, 9);
  master.link.send(rc.d, rc.len, master.clk.nowNs());
  slave.acc.setStatus(AirLinkStatus{Packets::AirState::kSynced, 12, 0, 0});
  uint8_t buf[253];
  for (int i = 0; i < 25; i++) {
    size_t n = master.acc.fillTxPayload(buf, sizeof buf, master.clk.nowNs());
    slave.acc.onRadioPacket(buf, n, slave.clk.nowNs(), -71, 7);
    TEST_ASSERT_TRUE(slave.acc.process());
    master.clk.advanceMs(40);
    slave.clk.advanceMs(40);
  }
  TEST_ASSERT_EQUAL_INT(1, countData(slave.sink.got, 0x10, 2, 1));
  // 40 ms after the last packet the first one is exactly 1 s old and out of the window: 24 of 25.
  LocalAirStats s = slave.acc.localAirStats();
  TEST_ASSERT_EQUAL_UINT8(96, s.rxLq);
  TEST_ASSERT_EQUAL_INT8(-71, s.rssi);
  TEST_ASSERT_EQUAL_INT8(7, s.snr);
  TEST_ASSERT_EQUAL_INT8(12, s.txPower);
  TEST_ASSERT_EQUAL_UINT32(25, s.airRxOk);
  TEST_ASSERT_EQUAL_UINT8(25, slave.link.airStats().lastRxSfc); // the master counted 25 superframes
  // An FHSS keep-alive (zero padding) is neither delivered nor counted as rejected.
  uint8_t zeros[253] = {0};
  slave.acc.onRadioPacket(zeros, sizeof zeros, slave.clk.nowNs(), -70, 7);
  TEST_ASSERT_FALSE(slave.acc.process());
  TEST_ASSERT_EQUAL_UINT32(1, slave.acc.counters.keepAlives);
  TEST_ASSERT_EQUAL_UINT32(0, slave.acc.localAirStats().airRxBad);
  // A corrupted packet is rejected and counted.
  size_t n = master.acc.fillTxPayload(buf, sizeof buf, master.clk.nowNs());
  buf[10] ^= 0x40;
  slave.acc.onRadioPacket(buf, n, slave.clk.nowNs(), -70, 7);
  TEST_ASSERT_FALSE(slave.acc.process());
  TEST_ASSERT_EQUAL_UINT32(1, slave.acc.localAirStats().airRxBad);
  // A second packet before process() overwrites the first (counted).
  slave.acc.onRadioPacket(buf, n, slave.clk.nowNs(), -70, 7);
  slave.acc.onRadioPacket(buf, n, slave.clk.nowNs(), -70, 7);
  TEST_ASSERT_EQUAL_UINT32(1, slave.acc.counters.mailboxOverrun);
  slave.acc.process();
  // Searching: LQ is 0 at once.
  slave.acc.setStatus(AirLinkStatus{Packets::AirState::kSearching, 12, 0, 0});
  TEST_ASSERT_EQUAL_UINT8(0, slave.acc.localAirStats().rxLq);
}

void test_direction_master_does_not_accept_master() {
  AccessRig a(true), b(true);
  uint8_t buf[253];
  size_t n = a.acc.fillTxPayload(buf, sizeof buf, a.clk.nowNs());
  b.acc.onRadioPacket(buf, n, b.clk.nowNs(), -70, 7);
  TEST_ASSERT_FALSE(b.acc.process());
  TEST_ASSERT_EQUAL_UINT32(1, b.acc.localAirStats().airRxBad);
}

void test_access_no_heap_after_construction() {
  AccessRig master(true), slave(false);
  slave.sink.got.reserve(1000); // the test's own recorder
  Frame rc = rcStateFrame(2, 1, 1);
  uint8_t buf[253];
  const unsigned long before = g_allocs.load();
  for (int i = 0; i < 100; i++) {
    master.link.send(rc.d, rc.len, master.clk.nowNs());
    size_t n = master.acc.fillTxPayload(buf, sizeof buf, master.clk.nowNs());
    slave.acc.onRadioPacket(buf, n, slave.clk.nowNs(), -70, 7);
    slave.acc.process();
    slave.acc.localAirStats();
    master.clk.advanceMs(20);
    slave.clk.advanceMs(20);
  }
  TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)(g_allocs.load() - before));
}

// =========================================================================================== RxRouter

struct RxRig {
  FakeClock clk;
  RecordLink uart{clk};
  NetAirAccess::Link link;
  NetAirAccess acc{link, clk, false, 25};
  RxRouter router{clk, uart, link, acc};
  /// One packet from the TX module carrying these frames, through the mailbox path.
  void fromAir(std::initializer_list<Frame> frames) {
    uint8_t buf[253];
    net::wire::AirHeader h;
    h.dirSlaveToMaster = false;
    net::wire::AirPacketWriter w;
    TEST_ASSERT_TRUE(w.begin(buf, sizeof buf, h));
    for (auto &f : frames) TEST_ASSERT_TRUE(w.add(f.d, f.len));
    size_t n = w.finish(253);
    acc.onRadioPacket(buf, n, clk.nowNs(), -80, 3);
    TEST_ASSERT_TRUE(acc.process());
  }
  std::vector<Frame> toAir() {
    uint8_t buf[253];
    size_t n = acc.fillTxPayload(buf, sizeof buf, clk.nowNs());
    return parseAir(buf, n);
  }
};

void test_rx_filters_from_air_to_uart() {
  RxRig r;
  r.uart.sent.clear();
  const uint8_t pl[12] = {0};
  r.fromAir({makeHeartbeat(0, 0),                                   // ground station heartbeat: dropped
             rcStateFrame(0, 1, 1),                                 // ground station RC_STATE: dropped
             makeData(0, 1, 0x11, 1, pl, 12, net::wire::Qos::Event, false), // ground station REQUEST: passes
             makeData(0, 1, 0x20, 2, pl, 12),                       // other ground station data: passes
             rcStateFrame(2, 1, 1),                                 // RC_STATE from any source: dropped (GS-control)
             cmdStateFrame(0, 1, 1),                                // CMD_STATE from node 0: passes
             cmdStateFrame(1, 1, 2),                                // CMD_STATE from node 1, 2, 3: dropped
             cmdStateFrame(2, 1, 3),
             cmdStateFrame(3, 1, 4),
             makeHeartbeat(2, 2)});                                 // TX module heartbeat
  TEST_ASSERT_EQUAL_INT(0, countHb(r.uart.sent, 0));
  TEST_ASSERT_EQUAL_INT(0, countData(r.uart.sent, 0x10, 0));
  TEST_ASSERT_EQUAL_INT(1, countData(r.uart.sent, 0x11, 0, 1));
  TEST_ASSERT_EQUAL_INT(1, countData(r.uart.sent, 0x20, 0, 1));
  TEST_ASSERT_EQUAL_INT(0, countData(r.uart.sent, 0x10)); // no RC_STATE on the vehicle UART, whatever its source
  TEST_ASSERT_EQUAL_INT(1, countData(r.uart.sent, 0x13));
  TEST_ASSERT_EQUAL_INT(1, countData(r.uart.sent, 0x13, 0, 1)); // src 0 preserved
  TEST_ASSERT_EQUAL_INT(1, countHb(r.uart.sent, 2));
  TEST_ASSERT_TRUE(r.router.node().stats.egressFiltered >= 2);
}

void test_rx_cmd_state_from_vehicle_side_not_on_air_and_downlink_unaffected() {
  RxRig r;
  // CMD_STATE arriving from the vehicle UART, any source (including a forged node 0): never on the air.
  r.uart.inject(cmdStateFrame(0, 1, 1));
  r.uart.inject(cmdStateFrame(1, 0, 2));
  r.uart.inject(cmdStateFrame(1, 2, 3));
  r.uart.inject(cmdStateFrame(1, 0xFF, 4));
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(0, countData(r.toAir(), 0x13));
  // Downlink is unaffected: vehicle telemetry (port 0x20) and an ACK still go on the air.
  const uint8_t pl[12] = {0};
  r.uart.inject(makeData(1, 0, 0x20, 1, pl, 12));
  r.uart.inject(makeData(1, 0, 0x12, 1, pl, 4, net::wire::Qos::Event, false));
  r.router.poll();
  auto air = r.toAir();
  TEST_ASSERT_EQUAL_INT(1, countData(air, 0x20, 1, 0));
  TEST_ASSERT_EQUAL_INT(1, countData(air, 0x12, 1, 0));
  TEST_ASSERT_EQUAL_INT(0, countData(air, 0x13));
}

void test_rx_vehicle_request_ack_to_gs_goes_on_air() {
  RxRig r;
  Packets::RequestAck ack;
  ack.req_id = 3;
  uint8_t buf[Packets::RequestAck::kMaxSize];
  size_t n = ack.encode(buf, sizeof buf);
  r.uart.inject(makeData(1, 0, Packets::RequestAck::kPort, 1, buf, n, net::wire::Qos::Event, false));
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(1, countData(r.toAir(), 0x12, 1, 0));
}

static Frame rxUploadRequest(uint8_t src, uint8_t dst, uint8_t seq, uint16_t reqId, uint8_t target = 3,
                             uint8_t action = 10) {
  Packets::Request q;
  q.req_id = reqId;
  q.action = action;
  q.target = target;
  uint8_t buf[Packets::Request::kMaxSize];
  size_t n = q.encode(buf, sizeof buf);
  return makeData(src, dst, Packets::Request::kPort, seq, buf, n, net::wire::Qos::Event, false);
}

void test_rx_upload_command_from_uart_and_air() {
  { // from the UART (vehicle host): ACK back on the UART, due after the delay
    RxRig r;
    r.uart.inject(rxUploadRequest(1, 3, 1, 21));
    r.router.poll();
    TEST_ASSERT_EQUAL_INT(1, countData(r.uart.sent, 0x12, 3, 1));
    for (auto &f : r.uart.sent)
      if (isData(f, 0x12)) {
        Packets::RequestAck a;
        TEST_ASSERT_TRUE(a.decode(f.d + 6, f.len - 6));
        TEST_ASSERT_EQUAL_UINT16(21, a.req_id);
        TEST_ASSERT_TRUE(a.status == Packets::RequestStatus::kDone);
        TEST_ASSERT_EQUAL_UINT8(0, a.detail);
      }
    TEST_ASSERT_FALSE(r.router.uploadCommandDue());
    r.clk.t += LinkUploadCommand::kUploadDelayNs - 1000000LL;
    r.router.poll();
    TEST_ASSERT_FALSE(r.router.uploadCommandDue());
    r.clk.t += 2000000LL;
    r.router.poll();
    TEST_ASSERT_TRUE(r.router.uploadCommandDue());
  }
  { // over the air (ground station through the TX module): ACK goes on the air, addressed to node 0
    RxRig r;
    r.fromAir({rxUploadRequest(0, 3, 1, 22)});
    r.router.poll();
    auto air = r.toAir();
    TEST_ASSERT_EQUAL_INT(1, countData(air, 0x12, 3, 0));
    TEST_ASSERT_EQUAL_INT(0, countData(r.uart.sent, 0x12));
    TEST_ASSERT_EQUAL_INT(0, countData(r.uart.sent, 0x11)); // the command is not passed on to the vehicle
    r.clk.t += LinkUploadCommand::kUploadDelayNs + 1000000LL;
    r.router.poll();
    TEST_ASSERT_TRUE(r.router.uploadCommandDue());
  }
}

void test_rx_reboot_command_from_uart_and_air_idempotent() {
  { // from the UART: ACK, then due after the delay; upload not due
    RxRig r;
    r.uart.inject(rxUploadRequest(1, 3, 1, 61, 3, 11));
    r.uart.inject(rxUploadRequest(1, 3, 2, 61, 3, 11)); // retry, same id
    r.router.poll();
    const int64_t first = r.clk.nowNs();
    TEST_ASSERT_EQUAL_INT(2, countData(r.uart.sent, 0x12, 3, 1));
    TEST_ASSERT_FALSE(r.router.rebootCommandDue());
    r.clk.t = first + LinkCommand::kDelayNs - 1000000LL;
    r.router.poll();
    TEST_ASSERT_FALSE(r.router.rebootCommandDue());
    r.clk.t += 2000000LL;
    r.router.poll();
    TEST_ASSERT_TRUE(r.router.rebootCommandDue());
    TEST_ASSERT_FALSE(r.router.uploadCommandDue());
  }
  { // over the air: ACK on the air to node 0, not passed to the vehicle
    RxRig r;
    r.fromAir({rxUploadRequest(0, 3, 1, 62, 3, 11)});
    r.router.poll();
    TEST_ASSERT_EQUAL_INT(1, countData(r.toAir(), 0x12, 3, 0));
    TEST_ASSERT_EQUAL_INT(0, countData(r.uart.sent, 0x11));
    r.clk.t += LinkCommand::kDelayNs + 1000000LL;
    r.router.poll();
    TEST_ASSERT_TRUE(r.router.rebootCommandDue());
  }
  { // wrong target, req_id 0, unknown action: ignored, no ACK
    RxRig q;
    q.uart.inject(rxUploadRequest(1, 3, 1, 1, 2, 11));
    q.uart.inject(rxUploadRequest(1, 3, 2, 0, 3, 11));
    q.uart.inject(rxUploadRequest(1, 3, 3, 4, 3, 12));
    q.router.poll();
    q.clk.t += 2 * LinkCommand::kDelayNs;
    q.router.poll();
    TEST_ASSERT_FALSE(q.router.rebootCommandDue());
    TEST_ASSERT_EQUAL_INT(0, countData(q.uart.sent, 0x12));
  }
}

void test_rx_upload_command_idempotent_and_ignored_cases() {
  RxRig r;
  r.uart.inject(rxUploadRequest(1, 3, 1, 5));
  r.router.poll();
  const int64_t first = r.clk.t;
  r.clk.t += 300000000LL;
  r.uart.inject(rxUploadRequest(1, 3, 2, 5)); // retry, same id
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(2, countData(r.uart.sent, 0x12, 3, 1)); // every copy acknowledged
  r.clk.t = first + LinkUploadCommand::kUploadDelayNs + 1000000LL;
  r.router.poll();
  TEST_ASSERT_TRUE(r.router.uploadCommandDue()); // from the first copy
  TEST_ASSERT_EQUAL_UINT32(2, r.router.uploadCommand().stats.accepted);

  RxRig q;
  const uint8_t shortPl[11] = {1, 0, 10, 3};
  q.uart.inject(rxUploadRequest(1, 3, 1, 1, 2));        // target the TX module
  q.uart.inject(rxUploadRequest(1, 3, 2, 2, 3, 9));     // other action
  q.uart.inject(rxUploadRequest(1, 3, 3, 0));           // req_id 0
  q.uart.inject(makeData(1, 3, 0x11, 4, shortPl, sizeof shortPl, net::wire::Qos::Event, false)); // too short
  q.uart.inject(rxUploadRequest(1, 2, 5, 3, 2));        // forwarded, not for us
  q.clk.t += 2 * LinkUploadCommand::kUploadDelayNs;
  q.router.poll();
  TEST_ASSERT_FALSE(q.router.uploadCommandDue());
  TEST_ASSERT_EQUAL_INT(0, countData(q.uart.sent, 0x12));
  TEST_ASSERT_EQUAL_UINT32(0, q.router.uploadCommand().stats.accepted);
}

void test_rx_filters_from_uart_to_air() {
  RxRig r;
  const uint8_t pl[20] = {0};
  r.uart.inject(makeData(1, 0, 0x20, 1, pl, 20));   // vehicle telemetry to the ground station
  r.uart.inject(makeData(1, 0, 0x90, 1, pl, 20));   // USB-only port: never on air
  r.uart.inject(makeHeartbeat(1, 1));
  r.router.poll(); // own heartbeat (first poll) and LINK_STATS
  auto air = r.toAir();
  TEST_ASSERT_EQUAL_INT(1, countData(air, 0x20, 1, 0));
  TEST_ASSERT_EQUAL_INT(0, countData(air, 0x90));
  TEST_ASSERT_EQUAL_INT(1, countHb(air, 1));
  TEST_ASSERT_EQUAL_INT(1, countHb(air, 3)); // own heartbeat
  // nothing the module sends goes back to the vehicle addressed to node 0, nothing addressed to node 1 goes on air
  for (auto &f : r.uart.sent) TEST_ASSERT_NOT_EQUAL(0, f.d[2]);
  for (auto &f : air) TEST_ASSERT_NOT_EQUAL(1, f.d[2]);
  TEST_ASSERT_TRUE(r.router.uartHostSeen());
}

void test_rx_link_stats_5hz_and_searching() {
  RxRig r;
  r.acc.setStatus(AirLinkStatus{Packets::AirState::kSearching, 12, 0, 0});
  for (int i = 0; i < 100; i++) { // 1 s in 10 ms steps
    r.router.poll();
    r.clk.advanceMs(10);
  }
  int n = 0;
  for (auto &f : r.uart.sent)
    if (isData(f, 0x01) && f.d[3] == 3) {
      Packets::LinkStats ls;
      TEST_ASSERT_TRUE(ls.decode(f.d + 6, f.len - 6));
      TEST_ASSERT_EQUAL_INT((int)Packets::AirState::kSearching, (int)ls.air_state);
      TEST_ASSERT_EQUAL_UINT8(0, ls.rx_lq);
      TEST_ASSERT_EQUAL_INT(12, ls.tx_power);
      n++;
    }
  TEST_ASSERT_INT_WITHIN(1, 5, n);
  TEST_ASSERT_EQUAL_INT(1, countHb(r.uart.sent, 3));
  // no air: no RC state of any source is ever sent to the vehicle
  TEST_ASSERT_EQUAL_INT(0, countData(r.uart.sent, 0x10));
}

void test_rx_router_no_heap_after_construction() {
  RxRig r;
  const uint8_t pl[20] = {0};
  const Frame tel = makeData(1, 0, 0x20, 1, pl, 20);
  const Frame rc = rcStateFrame(2, 1, 1);
  uint8_t air[253];
  // the TX module's packet, built once up front
  net::wire::AirHeader h;
  net::wire::AirPacketWriter w;
  w.begin(air, sizeof air, h);
  w.add(rc.d, rc.len);
  const size_t an = w.finish(253);
  r.uart.sent.clear(); // capacity stays reserved
  const unsigned long before = g_allocs.load();
  for (int i = 0; i < 200; i++) {
    r.uart.inject(tel);
    r.acc.onRadioPacket(air, an, r.clk.nowNs(), -70, 5);
    r.acc.process();
    r.router.poll();
    uint8_t buf[253];
    r.acc.fillTxPayload(buf, sizeof buf, r.clk.nowNs());
    r.clk.advanceMs(20);
  }
  TEST_ASSERT_TRUE(r.uart.sent.size() < 8192); // the recorder never had to grow
  TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)(g_allocs.load() - before));
}

// =========================================================================================== triggers, power

void test_upload_triggers() {
  const int64_t boot = 1000000000LL;
  using R = UploadTriggers::Reason;
  {
    UploadTriggers t(boot, true);
    TEST_ASSERT_EQUAL_INT((int)R::None, (int)t.update(boot + 59000000000LL, false, true));
    TEST_ASSERT_EQUAL_INT((int)R::SyncFailsafe, (int)t.update(boot + 60000000000LL, false, true));
    TEST_ASSERT_EQUAL_INT((int)R::None, (int)t.update(boot + 61000000000LL, false, true)); // once
  }
  {
    UploadTriggers t(boot, true);
    t.update(boot + 1000000000LL, true, true); // synced once and host seen: both cancelled for good
    TEST_ASSERT_EQUAL_INT((int)R::None, (int)t.update(boot + 600000000000LL, false, false));
  }
  {
    UploadTriggers t(boot, true);
    t.update(boot + 1000000000LL, true, false); // air link seen, no UART host: stays up (user 2026-11-20)
    TEST_ASSERT_EQUAL_INT((int)R::None, (int)t.update(boot + 60000000000LL, false, false));
  }
  {
    UploadTriggers t(boot, true); // no UART host, air link never seen: sync failsafe wins at 60 s
    TEST_ASSERT_EQUAL_INT((int)R::SyncFailsafe, (int)t.update(boot + 60000000000LL, false, false));
  }
  {
    UploadTriggers t(boot, false); // TX module: no UART trigger
    t.update(boot + 1000000000LL, true, false);
    TEST_ASSERT_EQUAL_INT((int)R::None, (int)t.update(boot + 600000000000LL, false, false));
  }
  {
    UploadTriggers t(boot, true);
    t.update(boot, true, true);
    t.requestCommand();
    TEST_ASSERT_EQUAL_INT((int)R::Command, (int)t.update(boot + 1, true, true));
  }
}

void test_power_policy() {
  NetPowerPolicy p;
  TEST_ASSERT_FALSE(p.onRxSlotEnd(true, true));
  TEST_ASSERT_FALSE(p.onRxSlotEnd(false, true));
  TEST_ASSERT_TRUE(p.onRxSlotEnd(false, true)); // 2 lost in a row
  TEST_ASSERT_FALSE(p.onRxSlotEnd(true, true));
  TEST_ASSERT_TRUE(p.onRxSlotEnd(true, false)); // remote stats stale
  TEST_ASSERT_TRUE(p.newRemoteStats(5));
  TEST_ASSERT_FALSE(p.newRemoteStats(5));
  TEST_ASSERT_TRUE(NetPowerPolicy::remoteFresh(true, 0, 999999999));
  TEST_ASSERT_FALSE(NetPowerPolicy::remoteFresh(true, 0, 1000000000));
  TEST_ASSERT_FALSE(NetPowerPolicy::remoteFresh(false, 0, 1));
}

// =========================================================================================== end to end

struct Gate : LinkGateI {
  bool open = true;
  bool admits() override { return open; }
};

/// TX module (node 2) and RX module (node 3) joined by their air accesses over a simulated FHSS schedule: 20 ms
/// slots, the master transmits in odd slots, the follower in even ones. Each packet arrives `toa` later. A loss
/// window drops packets.
struct EndToEnd {
  FakeClock clk;
  RecordLink usb{clk};
  Gate gate;
  NetAirAccess::Link txLink;
  NetAirAccess txAcc{txLink, clk, true, 25};
  NetworkRouter tx{clk, usb, &gate, txLink, txAcc};
  RecordLink uart{clk};
  NetAirAccess::Link rxLink;
  NetAirAccess rxAcc{rxLink, clk, false, 25};
  RxRouter rx{clk, uart, rxLink, rxAcc};
  int64_t lossStart = 0, lossEnd = 0;
  uint32_t slot = 0;
  bool pocketOn = true; ///< the handset: RC_STATE to the GS on USB only
  bool gsOn = true;     ///< the ground station: CMD_STATE over USB at 25 Hz
  uint32_t rcOnUart = 0;  ///< CMD_STATE frames seen on the vehicle UART
  uint32_t pocketRcOnUart = 0; ///< RC_STATE (0x10) frames seen on the vehicle UART: must stay 0
  int64_t lastRcOnUartNs = 0;

  EndToEnd() {
    txAcc.setStatus(AirLinkStatus{Packets::AirState::kSynced, 20, 0, 0});
    rxAcc.setStatus(AirLinkStatus{Packets::AirState::kSynced, 12, 0, 0});
  }

  void step1ms() {
    // CRSF frames from the Pocket at 250 Hz
    if (pocketOn && (clk.t / 1000000LL) % 4 == 0) {
      uint16_t ch[16];
      for (int i = 0; i < 16; i++) ch[i] = 992;
      tx.onCrsfRcFrame(ch);
    }
    if (gsOn && (clk.t / 1000000LL) % 40 == 7) usb.inject(cmdStateFrame(0, 1, (uint8_t)(clk.t / 40000000LL)));
    if ((clk.t / 1000000LL) % 20 == 0) {
      const bool masterSlot = (slot++ % 2) == 1;
      NetAirAccess &from = masterSlot ? txAcc : rxAcc;
      NetAirAccess &to = masterSlot ? rxAcc : txAcc;
      uint8_t buf[253];
      size_t n = from.fillTxPayload(buf, sizeof buf, clk.nowNs());
      if (!(clk.t >= lossStart && clk.t < lossEnd)) {
        to.onRadioPacket(buf, n, clk.nowNs(), -75, 6);
      }
    }
    if ((clk.t / 1000000LL) % 20 == 11) { // after the 10.56 ms time on air
      txAcc.process();
      rxAcc.process();
    }
    tx.poll();
    rx.poll();
    const size_t before = uart.sent.size();
    (void)before;
    clk.advanceMs(1);
  }
  void run(int ms) {
    for (int i = 0; i < ms; i++) {
      const size_t n0 = uart.sent.size();
      step1ms();
      for (size_t k = n0; k < uart.sent.size(); k++)
      {
        if (isData(uart.sent[k], 0x13)) {
          rcOnUart++;
          lastRcOnUartNs = clk.nowNs();
        }
        if (isData(uart.sent[k], 0x10)) pocketRcOnUart++;
      }
    }
  }
};

void test_e2e_cmd_state_reaches_uart_at_25hz_and_pocket_rc_never() {
  EndToEnd e;
  e.run(1000);
  const uint32_t rc0 = e.rcOnUart;
  e.run(2000);
  const uint32_t rc = e.rcOnUart - rc0;
  char msg[60];
  snprintf(msg, sizeof msg, "CMD_STATE on the RX UART in 2 s: %u", (unsigned)rc);
  TEST_MESSAGE(msg);
  TEST_ASSERT_UINT32_WITHIN(1, 50, rc); // 25 Hz: one per uplink slot
  TEST_ASSERT_EQUAL_INT(0, countData(e.uart.sent, 0x13, -1, 2)); // only the GS's frames
  TEST_ASSERT_TRUE(countData(e.uart.sent, 0x13, 0, 1) > 0);       // src 0 preserved
  // The Pocket is on at 250 Hz: its RC_STATE goes to the GS on USB, never on the air or the UART.
  TEST_ASSERT_EQUAL_UINT32(0, e.pocketRcOnUart);
  TEST_ASSERT_EQUAL_INT(0, countData(e.uart.sent, 0x10));
  TEST_ASSERT_TRUE(countData(e.usb.sent, 0x10, 2, 0) > 100);
  // LINK_STATS of node 3 reaches the TX module (and the ground station on USB)
  TEST_ASSERT_TRUE(e.tx.linkStatsPort().hasValue());
  TEST_ASSERT_TRUE(countData(e.usb.sent, 0x01, 3) > 0);
  TEST_ASSERT_TRUE(e.tx.node().isPresent(3));
  TEST_ASSERT_TRUE(e.rx.node().isPresent(2));
}

void test_e2e_gs_heartbeat_and_rc_never_reach_uart_but_request_does() {
  EndToEnd e;
  e.gsOn = false;
  const uint8_t pl[12] = {0};
  for (int i = 0; i < 100; i++) {
    e.usb.inject(makeHeartbeat(0, 0));                 // ground-station heartbeat
    e.usb.inject(rcStateFrame(0, 1, (uint8_t)i));      // old virtual radio RC_STATE
    e.usb.inject(rcStateFrame(1, 1, (uint8_t)i));      // RC_STATE injected over USB with a forged source
    e.usb.inject(makeData(0, 1, 0x11, (uint8_t)i, pl, 12, net::wire::Qos::Event, false)); // REQUEST
    e.run(20);
  }
  TEST_ASSERT_EQUAL_INT(0, countHb(e.uart.sent, 0));
  TEST_ASSERT_EQUAL_INT(0, countData(e.uart.sent, 0x10));
  TEST_ASSERT_TRUE(countData(e.uart.sent, 0x11, 0, 1) > 0); // ground-station REQUESTs reach the vehicle UART
}

void test_e2e_cmd_state_spoof_sources_never_reach_uart() {
  EndToEnd e;
  e.gsOn = false;
  for (int i = 0; i < 100; i++) {
    e.usb.inject(cmdStateFrame(1, 1, (uint8_t)i)); // forged vehicle source over USB
    e.usb.inject(cmdStateFrame(2, 1, (uint8_t)i)); // forged TX module source
    e.usb.inject(cmdStateFrame(3, 1, (uint8_t)i)); // forged RX module source
    e.run(20);
  }
  TEST_ASSERT_EQUAL_INT(0, countData(e.uart.sent, 0x13));
  TEST_ASSERT_EQUAL_INT(0, countData(e.usb.sent, 0x13)); // and never echoed back on USB
}

void test_e2e_request_ack_from_vehicle_reaches_usb() {
  EndToEnd e;
  e.run(1000); // sync both routes
  Packets::RequestAck ack;
  ack.req_id = 11;
  uint8_t buf[Packets::RequestAck::kMaxSize];
  size_t n = ack.encode(buf, sizeof buf);
  e.uart.inject(makeData(1, 0, Packets::RequestAck::kPort, 1, buf, n, net::wire::Qos::Event, false));
  e.run(300);
  TEST_ASSERT_EQUAL_INT(1, countData(e.usb.sent, 0x12, 1, 0));
}

void test_e2e_upload_command_to_rx_over_air_ack_before_entry() {
  EndToEnd e;
  e.run(1000);
  Packets::Request q;
  q.req_id = 600;
  q.action = 10;
  q.target = 3;
  uint8_t buf[Packets::Request::kMaxSize];
  size_t n = q.encode(buf, sizeof buf);
  e.usb.inject(makeData(0, 3, Packets::Request::kPort, 1, buf, n, net::wire::Qos::Event, false));
  int64_t ackNs = 0, dueNs = 0;
  const int64_t t0 = e.clk.nowNs();
  for (int i = 0; i < 1000; i++) {
    const size_t u0 = e.usb.sent.size();
    e.run(1);
    for (size_t k = u0; k < e.usb.sent.size(); k++)
      if (isData(e.usb.sent[k], 0x12) && e.usb.sent[k].d[3] == 3 && e.usb.sent[k].d[2] == 0 && !ackNs)
        ackNs = e.clk.nowNs();
    if (e.rx.uploadCommandDue() && !dueNs) dueNs = e.clk.nowNs();
  }
  TEST_ASSERT_TRUE(ackNs > t0);
  TEST_ASSERT_TRUE(dueNs > ackNs);                                   // the ACK crossed the air first
  TEST_ASSERT_TRUE(dueNs - t0 >= LinkUploadCommand::kUploadDelayNs);
  TEST_ASSERT_FALSE(e.tx.uploadCommandDue());                        // the TX module was not the target
  // the REQUEST for node 3 did not reach the vehicle UART
  TEST_ASSERT_EQUAL_INT(0, countData(e.uart.sent, 0x11));
}

void test_e2e_upload_command_to_tx_acked_on_usb() {
  EndToEnd e;
  e.run(1000);
  Packets::Request q;
  q.req_id = 601;
  q.action = 10;
  q.target = 2;
  uint8_t buf[Packets::Request::kMaxSize];
  size_t n = q.encode(buf, sizeof buf);
  e.usb.inject(makeData(0, 2, Packets::Request::kPort, 1, buf, n, net::wire::Qos::Event, false));
  e.run(100);
  TEST_ASSERT_EQUAL_INT(1, countData(e.usb.sent, 0x12, 2, 0));
  TEST_ASSERT_FALSE(e.tx.uploadCommandDue());
  e.run(400);
  TEST_ASSERT_TRUE(e.tx.uploadCommandDue());
  TEST_ASSERT_FALSE(e.rx.uploadCommandDue());
}

void test_e2e_gs_off_silence_and_no_late_release() {
  EndToEnd e;
  e.run(1000);
  e.gsOn = false;
  const int64_t off = e.clk.nowNs();
  e.run(1000);
  // The last CMD_STATE on the UART was originated before the GS stopped; it arrives at most one uplink slot plus
  // the expiry limit later.
  TEST_ASSERT_TRUE(e.lastRcOnUartNs - off <= 100000000LL + 40000000LL);
  // RF block of 5 s with the GS on: nothing old is released afterwards.
  e.gsOn = true;
  e.run(500);
  e.lossStart = e.clk.t;
  e.lossEnd = e.clk.t + 5000000000LL;
  e.run(5000);
  const uint32_t rcAtEnd = e.rcOnUart;
  e.run(60);
  TEST_ASSERT_TRUE(e.rcOnUart - rcAtEnd <= 2); // only fresh states, one per uplink slot
}


// =========================================================================================== power policy, LINK_CONFIG, Lua menu

static Frame linkConfigFrame(uint8_t src, uint8_t dst, uint8_t seq, int8_t dbm) {
  Packets::LinkConfig lc;
  lc.tx_power_dbm = dbm;
  lc.flags = 0;
  uint8_t buf[Packets::LinkConfig::kMaxSize];
  const size_t n = lc.encode(buf, sizeof buf);
  return makeData(src, dst, Packets::LinkConfig::kPort, seq, buf, n, net::wire::Qos::Link, true);
}

void test_link_config_packet_bytes() {
  Packets::LinkConfig lc;
  lc.tx_power_dbm = -3;
  lc.flags = 5;
  uint8_t buf[Packets::LinkConfig::kMaxSize];
  TEST_ASSERT_EQUAL_UINT32(2, (uint32_t)lc.encode(buf, sizeof buf));
  TEST_ASSERT_EQUAL_HEX8(0xFD, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x05, buf[1]);
  Packets::LinkConfig d;
  TEST_ASSERT_TRUE(d.decode(buf, 2));
  TEST_ASSERT_EQUAL_INT8(-3, d.tx_power_dbm);
  TEST_ASSERT_EQUAL_UINT8(5, d.flags);
  TEST_ASSERT_EQUAL_UINT8(0x04, Packets::LinkConfig::kPort);
  TEST_ASSERT_FALSE(d.decode(buf, 1)); // too short
}

void test_power_default_and_ceiling_clamp() {
  TEST_ASSERT_EQUAL_INT8(10, kNetTxPowerDefaultDbm);
  NetTxPowerSetting router(27);
  TEST_ASSERT_EQUAL_INT8(10, router.get());     // boots at the legal 10 dBm
  TEST_ASSERT_EQUAL_INT8(27, router.set(27));
  TEST_ASSERT_EQUAL_INT8(27, router.set(40));   // above the board maximum: clamped
  TEST_ASSERT_EQUAL_INT8(0, router.set(-5));    // never below 0
  router.set(20);
  router.reset();
  TEST_ASSERT_EQUAL_INT8(10, router.get());
  NetTxPowerSetting ship(12);
  TEST_ASSERT_EQUAL_INT8(10, ship.get());
  TEST_ASSERT_EQUAL_INT8(12, ship.set(27));
  NetTxPowerSetting tiny(8);                    // board below the default: default is clamped too
  TEST_ASSERT_EQUAL_INT8(8, tiny.get());
  TEST_ASSERT_EQUAL_INT8(10, clampNetPower(10, 27));
  TEST_ASSERT_EQUAL_INT8(0, clampNetPower(10, -1));
}

void test_link_config_follower_apply_and_failsafe() {
  LinkConfigFollower f(12);
  const int64_t t0 = 5000000000LL;
  TEST_ASSERT_FALSE(f.fresh(t0));
  TEST_ASSERT_EQUAL_INT8(10, f.power(t0));                 // never received: default
  f.onLinkConfig(27, t0);                                  // above the ship board maximum
  TEST_ASSERT_EQUAL_INT8(12, f.power(t0));                 // clamped to the board maximum
  f.onLinkConfig(5, t0 + 1000000000LL);
  TEST_ASSERT_EQUAL_INT8(5, f.power(t0 + 1000000000LL));   // lower than the default is honoured
  f.onLinkConfig(-20, t0 + 2000000000LL);
  TEST_ASSERT_EQUAL_INT8(0, f.power(t0 + 2000000000LL));   // never below 0
  const int64_t last = t0 + 3000000000LL;
  f.onLinkConfig(12, last);
  TEST_ASSERT_EQUAL_INT8(12, f.power(last + LinkConfigFollower::kTimeoutNs - 1));
  TEST_ASSERT_TRUE(f.fresh(last + LinkConfigFollower::kTimeoutNs - 1));
  TEST_ASSERT_EQUAL_INT8(10, f.power(last + LinkConfigFollower::kTimeoutNs)); // 10 s: back to 10 dBm
  TEST_ASSERT_FALSE(f.fresh(last + LinkConfigFollower::kTimeoutNs));
  f.onLinkConfig(12, last + 20000000000LL);                // a new copy brings it back
  TEST_ASSERT_EQUAL_INT8(12, f.power(last + 20000000000LL));
}

struct TxRig {
  FakeClock clk;
  RecordLink usb{clk};
  RecordLink air{clk};
  Gate gate;
  StubAirStats stats;
  NetworkRouter router{clk, usb, &gate, air, stats};
};

void test_tx_router_sends_link_config_continuously() {
  TxRig r;
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(0, countData(r.air.sent, 0x04)); // nothing before a power is set
  r.router.setConfigPower(10);
  for (int i = 0; i < 5000; i++) { // 5 s at 1 kHz
    r.router.poll();
    r.clk.advanceMs(1);
  }
  const int n = countData(r.air.sent, 0x04, 2, 3);
  char msg[40];
  snprintf(msg, sizeof msg, "LINK_CONFIG in 5 s: %d", n);
  TEST_MESSAGE(msg);
  TEST_ASSERT_INT_WITHIN(1, 10, n); // about 2 Hz
  TEST_ASSERT_EQUAL_INT(0, countData(r.air.sent, 0x04, -1, 0xFF)); // unicast to node 3 only
  TEST_ASSERT_EQUAL_INT(0, countData(r.usb.sent, 0x04));            // never on USB
  // last one carries the value; a change goes out at the next poll, not at the next period
  r.air.sent.clear();
  r.router.setConfigPower(23);
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(1, countData(r.air.sent, 0x04, 2, 3));
  Packets::LinkConfig lc;
  for (auto &f : r.air.sent)
    if (isData(f, 0x04)) TEST_ASSERT_TRUE(lc.decode(f.d + 6, f.len - 6));
  TEST_ASSERT_EQUAL_INT8(23, lc.tx_power_dbm);
}

void test_tx_router_drops_link_config_from_usb() {
  TxRig r;
  r.router.poll();
  r.usb.inject(linkConfigFrame(0, 3, 1, 27)); // ground station tries to raise the ship node's power
  r.usb.inject(linkConfigFrame(0, 0xFF, 2, 27));
  r.usb.inject(linkConfigFrame(2, 3, 3, 27)); // forged source: still dropped (ingress rule, not source rule)
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(0, countData(r.air.sent, 0x04));
  // other GS traffic still passes to the air (REQUEST), so the rule is specific
  Packets::Request q;
  q.req_id = 7;
  q.action = 11;
  q.target = 3;
  uint8_t buf[Packets::Request::kMaxSize];
  const size_t n = q.encode(buf, sizeof buf);
  r.usb.inject(makeData(0, 3, Packets::Request::kPort, 4, buf, n, net::wire::Qos::Event, false));
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(1, countData(r.air.sent, 0x11, 0, 3));
  // and the Router's own config frames are still sent
  r.router.setConfigPower(10);
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(1, countData(r.air.sent, 0x04, 2, 3));
}

void test_ship_accepts_link_config_only_from_router_over_air() {
  RxRig r;
  TEST_ASSERT_EQUAL_INT8(10, r.router.desiredPowerDbm());
  r.uart.inject(linkConfigFrame(2, 3, 1, 12)); // forged by the vehicle side: rejected, whatever the source
  r.router.poll();
  TEST_ASSERT_EQUAL_INT8(10, r.router.desiredPowerDbm());
  TEST_ASSERT_EQUAL_UINT32(1, r.router.linkConfigHandler().rejected);
  r.fromAir({linkConfigFrame(0, 3, 2, 12)});   // wrong source node (the ground station)
  TEST_ASSERT_EQUAL_INT8(10, r.router.desiredPowerDbm());
  r.fromAir({linkConfigFrame(2, 3, 3, 12)});   // the Router over the air
  TEST_ASSERT_EQUAL_INT8(12, r.router.desiredPowerDbm());
  TEST_ASSERT_TRUE(r.router.linkConfigFresh());
  r.fromAir({linkConfigFrame(2, 3, 4, 27)});   // clamped to the default board maximum of the ship node
  TEST_ASSERT_EQUAL_INT8(12, r.router.desiredPowerDbm());
  r.clk.t += LinkConfigFollower::kTimeoutNs + 1;
  TEST_ASSERT_EQUAL_INT8(10, r.router.desiredPowerDbm()); // fail-safe
  TEST_ASSERT_FALSE(r.router.linkConfigFresh());
  TEST_ASSERT_EQUAL_INT(0, countData(r.uart.sent, 0x04)); // not forwarded to the vehicle
}

void test_e2e_link_config_mirrors_power_and_failsafe() {
  EndToEnd e;
  e.tx.setConfigPower(10);
  e.run(1500);
  TEST_ASSERT_TRUE(e.rx.linkConfigFresh());
  TEST_ASSERT_EQUAL_INT8(10, e.rx.desiredPowerDbm());
  e.tx.setConfigPower(12);
  e.run(120); // one uplink slot (40 ms) plus the air time
  TEST_ASSERT_EQUAL_INT8(12, e.rx.desiredPowerDbm());
  TEST_ASSERT_EQUAL_INT(0, countData(e.uart.sent, 0x04)); // consumed by the ship node, not passed to the vehicle
  e.tx.setConfigPower(7);                                  // lowering works too
  e.run(120);
  TEST_ASSERT_EQUAL_INT8(7, e.rx.desiredPowerDbm());
  // the Router stops commanding (link lost for 12 s): the ship node falls back to 10 dBm within 10 s
  e.tx.setConfigPower(12);
  e.run(120);
  TEST_ASSERT_EQUAL_INT8(12, e.rx.desiredPowerDbm());
  e.lossStart = e.clk.nowNs();
  e.lossEnd = e.lossStart + 12000000000LL;
  e.run(9000);
  TEST_ASSERT_EQUAL_INT8(12, e.rx.desiredPowerDbm());
  e.run(1500);
  TEST_ASSERT_EQUAL_INT8(10, e.rx.desiredPowerDbm());
  e.run(4000); // link back: the mirrored value returns
  TEST_ASSERT_EQUAL_INT8(12, e.rx.desiredPowerDbm());
}

void test_ship_reboot_requester_retries_until_ack() {
  { // no ack: 5 tries one second apart, then timed out
    TxRig r;
    TEST_ASSERT_TRUE(r.router.requestShipReboot());
    TEST_ASSERT_FALSE(r.router.requestShipReboot()); // already running
    for (int i = 0; i < 7000; i++) {
      r.router.poll();
      r.clk.advanceMs(1);
    }
    TEST_ASSERT_EQUAL_INT(5, countData(r.air.sent, 0x11, 2, 3));
    TEST_ASSERT_TRUE(r.router.shipReboot().state() == ShipRebootRequester::State::TimedOut);
    Packets::Request q;
    for (auto &f : r.air.sent)
      if (isData(f, 0x11)) {
        TEST_ASSERT_TRUE(q.decode(f.d + 6, f.len - 6));
        TEST_ASSERT_EQUAL_UINT8(11, q.action);
        TEST_ASSERT_EQUAL_UINT8(3, q.target);
        TEST_ASSERT_TRUE(q.req_id == r.router.shipReboot().reqId() && q.req_id != 0);
      }
    TEST_ASSERT_TRUE(r.router.requestShipReboot()); // a new request is possible afterwards
  }
  { // ack from the ship node stops it
    TxRig r;
    r.router.requestShipReboot();
    r.router.poll();
    TEST_ASSERT_EQUAL_INT(1, countData(r.air.sent, 0x11, 2, 3));
    Packets::RequestAck a;
    a.req_id = r.router.shipReboot().reqId();
    a.status = Packets::RequestStatus::kDone;
    a.detail = 0;
    uint8_t buf[Packets::RequestAck::kMaxSize];
    const size_t n = a.encode(buf, sizeof buf);
    r.air.inject(makeData(3, 2, Packets::RequestAck::kPort, 1, buf, n, net::wire::Qos::Event, false));
    r.router.poll();
    TEST_ASSERT_TRUE(r.router.shipReboot().state() == ShipRebootRequester::State::Acked);
    for (int i = 0; i < 3000; i++) {
      r.router.poll();
      r.clk.advanceMs(1);
    }
    TEST_ASSERT_EQUAL_INT(1, countData(r.air.sent, 0x11, 2, 3));
  }
}

void test_e2e_ship_reboot_request_reaches_ship_node() {
  EndToEnd e;
  e.run(1000);
  TEST_ASSERT_TRUE(e.tx.requestShipReboot());
  e.run(1500);
  TEST_ASSERT_TRUE(e.tx.shipReboot().state() == ShipRebootRequester::State::Acked);
  TEST_ASSERT_TRUE(e.rx.rebootCommandDue());
  TEST_ASSERT_FALSE(e.rx.uploadCommandDue());
  TEST_ASSERT_FALSE(e.tx.rebootCommandDue());
  TEST_ASSERT_EQUAL_INT(0, countData(e.uart.sent, 0x11));
}

// ---- Lua menu

/// Parsed 0x2B parameter entry (own decoder: independent of the encoder under test).
struct Entry {
  bool ok = false;
  uint8_t dest, orig, param, chunks, parent, type;
  char name[40] = "", text[48] = "", unit[8] = "";
  uint8_t value = 0, min = 0, max = 0, def = 0, status = 0, timeout = 0;
};

static Entry parseEntry(const CrsfFrame &f) {
  Entry e;
  if (f.len < 11 || f.len > 64 || f.bytes[0] != 0xC8 || f.bytes[1] != f.len - 2 || f.bytes[2] != 0x2B) return e;
  if (crsfCrc8(f.bytes + 2, f.len - 3) != f.bytes[f.len - 1]) return e;
  e.dest = f.bytes[3];
  e.orig = f.bytes[4];
  e.param = f.bytes[5];
  e.chunks = f.bytes[6];
  e.parent = f.bytes[7];
  e.type = f.bytes[8];
  const uint8_t *p = f.bytes + 9, *end = f.bytes + f.len - 1;
  auto str = [&](char *dst, size_t cap) {
    size_t i = 0;
    while (p < end && *p) {
      if (i + 1 < cap) dst[i++] = static_cast<char>(*p);
      p++;
    }
    dst[i] = 0;
    if (p < end) p++;
  };
  str(e.name, sizeof e.name);
  if (e.type == 0x09) {
    str(e.text, sizeof e.text);
    if (end - p < 5) return e;
    e.value = *p++, e.min = *p++, e.max = *p++, e.def = *p++;
    str(e.unit, sizeof e.unit);
  } else if (e.type == 0x0C) {
    str(e.text, sizeof e.text);
  } else if (e.type == 0x0D) {
    if (end - p < 2) return e;
    e.status = *p++;
    e.timeout = *p++;
    str(e.text, sizeof e.text);
  } else {
    return e;
  }
  e.ok = p == end;
  return e;
}

struct MenuActions : LuaMenuActions {
  int power = -100, antenna = -1, rebootRouterCalls = 0, rebootShipCalls = 0, powerCalls = 0;
  void setTxPower(int8_t dbm) override { power = dbm; powerCalls++; }
  void setTxAntenna(uint8_t s) override { antenna = s; }
  void rebootRouter() override { rebootRouterCalls++; }
  void rebootShipNode() override { rebootShipCalls++; }
};

void test_lua_power_levels_and_options() {
  NetLuaMenu m(27);
  TEST_ASSERT_EQUAL_UINT8(8, m.levelCount());
  char opts[48];
  m.buildPowerOptions(opts, sizeof opts);
  TEST_ASSERT_EQUAL_STRING("0;5;10;13;17;20;23;27", opts);
  TEST_ASSERT_EQUAL_INT8(0, m.levelAt(0));
  TEST_ASSERT_EQUAL_INT8(27, m.levelAt(7));
  TEST_ASSERT_EQUAL_INT8(27, m.levelAt(200)); // out of range index clamps
  TEST_ASSERT_EQUAL_UINT8(2, m.indexFor(10));
  TEST_ASSERT_EQUAL_UINT8(2, m.indexFor(11)); // snaps down
  TEST_ASSERT_EQUAL_UINT8(0, m.indexFor(-4));
  TEST_ASSERT_EQUAL_UINT8(7, m.indexFor(99));
  TEST_ASSERT_EQUAL_UINT32(1, NetLuaMenu::milliwatts(0));
  TEST_ASSERT_EQUAL_UINT32(10, NetLuaMenu::milliwatts(10));
  TEST_ASSERT_EQUAL_UINT32(100, NetLuaMenu::milliwatts(20));
  TEST_ASSERT_EQUAL_UINT32(501, NetLuaMenu::milliwatts(27));
  NetLuaMenu ship(12); // a board maximum of 12 dBm offers 0..10 only
  TEST_ASSERT_EQUAL_UINT8(3, ship.levelCount());
  ship.buildPowerOptions(opts, sizeof opts);
  TEST_ASSERT_EQUAL_STRING("0;5;10", opts);
}

void test_lua_read_entries() {
  NetLuaMenu m(27);
  LuaMenuStatus st;
  CrsfFrame f;
  Entry e;
  const int64_t t = 1000;
  // power select, addressed to the handset when the ELRS Lua (0xEF) asks
  TEST_ASSERT_TRUE(m.buildRead(1, 0xEF, st, t, f));
  e = parseEntry(f);
  TEST_ASSERT_TRUE(e.ok);
  TEST_ASSERT_EQUAL_HEX8(0xEA, e.dest);
  TEST_ASSERT_EQUAL_HEX8(0xEE, e.orig);
  TEST_ASSERT_EQUAL_UINT8(1, e.param);
  TEST_ASSERT_EQUAL_UINT8(0, e.chunks);
  TEST_ASSERT_EQUAL_UINT8(0, e.parent);
  TEST_ASSERT_EQUAL_HEX8(0x09, e.type);
  TEST_ASSERT_EQUAL_STRING("Power (10mW legal)", e.name);
  TEST_ASSERT_EQUAL_STRING("0;5;10;13;17;20;23;27", e.text);
  TEST_ASSERT_EQUAL_UINT8(2, e.value); // 10 dBm
  TEST_ASSERT_EQUAL_UINT8(0, e.min);
  TEST_ASSERT_EQUAL_UINT8(7, e.max);
  TEST_ASSERT_EQUAL_UINT8(2, e.def);
  TEST_ASSERT_EQUAL_STRING("dBm", e.unit);
  TEST_ASSERT_TRUE(f.len <= 64);
  st.txPowerDbm = 27;
  TEST_ASSERT_TRUE(m.buildRead(1, 0xEA, st, t, f));
  e = parseEntry(f);
  TEST_ASSERT_EQUAL_HEX8(0xEA, e.dest);
  TEST_ASSERT_EQUAL_UINT8(7, e.value);
  // mW info
  TEST_ASSERT_TRUE(m.buildRead(2, 0xEA, st, t, f));
  e = parseEntry(f);
  TEST_ASSERT_TRUE(e.ok);
  TEST_ASSERT_EQUAL_HEX8(0x0C, e.type);
  TEST_ASSERT_EQUAL_STRING("Power (mW)", e.name);
  TEST_ASSERT_EQUAL_STRING("501 mW", e.text);
  // antenna
  st.txAntennaSel = 2;
  TEST_ASSERT_TRUE(m.buildRead(3, 0xEA, st, t, f));
  e = parseEntry(f);
  TEST_ASSERT_TRUE(e.ok);
  TEST_ASSERT_EQUAL_STRING("TX antenna", e.name);
  TEST_ASSERT_EQUAL_STRING("Auto;A;B", e.text);
  TEST_ASSERT_EQUAL_UINT8(2, e.value);
  TEST_ASSERT_EQUAL_UINT8(2, e.max);
  // link
  st.airState = Packets::AirState::kSynced;
  st.rxLq = 97;
  TEST_ASSERT_TRUE(m.buildRead(4, 0xEA, st, t, f));
  e = parseEntry(f);
  TEST_ASSERT_TRUE(e.ok);
  TEST_ASSERT_EQUAL_STRING("Link", e.name);
  TEST_ASSERT_EQUAL_STRING("Synced LQ 97%", e.text);
  // ship power: no data, then valid
  TEST_ASSERT_TRUE(m.buildRead(5, 0xEA, st, t, f));
  TEST_ASSERT_EQUAL_STRING("no data", parseEntry(f).text);
  st.shipValid = true;
  st.shipPowerDbm = 10;
  TEST_ASSERT_TRUE(m.buildRead(5, 0xEA, st, t, f));
  e = parseEntry(f);
  TEST_ASSERT_EQUAL_STRING("Ship power", e.name);
  TEST_ASSERT_EQUAL_STRING("10 dBm", e.text);
  // commands, and unknown numbers are not answered
  TEST_ASSERT_TRUE(m.buildRead(6, 0xEA, st, t, f));
  e = parseEntry(f);
  TEST_ASSERT_TRUE(e.ok);
  TEST_ASSERT_EQUAL_HEX8(0x0D, e.type);
  TEST_ASSERT_EQUAL_STRING("Reboot router", e.name);
  TEST_ASSERT_EQUAL_UINT8(0, e.status);
  TEST_ASSERT_TRUE(m.buildRead(7, 0xEA, st, t, f));
  TEST_ASSERT_EQUAL_STRING("Reboot ship node", parseEntry(f).name);
  TEST_ASSERT_FALSE(m.buildRead(0, 0xEA, st, t, f));
  TEST_ASSERT_FALSE(m.buildRead(8, 0xEA, st, t, f));
  TEST_ASSERT_FALSE(m.buildRead(255, 0xEA, st, t, f));
  // every entry fits one frame, with the longest values
  st.txPowerDbm = 23;
  for (uint8_t p = 1; p <= NetLuaMenu::kParamCount; p++) {
    TEST_ASSERT_TRUE(m.buildRead(p, 0xEA, st, t, f));
    TEST_ASSERT_TRUE(f.len <= 64);
    TEST_ASSERT_TRUE(parseEntry(f).ok);
  }
  TEST_ASSERT_TRUE(NetLuaMenu::pingIsForUs(0x00));
  TEST_ASSERT_TRUE(NetLuaMenu::pingIsForUs(0xEE));
  TEST_ASSERT_FALSE(NetLuaMenu::pingIsForUs(0xC8));
  TEST_ASSERT_EQUAL_HEX8(0xEA, NetLuaMenu::replyDest(0xEF));
  TEST_ASSERT_EQUAL_HEX8(0xEA, NetLuaMenu::replyDest(0xEA));
}

void test_lua_power_write_and_antenna_write() {
  NetLuaMenu m(27);
  LuaMenuStatus st;
  MenuActions act;
  CrsfFrame f;
  TEST_ASSERT_FALSE(m.handleWrite(1, 7, 0xEF, st, 0, act, f)); // selections reply nothing
  TEST_ASSERT_EQUAL_INT(27, act.power);
  m.handleWrite(1, 0, 0xEF, st, 0, act, f);
  TEST_ASSERT_EQUAL_INT(0, act.power);
  m.handleWrite(1, 2, 0xEF, st, 0, act, f);
  TEST_ASSERT_EQUAL_INT(10, act.power);
  m.handleWrite(1, 99, 0xEF, st, 0, act, f); // out of range index -> highest level, never above the board maximum
  TEST_ASSERT_EQUAL_INT(27, act.power);
  NetLuaMenu ship(12);
  ship.handleWrite(1, 7, 0xEF, st, 0, act, f);
  TEST_ASSERT_EQUAL_INT(10, act.power);
  m.handleWrite(3, 1, 0xEF, st, 0, act, f);
  TEST_ASSERT_EQUAL_INT(1, act.antenna);
  m.handleWrite(3, 2, 0xEF, st, 0, act, f);
  TEST_ASSERT_EQUAL_INT(2, act.antenna);
  m.handleWrite(3, 9, 0xEF, st, 0, act, f);
  TEST_ASSERT_EQUAL_INT(0, act.antenna);
  const int calls = act.powerCalls;
  TEST_ASSERT_FALSE(m.handleWrite(2, 1, 0xEF, st, 0, act, f)); // read-only entries
  TEST_ASSERT_FALSE(m.handleWrite(4, 1, 0xEF, st, 0, act, f));
  TEST_ASSERT_FALSE(m.handleWrite(77, 1, 0xEF, st, 0, act, f));
  TEST_ASSERT_EQUAL_INT(calls, act.powerCalls);
}

void test_lua_command_confirmation_flow() {
  NetLuaMenu m(27);
  LuaMenuStatus st;
  MenuActions act;
  CrsfFrame f;
  Entry e;
  int64_t t = 1000000000LL;
  // click: confirmation first, nothing executed
  TEST_ASSERT_TRUE(m.handleWrite(6, 1, 0xEF, st, t, act, f));
  e = parseEntry(f);
  TEST_ASSERT_TRUE(e.ok);
  TEST_ASSERT_EQUAL_UINT8(3, e.status); // ConfirmationNeeded
  TEST_ASSERT_EQUAL_STRING("Reboot router?", e.text);
  TEST_ASSERT_EQUAL_INT(0, act.rebootRouterCalls);
  // poll while waiting: the handset gets the same state
  t += 500000000LL;
  TEST_ASSERT_TRUE(m.handleWrite(6, 6, 0xEF, st, t, act, f));
  TEST_ASSERT_EQUAL_UINT8(3, parseEntry(f).status);
  // confirm: executes once
  TEST_ASSERT_TRUE(m.handleWrite(6, 4, 0xEF, st, t, act, f));
  TEST_ASSERT_EQUAL_UINT8(2, parseEntry(f).status); // Progress
  TEST_ASSERT_EQUAL_INT(1, act.rebootRouterCalls);
  TEST_ASSERT_EQUAL_INT(0, act.rebootShipCalls);
  // a repeated confirm does not execute again
  m.handleWrite(6, 4, 0xEF, st, t, act, f);
  TEST_ASSERT_EQUAL_INT(1, act.rebootRouterCalls);
  // progress is shown for 2 s, then it is ready again
  TEST_ASSERT_FALSE(m.handleWrite(6, 6, 0xEF, st, t + 1000000000LL, act, f));
  TEST_ASSERT_TRUE(m.buildRead(6, 0xEA, st, t + 2500000000LL, f));
  TEST_ASSERT_EQUAL_UINT8(0, parseEntry(f).status);

  // ship node: click, cancel -> nothing; click, confirm -> one call
  t += 10000000000LL;
  TEST_ASSERT_TRUE(m.handleWrite(7, 1, 0xEF, st, t, act, f));
  TEST_ASSERT_EQUAL_STRING("Reboot ship node?", parseEntry(f).text);
  TEST_ASSERT_TRUE(m.handleWrite(7, 5, 0xEF, st, t, act, f));
  TEST_ASSERT_EQUAL_UINT8(0, parseEntry(f).status);
  TEST_ASSERT_EQUAL_INT(0, act.rebootShipCalls);
  // a confirm without a click executes nothing
  TEST_ASSERT_TRUE(m.handleWrite(7, 4, 0xEF, st, t, act, f));
  TEST_ASSERT_EQUAL_UINT8(0, parseEntry(f).status);
  TEST_ASSERT_EQUAL_INT(0, act.rebootShipCalls);
  m.handleWrite(7, 1, 0xEF, st, t, act, f);
  TEST_ASSERT_TRUE(m.handleWrite(7, 4, 0xEF, st, t, act, f));
  TEST_ASSERT_EQUAL_INT(1, act.rebootShipCalls);
  TEST_ASSERT_EQUAL_INT(1, act.rebootRouterCalls);
  // the confirmation lapses after 10 s: a late confirm does nothing
  t += 10000000000LL;
  m.handleWrite(6, 1, 0xEF, st, t, act, f);
  t += NetLuaMenu::kConfirmTimeoutNs + 1;
  TEST_ASSERT_TRUE(m.handleWrite(6, 4, 0xEF, st, t, act, f));
  TEST_ASSERT_EQUAL_UINT8(0, parseEntry(f).status);
  TEST_ASSERT_EQUAL_INT(1, act.rebootRouterCalls);
  TEST_ASSERT_EQUAL_UINT8(0, m.commandStep(6));
  // unknown command status values are ignored
  TEST_ASSERT_FALSE(m.handleWrite(6, 77, 0xEF, st, t, act, f));
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_time_on_air_profile_v1);
  RUN_TEST(test_slot_budget);
  RUN_TEST(test_expected_rates);
  RUN_TEST(test_slot_roles_one_to_one);
  RUN_TEST(test_trailer_counters_and_seeds);
  RUN_TEST(test_phase_wrap_and_filters);
  RUN_TEST(test_lq_counter);
  RUN_TEST(test_rc_state_expiry_is_evaluated_at_slot_start);
  RUN_TEST(test_priority_order_in_air_packet);
  RUN_TEST(test_downlink_fill_throughput);
  RUN_TEST(test_receive_mailbox_keepalive_and_lq);
  RUN_TEST(test_direction_master_does_not_accept_master);
  RUN_TEST(test_access_no_heap_after_construction);
  RUN_TEST(test_rx_filters_from_air_to_uart);
  RUN_TEST(test_rx_vehicle_request_ack_to_gs_goes_on_air);
  RUN_TEST(test_rx_upload_command_from_uart_and_air);
  RUN_TEST(test_rx_reboot_command_from_uart_and_air_idempotent);
  RUN_TEST(test_rx_upload_command_idempotent_and_ignored_cases);
  RUN_TEST(test_rx_filters_from_uart_to_air);
  RUN_TEST(test_rx_link_stats_5hz_and_searching);
  RUN_TEST(test_rx_router_no_heap_after_construction);
  RUN_TEST(test_upload_triggers);
  RUN_TEST(test_power_policy);
  RUN_TEST(test_link_config_packet_bytes);
  RUN_TEST(test_power_default_and_ceiling_clamp);
  RUN_TEST(test_link_config_follower_apply_and_failsafe);
  RUN_TEST(test_tx_router_sends_link_config_continuously);
  RUN_TEST(test_tx_router_drops_link_config_from_usb);
  RUN_TEST(test_ship_accepts_link_config_only_from_router_over_air);
  RUN_TEST(test_e2e_link_config_mirrors_power_and_failsafe);
  RUN_TEST(test_ship_reboot_requester_retries_until_ack);
  RUN_TEST(test_e2e_ship_reboot_request_reaches_ship_node);
  RUN_TEST(test_lua_power_levels_and_options);
  RUN_TEST(test_lua_read_entries);
  RUN_TEST(test_lua_power_write_and_antenna_write);
  RUN_TEST(test_lua_command_confirmation_flow);
  RUN_TEST(test_e2e_cmd_state_reaches_uart_at_25hz_and_pocket_rc_never);
  RUN_TEST(test_e2e_cmd_state_spoof_sources_never_reach_uart);
  RUN_TEST(test_rx_cmd_state_from_vehicle_side_not_on_air_and_downlink_unaffected);
  RUN_TEST(test_e2e_gs_heartbeat_and_rc_never_reach_uart_but_request_does);
  RUN_TEST(test_e2e_request_ack_from_vehicle_reaches_usb);
  RUN_TEST(test_e2e_upload_command_to_rx_over_air_ack_before_entry);
  RUN_TEST(test_e2e_upload_command_to_tx_acked_on_usb);
  RUN_TEST(test_e2e_gs_off_silence_and_no_late_release);
  return UNITY_END();
}
