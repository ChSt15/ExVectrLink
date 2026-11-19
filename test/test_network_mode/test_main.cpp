// Host tests of the TX-module network mode (src_network/): CRSF <-> packet mapping, RC state expiry, egress
// filters, heartbeat, telemetry conversions and the handset reply round robin.
// Run: ~/.platformio/penv/bin/pio test -e native
#include <unity.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <atomic>
#include <limits>
#include <new>
#include <vector>

#include "NetworkRouter.hpp"

using namespace VCTR;
using namespace VCTR::ExVectrLink::netmode;
namespace net = VCTR::network;

// ---- allocation hook: no heap use once the router is built ----
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

// ---- test doubles ----
struct FakeClock : net::TimeSourceI {
  int64_t t = 1000000000LL;
  int64_t nowNs() override { return t; }
  void advanceMs(int ms) { t += static_cast<int64_t>(ms) * 1000000LL; }
};

/// Records frames sent to it; inject() delivers a frame as if it had arrived on this link.
struct RecordLink : net::FrameLinkI {
  struct Item { uint8_t len; uint8_t d[net::kMaxNetFrame]; };
  std::vector<Item> sent;
  FakeClock *clk;
  explicit RecordLink(FakeClock &c) : clk(&c) {}
  bool send(const uint8_t *f, uint8_t len, int64_t) override {
    Item it;
    it.len = len;
    memcpy(it.d, f, len);
    sent.push_back(it);
    stats.txOk++;
    return true;
  }
  uint8_t mtu() const override { return net::kMaxNetFrame; }
  void inject(const uint8_t *f, uint8_t len) {
    net::FrameView v{f, len, clk->nowNs()};
    deliver(v);
  }
};

/// The air face of the TX module as the target builds it: AirFrameLink plus the FHSS air access (master). The tests
/// play the radio: drainAir() is one own TX slot, injectFromAir() one received packet.
struct AirSide {
  NetAirAccess::Link link;
  NetAirAccess access;
  NetAirAccess &stats;
  explicit AirSide(net::TimeSourceI &t) : access(link, t, true, 25), stats(access) {}
};

struct Gate : LinkGateI {
  bool open = true;
  bool admits() override { return open; }
};

// ---- frame helpers ----
struct Frame { uint8_t d[net::kMaxNetFrame]; uint8_t len = 0; };

static Frame makeData(uint8_t src, uint8_t dst, uint8_t port, uint8_t seq, const uint8_t *pl, size_t n,
                      net::wire::Qos qos = net::wire::Qos::Telemetry, bool latest = true, uint8_t ttl = 3) {
  Frame f;
  net::wire::NetHeader h;
  h.type = 0;
  h.qos = static_cast<uint8_t>(qos);
  h.latest = latest;
  h.ttl = ttl;
  h.dst = dst;
  h.src = src;
  net::wire::encodeNetHeader(h, f.d, sizeof f.d);
  net::wire::DataHeader dh;
  dh.port = port;
  dh.seq = seq;
  net::wire::encodeDataHeader(dh, f.d + 4, sizeof f.d - 4);
  memcpy(f.d + 6, pl, n);
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
  net::wire::encodeNetHeader(h, f.d, sizeof f.d);
  net::wire::Heartbeat hb;
  hb.role = role;
  hb.schemaHash = Packets::kSchemaHash;
  net::wire::encodeHeartbeat(hb, f.d + 4, sizeof f.d - 4);
  f.len = 15;
  return f;
}

template <typename T>
static Frame makePacket(uint8_t src, uint8_t dst, uint8_t seq, const T &p, net::wire::Qos qos,
                        bool latest = true) {
  uint8_t buf[T::kMaxSize];
  size_t n = p.encode(buf, sizeof buf);
  TEST_ASSERT_TRUE(n > 0);
  return makeData(src, dst, T::kPort, seq, buf, n, qos, latest);
}

static bool isData(const uint8_t *f, uint8_t len, uint8_t port) {
  return len >= 6 && (f[0] & 0x30) == 0 && f[4] == port;
}
static bool isHeartbeat(const uint8_t *f, uint8_t len) { return len >= 15 && ((f[0] >> 4) & 3) == 1; }

static int countUsb(const RecordLink &l, uint8_t port, int dst = -1, int src = -1) {
  int n = 0;
  for (auto &i : l.sent)
    if (isData(i.d, i.len, port) && (dst < 0 || i.d[2] == dst) && (src < 0 || i.d[3] == src)) n++;
  return n;
}
static int countUsbHeartbeats(const RecordLink &l, int src) {
  int n = 0;
  for (auto &i : l.sent)
    if (isHeartbeat(i.d, i.len) && i.d[3] == src) n++;
  return n;
}

struct AirContents {
  std::vector<Frame> frames;
  int count(uint8_t port, int dst = -1, int src = -1) const {
    int n = 0;
    for (auto &f : frames)
      if (isData(f.d, f.len, port) && (dst < 0 || f.d[2] == dst) && (src < 0 || f.d[3] == src)) n++;
    return n;
  }
  int heartbeats(int src) const {
    int n = 0;
    for (auto &f : frames)
      if (isHeartbeat(f.d, f.len) && f.d[3] == src) n++;
    return n;
  }
  const Frame *find(uint8_t port) const {
    for (auto &f : frames)
      if (isData(f.d, f.len, port)) return &f;
    return nullptr;
  }
};

/// Plays the far end of the air link for one slot: asks the air link for its next packet and parses it.
static AirContents drainAir(AirSide &air, int64_t nowNs) {
  uint8_t buf[255];
  size_t n = air.link.buildAirPacket(buf, sizeof buf, nowNs);
  TEST_ASSERT_TRUE(n >= 7);
  net::wire::AirPacketReader r;
  TEST_ASSERT_TRUE(r.parse(buf, n));
  AirContents c;
  const uint8_t *f;
  size_t fl;
  while (r.next(f, fl)) {
    Frame fr;
    memcpy(fr.d, f, fl);
    fr.len = static_cast<uint8_t>(fl);
    c.frames.push_back(fr);
  }
  return c;
}

/// Delivers frames from the vehicle side into the air link (as the FHSS access would after a received packet).
static void injectFromAir(AirSide &air, FakeClock &clk, const Frame &f) {
  uint8_t buf[255];
  net::wire::AirHeader h;
  h.dirSlaveToMaster = true; // vehicle receiver -> TX module
  net::wire::AirPacketWriter w;
  TEST_ASSERT_TRUE(w.begin(buf, sizeof buf, h));
  TEST_ASSERT_TRUE(w.add(f.d, f.len));
  size_t n = w.finish(255);
  air.link.onAirPacket(buf, n, clk.nowNs(), -70, 5);
}

struct Rig {
  FakeClock clk;
  RecordLink usb{clk};
  AirSide air{clk};
  Gate gate;
  NetworkRouter router;
  Rig() : router(clk, usb, &gate, air.link, air.stats) {}
};

static void rawChannels(uint16_t ch[16], uint16_t v) {
  for (int i = 0; i < 16; i++) ch[i] = v;
}

// ---------------------------------------------------------------------------------------------------------
// CRSF -> RC_STATE mapping
// ---------------------------------------------------------------------------------------------------------

void test_mapping_points() {
  TEST_ASSERT_EQUAL_INT16(0, crsfToRcChannel(992));
  TEST_ASSERT_EQUAL_INT16(1000, crsfToRcChannel(992 + 820));
  TEST_ASSERT_EQUAL_INT16(-1000, crsfToRcChannel(992 - 820)); // 172
  TEST_ASSERT_EQUAL_INT16(999, crsfToRcChannel(1811));         // CRSF max of the 988-2012 us range
  // rounding, symmetric about zero: 1 tick = 1.2195 -> 1; 410/1000 ticks round up at the half
  TEST_ASSERT_EQUAL_INT16(1, crsfToRcChannel(993));
  TEST_ASSERT_EQUAL_INT16(-1, crsfToRcChannel(991));
  TEST_ASSERT_EQUAL_INT16(11, crsfToRcChannel(1001)); // 10.98
  TEST_ASSERT_EQUAL_INT16(-11, crsfToRcChannel(983));
  TEST_ASSERT_EQUAL_INT16(500, crsfToRcChannel(992 + 410)); // exactly 500
}

void test_mapping_out_of_range_clamps() {
  TEST_ASSERT_EQUAL_INT16(-1000, crsfToRcChannel(0));
  TEST_ASSERT_EQUAL_INT16(-1000, crsfToRcChannel(100));
  TEST_ASSERT_EQUAL_INT16(1000, crsfToRcChannel(1813));
  TEST_ASSERT_EQUAL_INT16(1000, crsfToRcChannel(2047));
  TEST_ASSERT_EQUAL_INT16(1000, crsfToRcChannel(0xFFFF));
}

void test_mapping_is_monotonic_and_bounded() {
  int16_t prev = -1000;
  for (uint32_t v = 0; v <= 2047; v++) {
    int16_t c = crsfToRcChannel(static_cast<uint16_t>(v));
    TEST_ASSERT_TRUE(c >= prev);
    TEST_ASSERT_TRUE(c >= -1000 && c <= 1000);
    prev = c;
  }
}

void test_rcstate_all_channels_and_source_ok() {
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) ch[i] = static_cast<uint16_t>(172 + i * 100);
  Packets::RcState rc;
  rc.reserved = 0x55;
  rcStateFromCrsf(ch, rc);
  TEST_ASSERT_EQUAL_UINT8(Packets::RcState::kFlagsSourceOk, rc.flags);
  TEST_ASSERT_EQUAL_UINT8(0, rc.reserved);
  for (int i = 0; i < 16; i++) TEST_ASSERT_EQUAL_INT16(crsfToRcChannel(ch[i]), rc.ch[i]);
  uint8_t buf[34];
  TEST_ASSERT_EQUAL_UINT(34, rc.encode(buf, sizeof buf));
  TEST_ASSERT_EQUAL_UINT8(1, buf[0]);
}

void test_crsf_channel_pack_roundtrip() {
  uint16_t in[16], out[16];
  for (int i = 0; i < 16; i++) in[i] = static_cast<uint16_t>((i * 131 + 7) & 0x7FF);
  uint8_t pl[22];
  crsfPackRcChannels(in, pl);
  crsfUnpackRcChannels(pl, out);
  for (int i = 0; i < 16; i++) TEST_ASSERT_EQUAL_UINT16(in[i], out[i]);
}

// ---------------------------------------------------------------------------------------------------------
// Router: RC_STATE out, expiry, filters, heartbeat
// ---------------------------------------------------------------------------------------------------------

void test_rc_state_goes_to_air_dst1_and_usb_dst0() {
  Rig r;
  uint16_t ch[16];
  rawChannels(ch, 992);
  ch[0] = 172;
  ch[1] = 1811;
  ch[15] = 2047;
  r.router.onCrsfRcFrame(ch);

  // USB: exactly one RC_STATE, dst 0, src 2, QoS control, latest
  TEST_ASSERT_EQUAL_INT(1, countUsb(r.usb, 0x10));
  TEST_ASSERT_EQUAL_INT(1, countUsb(r.usb, 0x10, 0, 2));
  const auto &u = r.usb.sent.back();
  TEST_ASSERT_EQUAL_UINT8(0x02, u.d[0] & 0x0F); // QoS 0, LATEST
  Packets::RcState rc;
  TEST_ASSERT_TRUE(rc.decode(u.d + 6, u.len - 6));
  TEST_ASSERT_EQUAL_UINT8(1, rc.flags & 1);
  TEST_ASSERT_EQUAL_INT16(-1000, rc.ch[0]);
  TEST_ASSERT_EQUAL_INT16(999, rc.ch[1]);
  TEST_ASSERT_EQUAL_INT16(0, rc.ch[2]);
  TEST_ASSERT_EQUAL_INT16(1000, rc.ch[15]);
  // No dst-1 copy on USB
  TEST_ASSERT_EQUAL_INT(0, countUsb(r.usb, 0x10, 1));

  // Air: exactly one RC_STATE, dst 1, and never a dst-0 copy
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, a.count(0x10));
  TEST_ASSERT_EQUAL_INT(1, a.count(0x10, 1, 2));
  TEST_ASSERT_EQUAL_INT(0, a.count(0x10, 0));
  TEST_ASSERT_EQUAL_UINT32(2, r.router.stats.rcFramesOffered * 2); // 1 frame, two copies offered
  TEST_ASSERT_EQUAL_UINT32(0, r.router.stats.rcSendFailed);
}

void test_each_frame_replaces_queued_state() {
  Rig r;
  uint16_t ch[16];
  for (int k = 0; k < 6; k++) {
    rawChannels(ch, static_cast<uint16_t>(992 + k * 50));
    r.router.onCrsfRcFrame(ch);
    r.clk.advanceMs(4);
  }
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, a.count(0x10)); // one queued state, the newest
  Packets::RcState rc;
  const Frame *f = a.find(0x10);
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_TRUE(rc.decode(f->d + 6, f->len - 6));
  TEST_ASSERT_EQUAL_INT16(crsfToRcChannel(992 + 250), rc.ch[0]);
}

void test_expiry_100ms_and_no_reemission() {
  Rig r;
  uint16_t ch[16];
  rawChannels(ch, 1500);
  r.router.onCrsfRcFrame(ch);
  TEST_ASSERT_TRUE(r.router.rcSourceFresh());

  r.clk.advanceMs(99);
  TEST_ASSERT_TRUE(r.router.rcSourceFresh());
  r.clk.advanceMs(1); // 100 ms: expired
  TEST_ASSERT_FALSE(r.router.rcSourceFresh());

  size_t usbBefore = r.usb.sent.size();
  const int rcOnUsb = countUsb(r.usb, 0x10);

  // The queued air state is gone when the next slot builds its packet: never released late.
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, a.count(0x10));

  // Nothing is synthesised or repeated while the handset is silent, however long that lasts.
  for (int i = 0; i < 100; i++) {
    r.clk.advanceMs(20);
    r.router.poll();
    AirContents b = drainAir(r.air, r.clk.nowNs());
    TEST_ASSERT_EQUAL_INT(0, b.count(0x10));
  }
  TEST_ASSERT_EQUAL_INT(rcOnUsb, countUsb(r.usb, 0x10));
  TEST_ASSERT_TRUE(r.usb.sent.size() >= usbBefore); // heartbeats/link stats still flow
  TEST_ASSERT_EQUAL_UINT32(1, r.router.stats.rcFramesOffered);

  // A fresh CRSF frame resumes it, with the new value only.
  rawChannels(ch, 800);
  r.router.onCrsfRcFrame(ch);
  AirContents c = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, c.count(0x10));
  TEST_ASSERT_TRUE(r.router.rcSourceFresh());
}

void test_expired_state_not_released_after_air_stall() {
  // The air side is not drained for 150 ms (no slot): the queued RC_STATE must be dropped, not sent late.
  Rig r;
  uint16_t ch[16];
  rawChannels(ch, 1200);
  r.router.onCrsfRcFrame(ch);
  r.clk.advanceMs(150);
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, a.count(0x10));
}

void test_usb_host_absent_only_stops_usb() {
  Rig r;
  r.gate.open = false; // host gone
  uint16_t ch[16];
  rawChannels(ch, 1300);
  r.router.onCrsfRcFrame(ch);
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(0, (int)r.usb.sent.size());
  TEST_ASSERT_TRUE(r.router.usbGuard().guardStats.droppedByGate > 0);
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, a.count(0x10, 1));
  TEST_ASSERT_EQUAL_INT(1, a.heartbeats(2)); // the module still announces itself on the air side

  // Host returns later: nothing from the outage is released.
  r.clk.advanceMs(500);
  r.gate.open = true;
  size_t n = r.usb.sent.size();
  TEST_ASSERT_EQUAL_UINT(0, n);
  TEST_ASSERT_EQUAL_INT(0, countUsb(r.usb, 0x10));
}

void test_heartbeat_contents() {
  Rig r;
  r.router.poll(); // first heartbeat on the first poll
  const RecordLink::Item *hb = nullptr;
  for (auto &i : r.usb.sent)
    if (isHeartbeat(i.d, i.len) && i.d[3] == 2) hb = &i;
  TEST_ASSERT_NOT_NULL(hb);
  TEST_ASSERT_EQUAL_UINT8(0x56, hb->d[0]); // VER 1, HEARTBEAT, QoS link, LATEST
  TEST_ASSERT_EQUAL_UINT8(3, hb->d[1]);    // TTL
  TEST_ASSERT_EQUAL_UINT8(0xFF, hb->d[2]); // broadcast
  TEST_ASSERT_EQUAL_UINT8(2, hb->d[3]);    // src: TX module
  net::wire::Heartbeat h;
  TEST_ASSERT_TRUE(net::wire::decodeHeartbeat(hb->d + 4, hb->len - 4, h));
  TEST_ASSERT_EQUAL_UINT8(2, h.role); // link TX module
  TEST_ASSERT_EQUAL_UINT8(1, h.netVersion);
  TEST_ASSERT_EQUAL_HEX32(Packets::kSchemaHash, h.schemaHash);
  TEST_ASSERT_EQUAL_HEX8(0, h.flags);

  // and on the air too
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, a.heartbeats(2));

  // 1 Hz
  size_t count1 = 0;
  for (auto &i : r.usb.sent) if (isHeartbeat(i.d, i.len)) count1++;
  r.clk.advanceMs(500);
  r.router.poll();
  size_t count2 = 0;
  for (auto &i : r.usb.sent) if (isHeartbeat(i.d, i.len)) count2++;
  TEST_ASSERT_EQUAL_UINT(count1, count2);
  r.clk.advanceMs(500);
  r.router.poll();
  size_t count3 = 0;
  for (auto &i : r.usb.sent) if (isHeartbeat(i.d, i.len)) count3++;
  TEST_ASSERT_EQUAL_UINT(count1 + 1, count3);
}

void test_own_link_stats_to_usb_stub_values() {
  Rig r;
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(1, countUsb(r.usb, 0x01, 0xFF, 2));
  const RecordLink::Item *ls = nullptr;
  for (auto &i : r.usb.sent) if (isData(i.d, i.len, 0x01)) ls = &i;
  TEST_ASSERT_NOT_NULL(ls);
  Packets::LinkStats s;
  TEST_ASSERT_TRUE(s.decode(ls->d + 6, ls->len - 6));
  TEST_ASSERT_TRUE(s.air_state == Packets::AirState::kOff);
  TEST_ASSERT_EQUAL_UINT8(0, s.rx_lq);
}

void test_filter_node0_heartbeat_dropped_at_air_egress() {
  Rig r;
  r.router.poll();
  Frame hb0 = makeHeartbeat(0, 0);
  r.usb.inject(hb0.d, hb0.len);

  // It is node 0's heartbeat arriving on USB: the router would forward it (broadcast, TTL 3) to the air link.
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, a.heartbeats(0)); // the vehicle never sees the ground station over radio
  TEST_ASSERT_EQUAL_INT(1, a.heartbeats(2)); // module heartbeats pass
  TEST_ASSERT_TRUE(r.router.node().stats.egressFiltered >= 1);
  TEST_ASSERT_TRUE(r.router.node().isPresent(0)); // but the module itself sees node 0
}

void test_filter_vehicle_heartbeat_forwarded_to_usb() {
  Rig r;
  Frame hb1 = makeHeartbeat(1, 1);
  injectFromAir(r.air, r.clk, hb1);
  TEST_ASSERT_EQUAL_INT(1, countUsbHeartbeats(r.usb, 1));
}

void test_filter_rc_state_from_usb_not_forwarded_to_air() {
  Rig r;
  Packets::RcState rc;
  rc.flags = 1;
  for (int i = 0; i < 16; i++) rc.ch[i] = 1000;
  // The ground station's virtual radio: src 0, dst 1, port 0x10, arriving on USB.
  Frame f = makePacket(0, 1, 7, rc, net::wire::Qos::Control);
  r.usb.inject(f.d, f.len);

  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, a.count(0x10)); // not on the radio path
  TEST_ASSERT_TRUE(r.router.node().stats.egressFiltered >= 1);

  // User decision 2026-11-19: the ground station's other traffic passes over the radio: a REQUEST to the vehicle
  // (calibration commands) crosses the air.
  Packets::Request req;
  req.req_id = 5;
  Frame g = makePacket(0, 1, 1, req, net::wire::Qos::Event, false);
  r.usb.inject(g.d, g.len);
  AirContents b = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, b.count(0x11, 1, 0));
  // RC_STATE arriving over USB on behalf of another source is still dropped by the port rule (second layer).
  Frame h = makePacket(1, 1, 8, rc, net::wire::Qos::Control);
  r.usb.inject(h.d, h.len);
  AirContents c = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, c.count(0x10));
}

void test_filter_usb_only_ports_not_on_air_but_on_usb() {
  Rig r;
  uint8_t pl[8] = {1, 2, 3};
  Frame dbgFromVehicle = makeData(1, 0, 0x90, 1, pl, 3); // vehicle -> GS, USB-only range
  injectFromAir(r.air, r.clk, dbgFromVehicle);
  TEST_ASSERT_EQUAL_INT(1, countUsb(r.usb, 0x90)); // reaches the ground station
  Frame dbgFromGs = makeData(0, 1, 0x91, 1, pl, 3); // GS -> vehicle, USB-only range
  r.usb.inject(dbgFromGs.d, dbgFromGs.len);
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, a.count(0x91)); // dropped by the air egress port limit
}

void test_vehicle_rc_state_never_looped_back() {
  // RC_STATE originated by the module is never delivered to the module itself, and the router does not
  // mirror USB RC_STATE to USB (ingress link is excluded).
  Rig r;
  Packets::RcState rc;
  rc.flags = 1;
  Frame f = makePacket(0, 1, 9, rc, net::wire::Qos::Control);
  r.usb.inject(f.d, f.len);
  TEST_ASSERT_EQUAL_INT(0, countUsb(r.usb, 0x10));
}

// ---------------------------------------------------------------------------------------------------------
// CRSF telemetry
// ---------------------------------------------------------------------------------------------------------

static void checkFrame(const CrsfFrame &f, uint8_t type, size_t payloadLen) {
  TEST_ASSERT_EQUAL_UINT8(0xC8, f.bytes[0]);
  TEST_ASSERT_EQUAL_UINT8(payloadLen + 2, f.bytes[1]);
  TEST_ASSERT_EQUAL_UINT8(type, f.bytes[2]);
  TEST_ASSERT_EQUAL_UINT(payloadLen + 4, f.len);
  TEST_ASSERT_EQUAL_UINT8(crsfCrc8(f.bytes + 2, payloadLen + 1), f.bytes[3 + payloadLen]);
}

static int16_t be16(const uint8_t *p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }
static int32_t be32(const uint8_t *p) {
  return static_cast<int32_t>((uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3]);
}

void test_crc8_known_value() {
  // CRC-8/DVB-S2 of "123456789" is 0xBC
  const uint8_t d[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX8(0xBC, crsfCrc8(d, sizeof d));
}

static Packets::HandsetTelem sampleTelem() {
  Packets::HandsetTelem t;
  t.flags = Packets::HandsetTelem::kFlagsArmed | Packets::HandsetTelem::kFlagsGnssFix;
  memset(t.mode_text, 0, sizeof t.mode_text);
  memcpy(t.mode_text, "ARMD", 4);
  const double kDeg = M_PI / 180.0;
  t.latitude = 47.1234567 * kDeg;
  t.longitude = -8.7654321 * kDeg;
  t.ground_speed = 12.5f;   // m/s -> 45 km/h
  t.heading = 90.0f * float(kDeg);
  t.altitude = 123.4f;
  t.satellites = 11;
  t.vertical_speed = -2.5f; // m/s -> -250 cm/s
  t.pitch = 0.1f;
  t.roll = -0.2f;
  t.yaw = 3.0f;
  t.battery = 7.4f;
  return t;
}

/// Sends a HANDSET_TELEM through the wire (so it is quantised as on a real link) and returns what the router decoded.
static Packets::HandsetTelem viaWire(const Packets::HandsetTelem &in) {
  uint8_t b[34];
  TEST_ASSERT_EQUAL_UINT(34, in.encode(b, sizeof b));
  Packets::HandsetTelem out;
  TEST_ASSERT_TRUE(out.decode(b, sizeof b));
  return out;
}

void test_conversion_flight_mode() {
  CrsfFrame f;
  crsfBuildFlightMode(f, viaWire(sampleTelem()));
  checkFrame(f, 0x21, 5);
  TEST_ASSERT_EQUAL_STRING("ARMD", reinterpret_cast<const char *>(f.bytes + 3));
  // an unterminated 8-character mode text is cut at 8 and terminated
  Packets::HandsetTelem t = sampleTelem();
  memcpy(t.mode_text, "ABCDEFGH", 8);
  crsfBuildFlightMode(f, t);
  checkFrame(f, 0x21, 9);
  TEST_ASSERT_EQUAL_STRING("ABCDEFGH", reinterpret_cast<const char *>(f.bytes + 3));
  // non-printable bytes are replaced
  t.mode_text[0] = 0x01;
  crsfBuildFlightMode(f, t);
  TEST_ASSERT_EQUAL_UINT8('?', f.bytes[3]);
}

void test_conversion_gps() {
  CrsfFrame f;
  crsfBuildGps(f, viaWire(sampleTelem()));
  checkFrame(f, 0x02, 15);
  const uint8_t *p = f.bytes + 3;
  TEST_ASSERT_EQUAL_INT32(471234567, be32(p));
  TEST_ASSERT_EQUAL_INT32(-87654321, be32(p + 4));
  TEST_ASSERT_EQUAL_INT((12.5 * 3.6 * 10.0) + 0.5, (uint16_t)be16(p + 8)); // 0.1 km/h units: 450
  TEST_ASSERT_EQUAL_INT(9000, (uint16_t)be16(p + 10));                      // centi-degrees
  TEST_ASSERT_EQUAL_INT(1123, (uint16_t)be16(p + 12));                      // 123.4 m + 1000, rounded
  TEST_ASSERT_EQUAL_UINT8(11, p[14]);
}

void test_conversion_gps_negative_altitude_and_clamps() {
  CrsfFrame f;
  Packets::HandsetTelem t = sampleTelem();
  t.altitude = -50.0f;
  t.ground_speed = 100000.0f;
  crsfBuildGps(f, t);
  TEST_ASSERT_EQUAL_INT(950, (uint16_t)be16(f.bytes + 3 + 12));
  TEST_ASSERT_EQUAL_INT(65535, (uint16_t)be16(f.bytes + 3 + 8));
  t.altitude = -5000.0f; // below the representable range
  crsfBuildGps(f, t);
  TEST_ASSERT_EQUAL_INT(0, (uint16_t)be16(f.bytes + 3 + 12));
}

void test_conversion_attitude_vario_battery() {
  const Packets::HandsetTelem t = viaWire(sampleTelem());
  CrsfFrame f;
  crsfBuildAttitude(f, t);
  checkFrame(f, 0x1E, 6);
  TEST_ASSERT_EQUAL_INT16(1000, be16(f.bytes + 3));   // pitch 0.1 rad
  TEST_ASSERT_EQUAL_INT16(-2000, be16(f.bytes + 5));  // roll -0.2 rad
  TEST_ASSERT_EQUAL_INT16(30000, be16(f.bytes + 7));  // yaw 3.0 rad

  crsfBuildVario(f, t);
  checkFrame(f, 0x07, 2);
  TEST_ASSERT_EQUAL_INT16(-250, be16(f.bytes + 3));

  crsfBuildBattery(f, t);
  checkFrame(f, 0x08, 8);
  TEST_ASSERT_EQUAL_INT16(74, be16(f.bytes + 3)); // 7.4 V in 100 mV
  for (int i = 5; i < 11; i++) TEST_ASSERT_EQUAL_UINT8(0, f.bytes[i]);
}

void test_conversion_link_statistics() {
  Packets::LinkStats ls;
  ls.rssi = -85;
  ls.snr = -6;
  ls.rx_lq = 97;
  ls.antenna = 0x21; // RX winner 1, TX antenna 2
  LocalAirStats local;
  local.rssi = -90;
  local.snr = 4;
  local.rxLq = 88;
  local.txPower = 20; // 100 mW -> index 3
  CrsfFrame f;
  crsfBuildLinkStatistics(f, &ls, local);
  checkFrame(f, 0x14, 10);
  const uint8_t *p = f.bytes + 3;
  TEST_ASSERT_EQUAL_UINT8(85, p[0]);
  TEST_ASSERT_EQUAL_UINT8(0, p[1]);
  TEST_ASSERT_EQUAL_UINT8(97, p[2]);
  TEST_ASSERT_EQUAL_INT8(-6, static_cast<int8_t>(p[3]));
  TEST_ASSERT_EQUAL_UINT8(1, p[4]);
  TEST_ASSERT_EQUAL_UINT8(4, p[5]);
  TEST_ASSERT_EQUAL_UINT8(3, p[6]);
  TEST_ASSERT_EQUAL_UINT8(90, p[7]);
  TEST_ASSERT_EQUAL_UINT8(88, p[8]);
  TEST_ASSERT_EQUAL_INT8(4, static_cast<int8_t>(p[9]));
}

void test_link_statistics_zero_when_node3_absent() {
  LocalAirStats local;
  local.rssi = -90;
  local.rxLq = 88; // even a non-zero local reading is not shown without node 3
  CrsfFrame f;
  crsfBuildLinkStatistics(f, nullptr, local);
  checkFrame(f, 0x14, 10);
  for (int i = 3; i < 13; i++) TEST_ASSERT_EQUAL_UINT8(0, f.bytes[i]);
}

/// Collects the telemetry frames the router hands out, one per simulated handset frame (4 ms).
static std::vector<CrsfFrame> runHandset(Rig &r, int ms) {
  std::vector<CrsfFrame> out;
  for (int t = 0; t < ms; t += 4) {
    r.clk.advanceMs(4);
    CrsfFrame f;
    if (r.router.nextHandsetReply(f)) out.push_back(f);
  }
  return out;
}

void test_router_link_stats_10hz_zeroed_without_node3() {
  Rig r;
  auto frames = runHandset(r, 1000);
  int n = 0;
  for (auto &f : frames) {
    TEST_ASSERT_EQUAL_UINT8(0x14, f.bytes[2]); // nothing else without vehicle data
    for (int i = 3; i < 13; i++) TEST_ASSERT_EQUAL_UINT8(0, f.bytes[i]);
    n++;
  }
  TEST_ASSERT_TRUE(n >= 9 && n <= 11); // 10 Hz
}

void test_router_link_stats_from_node3_only() {
  Rig r;
  Packets::LinkStats ls;
  ls.rssi = -70;
  ls.rx_lq = 95;
  ls.snr = 7;
  // From the ground station claiming port 0x01: rejected (only node 3 reports the uplink).
  Frame bad = makePacket(0, 0xFF, 1, ls, net::wire::Qos::Link);
  r.usb.inject(bad.d, bad.len);
  TEST_ASSERT_FALSE(r.router.linkStatsPort().hasValue());
  TEST_ASSERT_EQUAL_UINT32(1, r.router.linkStatsPort().rejectedSource());

  // From node 3 (here injected over USB as a bench test would): accepted, and node 3 becomes present.
  Frame good = makePacket(3, 0xFF, 1, ls, net::wire::Qos::Link);
  r.usb.inject(good.d, good.len);
  TEST_ASSERT_TRUE(r.router.linkStatsPort().hasValue());
  TEST_ASSERT_TRUE(r.router.node().isPresent(3));
  CrsfFrame f;
  TEST_ASSERT_TRUE(r.router.nextHandsetReply(f));
  TEST_ASSERT_EQUAL_UINT8(0x14, f.bytes[2]);
  TEST_ASSERT_EQUAL_UINT8(70, f.bytes[3]);
  TEST_ASSERT_EQUAL_UINT8(95, f.bytes[5]);

  // Node 3 falls silent: after 3 s the next 10 Hz frame is zeroed again.
  r.clk.advanceMs(3100);
  TEST_ASSERT_TRUE(r.router.nextHandsetReply(f));
  TEST_ASSERT_EQUAL_UINT8(0x14, f.bytes[2]);
  for (int i = 3; i < 13; i++) TEST_ASSERT_EQUAL_UINT8(0, f.bytes[i]);
}

void test_router_handset_telem_round_robin() {
  Rig r;
  Frame t = makePacket(1, 0xFF, 1, sampleTelem(), net::wire::Qos::Link);
  r.usb.inject(t.d, t.len);
  TEST_ASSERT_TRUE(r.router.handsetTelemPort().hasValue());

  // One frame per handset frame: LINK_STATS first (due), then the others in order.
  CrsfFrame f;
  std::vector<uint8_t> order;
  for (int i = 0; i < 6; i++) {
    r.clk.advanceMs(4);
    if (r.router.nextHandsetReply(f)) order.push_back(f.bytes[2]);
    else order.push_back(0);
  }
  const uint8_t expect[] = {0x14, 0x21, 0x02, 0x1E, 0x07, 0x08};
  TEST_ASSERT_EQUAL_UINT(6, order.size());
  for (int i = 0; i < 6; i++) TEST_ASSERT_EQUAL_UINT8(expect[i], order[i]);

  // Afterwards each kind is limited to 10 Hz: 1 s of 4 ms handset frames gives about 10 of each.
  int counts[0x22] = {};
  auto frames = runHandset(r, 1000);
  for (auto &fr : frames) counts[fr.bytes[2]]++;
  const uint8_t kinds[] = {0x14, 0x21, 0x02, 0x1E, 0x07, 0x08};
  for (uint8_t k : kinds) TEST_ASSERT_TRUE(counts[k] >= 9 && counts[k] <= 11);
}

void test_router_handset_telem_from_vehicle_only() {
  Rig r;
  Frame bad = makePacket(0, 0xFF, 1, sampleTelem(), net::wire::Qos::Link); // ground station: not vehicle state
  r.usb.inject(bad.d, bad.len);
  TEST_ASSERT_FALSE(r.router.handsetTelemPort().hasValue());
  auto frames = runHandset(r, 500);
  for (auto &f : frames) TEST_ASSERT_EQUAL_UINT8(0x14, f.bytes[2]);
}

void test_router_battery_skipped_when_unknown() {
  Rig r;
  Packets::HandsetTelem t = sampleTelem();
  t.battery = std::numeric_limits<float>::quiet_NaN(); // encodes as wire 0 = unknown
  Frame f = makePacket(1, 0xFF, 1, t, net::wire::Qos::Link);
  r.usb.inject(f.d, f.len);
  auto frames = runHandset(r, 600);
  int bat = 0, mode = 0;
  for (auto &fr : frames) {
    if (fr.bytes[2] == 0x08) bat++;
    if (fr.bytes[2] == 0x21) mode++;
  }
  TEST_ASSERT_EQUAL_INT(0, bat);
  TEST_ASSERT_TRUE(mode >= 5);
}

void test_router_stale_vehicle_telemetry_not_shown() {
  Rig r;
  Frame f = makePacket(1, 0xFF, 1, sampleTelem(), net::wire::Qos::Link);
  r.usb.inject(f.d, f.len);
  auto early = runHandset(r, 400);
  bool sawMode = false;
  for (auto &fr : early) if (fr.bytes[2] == 0x21) sawMode = true;
  TEST_ASSERT_TRUE(sawMode);
  r.clk.advanceMs(3000); // vehicle silent for > 3 s
  auto late = runHandset(r, 500);
  for (auto &fr : late) TEST_ASSERT_EQUAL_UINT8(0x14, fr.bytes[2]);
}

void test_log_text_goes_to_usb_not_air() {
  Rig r;
  TEST_ASSERT_TRUE(r.router.log(Packets::Severity::kInfo, "hello"));
  TEST_ASSERT_EQUAL_INT(1, countUsb(r.usb, 0x02, 0, 2));
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(0, a.count(0x02));
}

void test_no_heap_use_after_construction() {
  Rig r;
  r.usb.sent.reserve(4096); // the recorder is the test's own heap use
  uint16_t ch[16];
  rawChannels(ch, 1000);
  const unsigned long before = g_allocs.load();
  for (int i = 0; i < 200; i++) {
    r.clk.advanceMs(4);
    r.router.onCrsfRcFrame(ch);
    r.router.poll();
    CrsfFrame f;
    r.router.nextHandsetReply(f);
    if (i % 5 == 0) {
      uint8_t buf[255];
      r.air.link.buildAirPacket(buf, sizeof buf, r.clk.nowNs());
    }
  }
  Frame hb = makeHeartbeat(0, 0);
  r.usb.inject(hb.d, hb.len);
  TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)(g_allocs.load() - before));
}

void test_filter_only_gs_heartbeat_and_rc_state_dropped_on_air() {
  Rig r;
  const uint8_t pl[12] = {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  Frame req = makeData(0, 1, 0x11, 1, pl, sizeof pl, net::wire::Qos::Event, false);   // REQUEST to the vehicle
  r.usb.inject(req.d, req.len);
  Frame reqRx = makeData(0, 3, 0x11, 2, pl, sizeof pl, net::wire::Qos::Event, false); // REQUEST to the RX module
  r.usb.inject(reqRx.d, reqRx.len);
  Frame other = makeData(0, 0xFF, 0x30, 1, pl, 4);                                    // other GS data, broadcast
  r.usb.inject(other.d, other.len);
  Frame hb0 = makeHeartbeat(0, 0);
  r.usb.inject(hb0.d, hb0.len);
  Frame rc0 = makeData(0, 1, 0x10, 3, pl, 4, net::wire::Qos::Control, true);          // RC_STATE from the GS
  r.usb.inject(rc0.d, rc0.len);
  Frame toGs = makeData(0, 0, 0x30, 1, pl, 4);                                        // dst 0 never on air
  r.usb.inject(toGs.d, toGs.len);
  // A frame injected over USB on behalf of the vehicle (src 1) still passes (bench test 8).
  Frame veh = makeData(1, 2, 0x30, 1, pl, 4);
  r.usb.inject(veh.d, veh.len);
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, a.count(0x11, 1, 0));
  TEST_ASSERT_EQUAL_INT(1, a.count(0x11, 3, 0));
  TEST_ASSERT_EQUAL_INT(1, a.count(0x30, 0xFF, 0));
  TEST_ASSERT_EQUAL_INT(0, a.heartbeats(0));
  TEST_ASSERT_EQUAL_INT(0, a.count(0x10));
  TEST_ASSERT_EQUAL_INT(0, a.count(0x30, 0, 0));
}

void test_vehicle_request_ack_reaches_usb() {
  Rig r;
  Packets::RequestAck ack;
  ack.req_id = 9;
  Frame f = makePacket(1, 0, 4, ack, net::wire::Qos::Event, false);
  injectFromAir(r.air, r.clk, f);
  TEST_ASSERT_EQUAL_INT(1, countUsb(r.usb, 0x12, 0, 1));
}

// ---- link upload command (REQUEST action 10, target 2), TX module ----

static Frame uploadRequest(uint8_t src, uint8_t dst, uint8_t seq, uint16_t reqId, uint8_t target = 2,
                           uint8_t action = 10) {
  Packets::Request q;
  q.req_id = reqId;
  q.action = action;
  q.target = target;
  return makePacket(src, dst, seq, q, net::wire::Qos::Event, false);
}

static void decodeAck(const uint8_t *d, uint8_t len, Packets::RequestAck &out) {
  TEST_ASSERT_TRUE(out.decode(d + 6, len - 6));
}

void test_upload_cmd_from_usb_acked_then_due_after_delay() {
  Rig r;
  Frame q = uploadRequest(0, 2, 1, 77);
  r.usb.inject(q.d, q.len);
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(1, countUsb(r.usb, 0x12, 0, 2)); // ACK addressed to the requester (dst 0, src 2)
  for (auto &i : r.usb.sent)
    if (isData(i.d, i.len, 0x12)) {
      Packets::RequestAck a;
      decodeAck(i.d, i.len, a);
      TEST_ASSERT_EQUAL_UINT16(77, a.req_id);
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
  // honoured always: nothing but the command was needed (no arm / ground state exists in the module)
  TEST_ASSERT_TRUE(LinkUploadCommand::kUploadDelayNs >= 300000000LL && LinkUploadCommand::kUploadDelayNs <= 500000000LL);
}

void test_upload_cmd_from_air_acked_to_air() {
  Rig r;
  Frame q = uploadRequest(1, 2, 1, 5); // any link: here over the air, from the vehicle side
  injectFromAir(r.air, r.clk, q);
  r.router.poll();
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, a.count(0x12, 1, 2));
  TEST_ASSERT_EQUAL_INT(0, countUsb(r.usb, 0x12)); // not to USB
  r.clk.t += LinkUploadCommand::kUploadDelayNs + 1000000LL;
  r.router.poll();
  TEST_ASSERT_TRUE(r.router.uploadCommandDue());
}

void test_upload_cmd_repeat_is_idempotent_and_each_copy_acked() {
  Rig r;
  Frame q1 = uploadRequest(0, 2, 1, 42);
  r.usb.inject(q1.d, q1.len);
  r.router.poll();
  const int64_t first = r.clk.t;
  r.clk.t += 300000000LL; // a retry 300 ms later must not restart the delay
  Frame q2 = uploadRequest(0, 2, 2, 42);
  r.usb.inject(q2.d, q2.len);
  r.router.poll();
  TEST_ASSERT_EQUAL_INT(2, countUsb(r.usb, 0x12, 0, 2));
  r.clk.t = first + LinkUploadCommand::kUploadDelayNs + 1000000LL;
  r.router.poll();
  TEST_ASSERT_TRUE(r.router.uploadCommandDue()); // due from the FIRST copy
  TEST_ASSERT_EQUAL_UINT32(2, r.router.uploadCommand().stats.accepted);
}

void test_upload_cmd_ignored_for_other_target_action_and_malformed() {
  Rig r;
  Frame wrongTarget = uploadRequest(0, 2, 1, 1, 3);       // target the RX module, delivered here
  Frame wrongAction = uploadRequest(0, 2, 2, 2, 2, 9);    // SYSTEM_RESET for us
  Frame zeroId = uploadRequest(0, 2, 3, 0);               // req_id 0 is invalid
  const uint8_t shortPl[11] = {1, 0, 10, 2};              // shorter than the fixed Request size
  Frame shortReq = makeData(0, 2, 0x11, 4, shortPl, sizeof shortPl, net::wire::Qos::Event, false);
  Frame fromVehicleOtherTarget = uploadRequest(1, 2, 5, 9, 1);
  for (const Frame *f : {&wrongTarget, &wrongAction, &zeroId, &shortReq, &fromVehicleOtherTarget}) r.usb.inject(f->d, f->len);
  r.clk.t += 2 * LinkUploadCommand::kUploadDelayNs;
  r.router.poll();
  TEST_ASSERT_FALSE(r.router.uploadCommandDue());
  TEST_ASSERT_EQUAL_INT(0, countUsb(r.usb, 0x12));
  TEST_ASSERT_EQUAL_UINT32(0, r.router.uploadCommand().stats.accepted);
  // a heartbeat or telemetry never triggers it either
  Frame hb = makeHeartbeat(0, 0);
  r.usb.inject(hb.d, hb.len);
  r.router.poll();
  TEST_ASSERT_FALSE(r.router.uploadCommandDue());
  TEST_ASSERT_FALSE(r.router.uploadCommand().pending());
}

void test_upload_cmd_for_rx_module_is_forwarded_not_executed() {
  Rig r;
  Frame q = uploadRequest(0, 3, 1, 8, 3);
  r.usb.inject(q.d, q.len);
  AirContents a = drainAir(r.air, r.clk.nowNs());
  TEST_ASSERT_EQUAL_INT(1, a.count(0x11, 3, 0)); // forwarded to the RX module over the air
  r.clk.t += 2 * LinkUploadCommand::kUploadDelayNs;
  r.router.poll();
  TEST_ASSERT_FALSE(r.router.uploadCommandDue());
}

void test_tx_link_stats_carry_air_access_counters() {
  Rig r;
  r.air.access.setStatus(AirLinkStatus{Packets::AirState::kSynced, 20, 1, 0});
  Frame f = makeHeartbeat(3, 3);
  injectFromAir(r.air, r.clk, f); // one packet straight into the link (not via the mailbox: no LQ)
  r.router.poll();
  // LINK_STATS from node 2 on USB: air state synced, tx power 20, antenna rx 1, 1 packet accepted.
  bool found = false;
  for (auto &it : r.usb.sent)
    if (isData(it.d, it.len, 0x01) && it.d[3] == 2) {
      Packets::LinkStats ls;
      TEST_ASSERT_TRUE(ls.decode(it.d + 6, it.len - 6));
      TEST_ASSERT_EQUAL_INT((int)Packets::AirState::kSynced, (int)ls.air_state);
      TEST_ASSERT_EQUAL_INT(20, ls.tx_power);
      TEST_ASSERT_EQUAL_UINT8(0x01, ls.antenna);
      TEST_ASSERT_EQUAL_UINT16(1, ls.air_rx_ok);
      found = true;
    }
  TEST_ASSERT_TRUE(found);
}

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_mapping_points);
  RUN_TEST(test_mapping_out_of_range_clamps);
  RUN_TEST(test_mapping_is_monotonic_and_bounded);
  RUN_TEST(test_rcstate_all_channels_and_source_ok);
  RUN_TEST(test_crsf_channel_pack_roundtrip);
  RUN_TEST(test_rc_state_goes_to_air_dst1_and_usb_dst0);
  RUN_TEST(test_each_frame_replaces_queued_state);
  RUN_TEST(test_expiry_100ms_and_no_reemission);
  RUN_TEST(test_expired_state_not_released_after_air_stall);
  RUN_TEST(test_usb_host_absent_only_stops_usb);
  RUN_TEST(test_heartbeat_contents);
  RUN_TEST(test_own_link_stats_to_usb_stub_values);
  RUN_TEST(test_filter_node0_heartbeat_dropped_at_air_egress);
  RUN_TEST(test_filter_vehicle_heartbeat_forwarded_to_usb);
  RUN_TEST(test_filter_rc_state_from_usb_not_forwarded_to_air);
  RUN_TEST(test_filter_usb_only_ports_not_on_air_but_on_usb);
  RUN_TEST(test_vehicle_rc_state_never_looped_back);
  RUN_TEST(test_crc8_known_value);
  RUN_TEST(test_conversion_flight_mode);
  RUN_TEST(test_conversion_gps);
  RUN_TEST(test_conversion_gps_negative_altitude_and_clamps);
  RUN_TEST(test_conversion_attitude_vario_battery);
  RUN_TEST(test_conversion_link_statistics);
  RUN_TEST(test_link_statistics_zero_when_node3_absent);
  RUN_TEST(test_router_link_stats_10hz_zeroed_without_node3);
  RUN_TEST(test_router_link_stats_from_node3_only);
  RUN_TEST(test_router_handset_telem_round_robin);
  RUN_TEST(test_router_handset_telem_from_vehicle_only);
  RUN_TEST(test_router_battery_skipped_when_unknown);
  RUN_TEST(test_router_stale_vehicle_telemetry_not_shown);
  RUN_TEST(test_log_text_goes_to_usb_not_air);
  RUN_TEST(test_no_heap_use_after_construction);
  RUN_TEST(test_filter_only_gs_heartbeat_and_rc_state_dropped_on_air);
  RUN_TEST(test_vehicle_request_ack_reaches_usb);
  RUN_TEST(test_upload_cmd_from_usb_acked_then_due_after_delay);
  RUN_TEST(test_upload_cmd_from_air_acked_to_air);
  RUN_TEST(test_upload_cmd_repeat_is_idempotent_and_each_copy_acked);
  RUN_TEST(test_upload_cmd_ignored_for_other_target_action_and_malformed);
  RUN_TEST(test_upload_cmd_for_rx_module_is_forwarded_not_executed);
  RUN_TEST(test_tx_link_stats_carry_air_access_counters);
  return UNITY_END();
}
