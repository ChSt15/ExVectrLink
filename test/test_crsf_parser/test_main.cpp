// Host tests of src/CrsfParser.hpp: legacy mode must equal the original CRSF::decodeByte() (FPV), hardened mode
// (network TX module) must resync. Run: ~/.platformio/penv/bin/pio test -e native -f test_crsf_parser
#include <unity.h>

#include <stdlib.h>
#include <string.h>

#include <vector>

#include "CrsfParser.hpp"

using VCTR::ExVectrLink::CrsfParser;
using Frame = std::vector<uint8_t>;

// The original decodeByte() as it was before the parser was extracted (reference for the FPV identity test).
struct OldParser {
  uint8_t buf[64];
  uint8_t pos = 0;
  std::vector<Frame> out;
  void feed(uint8_t b) {
    if (pos == 0) {
      if (b == 0xC8 || b == 0x00 || b == 0xEC || b == 0xEA || b == 0xEE) buf[pos++] = b;
      return;
    }
    buf[pos++] = b;
    if (pos == 2) {
      if (buf[1] < 2 || buf[1] > 64 - 2) pos = 0;
      return;
    }
    const uint8_t total = buf[1] + 2;
    if (pos >= total) {
      if (CrsfParser::crc8(&buf[2], buf[1] - 1) == buf[total - 1]) out.emplace_back(buf, buf + total);
      pos = 0;
    }
  }
};

static Frame makeFrame(uint8_t sync, uint8_t type, const std::vector<uint8_t> &payload) {
  Frame f{sync, static_cast<uint8_t>(payload.size() + 2), type};
  f.insert(f.end(), payload.begin(), payload.end());
  f.push_back(CrsfParser::crc8(&f[2], f.size() - 2));
  return f;
}
static Frame rcFrame(uint8_t seed) {
  std::vector<uint8_t> p(22);
  for (size_t i = 0; i < p.size(); i++) p[i] = static_cast<uint8_t>(seed * 7 + i);
  return makeFrame(0xEE, 0x16, p);
}

struct Run {
  CrsfParser p;
  std::vector<Frame> out;
  explicit Run(bool hardened) { p.setHardened(hardened); }
  void feed(const Frame &bytes) {
    for (uint8_t b : bytes) p.feed(b, [this](const uint8_t *f, size_t n) { out.emplace_back(f, f + n); });
  }
};

void test_legacy_identical_to_original_on_random_streams() {
  srand(1234);
  for (int round = 0; round < 300; round++) {
    Frame stream;
    for (int i = 0; i < 60; i++) {
      const int r = rand() % 10;
      if (r < 5) {
        Frame f = rcFrame(static_cast<uint8_t>(rand()));
        if (r == 0) f.back() ^= 0x5A;                       // corrupt CRC
        if (r == 1) f[1] = static_cast<uint8_t>(rand());    // corrupt length
        stream.insert(stream.end(), f.begin(), f.end());
      } else if (r < 8) {
        const int n = rand() % 6;
        for (int k = 0; k < n; k++) stream.push_back(static_cast<uint8_t>(rand() % 3 ? rand() : 0x00));
      } else {
        Frame f = makeFrame(0xC8, 0x2C, {0xEA, 0xEE, 1});
        stream.insert(stream.end(), f.begin(), f.end());
      }
    }
    OldParser old;
    Run legacy(false);
    for (uint8_t b : stream) old.feed(b);
    legacy.feed(stream);
    TEST_ASSERT_EQUAL(old.out.size(), legacy.out.size());
    for (size_t i = 0; i < old.out.size(); i++) TEST_ASSERT_TRUE(old.out[i] == legacy.out[i]);
  }
}

void test_hardened_clean_stream_equals_legacy() {
  Run a(false), b(true);
  for (int i = 0; i < 50; i++) {
    a.feed(rcFrame(i));
    b.feed(rcFrame(i));
  }
  TEST_ASSERT_EQUAL(50, a.out.size());
  TEST_ASSERT_TRUE(a.out == b.out);
  TEST_ASSERT_EQUAL_UINT32(50, b.p.stats().framesRc);
  TEST_ASSERT_EQUAL_UINT32(0, b.p.stats().crcFail);
}

void test_zero_byte_is_not_a_start_when_hardened() {
  Run a(false), b(true);
  // 0x00 0x18 would open a 26 byte frame in legacy mode and swallow the next frame.
  Frame s{0x00, 0x18};
  Frame f = rcFrame(1);
  s.insert(s.end(), f.begin(), f.end());
  a.feed(s);
  b.feed(s);
  TEST_ASSERT_EQUAL(0, a.out.size());
  TEST_ASSERT_EQUAL(1, b.out.size());
  TEST_ASSERT_EQUAL_UINT32(0, b.p.stats().zeroStarts);
  TEST_ASSERT_EQUAL_UINT32(1, a.p.stats().zeroStarts);
}

void test_rescan_after_false_start_recovers_following_frames() {
  Frame s{0xC8, 0x3E}; // false start, claims 64 bytes: legacy swallows the next two to three frames
  for (int i = 0; i < 6; i++) {
    Frame f = rcFrame(i);
    s.insert(s.end(), f.begin(), f.end());
  }
  Run a(false), b(true);
  a.feed(s);
  b.feed(s);
  TEST_ASSERT_TRUE(a.out.size() < 6);
  TEST_ASSERT_EQUAL(6, b.out.size());
  for (int i = 0; i < 6; i++) TEST_ASSERT_TRUE(b.out[i] == rcFrame(i));
  TEST_ASSERT_TRUE(b.p.stats().rescans > 0);
}

void test_bad_length_rescans_from_second_byte() {
  Frame s{0xEE}; // start, then the next frame's sync as the 'length' (0xC8 = 200: out of range)
  Frame f = makeFrame(0xC8, 0x16, std::vector<uint8_t>(22, 0x11));
  s.insert(s.end(), f.begin(), f.end());
  Run a(false), b(true);
  a.feed(s);
  b.feed(s);
  TEST_ASSERT_EQUAL(0, a.out.size());
  TEST_ASSERT_EQUAL(1, b.out.size());
  TEST_ASSERT_TRUE(b.out[0] == f);
  TEST_ASSERT_EQUAL_UINT32(1, b.p.stats().badLen);
}

void test_single_corrupt_byte_costs_one_frame() {
  Run b(true);
  for (int i = 0; i < 10; i++) {
    Frame f = rcFrame(i);
    if (i == 4) f[5] ^= 0xFF; // payload corruption: CRC fails
    b.feed(f);
  }
  TEST_ASSERT_EQUAL(9, b.out.size());
  TEST_ASSERT_EQUAL_UINT32(1, b.p.stats().crcFail);
}

void test_frame_classes_and_stats() {
  Run b(true);
  b.feed(rcFrame(1));
  b.feed(makeFrame(0xEE, 0x28, {0x00, 0xEA, 0xEE}));
  b.feed(makeFrame(0xEE, 0x2D, {0xEE, 0xEA, 1, 2}));
  b.feed(makeFrame(0xEE, 0x07, {1, 2, 3}));
  b.feed(Frame{0x11, 0x22, 0x33}); // noise while hunting
  const auto &st = b.p.stats();
  TEST_ASSERT_EQUAL_UINT32(1, st.framesRc);
  TEST_ASSERT_EQUAL_UINT32(2, st.framesLua);
  TEST_ASSERT_EQUAL_UINT32(1, st.framesOther);
  TEST_ASSERT_EQUAL_UINT32(3, st.droppedHunting);
  TEST_ASSERT_EQUAL_UINT32(b.out.size() ? 26 + 7 + 8 + 7 + 3 : 0, st.bytesIn);
}

void test_stale_partial_reset_and_recent_bytes() {
  Run b(true);
  Frame f = rcFrame(3);
  Frame head(f.begin(), f.begin() + 10);
  b.feed(head);
  TEST_ASSERT_TRUE(b.p.hasPartial());
  b.p.resetPartial();
  TEST_ASSERT_FALSE(b.p.hasPartial());
  TEST_ASSERT_EQUAL_UINT32(1, b.p.stats().staleResets);
  b.feed(rcFrame(4)); // not swallowed by the dead partial
  TEST_ASSERT_EQUAL(1, b.out.size());
  uint8_t recent[CrsfParser::kRecentBytes];
  b.p.copyRecent(recent);
  Frame last = rcFrame(4);
  for (int i = 0; i < 16; i++) TEST_ASSERT_EQUAL_UINT8(last[last.size() - 16 + i], recent[i]);
}

void test_replay_does_not_overflow_with_worst_case_garbage() {
  Run b(true);
  Frame s;
  for (int i = 0; i < 4000; i++) s.push_back(i % 2 ? 0x3E : 0xC8); // endless false starts claiming 64 bytes
  b.feed(s);
  // The first good frame may still be inside a pending false frame; a few more must get through.
  for (int i = 0; i < 4; i++) b.feed(rcFrame(9));
  TEST_ASSERT_TRUE(b.out.size() >= 1);
  TEST_ASSERT_TRUE(b.out.back() == rcFrame(9));
}

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_legacy_identical_to_original_on_random_streams);
  RUN_TEST(test_hardened_clean_stream_equals_legacy);
  RUN_TEST(test_zero_byte_is_not_a_start_when_hardened);
  RUN_TEST(test_rescan_after_false_start_recovers_following_frames);
  RUN_TEST(test_bad_length_rescans_from_second_byte);
  RUN_TEST(test_single_corrupt_byte_costs_one_frame);
  RUN_TEST(test_frame_classes_and_stats);
  RUN_TEST(test_stale_partial_reset_and_recent_bytes);
  RUN_TEST(test_replay_does_not_overflow_with_worst_case_garbage);
  return UNITY_END();
}
