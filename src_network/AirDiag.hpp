#pragma once
// AIRDIAG: one compact LOG_TEXT line per node at 1 Hz with the FHSS slot-path counters (docs/air-link-optimisation.md,
// "AIRDIAG"). Pure (no Arduino): the counters are plain integers in lib/ExVectrLink SlotDiag, this only formats them.
// The caller reads them from the main loop (never from the slot path), passes the line to router.log() and then
// calls FHSS::resetSlotDiagWindow().
#include <stdint.h>
#include <stdio.h>

#include "ExVectrLink/datalink/SlotDiag.hpp"

namespace VCTR::ExVectrLink::netmode {

/// Everything one AIRDIAG line shows. Filled by netAirDiagInput() (NetRadio.cpp) on the target, by hand in tests.
struct AirDiagInput {
  datalink::SlotDiag d;
  uint8_t state = 0;          ///< 0 Searching, 1 Syncing, 2 Synced
  uint32_t txDone = 0;        ///< TX-done count of the radio driver(s) (sum over both diversity radios)
  uint32_t missedSlots = 0;   ///< FHSS missedSlotCounter (slots fast-forwarded)
  uint32_t graceRescue = 0;
  uint32_t graceExpire = 0;
  uint32_t counterResync = 0;
};

class AirDiagReporter {
public:
  static constexpr int64_t kPeriodNs = 1000000000LL;
  /// Longest line the reporter hands out (callers use a buffer of kMaxLine + 1). Below the 203-byte limit of one
  /// LOG_TEXT in a 220 B air frame (see format()); a longer line would be truncated here instead of silently never
  /// leaving the ship.
  static constexpr size_t kMaxLine = 200;

  /// Writes the line into `buf` when a period has passed since the last one and returns its length, else 0. Counters
  /// are cumulative; prep_avg, air and the max values cover the time since the previous line.
  size_t poll(int64_t nowNs, const AirDiagInput &in, char *buf, size_t cap) {
    if (nowNs < nextNs_) return 0;
    nextNs_ = nowNs + kPeriodNs;
    return format(buf, cap, in);
  }

  size_t format(char *buf, size_t cap, const AirDiagInput &in) {
    const datalink::SlotDiag &d = in.d;
    const uint32_t started = d.txStarted - prevStarted_;
    const int64_t avgUs = started ? (d.prepSumNs - prevPrepSumNs_) / static_cast<int64_t>(started) / 1000 : 0;
    prevStarted_ = d.txStarted;
    prevPrepSumNs_ = d.prepSumNs;
    const uint32_t airN = d.airCount - prevAirCount_;
    const int64_t airAvgUs = airN ? (d.airSumNs - prevAirSumNs_) / static_cast<int64_t>(airN) / 1000 : 0;
    prevAirCount_ = d.airCount;
    prevAirSumNs_ = d.airSumNs;
    if (cap > kMaxLine + 1) cap = kMaxLine + 1;
    // Short keys: the line must fit ONE air frame. At 220 B on air the largest net frame the aggregator can carry is
    // 210 B (218 - 3 AHDR/SFC/NETID - 4 CRC - 1 LEN), i.e. at most 203 text bytes (210 - 4 net hdr - 2 data hdr - 1
    // severity); a longer line is never sent. Most telling fields first.
    int n = snprintf(buf, cap,
                     "AIRDIAG s=%u tx=%lu st=%lu dn=%lu ns=%lu nu=%lu tl=%lu rx=%lu ok=%lu no=%lu "
                     "wk=%lu w1=%lu ms=%lu/%lu gr=%lu ge=%lu "
                     "pr=%ld/%ld ar=%ld/%ld/%ld al=%lu wl=%ld sl=%ld",
                     static_cast<unsigned>(in.state), (unsigned long)d.txSlots, (unsigned long)d.txStarted,
                     (unsigned long)in.txDone, (unsigned long)d.txSkipNotSynced, (unsigned long)d.txSkipSetup,
                     (unsigned long)d.txLateStart, (unsigned long)d.rxSlots, (unsigned long)d.rxValid,
                     (unsigned long)d.rxNone, (unsigned long)d.wakeups, (unsigned long)d.wakeLate1ms,
                     (unsigned long)in.missedSlots, (unsigned long)d.missEvents, (unsigned long)in.graceRescue,
                     (unsigned long)in.graceExpire, static_cast<long>(avgUs), static_cast<long>(d.prepMaxNs / 1000),
                     static_cast<long>(d.airMinNs / 1000), static_cast<long>(airAvgUs),
                     static_cast<long>(d.airMaxNs / 1000), (unsigned long)d.airLate,
                     static_cast<long>(d.wakeLateMaxNs / 1000), static_cast<long>(d.startLateMaxNs / 1000));
    if (n < 0) return 0;
    return static_cast<size_t>(n) >= cap ? cap - 1 : static_cast<size_t>(n);
  }

private:
  int64_t nextNs_ = 0;
  uint32_t prevStarted_ = 0;
  int64_t prevPrepSumNs_ = 0;
  uint32_t prevAirCount_ = 0;
  int64_t prevAirSumNs_ = 0;
};

} // namespace VCTR::ExVectrLink::netmode
