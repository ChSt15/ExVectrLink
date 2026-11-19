#pragma once
// Automatic WiFi-upload triggers of the network-mode firmwares. Pure, host-tested (test/test_air_link).
#include <stdint.h>

namespace VCTR::ExVectrLink::netmode {

/// The receiver's (and TX module's) automatic WiFi-upload triggers, as kept by the workspace design (Revision 1):
///  - sync failsafe: the air link never left Searching within 60 s of boot (reaching Syncing once cancels it for
///    good, exactly as the FPV SyncFailsafeMonitor);
///  - stale UART (receiver only): no valid stream frame from the UART host within 60 s of boot (seeing one once
///    cancels it for good, so a host that goes quiet later, for example in flight, never stops the link);
///  - a command: requestCommand() (called by the main loop once LinkUploadCommand reports the REQUEST action 10 due,
///    docs/network-mode.md "Link upload command").
/// Each fires at most once. Entering upload mode stops the link until reset.
class UploadTriggers {
public:
  enum class Reason : uint8_t { None, SyncFailsafe, UartStale, Command };
  static constexpr int64_t kTimeoutNs = 60000000000LL; // 60 s

  UploadTriggers(int64_t bootNs, bool useUartTrigger) : boot_(bootNs), useUart_(useUartTrigger) {}

  void requestCommand() { command_ = true; }

  /// Call periodically. `airLeftSearching`: the FHSS state is not Searching now. Returns the reason once.
  Reason update(int64_t nowNs, bool airLeftSearching, bool uartHostSeen) {
    if (fired_ != Reason::None) return Reason::None;
    if (airLeftSearching) synced_ = true;
    if (uartHostSeen) hostSeen_ = true;
    Reason r = Reason::None;
    if (command_) r = Reason::Command;
    else if (!synced_ && nowNs - boot_ >= kTimeoutNs) r = Reason::SyncFailsafe;
    else if (useUart_ && !hostSeen_ && nowNs - boot_ >= kTimeoutNs) r = Reason::UartStale;
    fired_ = r;
    return r;
  }
  Reason fired() const { return fired_; }

private:
  int64_t boot_;
  bool useUart_;
  bool synced_ = false, hostSeen_ = false, command_ = false;
  Reason fired_ = Reason::None;
};

} // namespace VCTR::ExVectrLink::netmode
