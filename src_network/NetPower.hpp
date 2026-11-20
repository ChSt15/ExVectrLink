#pragma once
// Transmit power policy of the network-mode firmwares (docs/network-mode.md "Power policy"). Pure, host-tested.
//
// One manual power setting for the whole network (dynamic power is not used in network mode):
//  - The legal limit is 10 mW = 10 dBm. kNetTxPowerDefaultDbm is the power at EVERY boot, on both ends.
//  - The Router (node 2, the configuration master) may raise it temporarily from the Lua menu up to its board
//    maximum (runtime ceiling). The raise lives in RAM only: a reboot returns to 10 dBm.
//  - The Router sends its setting continuously as LINK_CONFIG to the ship radio node (node 3). The ship node
//    follows it, clamped to 0..its board maximum, and falls back to 10 dBm if LINK_CONFIG has not arrived for
//    10 s, so an elevated power never outlives the Router commanding it.
#include <stdint.h>

namespace VCTR::ExVectrLink::netmode {

/// Power at every boot and the fall-back power (10 mW legal limit, user decision 2026-11-20).
constexpr int8_t kNetTxPowerDefaultDbm = 10;

/// Clamps a requested power to [0, boardMaxDbm] (a board maximum below 0 counts as 0).
inline int8_t clampNetPower(int dbm, int boardMaxDbm) {
  if (boardMaxDbm < 0) boardMaxDbm = 0;
  if (boardMaxDbm > 127) boardMaxDbm = 127;
  if (dbm > boardMaxDbm) dbm = boardMaxDbm;
  if (dbm < 0) dbm = 0;
  return static_cast<int8_t>(dbm);
}

/// The Router's own power setting: starts at the default, settable up to the board maximum, never stored.
class NetTxPowerSetting {
public:
  explicit NetTxPowerSetting(int boardMaxDbm) : boardMax_(boardMaxDbm) { power_ = clampNetPower(kNetTxPowerDefaultDbm, boardMax_); }
  int8_t get() const { return power_; }
  int boardMax() const { return boardMax_; }
  /// Sets the power (clamped to 0..board maximum). Returns the value that is now in effect.
  int8_t set(int dbm) { power_ = clampNetPower(dbm, boardMax_); return power_; }
  void reset() { power_ = clampNetPower(kNetTxPowerDefaultDbm, boardMax_); }

private:
  int boardMax_;
  int8_t power_;
};

/// Ship radio node side: follows LINK_CONFIG, with the 10 s fail-safe. Time is injected (nanoseconds).
class LinkConfigFollower {
public:
  static constexpr int64_t kTimeoutNs = 10000000000LL; // 10 s

  explicit LinkConfigFollower(int boardMaxDbm) : boardMax_(boardMaxDbm) {}

  /// A valid LINK_CONFIG arrived at `nowNs` (any new copy refreshes the timeout).
  void onLinkConfig(int8_t requestedDbm, int64_t nowNs) {
    requested_ = requestedDbm;
    lastRxNs_ = nowNs;
    have_ = true;
  }
  /// True while the last LINK_CONFIG is younger than 10 s.
  bool fresh(int64_t nowNs) const { return have_ && nowNs - lastRxNs_ < kTimeoutNs; }
  /// The power to apply now: the requested value (clamped to 0..board maximum) while fresh, otherwise the default
  /// (also clamped to the board maximum). Level, evaluate every loop pass.
  int8_t power(int64_t nowNs) const {
    return clampNetPower(fresh(nowNs) ? requested_ : kNetTxPowerDefaultDbm, boardMax_);
  }

private:
  int boardMax_;
  int8_t requested_ = kNetTxPowerDefaultDbm;
  int64_t lastRxNs_ = 0;
  bool have_ = false;
};

} // namespace VCTR::ExVectrLink::netmode
