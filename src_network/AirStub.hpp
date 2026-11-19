#pragma once
// Air side of the TX-module router: a stub with the real seam in place (docs/network-mode.md "Air stub").
//
// The air link will be network::AirFrameLink driven by an FHSS adapter that implements network::AirAccessI. This
// stub already uses the real AirFrameLink (aggregator, expiry, latest-wins), so queued frames behave exactly as
// they will on air: an RC_STATE is gone after 100 ms and is never released later. Only the radio access is
// missing. StubAirAccess plays a "slot every 20 ms that carries nothing": it drains the queue into a scratch buffer
// and discards the bytes.
//
// To plug in the real air link, replace StubAirAccess by the FHSS access (setSource(), then
// buildAirPacket()/onAirPacket() from the slot hooks) and replace StubAirStats by the access' measurements.
#include <stdint.h>

#include "ExVectrNetwork/link/AirAccessI.hpp"
#include "ExVectrNetwork/link/AirFrameLink.hpp"

#include "NetCrsf.hpp"

namespace VCTR::ExVectrLink::netmode {

/// Where the router reads the module's own radio measurements (LINK_STATS originated by node 2, CRSF downlink
/// fields). The air task provides the real one.
class AirStatsSourceI {
public:
  virtual ~AirStatsSourceI() = default;
  virtual LocalAirStats localAirStats() = 0;
};

/// "Radio off": zeros, air state Off.
class StubAirStats : public AirStatsSourceI {
public:
  LocalAirStats localAirStats() override { return LocalAirStats(); }
};

/// Stand-in for the radio access. Discards what the aggregator would put in the next air packet.
class StubAirAccess : public network::AirAccessI {
public:
  static constexpr int64_t kSlotPeriodNs = 20000000LL; // 20 ms, the planned network-mode FHSS interval
  static constexpr size_t kAirMax = 255;

  void setSource(network::AirPacketSourceI &src) override { src_ = &src; }
  uint16_t expectedRxPacketsPerSecond() const override { return 0; }

  /// Call often (every loop). Once per slot period it builds and discards one air packet.
  void poll(int64_t nowNs) {
    if (!src_ || nowNs - lastSlotNs_ < kSlotPeriodNs) return;
    lastSlotNs_ = nowNs;
    uint8_t buf[kAirMax];
    if (src_->buildAirPacket(buf, sizeof buf, nowNs)) discardedPackets_++;
  }
  uint32_t discardedPackets() const { return discardedPackets_; }

private:
  network::AirPacketSourceI *src_ = nullptr;
  int64_t lastSlotNs_ = 0;
  uint32_t discardedPackets_ = 0;
};

/// The air face as the router sees it: a FrameLinkI plus the access seam.
struct AirSide {
  using Link = network::AirFrameLink<16, 8>;
  Link link;
  StubAirAccess access;
  StubAirStats stats;

  AirSide() { access.setSource(link); }
};

} // namespace VCTR::ExVectrLink::netmode
