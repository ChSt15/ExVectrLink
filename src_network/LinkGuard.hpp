#pragma once
// App-level wrapper around a FrameLinkI, used by the TX-module router for two jobs the library's egress filters
// do not cover (docs/network-mode.md "Filters"):
//   1. Drop frames by DESTINATION address on egress. Node::send floods a unicast frame to every link when it has
//      no fresh route to the destination, so without this the dst-0 copy of RC_STATE (for the ground station)
//      would also be queued on the air link whenever no ground station is attached, and the dst-1 copy
//      would go to USB. Neither is wanted.
//   2. Refuse frames while the link cannot take them without queueing stale data (USB host absent or backed up).
//      A refused frame is dropped, never held: an expired RC_STATE must not be delivered later.
// Inbound frames pass through unchanged (the guard is the inner link's sink and re-delivers to its own).
#include <stdint.h>

#include "ExVectrNetwork/link/FrameLinkI.hpp"

namespace VCTR::ExVectrLink::netmode {

/// Says whether a link can accept a frame right now without holding it back.
class LinkGateI {
public:
  virtual ~LinkGateI() = default;
  virtual bool admits() = 0;
};

class LinkGuard : public network::FrameLinkI, public network::FrameSinkI {
public:
  static constexpr size_t kMaxDropDst = 4;

  struct GuardStats {
    uint32_t droppedByDst = 0;
    uint32_t droppedByGate = 0;
  } guardStats;

  explicit LinkGuard(network::FrameLinkI &inner, LinkGateI *gate = nullptr)
      : inner_(inner), gate_(gate) {
    inner_.setSink(*this, 0);
  }

  /// Drops every frame with this destination on egress. False if the table is full.
  bool addDropDst(uint8_t dst) {
    if (dropCount_ >= kMaxDropDst) return false;
    dropDst_[dropCount_++] = dst;
    return true;
  }

  bool send(const uint8_t *frame, uint8_t len, int64_t enqueueTimeNs) override {
    if (len >= 4) {
      const uint8_t dst = frame[2]; // network header: CTRL, TTL, DST, SRC
      for (uint8_t i = 0; i < dropCount_; i++)
        if (dropDst_[i] == dst) {
          guardStats.droppedByDst++;
          return false;
        }
    }
    if (gate_ && !gate_->admits()) {
      guardStats.droppedByGate++;
      stats.txDropped++;
      return false;
    }
    const bool ok = inner_.send(frame, len, enqueueTimeNs);
    if (ok) stats.txOk++; else stats.txDropped++;
    return ok;
  }

  uint8_t mtu() const override { return inner_.mtu(); }

  void onFrame(const network::FrameView &f, uint8_t) override { deliver(f); }

private:
  network::FrameLinkI &inner_;
  LinkGateI *gate_;
  uint8_t dropDst_[kMaxDropDst] = {};
  uint8_t dropCount_ = 0;
};

} // namespace VCTR::ExVectrLink::netmode
