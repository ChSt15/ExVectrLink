#pragma once
// App-level wrapper around a FrameLinkI that refuses frames while the link cannot take them without queueing stale
// data (the TX module's USB host absent or backed up). A refused frame is dropped, never held: an expired RC_STATE
// must not be delivered later. Inbound frames pass through unchanged (the guard is the inner link's sink and
// re-delivers to its own).
//
// Destination drops used to live here too (workaround for Node::send flooding unicast frames without a route).
// They are now ExVectrNetwork egress rules (EgressFilter::addDstDrop / addSrcDrop), see docs/network-mode.md.
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
  struct GuardStats {
    uint32_t droppedByGate = 0;
  } guardStats;

  explicit LinkGuard(network::FrameLinkI &inner, LinkGateI *gate = nullptr)
      : inner_(inner), gate_(gate) {
    inner_.setSink(*this, 0);
  }

  bool send(const uint8_t *frame, uint8_t len, int64_t enqueueTimeNs) override {
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
};

} // namespace VCTR::ExVectrLink::netmode
