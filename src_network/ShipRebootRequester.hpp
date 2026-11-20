#pragma once
// Router side of "Reboot ship node" (Lua command): sends REQUEST LINK_REBOOT (action 11) with target = the ship radio
// node (3) across the air, the way the ground station would, and repeats it until the REQUEST_ACK arrives. Retry
// rule as the ground station's (design 3.5.5): every 1 s with the same req_id, at most 5 tries. Pure, host-tested,
// time injected. The ack is recorded in the port handler (no work there), the sending happens in poll().
#include <stdint.h>

#include "ExVectrNetwork.hpp"
#include "ExVectrPackets/gen/all.hpp"

namespace VCTR::ExVectrLink::netmode {

class ShipRebootRequester : public network::PortHandlerI {
public:
  static constexpr uint8_t kShipNode = 3;
  static constexpr int64_t kRetryNs = 1000000000LL;
  static constexpr uint8_t kMaxTries = 5;

  enum class State : uint8_t { Idle, Sending, Acked, TimedOut };

  ShipRebootRequester(network::Node &node, network::PortRegistryI &mux) : node_(node) {
    subscribed_ = mux.subscribe(Packets::RequestAck::kPort, *this);
  }

  bool subscribed() const { return subscribed_; }
  State state() const { return state_; }
  uint8_t tries() const { return tries_; }

  /// Starts (or, while one is running, keeps) a reboot request. `nowNs` seeds the request id.
  /// Returns false if one is already being sent.
  bool request(int64_t nowNs) {
    if (state_ == State::Sending) return false;
    // Non-zero, new per request (the receiver deduplicates by id).
    reqId_ = static_cast<uint16_t>(reqId_ + 1u + static_cast<uint16_t>(nowNs >> 20));
    if (reqId_ == 0) reqId_ = 1;
    tries_ = 0;
    nextNs_ = nowNs;
    state_ = State::Sending;
    return true;
  }

  /// Ack of this request from the ship node. Records only.
  void onPacket(const network::RxInfo &info, const uint8_t *payload, uint8_t len) override {
    Packets::RequestAck ack;
    if (info.src != kShipNode || !ack.decode(payload, len)) return;
    if (state_ == State::Sending && ack.req_id == reqId_) state_ = State::Acked;
  }

  /// Sends the request when due. Call from the router's poll().
  void poll(int64_t nowNs) {
    if (state_ != State::Sending || nowNs < nextNs_) return;
    if (tries_ >= kMaxTries) {
      state_ = State::TimedOut;
      return;
    }
    Packets::Request req;
    req.req_id = reqId_;
    req.action = static_cast<uint8_t>(Packets::RequestAction::kLinkReboot);
    req.target = kShipNode;
    req.arg_i = 0;
    req.arg_f = 0.0f;
    uint8_t buf[Packets::Request::kMaxSize];
    const size_t n = req.encode(buf, sizeof buf);
    if (n && node_.send(kShipNode, Packets::Request::kPort, buf, n, network::wire::Qos::Event, false)) {
      tries_++;
      nextNs_ = nowNs + kRetryNs;
    } else {
      nextNs_ = nowNs + 50000000LL; // queue full: try again shortly (not counted)
    }
  }

  /// Last request id (tests).
  uint16_t reqId() const { return reqId_; }

private:
  network::Node &node_;
  bool subscribed_ = false;
  State state_ = State::Idle;
  uint16_t reqId_ = 0;
  uint8_t tries_ = 0;
  int64_t nextNs_ = 0;
};

} // namespace VCTR::ExVectrLink::netmode
