#pragma once
// The link-upload command of the network-mode firmwares (both modules). Pure, host-tested (test/test_air_link).
//
// REQUEST (port 0x11) with action LINK_UPLOAD_MODE (10) and `target` = this module's node id is honoured from ANY
// link (USB, UART, air) and ALWAYS: no armed or on-ground condition, no other gating (user decision 2026-11-19,
// exception to safety.md 2.5; the module cannot know the vehicle state, the senders guard against accidents).
//  - Idempotent: a repeated request (same req_id, or any later request while the command is pending) does not move
//    the deadline; every copy is answered with REQUEST_ACK (status Done, detail 0) to the requester (dst = request src).
//  - After kUploadDelayNs (long enough for the ACK to leave, on the receiver across the air) poll() reports `due`;
//    the application then stops the link and enters WiFi upload mode through UploadTriggers::requestCommand().
//  - Never triggers on anything else: other actions, other targets (the router forwards them as usual), a request
//    shorter than its fixed size, or req_id 0 (the schema says non-zero) are ignored and not acknowledged.
// The ACK is queued in the receive callback and sent from poll(), so no routing work happens in the callback.
#include <stdint.h>

#include "ExVectrNetwork.hpp"
#include "ExVectrPackets/gen/all.hpp"

namespace VCTR::ExVectrLink::netmode {

class LinkUploadCommand : public network::PortHandlerI {
public:
  /// Delay between the first accepted request and entering upload mode. One place for the constant.
  static constexpr int64_t kUploadDelayNs = 400000000LL; // 400 ms
  static constexpr uint8_t kAction = static_cast<uint8_t>(Packets::RequestAction::kLinkUploadMode);

  struct CommandStats {
    uint32_t accepted = 0;  ///< valid requests for this module (including repeats)
    uint32_t ignored = 0;   ///< other action, other target, malformed
    uint32_t acksSent = 0;
    uint32_t acksLost = 0;  ///< ACKs the node could not send before the delay ran out
  } stats;

  LinkUploadCommand(network::Node &node, network::PortRegistryI &mux)
      : node_(node) {
    subscribed_ = mux.subscribe(Packets::Request::kPort, *this);
  }

  bool subscribed() const { return subscribed_; }
  bool pending() const { return armed_; }

  void onPacket(const network::RxInfo &info, const uint8_t *payload, uint8_t len) override {
    Packets::Request req;
    if (!req.decode(payload, len) || req.req_id == 0 || req.action != kAction || req.target != node_.address()) {
      stats.ignored++;
      return;
    }
    stats.accepted++;
    if (!armed_) {
      armed_ = true;
      dueNs_ = info.rxTimeNs + kUploadDelayNs;
    }
    // Queue one ACK per copy (a full queue drops the newest: the sender retries).
    if (ackCount_ < kMaxAcks) {
      acks_[ackCount_].dst = info.src;
      acks_[ackCount_].reqId = req.req_id;
      ackCount_++;
    }
  }

  /// Sends queued ACKs. Returns true once the delay has passed (level: stays true). Call from the router's poll().
  bool poll(int64_t nowNs) {
    uint8_t kept = 0;
    for (uint8_t i = 0; i < ackCount_; i++) {
      Packets::RequestAck ack;
      ack.req_id = acks_[i].reqId;
      ack.status = Packets::RequestStatus::kDone;
      ack.detail = 0;
      uint8_t buf[Packets::RequestAck::kMaxSize];
      const size_t n = ack.encode(buf, sizeof buf);
      if (n && node_.send(acks_[i].dst, Packets::RequestAck::kPort, buf, n, network::wire::Qos::Event, false)) {
        stats.acksSent++;
      } else if (armed_ && nowNs < dueNs_) {
        acks_[kept++] = acks_[i]; // try again on the next poll while there is time
      } else {
        stats.acksLost++;
      }
    }
    ackCount_ = kept;
    return armed_ && nowNs >= dueNs_;
  }

private:
  static constexpr uint8_t kMaxAcks = 4;
  struct PendingAck {
    uint8_t dst = 0;
    uint16_t reqId = 0;
  };
  network::Node &node_;
  bool subscribed_ = false;
  bool armed_ = false;
  int64_t dueNs_ = 0;
  PendingAck acks_[kMaxAcks];
  uint8_t ackCount_ = 0;
};

} // namespace VCTR::ExVectrLink::netmode
