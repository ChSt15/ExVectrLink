#pragma once
// The link commands of the network-mode firmwares (both modules): REQUEST action LINK_UPLOAD_MODE (10, enter WiFi
// upload mode) and LINK_REBOOT (11, clean software restart). Pure, host-tested (test/test_air_link, test_network_mode).
//
// REQUEST (port 0x11) with one of these actions and `target` = this module's node id is honoured from ANY link (USB,
// UART, air) and ALWAYS: no armed or on-ground condition, no other gating (user decisions 2026-11-19 and 2026-11-20,
// exception to safety.md 2.5; the module cannot know the vehicle state, the senders guard against accidents; a reboot
// of the receiver in flight stops RC for a few seconds). Each action has its own state, with identical semantics:
//  - Idempotent: a repeated request (same req_id, or any later request while that action is pending) does not move
//    the deadline; every copy is answered with REQUEST_ACK (status Done, detail 0) to the requester (dst = request src).
//  - After kDelayNs (long enough for the ACK to leave, on the receiver across the air) the action is `due`
//    (level: stays true); the application then executes it: upload mode through UploadTriggers::requestCommand(),
//    or ESP.restart().
//  - Never triggers on anything else: other actions, other targets (the router forwards them as usual), a request
//    shorter than its fixed size, or req_id 0 (the schema says non-zero) are ignored and not acknowledged.
// The ACK is queued in the receive callback and sent from poll(), so no routing work happens in the callback.
#include <stdint.h>

#include "ExVectrNetwork.hpp"
#include "ExVectrPackets/gen/all.hpp"

namespace VCTR::ExVectrLink::netmode {

class LinkCommand : public network::PortHandlerI {
public:
  /// Delay between the first accepted request and execution. One place for the constant.
  static constexpr int64_t kDelayNs = 400000000LL; // 400 ms
  static constexpr int64_t kUploadDelayNs = kDelayNs; // historical name
  static constexpr uint8_t kUploadAction = static_cast<uint8_t>(Packets::RequestAction::kLinkUploadMode);
  static constexpr uint8_t kRebootAction = static_cast<uint8_t>(Packets::RequestAction::kLinkReboot);
  static constexpr uint8_t kAction = kUploadAction; // historical name

  struct CommandStats {
    uint32_t accepted = 0;  ///< valid requests for this module (including repeats, both actions)
    uint32_t ignored = 0;   ///< other action, other target, malformed
    uint32_t acksSent = 0;
    uint32_t acksLost = 0;  ///< ACKs the node could not send before the delay ran out
  } stats;

  LinkCommand(network::Node &node, network::PortRegistryI &mux)
      : node_(node) {
    subscribed_ = mux.subscribe(Packets::Request::kPort, *this);
  }

  bool subscribed() const { return subscribed_; }
  /// True while an accepted command (any action, or the given one) has not been executed yet.
  bool pending() const { return slots_[0].armed || slots_[1].armed; }
  bool pending(uint8_t action) const { const Slot *s = slot(action); return s && s->armed; }
  /// True once the action has been pending for its delay (level), set by poll().
  bool due(uint8_t action) const { const Slot *s = slot(action); return s && s->due; }

  void onPacket(const network::RxInfo &info, const uint8_t *payload, uint8_t len) override {
    Packets::Request req;
    if (!req.decode(payload, len) || req.req_id == 0 || req.target != node_.address()) {
      stats.ignored++;
      return;
    }
    Slot *s = slot(req.action);
    if (!s) {
      stats.ignored++;
      return;
    }
    stats.accepted++;
    if (!s->armed) {
      s->armed = true;
      s->dueNs = info.rxTimeNs + kDelayNs;
    }
    // Queue one ACK per copy (a full queue drops the newest: the sender retries).
    if (ackCount_ < kMaxAcks) {
      acks_[ackCount_].dst = info.src;
      acks_[ackCount_].reqId = req.req_id;
      acks_[ackCount_].action = req.action;
      ackCount_++;
    }
  }

  /// Sends queued ACKs and updates the due flags. Call from the router's poll().
  void poll(int64_t nowNs) {
    uint8_t kept = 0;
    for (uint8_t i = 0; i < ackCount_; i++) {
      Packets::RequestAck ack;
      ack.req_id = acks_[i].reqId;
      ack.status = Packets::RequestStatus::kDone;
      ack.detail = 0;
      uint8_t buf[Packets::RequestAck::kMaxSize];
      const size_t n = ack.encode(buf, sizeof buf);
      const Slot *s = slot(acks_[i].action);
      if (n && node_.send(acks_[i].dst, Packets::RequestAck::kPort, buf, n, network::wire::Qos::Event, false)) {
        stats.acksSent++;
      } else if (s && s->armed && nowNs < s->dueNs) {
        acks_[kept++] = acks_[i]; // try again on the next poll while there is time
      } else {
        stats.acksLost++;
      }
    }
    ackCount_ = kept;
    for (Slot &s : slots_) s.due = s.armed && nowNs >= s.dueNs;
  }

private:
  static constexpr uint8_t kMaxAcks = 4;
  struct PendingAck {
    uint8_t dst = 0;
    uint16_t reqId = 0;
    uint8_t action = 0;
  };
  struct Slot {
    uint8_t action;
    bool armed = false;
    bool due = false;
    int64_t dueNs = 0;
  };
  Slot *slot(uint8_t action) {
    for (Slot &s : slots_) if (s.action == action) return &s;
    return nullptr;
  }
  const Slot *slot(uint8_t action) const {
    for (const Slot &s : slots_) if (s.action == action) return &s;
    return nullptr;
  }
  network::Node &node_;
  bool subscribed_ = false;
  Slot slots_[2] = {{kUploadAction}, {kRebootAction}};
  PendingAck acks_[kMaxAcks];
  uint8_t ackCount_ = 0;
};

/// Historical name (REQUEST action 10 was the only command at first).
using LinkUploadCommand = LinkCommand;

} // namespace VCTR::ExVectrLink::netmode
