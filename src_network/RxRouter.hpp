#pragma once
// Vehicle-side receiver for ExVectrNetwork mode: node 3, role link RX module, forwarding = true
// (docs/network-mode.md "RX module"). Host-testable: no Arduino, no ExVectrCore, no heap after construction.
//
// Links: UART (StreamFrameLink, 460800 8N1, towards the vehicle, node 1) and the air side (AirFrameLink on the FHSS
// slots, towards the TX module). Own frames: HEARTBEAT 1 Hz (by the Node), LINK_STATS 5 Hz broadcast (to the vehicle
// and over the air to the TX module and the ground station), with LQ 0 / Searching while the air link is down.
// The module never creates RC state: if nothing arrives over the air, nothing goes to the vehicle.
#include <stdint.h>

#include "ExVectrNetwork.hpp"
#include "ExVectrNetwork/TimeSourceI.hpp"

#include "LinkUploadCommand.hpp"
#include "NetAirAccess.hpp"
#include "NetPower.hpp"
#include "UploadTriggers.hpp"

namespace VCTR::ExVectrLink::netmode {

/// LINK_CONFIG (port 0x04) handler of the ship radio node: accepted only from node 2 and only when it came in over
/// the air link (never from the UART, whatever source it claims). Copies into the follower, no other work.
class LinkConfigHandler : public network::PortHandlerI {
public:
  LinkConfigHandler(LinkConfigFollower &follower) : follower_(follower) {}
  void setAirLink(int idx) { airIdx_ = idx; }
  void onPacket(const network::RxInfo &info, const uint8_t *payload, uint8_t len) override {
    Packets::LinkConfig lc;
    if (info.src != 2 || airIdx_ < 0 || info.ingressLink != static_cast<uint8_t>(airIdx_)) {
      rejected++;
      return;
    }
    if (!lc.decode(payload, len)) {
      decodeErrors++;
      return;
    }
    follower_.onLinkConfig(lc.tx_power_dbm, info.rxTimeNs);
    accepted++;
  }
  uint32_t accepted = 0, rejected = 0, decodeErrors = 0;

private:
  LinkConfigFollower &follower_;
  int airIdx_ = -1;
};

class RxRouter {
public:
  static constexpr uint8_t kNodeId = 3; // RX module
  static constexpr uint8_t kGroundStation = 0;
  static constexpr uint8_t kVehicle = 1;
  static constexpr uint8_t kCmdStatePort = Packets::CmdState::kPort; // 0x13, node 0 -> vehicle, uplink only
  static constexpr int64_t kLinkStatsPeriodNs = 200000000LL; // 5 Hz

  /// `boardMaxDbm`: the board's transmit power maximum, the ceiling of the power LINK_CONFIG can request.
  RxRouter(network::TimeSourceI &time, network::FrameLinkI &uart, network::FrameLinkI &air,
           AirStatsSourceI &airStats, int boardMaxDbm = 12);

  /// Power to apply now (dBm): the Router's LINK_CONFIG value clamped to 0..board maximum, or the 10 dBm default
  /// while no LINK_CONFIG arrived for 10 s (or never). Level, evaluate every loop pass.
  int8_t desiredPowerDbm() const { return follower_.power(time_.nowNs()); }
  bool linkConfigFresh() const { return follower_.fresh(time_.nowNs()); }
  const LinkConfigHandler &linkConfigHandler() const { return linkConfig_; }

  /// Periodic work (heartbeat, own LINK_STATS). Call at least at 10 Hz, together with the links' poll().
  void poll();

  /// True once any valid stream frame has arrived from the UART host (used by the stale-UART upload trigger).
  bool uartHostSeen() const { return uart_.stats.rxOk > 0; }

  /// True once an accepted link-upload command (REQUEST action 10, target 3) has been pending for its delay (the
  /// ACK has been sent across the air by then): the application then enters upload mode. Level, set by poll().
  bool uploadCommandDue() const { return upload_.due(LinkCommand::kUploadAction); }
  /// True once an accepted LINK_REBOOT request (REQUEST action 11, same target and delay) is due: the application
  /// then restarts the module (ESP.restart()). Level.
  bool rebootCommandDue() const { return upload_.due(LinkCommand::kRebootAction); }
  const LinkCommand &uploadCommand() const { return upload_; }

  /// LOG_TEXT to the ground station (over the air). Text is truncated to 233 bytes.
  bool log(Packets::Severity severity, const char *text);

  network::Node &node() { return node_; }
  const network::Node &node() const { return node_; }
  int uartLinkIndex() const { return uartIdx_; }
  int airLinkIndex() const { return airIdx_; }

private:
  static network::NodeConfig makeConfig();
  void sendOwnLinkStats(int64_t now);

  network::TimeSourceI &time_;
  network::FrameLinkI &uart_;
  AirStatsSourceI &airStats_;
  network::Node node_;
  network::PortMux<2> mux_; // REQUEST, LINK_CONFIG
  LinkConfigFollower follower_;
  LinkConfigHandler linkConfig_;
  LinkCommand upload_; // both link commands (upload mode, reboot)
  int uartIdx_ = -1, airIdx_ = -1;
  int64_t startNs_;
  int64_t nextLinkStatsNs_;
};

} // namespace VCTR::ExVectrLink::netmode
