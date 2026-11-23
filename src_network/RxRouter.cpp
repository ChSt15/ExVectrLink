#include "RxRouter.hpp"

#include <string.h>

namespace VCTR::ExVectrLink::netmode {

using network::EgressFilter;
using network::wire::Qos;

network::NodeConfig RxRouter::makeConfig() {
  network::NodeConfig cfg;
  cfg.address = kNodeId;
  cfg.role = network::wire::Role::LinkRx;
  cfg.forwarding = true;
  cfg.schemaHash = Packets::kSchemaHash;
  return cfg;
}

RxRouter::RxRouter(network::TimeSourceI &time, network::FrameLinkI &uart, network::FrameLinkI &air,
                   AirStatsSourceI &airStats, int boardMaxDbm)
    : time_(time), uart_(uart), airStats_(airStats), node_(makeConfig(), time),
      follower_(boardMaxDbm), linkConfig_(follower_), upload_(node_, mux_), startNs_(time.nowNs()), nextLinkStatsNs_(startNs_) {
  node_.setDataSink(mux_);
  mux_.subscribe(Packets::LinkConfig::kPort, linkConfig_);

  // Where frames must not go (design 3.3.3, docs/network-mode.md "Filters", docs/gs-control-design.md section 7):
  //  - UART (to the vehicle): the ground station's heartbeat is dropped; its REQUESTs and other frames pass.
  //    RC_STATE (0x10) from any source is dropped (the vehicle takes no handset channels over the radio). CMD_STATE
  //    (0x13) passes only from node 0 arriving over the air: it is dropped for sources 1, 2 and 3. Nothing
  //    addressed to the ground station (the vehicle is an end node and never forwards), no USB-only ports.
  //  - Air: nothing addressed to the vehicle (Node::send floods unicast frames without a route), no USB-only ports,
  //    and no CMD_STATE that came in from the vehicle side (uplink only, anti-spoof).
  EgressFilter uartFilter;
  uartFilter.portLimit = 0x80;
  uartFilter.addHeartbeatDrop(kGroundStation);
  uartFilter.addSrcPortDrop(kGroundStation, Packets::RcState::kPort);
  uartFilter.addRule(network::kAnyLink, Packets::RcState::kPort);
  uartFilter.addSrcPortDrop(kVehicle, kCmdStatePort);
  uartFilter.addSrcPortDrop(2, kCmdStatePort);
  uartFilter.addSrcPortDrop(kNodeId, kCmdStatePort);
  uartFilter.addDstDrop(kGroundStation);
  uartIdx_ = node_.addLink(uart, uartFilter);

  EgressFilter airFilter;
  airFilter.portLimit = 0x80;
  airFilter.addDstDrop(kVehicle);
  if (uartIdx_ >= 0) airFilter.addRule(static_cast<uint8_t>(uartIdx_), kCmdStatePort);
  airIdx_ = node_.addLink(air, airFilter);
  linkConfig_.setAirLink(airIdx_);

  // Fixed topology: the vehicle is behind the UART, the TX module and the ground station behind the air link.
  if (uartIdx_ >= 0) node_.pinRoute(kVehicle, static_cast<uint8_t>(uartIdx_));
  if (airIdx_ >= 0) {
    node_.pinRoute(kGroundStation, static_cast<uint8_t>(airIdx_));
    node_.pinRoute(2, static_cast<uint8_t>(airIdx_));
  }
}

void RxRouter::poll() {
  node_.poll();
  const int64_t now = time_.nowNs();
  upload_.poll(now);
  if (now >= nextLinkStatsNs_) {
    sendOwnLinkStats(now);
    nextLinkStatsNs_ += kLinkStatsPeriodNs;
    if (nextLinkStatsNs_ <= now) nextLinkStatsNs_ = now + kLinkStatsPeriodNs;
  }
}

void RxRouter::sendOwnLinkStats(int64_t now) {
  const Packets::LinkStats ls =
      makeLinkStats(airStats_.localAirStats(), static_cast<double>(now - startNs_) * 1e-9);
  uint8_t buf[Packets::LinkStats::kMaxSize];
  const size_t n = ls.encode(buf, sizeof buf);
  if (n) node_.send(network::kBroadcastAddr, Packets::LinkStats::kPort, buf, n, Qos::Link, true);
}

bool RxRouter::log(Packets::Severity severity, const char *text) {
  Packets::LogText lt;
  lt.severity = severity;
  size_t n = text ? strlen(text) : 0;
  if (n > sizeof lt.text) n = sizeof lt.text;
  memcpy(lt.text, text, n);
  lt.text_len = static_cast<uint16_t>(n);
  uint8_t buf[Packets::LogText::kMaxSize];
  const size_t len = lt.encode(buf, sizeof buf);
  if (!len) return false;
  return node_.send(kGroundStation, Packets::LogText::kPort, buf, len, Qos::Event, false);
}

} // namespace VCTR::ExVectrLink::netmode
