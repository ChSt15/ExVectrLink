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

  // Where frames must not go (design 3.3.3, Revision 1, docs/network-mode.md "Filters"):
  //  - UART (to the vehicle): from node 0 only the heartbeat and RC_STATE are dropped (second layer behind the TX
  //    module: no RC from the ground station over radio ever reaches the vehicle); its REQUESTs and other frames
  //    pass. Nothing addressed to the ground station (the vehicle is an end node and never forwards), no USB-only
  //    ports.
  //  - Air: nothing addressed to the vehicle (Node::send floods unicast frames without a route), no USB-only ports.
  EgressFilter uartFilter;
  uartFilter.portLimit = 0x80;
  uartFilter.addHeartbeatDrop(kGroundStation);
  uartFilter.addSrcPortDrop(kGroundStation, Packets::RcState::kPort);
  uartFilter.addDstDrop(kGroundStation);
  uartIdx_ = node_.addLink(uart, uartFilter);

  EgressFilter airFilter;
  airFilter.portLimit = 0x80;
  airFilter.addDstDrop(kVehicle);
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
