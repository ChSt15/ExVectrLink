#include "NetworkRouter.hpp"

#include <string.h>

namespace VCTR::ExVectrLink::netmode {

using network::EgressFilter;
using network::wire::Qos;

network::NodeConfig NetworkRouter::makeConfig() {
  network::NodeConfig cfg;
  cfg.address = kNodeId;
  cfg.role = network::wire::Role::LinkTx;
  cfg.forwarding = true;
  cfg.schemaHash = Packets::kSchemaHash;
  return cfg;
}

NetworkRouter::NetworkRouter(network::TimeSourceI &time, network::FrameLinkI &usb,
                             LinkGateI *usbGate, network::FrameLinkI &air,
                             AirStatsSourceI &airStats)
    : time_(time), airStats_(airStats), usbGuard_(usb, usbGate), node_(makeConfig(), time),
      linkStatsPort_(mux_, Packets::LinkStats::kPort, kRxModule),
      handsetTelemPort_(mux_, Packets::HandsetTelem::kPort, kVehicle),
      upload_(node_, mux_), startNs_(time.nowNs()), nextLinkStatsNs_(startNs_) {
  node_.setDataSink(mux_);

  // Where frames must not go (design 3.3.3, the Revision 1 heartbeat rule, docs/network-mode.md "Filters"):
  //  - USB: everything may go to the ground station, except the copies addressed to the vehicle (Node::send floods
  //    a unicast frame to every link while it has no route).
  //  - Air: only the ground station's HEARTBEAT is dropped (user decision 2026-11-19); its REQUESTs and every other
  //    frame pass. Also dropped: anything addressed to the ground station (it is on USB), no USB-only ports
  //    (>= 0x80), and no RC_STATE that came in over USB whatever its source (the Pocket is the only RC source on the
  //    radio path).
  EgressFilter usbFilter;
  usbFilter.addDstDrop(kVehicle);
  usbIdx_ = node_.addLink(usbGuard_, usbFilter);

  EgressFilter airFilter;
  airFilter.portLimit = 0x80;
  airFilter.addDstDrop(kGroundStation);
  if (usbIdx_ >= 0) airFilter.addRule(static_cast<uint8_t>(usbIdx_), kRcStatePort);
  airFilter.addHeartbeatDrop(kGroundStation);
  airIdx_ = node_.addLink(air, airFilter);

  // Fixed topology: the vehicle and the RX module are always behind the air link, the ground station behind USB.
  // Pinned routes keep a frame with a forged source (for example a bench injection with src 1 over USB) from
  // diverting the Pocket's RC_STATE away from the air link.
  if (airIdx_ >= 0) {
    node_.pinRoute(kVehicle, static_cast<uint8_t>(airIdx_));
    node_.pinRoute(kRxModule, static_cast<uint8_t>(airIdx_));
  }
  if (usbIdx_ >= 0) node_.pinRoute(kGroundStation, static_cast<uint8_t>(usbIdx_));
}

void NetworkRouter::poll() {
  node_.poll();
  const int64_t now = time_.nowNs();
  uploadDue_ = upload_.poll(now);
  if (now >= nextLinkStatsNs_) {
    sendOwnLinkStats(now);
    nextLinkStatsNs_ += kLinkStatsPeriodNs;
    if (nextLinkStatsNs_ <= now) nextLinkStatsNs_ = now + kLinkStatsPeriodNs;
  }
}

void NetworkRouter::sendOwnLinkStats(int64_t now) {
  const Packets::LinkStats ls =
      makeLinkStats(airStats_.localAirStats(), static_cast<double>(now - startNs_) * 1e-9);
  uint8_t buf[Packets::LinkStats::kMaxSize];
  const size_t n = ls.encode(buf, sizeof buf);
  if (n) node_.send(network::kBroadcastAddr, Packets::LinkStats::kPort, buf, n, Qos::Link, true);
}

void NetworkRouter::onCrsfRcFrame(const uint16_t *channels) {
  Packets::RcState rc;
  rcStateFromCrsf(channels, rc);
  uint8_t buf[Packets::RcState::kMaxSize];
  const size_t n = rc.encode(buf, sizeof buf);
  lastRcNs_ = time_.nowNs();
  haveRc_ = true;
  if (n == 0) return;
  stats.rcFramesOffered++;
  // Control QoS, latest-wins: a newer state replaces the queued one on the air side.
  if (!node_.send(kVehicle, kRcStatePort, buf, n, Qos::Control, true)) stats.rcSendFailed++;
  if (!node_.send(kGroundStation, kRcStatePort, buf, n, Qos::Control, true)) stats.rcSendFailed++;
}

bool NetworkRouter::rcSourceFresh() const {
  return haveRc_ && time_.nowNs() - lastRcNs_ < kRcSourceTimeoutNs;
}

bool NetworkRouter::nextHandsetReply(CrsfFrame &out) {
  const int64_t now = time_.nowNs();
  const LocalAirStats local = airStats_.localAirStats();

  const Packets::LinkStats *remote = nullptr;
  if (node_.isPresent(kRxModule) && linkStatsPort_.hasValue() &&
      now - linkStatsPort_.rxNs() < CrsfTelemetryScheduler::kStaleNs)
    remote = &linkStatsPort_.value();

  const Packets::HandsetTelem *telem =
      handsetTelemPort_.hasValue() ? &handsetTelemPort_.value() : nullptr;
  return scheduler_.next(now, remote, local, telem, handsetTelemPort_.rxNs(), out);
}

bool NetworkRouter::log(Packets::Severity severity, const char *text) {
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
