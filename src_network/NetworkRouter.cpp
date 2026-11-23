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
      upload_(node_, mux_), linkStatsPort_(mux_, Packets::LinkStats::kPort, kRxModule),
      handsetTelemPort_(mux_, Packets::HandsetTelem::kPort, kVehicle), shipReboot_(node_, mux_), startNs_(time.nowNs()), nextLinkStatsNs_(startNs_) {
  node_.setDataSink(mux_);

  // Where frames must not go (design 3.3.3, docs/network-mode.md "Filters", docs/gs-control-design.md section 7):
  //  - USB: everything may go to the ground station, except the copies addressed to the vehicle (Node::send floods
  //    a unicast frame to every link while it has no route), and CMD_STATE (the ground station never needs it back).
  //  - Air: the ground station's HEARTBEAT is dropped; its REQUESTs and every other frame pass. Also dropped:
  //    anything addressed to the ground station (it is on USB), no USB-only ports (>= 0x80), and RC_STATE (0x10)
  //    from every ingress: the handset's RC_STATE goes to the ground station over USB only and never on the radio.
  //    CMD_STATE (0x13) is the only control traffic that crosses: it passes only when it came in over USB with
  //    source 0 (a drop for sources 1, 2, 3 on any ingress; the only other ingresses, the air link and this node
  //    itself, are never sources of CMD_STATE). Uplink only: the USB link drops it on egress.
  EgressFilter usbFilter;
  usbFilter.addDstDrop(kVehicle);
  usbFilter.addRule(network::kAnyLink, kCmdStatePort);
  usbIdx_ = node_.addLink(usbGuard_, usbFilter);

  EgressFilter airFilter;
  airFilter.portLimit = 0x80;
  airFilter.addDstDrop(kGroundStation);
  airFilter.addRule(network::kAnyLink, kRcStatePort);
  if (usbIdx_ >= 0) {
    // LINK_CONFIG only travels Router -> air: the ground station can never inject it (it raises the ship node's
    // transmit power; docs/known-issues.md).
    airFilter.addRule(static_cast<uint8_t>(usbIdx_), Packets::LinkConfig::kPort);
  }
  airFilter.addSrcPortDrop(kVehicle, kCmdStatePort);
  airFilter.addSrcPortDrop(kNodeId, kCmdStatePort);
  airFilter.addSrcPortDrop(kRxModule, kCmdStatePort);
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
  upload_.poll(now);
  shipReboot_.poll(now);
  if (haveConfig_ && now >= nextLinkConfigNs_) sendLinkConfig(now);
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

/// LINK_CONFIG (port 0x04): the Router's power setting as continuously repeated, latest-wins link state to the
/// ship radio node, about 2 Hz, never as a one-shot event.
void NetworkRouter::sendLinkConfig(int64_t now) {
  Packets::LinkConfig lc;
  lc.tx_power_dbm = configPowerDbm_;
  lc.flags = 0;
  uint8_t buf[Packets::LinkConfig::kMaxSize];
  const size_t n = lc.encode(buf, sizeof buf);
  // Queue full: retry shortly (the next period would also do; the receiver tolerates 10 s of silence).
  nextLinkConfigNs_ = now + (n && node_.send(kRxModule, Packets::LinkConfig::kPort, buf, n, Qos::Link, true)
                                  ? kLinkConfigPeriodNs
                                  : 50000000LL);
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
  // Handset channels go to the ground station over USB only (GS-control design: the vehicle no longer takes the
  // handset over the air, the ground station sends CMD_STATE instead).
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
