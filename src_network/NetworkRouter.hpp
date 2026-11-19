#pragma once
// TX-module router application for ExVectrNetwork mode: node 2, role link TX module, forwarding = true.
// Host-testable: no Arduino, no ExVectrCore, no heap after construction (docs/network-mode.md).
//
// Links: USB (towards the ground station, node 0) and the air side (AirFrameLink on the FHSS slots, towards the
// vehicle receiver).
// The CRSF half-duplex port is not a network link: this class turns CRSF RC frames into RC_STATE and builds the
// telemetry frames for the handset.
#include <stdint.h>

#include "ExVectrNetwork.hpp"
#include "ExVectrNetwork/TimeSourceI.hpp"

#include "LinkGuard.hpp"
#include "LinkUploadCommand.hpp"
#include "NetAirAccess.hpp"
#include "NetCrsf.hpp"

namespace VCTR::ExVectrLink::netmode {

/// Latest value of one subscribed port, accepted from one source node only. Plain port handler: copies the
/// decoded packet and does no control work (the library's receive-callback rule).
template <typename T>
class SourcePort : public network::PortHandlerI {
public:
  SourcePort(network::PortRegistryI &mux, uint8_t port, uint8_t acceptedSrc)
      : src_(acceptedSrc) {
    subscribed_ = mux.subscribe(port, *this);
  }
  bool subscribed() const { return subscribed_; }
  bool hasValue() const { return has_; }
  const T &value() const { return value_; }
  int64_t rxNs() const { return rxNs_; }
  uint32_t rejectedSource() const { return rejectedSrc_; }
  uint32_t decodeErrors() const { return decodeErrors_; }

  void onPacket(const network::RxInfo &info, const uint8_t *payload, uint8_t len) override {
    if (info.src != src_) {
      rejectedSrc_++;
      return;
    }
    T tmp;
    if (!tmp.decode(payload, len)) {
      decodeErrors_++;
      return;
    }
    value_ = tmp;
    rxNs_ = info.rxTimeNs;
    has_ = true;
  }

private:
  uint8_t src_;
  bool subscribed_ = false;
  bool has_ = false;
  T value_{};
  int64_t rxNs_ = 0;
  uint32_t rejectedSrc_ = 0, decodeErrors_ = 0;
};

class NetworkRouter {
public:
  static constexpr uint8_t kNodeId = 2; // TX module
  static constexpr uint8_t kGroundStation = 0;
  static constexpr uint8_t kVehicle = 1;
  static constexpr uint8_t kRxModule = 3;
  static constexpr uint8_t kRcStatePort = Packets::RcState::kPort;
  static constexpr int64_t kRcSourceTimeoutNs = 100000000LL; // 100 ms
  static constexpr int64_t kLinkStatsPeriodNs = 500000000LL; // 2 Hz (schema air rate)

  struct RouterStats {
    uint32_t rcFramesOffered = 0; ///< CRSF RC frames turned into RC_STATE
    uint32_t rcSendFailed = 0;    ///< RC_STATE copies no link accepted
  } stats;

  /// `usb` and `air` are the raw datalinks. `usbGate` (optional) refuses frames while the host is absent or the
  /// stream is backed up. `airStats` reports the module's own radio.
  NetworkRouter(network::TimeSourceI &time, network::FrameLinkI &usb, LinkGateI *usbGate,
                network::FrameLinkI &air, AirStatsSourceI &airStats);

  /// Periodic work (heartbeat, own LINK_STATS). Call at least at 10 Hz, together with the links' poll().
  void poll();

  /// One CRSF RC_CHANNELS_PACKED frame (16 raw 11-bit values) arrived. Replaces the queued state: offers
  /// RC_STATE to the air side (dst 1) and to USB (dst 0). Called once per CRSF frame, and never otherwise:
  /// RC state is not synthesised or repeated.
  void onCrsfRcFrame(const uint16_t channels[16]);

  /// True while the last CRSF RC frame is younger than 100 ms (the SOURCE_OK condition).
  bool rcSourceFresh() const;

  /// To be called once per frame received from the handset. Returns true and fills `out` if a telemetry frame
  /// is to be written as the (single) pending reply.
  bool nextHandsetReply(CrsfFrame &out);

  /// LOG_TEXT to the ground station (replaces the debug monitor). Text is truncated to 233 bytes.
  bool log(Packets::Severity severity, const char *text);

  /// True once an accepted link-upload command (REQUEST action 10, target 2) has been pending for its delay: the
  /// application then enters upload mode (UploadTriggers::requestCommand()). Level, set by poll().
  bool uploadCommandDue() const { return uploadDue_; }
  const LinkUploadCommand &uploadCommand() const { return upload_; }

  network::Node &node() { return node_; }
  const network::Node &node() const { return node_; }
  int usbLinkIndex() const { return usbIdx_; }
  int airLinkIndex() const { return airIdx_; }
  const LinkGuard &usbGuard() const { return usbGuard_; }
  const SourcePort<Packets::LinkStats> &linkStatsPort() const { return linkStatsPort_; }
  const SourcePort<Packets::HandsetTelem> &handsetTelemPort() const { return handsetTelemPort_; }

private:
  static network::NodeConfig makeConfig();
  void sendOwnLinkStats(int64_t now);

  network::TimeSourceI &time_;
  AirStatsSourceI &airStats_;
  LinkGuard usbGuard_;
  network::Node node_;
  network::PortMux<8> mux_;
  LinkUploadCommand upload_;
  bool uploadDue_ = false;
  SourcePort<Packets::LinkStats> linkStatsPort_;     // accepted from node 3 only
  SourcePort<Packets::HandsetTelem> handsetTelemPort_; // accepted from node 1 only
  CrsfTelemetryScheduler scheduler_;
  int usbIdx_ = -1, airIdx_ = -1;
  int64_t startNs_;
  int64_t nextLinkStatsNs_;
  int64_t lastRcNs_ = 0;
  bool haveRc_ = false;
};

} // namespace VCTR::ExVectrLink::netmode
