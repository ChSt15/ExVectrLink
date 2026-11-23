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
#include "ShipRebootRequester.hpp"
#include "SourcePort.hpp"

namespace VCTR::ExVectrLink::netmode {

class NetworkRouter {
public:
  static constexpr uint8_t kNodeId = 2; // TX module
  static constexpr uint8_t kGroundStation = 0;
  static constexpr uint8_t kVehicle = 1;
  static constexpr uint8_t kRxModule = 3;
  static constexpr uint8_t kRcStatePort = Packets::RcState::kPort;
  static constexpr uint8_t kCmdStatePort = Packets::CmdState::kPort; // 0x13, node 0 -> vehicle, uplink only
  static constexpr int64_t kRcSourceTimeoutNs = 100000000LL; // 100 ms
  static constexpr int64_t kLinkStatsPeriodNs = 500000000LL; // 2 Hz (schema air rate)
  static constexpr int64_t kLinkConfigPeriodNs = 500000000LL; // 2 Hz (schema air rate)

  struct RouterStats {
    uint32_t rcFramesOffered = 0; ///< CRSF RC frames turned into RC_STATE
    uint32_t rcSendFailed = 0;    ///< RC_STATE copies (USB) no link accepted
  } stats;

  /// `usb` and `air` are the raw datalinks. `usbGate` (optional) refuses frames while the host is absent or the
  /// stream is backed up. `airStats` reports the module's own radio.
  NetworkRouter(network::TimeSourceI &time, network::FrameLinkI &usb, LinkGateI *usbGate,
                network::FrameLinkI &air, AirStatsSourceI &airStats);

  /// Periodic work (heartbeat, own LINK_STATS). Call at least at 10 Hz, together with the links' poll().
  void poll();

  /// One CRSF RC_CHANNELS_PACKED frame (16 raw 11-bit values) arrived. Offers RC_STATE to USB (dst 0)
  /// only; it never goes on the air (CMD_STATE from the ground station replaces it). Called once per CRSF frame, and never otherwise:
  /// RC state is not synthesised or repeated.
  void onCrsfRcFrame(const uint16_t channels[16]);

  /// True while the last CRSF RC frame is younger than 100 ms (the SOURCE_OK condition).
  bool rcSourceFresh() const;

  /// To be called once per frame received from the handset. Returns true and fills `out` if a telemetry frame
  /// is to be written as the (single) pending reply.
  bool nextHandsetReply(CrsfFrame &out);

  /// The power the Router is using now (dBm): the single variable it applies to its radios and broadcasts as
  /// LINK_CONFIG to the ship radio node. Call whenever it changes (and once at start); a change is sent at the
  /// next poll() instead of waiting for the 2 Hz period.
  void setConfigPower(int8_t dbm) {
    if (!haveConfig_ || dbm != configPowerDbm_) nextLinkConfigNs_ = 0;
    configPowerDbm_ = dbm;
    haveConfig_ = true;
  }
  int8_t configPower() const { return configPowerDbm_; }

  /// Starts a LINK_REBOOT request to the ship radio node (REQUEST action 11, target 3), repeated until acked
  /// (the Lua "Reboot ship node" command). False if one is already running.
  bool requestShipReboot() { return shipReboot_.request(time_.nowNs()); }
  const ShipRebootRequester &shipReboot() const { return shipReboot_; }

  /// LOG_TEXT to the ground station (replaces the debug monitor). Text is truncated to 233 bytes.
  bool log(Packets::Severity severity, const char *text);

  /// True once an accepted link-upload command (REQUEST action 10, target 2) has been pending for its delay: the
  /// application then enters upload mode (UploadTriggers::requestCommand()). Level, set by poll().
  bool uploadCommandDue() const { return upload_.due(LinkCommand::kUploadAction); }
  /// True once an accepted LINK_REBOOT request (REQUEST action 11, same target and delay) is due: the application
  /// then restarts the module (ESP.restart()). Level.
  bool rebootCommandDue() const { return upload_.due(LinkCommand::kRebootAction); }
  const LinkCommand &uploadCommand() const { return upload_; }

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
  void sendLinkConfig(int64_t now);

  network::TimeSourceI &time_;
  AirStatsSourceI &airStats_;
  LinkGuard usbGuard_;
  network::Node node_;
  network::PortMux<8> mux_; // LINK_STATS, HANDSET_TELEM, REQUEST, REQUEST_ACK
  LinkCommand upload_; // both link commands (upload mode, reboot)
  SourcePort<Packets::LinkStats> linkStatsPort_;     // accepted from node 3 only
  SourcePort<Packets::HandsetTelem> handsetTelemPort_; // accepted from node 1 only
  ShipRebootRequester shipReboot_;
  CrsfTelemetryScheduler scheduler_;
  int8_t configPowerDbm_ = 10;
  bool haveConfig_ = false;
  int64_t nextLinkConfigNs_ = 0;
  int usbIdx_ = -1, airIdx_ = -1;
  int64_t startNs_;
  int64_t nextLinkStatsNs_;
  int64_t lastRcNs_ = 0;
  bool haveRc_ = false;
};

} // namespace VCTR::ExVectrLink::netmode
