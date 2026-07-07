#ifndef EXVECRLINK_CRSFRCINPUT_HPP
#define EXVECRLINK_CRSFRCINPUT_HPP

#ifdef ENABLE_CRSF_INPUT

#include <Arduino.h>

#include <functional>

#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

#include "CRSF.hpp"

namespace VCTR::ExVectrLink {

/**
 * @brief Receives CRSF RC_CHANNELS_PACKED frames from a handset's external
 * module bay, built on the generic CRSF class, and forwards them over the
 * FHSS radio link as RadioLinkPacket_A4D4 packets.
 *
 * This is the first step of moving CRSFInput's responsibilities onto the
 * new CRSF system: only RC-channel input is handled here. Telemetry-out and
 * the diagnostic counters/raw-byte snapshot are not ported yet -- see
 * CRSFInput for the previous, still-complete implementation of those.
 *
 * Runs the underlying CRSF object in CRSFDuplex::Half. On a shared
 * single-wire module bay pin (rxPin == txPin), the port is initially bound
 * with only RX attached (see taskInit()); CRSF then reroutes the pin to
 * UART TX for the duration of a single reply frame immediately after each
 * RC_CHANNELS_PACKED frame is received, via the GPIO matrix (see
 * CRSF::switchToRx()/switchToTx()), and back to RX once it's sent. Queue
 * telemetry to send back to the handset with sendTelemetry().
 *
 * Also relays FC -> handset telemetry: any VCTR::packets::telecoms
 * Telemetry_* packet arriving over the FHSS downlink (tagged
 * PacketTypes::Data, same as CRSFOutput sends when it decodes them off the
 * FC's own CRSF wire) is converted to the matching CRSF telemetry frame and
 * queued via sendTelemetry() -- see registerTelemetryForwarding(). Adding a
 * new telemetry type to relay is a single forwardTelemetry<>() call there,
 * not a new hand-rolled decode/convert/send path.
 */
class CRSFRcInput : public Core::Task_Periodic {
private:
  HardwareSerial &serial_;
  int8_t rxPin_;
  int8_t txPin_;
  uint32_t baudRate_;
  bool invert_;
  uint8_t uartNum_;

  CRSF crsf_;

  VCTR::packets::radiolink::RadioLinkPacket_A4D4 lastRcPacket_{};
  int64_t lastRcPacketTime_ = 0;
  uint32_t rcFrameCount_ = 0;

  // Link-stats telemetry cadence (sent back to the handset).
  static constexpr int64_t LINK_STATS_INTERVAL_NS = 100 * Core::MILLISECONDS;
  int64_t lastLinkStatsSendNs_ = 0;
  void writeLinkStatsFrame();

  static int16_t crsfToNorm(uint16_t val);
  static uint8_t digitalFromCrsf(uint16_t val);

  void forwardRcChannels(
      const VCTR::packets::crsf::CRSFPacket_RcChannelsPacked &pkt);

  // ── CRSF parameter/command protocol (handset Lua config menu) ──
  // Exposes a single CRSF COMMAND parameter, "Bind", that the standard
  // ELRS/EdgeTX Lua config screen can list and execute. Unlike a real
  // ExpressLRS bind, executing it just relays a one-shot OTA trigger to the
  // far end (see DatalinkCommandManager's PacketTypes::UploadMode handling)
  // to make the RX enter WiFi firmware-upload mode. See TXModuleParameters.cpp
  // in extern/ExpressLRS-master for the reference command-parameter state
  // machine this mirrors (handleSimpleSendCmd()).
  static constexpr uint8_t kBindParameterNumber = 1;
  // Mirrors ExpressLRS's handleSimpleSendCmd() 2 s auto-revert-to-idle so a
  // lost/never-polled-again command doesn't leave the Lua entry stuck
  // showing "executing" forever.
  static constexpr int64_t kBindStatusTimeoutNs = 2000 * Core::MILLISECONDS;

  VCTR::packets::crsf::CRSFCommandStep bindStep_ =
      VCTR::packets::crsf::CRSFCommandStep::Ready;
  char bindInfo_[32] = "";
  int64_t lastBindClickNs_ = 0;

  void handleParameterPing(
      const VCTR::packets::crsf::CRSFPacket_ParameterPing &ping);
  void handleParameterRead(
      const VCTR::packets::crsf::CRSFPacket_ParameterRead &req);
  void handleParameterWrite(
      const VCTR::packets::crsf::CRSFPacket_ParameterWrite &req);
  void sendBindEntry(VCTR::packets::crsf::CRSFCommandStep status,
                     const char *info, uint8_t destAddress);
  // Sends a CRSF 0x2E ELRS-status frame (good/bad packet counter + connection
  // flag) in response to the ELRS Lua's periodic parameter-0 link-stat poll.
  void sendElrsStatus(uint8_t destAddress);
  void triggerRxWifiUpload();

  // ── FC -> handset telemetry relay ──
  // Decodes VCTR::packets::telecoms Telemetry_* packets out of the FHSS
  // downlink's raw PacketTypes::Data payloads; see
  // registerTelemetryForwarding() for what's currently wired up.
  VCTR::packets::PacketManager telemetryPacketManager_;
  void registerTelemetryForwarding();

  // GPS arrives as two independently-timed packets (stats and position --
  // see registerTelemetryForwarding()); this holds the latest of each so
  // every send to the handset carries the freshest known values for both,
  // even though only one half updates at a time.
  VCTR::packets::crsf::CRSFPacket_GPS gpsCache_{};

  /**
   * @brief Registers a conversion from an incoming ExVectrPackets telemetry
   * type to a CRSF telemetry frame: whenever `SourceT` is decoded out of
   * the FHSS downlink, `convert` is applied and the result queued via
   * sendTelemetry(). This is the one place new telemetry sources need to
   * hook into -- no other code needs to change.
   */
  template <VCTR::packets::IsPacket SourceT, VCTR::packets::IsPacket CrsfT>
  void forwardTelemetry(std::function<CrsfT(const SourceT &)> convert) {
    telemetryPacketManager_.addPacketReceiveHandler<SourceT>(
        [this, convert](const SourceT &packet) {
          sendTelemetry(convert(packet));
        });
  }

public:
  /**
   * @param serial Serial port wired to the handset's external module bay
   * CRSF pins.
   * @param rxPin RX pin to use, or -1 to leave the port's default pin
   * mapping untouched. If rxPin == txPin, only RX is bound (shared
   * single-wire bay pin, as used by e.g. the RadioMaster Pocket's nano
   * module bay); see the class doc comment.
   * @param txPin TX pin to use, or -1 to leave the port's default pin
   * mapping untouched.
   * @param baudRate CRSF baud rate. Must match the handset's configured
   * external-module baud rate exactly.
   * @param invert Set true if the physical signal is logic-inverted, as is
   * common on S.Port-style single-wire module bay pins.
   * @param uartNum The ESP32 UART peripheral number `serial` wraps (e.g.
   * HardwareSerial(1) -> 1). Only used (and required) when rxPin == txPin,
   * to drive the GPIO-matrix direction switching for the shared pin.
   */
  explicit CRSFRcInput(HardwareSerial &serial, int8_t rxPin = -1,
                       int8_t txPin = -1, uint32_t baudRate = 400000,
                       bool invert = false, uint8_t uartNum = 0);

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;

  /**
   * @brief Queues a CRSF packet to be sent back to the handset as
   * telemetry. On the shared half-duplex pin this is only ever actually
   * put on the wire the instant the next RC_CHANNELS_PACKED frame is fully
   * received (see CRSF::sendHalfDuplexReply()); only the single freshest
   * queued packet is kept, so calling this faster than frames arrive just
   * replaces the pending one.
   * @return false if the packet is too large to fit in one CRSF frame.
   */
  template <VCTR::packets::IsPacket T> bool sendTelemetry(const T &packet) {
    return crsf_.write(packet);
  }

  // ── Diagnostics ──
  // Minimal visibility for bench testing; see CRSFInput for the fuller
  // byte-level diagnostics (raw snapshot, CRC error/resync counters) not
  // ported here yet.

  /// @returns the most recently decoded RC channels packet.
  const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &
  getLastRcPacket() const {
    return lastRcPacket_;
  }

  /// @returns the timestamp (Core::NowNs()) the last RC_CHANNELS_PACKED
  /// frame was received.
  int64_t getLastRcPacketTime() const { return lastRcPacketTime_; }

  /// @returns total number of RC_CHANNELS_PACKED frames forwarded since
  /// boot (free-running; callers wanting a rate should diff this
  /// themselves over time, e.g. once per second).
  uint32_t getRcFrameCount() const { return rcFrameCount_; }
};

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF_INPUT
#endif // EXVECRLINK_CRSFRCINPUT_HPP
