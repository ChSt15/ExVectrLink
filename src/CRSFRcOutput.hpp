#ifndef EXVECRLINK_CRSFRCOUTPUT_HPP
#define EXVECRLINK_CRSFRCOUTPUT_HPP

#ifdef ENABLE_CRSF

#include <Arduino.h>
#include <functional>

#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"

#include "CRSF.hpp"

namespace VCTR::ExVectrLink {

/// @returns how many RC_CHANNELS_PACKED frames were received from the radio
/// link and sent out over CRSF in the last completed 1-second window.
uint16_t getCrsfRcPacketRateHz();
/// @returns how many CRSF frame writes failed (send buffer full) in the last
/// completed 1-second window.
uint16_t getCrsfTxFailureRateHz();

/**
 * @brief Sends CRSF RC_CHANNELS_PACKED frames to a flight controller, built
 * on the generic CRSF class. Also relays FC -> ground telemetry: any
 * supported CRSF frame the FC sends back over the wire (battery sensor,
 * GPS, ...) is converted to the matching VCTR::packets::telecoms
 * Telemetry_* packet and sent over the FHSS downlink -- see
 * registerCrsfTelemetryForwarding(). The ground TX module's CRSFRcInput
 * mirrors this: it decodes the same Telemetry_* packets back out of the
 * downlink and re-encodes them as CRSF frames for the handset, so adding a
 * new telemetry type only needs a forwardCrsfTelemetry<>() call here plus a
 * matching forwardTelemetry<>() call in CRSFRcInput.
 *
 * Link statistics and bind-command detection are not yet ported -- see
 * CRSFOutput for the previous, still-complete implementation of those.
 */
class CRSFRcOutput : public Core::Task_Periodic {
private:
  static constexpr uint16_t CRSF_CHANNEL_MIN = 172;
  static constexpr uint16_t CRSF_CHANNEL_MID = 992;
  static constexpr uint16_t CRSF_CHANNEL_MAX = 1811;
  static constexpr uint32_t CRSF_BAUD_RATE = 420000;

  CRSF crsf_;

  VCTR::packets::PacketManager packetManager_;
  VCTR::packets::radiolink::RadioLinkPacket_A4D4 packet_;
  VCTR::packets::radiolink::RadioLinkPacketFilter<
      4, 4, VCTR::packets::radiolink::RadioLinkTypes::A4D4>
      packetFilter_;
  int64_t lastPacketTime_ = 0;
  bool newRcPacket_ = false;

  static uint16_t normToCrsf(int16_t val);

  void writeRcChannelsFrame();

  // ── FC -> ground telemetry relay ──
  // Each entry attempts to read one supported CRSF telemetry frame type off
  // crsf_'s received-frame queue and, on success, forwards it over FHSS;
  // returns false (queue untouched) if the next queued frame isn't its
  // type, so taskThread() can just try each in turn. See
  // registerCrsfTelemetryForwarding() for what's currently wired up.
  Core::ListArray<std::function<bool(CRSF &)>> crsfTelemetryHandlers_;
  void registerCrsfTelemetryForwarding();

  /**
   * @brief Registers a conversion from a CRSF telemetry frame type to an
   * ExVectrPackets telemetry type: whenever `CrsfT` is the next queued
   * frame read off the FC's CRSF wire, `convert` is applied and the result
   * sent over the FHSS downlink. This is the one place new FC->ground
   * telemetry types need to hook into.
   */
  template <VCTR::packets::IsPacket CrsfT, VCTR::packets::IsPacket ExVectrT>
  void forwardCrsfTelemetry(std::function<ExVectrT(const CrsfT &)> convert);

  /// Serializes `packet`, tags it PacketTypes::Data, and sends it over the
  /// FHSS downlink -- same tagging CRSFRcInput's RC-channel uplink and
  /// DatalinkCommandManager's generic Data forwarding use.
  template <VCTR::packets::IsPacket T> void sendOverFhss(const T &packet);

public:
  /**
   * @param serial Serial port wired to the flight controller's CRSF pins
   * (separate RX/TX lines -- full duplex).
   */
  explicit CRSFRcOutput(HardwareSerial &serial);

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;
};

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF
#endif // EXVECRLINK_CRSFRCOUTPUT_HPP
