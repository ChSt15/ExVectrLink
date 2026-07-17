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
#include "DatalinkCommandManager.hpp"
#include "RcLinkChannels.hpp"

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
 * Also sends a CRSF Link Statistics (0x14) frame to the FC at ~10 Hz so it
 * can display uplink LQ/RSSI/SNR/TX-power (see writeLinkStatsFrame()) -- this
 * is what drives the FC's link telemetry readouts. Bind-command detection
 * (the FC/Betaflight "Bind" button) is not yet ported -- see CRSFOutput for
 * the previous implementation of that.
 */
class CRSFRcOutput : public Core::Task_Periodic {
private:
  static constexpr uint16_t CRSF_CHANNEL_MIN = 172;
  static constexpr uint16_t CRSF_CHANNEL_MID = 992;
  static constexpr uint16_t CRSF_CHANNEL_MAX = 1811;
  static constexpr uint32_t CRSF_BAUD_RATE = 420000;

  CRSF crsf_;

  VCTR::packets::PacketManager packetManager_;
  /// Accumulates the newest value of every RC channel out of the incoming
  /// RcLinkPackets (priority channels every packet, one multiplexed aux
  /// channel per packet -- see RcLinkChannels.hpp).
  RcLinkAccumulator rcAccumulator_;
  /// Glitch filter over the accumulated full channel set. The LinkType tag
  /// on the filter's internal packet is inert -- it is never serialized.
  VCTR::packets::radiolink::RadioLinkPacketFilter<
      kRcPriorityAnalog + kRcMuxAnalog, kRcPriorityDigital + kRcMuxDigital,
      VCTR::packets::radiolink::RadioLinkTypes::A4D0>
      packetFilter_;
  int64_t lastPacketTime_ = 0;
  bool newRcPacket_ = false;

  // Link-stats cadence to the FC. The FC only refreshes its LQ/RSSI/etc.
  // readouts when it receives a 0x14 frame, so this is sent on its own timer
  // (independent of RC-packet arrival) at roughly the same ~10 Hz CRSFOutput
  // and CRSFRcInput use.
  static constexpr int64_t LINK_STATS_INTERVAL_NS = 100 * Core::MILLISECONDS;
  int64_t lastLinkStatsSendNs_ = 0;

  static uint16_t normToCrsf(int16_t val);

  void writeRcChannelsFrame();
  void writeLinkStatsFrame();

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
   * queued for the FHSS downlink under `key` (see
   * DatalinkCommandManager.hpp's OtaSendKey/queueOtaPacket()). This is the
   * one place new FC->ground telemetry types need to hook into.
   */
  template <VCTR::packets::IsPacket CrsfT, VCTR::packets::IsPacket ExVectrT>
  void forwardCrsfTelemetry(datalink::OtaSendKey key,
                            std::function<ExVectrT(const CrsfT &)> convert);

  /**
   * @brief Like forwardCrsfTelemetry() above, but for a CRSF frame that
   * needs to become more than one OTA packet (e.g. GPS position doesn't fit
   * in the same frame as the rest of the GPS fields, so it's manually split
   * into two independently-keyed sends). `handle` is called with the
   * decoded `CrsfT` and is expected to call sendOverFhss() itself, once per
   * piece, each under its own OtaSendKey.
   */
  template <VCTR::packets::IsPacket CrsfT>
  void forwardCrsfTelemetry(std::function<void(const CrsfT &)> handle);

  /// Serializes `packet`, tags it PacketTypes::Data, and queues it for the
  /// FHSS downlink under `key` via DatalinkCommandManager's shared
  /// coalescing send queue -- same path CRSFRcInput's RC-channel uplink and
  /// DatalinkCommandManager's own link-stats sends use, so none of them
  /// race to overwrite each other's pending frame.
  template <VCTR::packets::IsPacket T>
  void sendOverFhss(datalink::OtaSendKey key, const T &packet);

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
