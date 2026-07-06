#ifndef EXVECRLINK_CRSFRCINPUT_HPP
#define EXVECRLINK_CRSFRCINPUT_HPP

#ifdef ENABLE_CRSF_INPUT

#include <Arduino.h>

#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"

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
 * Runs the underlying CRSF object in CRSFDuplex::Half. On this hardware a
 * shared single-wire module bay pin (rxPin == txPin) is handled by binding
 * only RX to the pin and leaving TX unmapped (see taskInit()) -- this class
 * never calls crsf_.write(), so CRSF::write() always failing in Half mode
 * never matters here.
 */
class CRSFRcInput : public Core::Task_Periodic {
private:
  HardwareSerial &serial_;
  int8_t rxPin_;
  int8_t txPin_;
  uint32_t baudRate_;
  bool invert_;

  CRSF crsf_;

  VCTR::packets::radiolink::RadioLinkPacket_A4D4 lastRcPacket_{};
  int64_t lastRcPacketTime_ = 0;
  uint32_t rcFrameCount_ = 0;

  static int16_t crsfToNorm(uint16_t val);
  static uint8_t digitalFromCrsf(uint16_t val);

  void forwardRcChannels(
      const VCTR::packets::crsf::CRSFPacket_RcChannelsPacked &pkt);

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
   */
  explicit CRSFRcInput(HardwareSerial &serial, int8_t rxPin = -1,
                       int8_t txPin = -1, uint32_t baudRate = 400000,
                       bool invert = false);

  void taskInit() override;
  void taskThread() override;

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
