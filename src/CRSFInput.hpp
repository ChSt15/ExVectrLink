#ifndef EXVECRLINK_CRSFINPUT_HPP
#define EXVECRLINK_CRSFINPUT_HPP

#ifdef ENABLE_CRSF_INPUT

#include <Arduino.h>
#include <functional>

#include "ExVectrCore.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"

namespace VCTR::ExVectrLink {

/**
 * @brief Receives CRSF protocol frames from a radio transmitter's external
 * module bay (e.g. the RadioMaster Pocket) over a HardwareSerial port.
 *
 * This is the mirror counterpart of CRSFOutput: instead of generating CRSF
 * frames to feed a flight controller, CRSFInput parses CRSF frames sent BY
 * the handset and converts RC_CHANNELS_PACKED frames into the internal
 * RadioLinkPacket_A4D4 representation used throughout ExVectrLink (same
 * channel ordering as CRSFOutput/SBusTx, so packets produced here are a
 * drop-in match for the rest of the pipeline).
 *
 * It also exposes a small, generic building block
 * (queueTelemetryFrame()) for sending arbitrary CRSF telemetry frames back
 * to the handset, plus one concrete example (sendBatteryTelemetry()).
 * Adding more telemetry types in the future (GPS, link stats, attitude,
 * flight mode, ...) only requires calling queueTelemetryFrame() with the
 * appropriate frame type/payload -- no changes to this class are needed.
 *
 * Any CRSF frame type this class does not natively understand can still be
 * observed/handled elsewhere via addRawFrameHandler(), without having to
 * modify CRSFInput itself (e.g. to support device ping/info, parameter
 * read/write, MSP passthrough, etc. down the line).
 *
 * Designed as a non-blocking periodic task; all serial I/O is buffered so a
 * single taskThread() execution never blocks on the UART.
 */
class CRSFInput : public Core::Task_Periodic {
private:
  // ── CRSF protocol constants ──
  // See https://github.com/crsf-wg/crsf/wiki for the public specification.
  static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

  // Device addresses. RC channel frames conventionally use CRSF_SYNC_BYTE
  // as their address regardless of the physical destination device; the
  // other addresses below are accepted so extended-header frames (bind,
  // parameters, device ping, ...) addressed directly to us are not dropped.
  static constexpr uint8_t CRSF_ADDRESS_BROADCAST = 0x00;
  static constexpr uint8_t CRSF_ADDRESS_CRSF_RECEIVER = 0xEC;
  static constexpr uint8_t CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA;
  static constexpr uint8_t CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE;

  static constexpr uint8_t CRSF_FRAMETYPE_GPS = 0x02;
  static constexpr uint8_t CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08;
  static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;
  static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
  static constexpr uint8_t CRSF_FRAMETYPE_COMMAND = 0x32;

  static constexpr uint16_t CRSF_CHANNEL_MIN = 172;
  static constexpr uint16_t CRSF_CHANNEL_MID = 992;
  static constexpr uint16_t CRSF_CHANNEL_MAX = 1811;
  static constexpr uint8_t CRSF_RC_CHANNELS_PAYLOAD_SIZE = 22;

  static constexpr uint8_t CRSF_MAX_FRAME_LEN = 64;
  static constexpr uint16_t SERIAL_WRITE_BUFFER_SIZE = 128;
  static constexpr uint8_t SERIAL_WRITE_MAX_CHUNK = 32;

  // RC frames typically arrive at 4-500 Hz depending on the handset's radio
  // protocol; polling every 2 ms keeps up without risking UART FIFO overrun.
  static constexpr int64_t TASK_INTERVAL_NS = 2 * Core::MILLISECONDS;

  // How long without a valid RC_CHANNELS_PACKED frame before isReceiving()
  // reports a loss of signal from the handset.
  static constexpr int64_t FAILSAFE_TIMEOUT_NS = 300 * Core::MILLISECONDS;

  HardwareSerial &serial_;
  int8_t rxPin_;
  int8_t txPin_;
  uint32_t baudRate_;
  bool invert_;

  uint8_t rxBuf_[CRSF_MAX_FRAME_LEN];
  uint8_t rxBufPos_ = 0;

  VCTR::packets::radiolink::RadioLinkPacket_A4D4 lastRcPacket_{};
  int64_t lastRcPacketTime_ = 0;

  Core::HandlerGroup<const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &>
      rcChannelHandlers_;
  Core::HandlerGroup<uint8_t, const uint8_t *, uint8_t> rawFrameHandlers_;

  Core::ListBuffer<uint8_t, SERIAL_WRITE_BUFFER_SIZE> serialWriteBuffer_;

  // ── Diagnostics ──
  // Free-running counters used purely for debugging link problems (wiring,
  // baud rate, inversion, ...). None of these affect normal operation.
  uint32_t rxByteCount_ = 0; // total bytes seen on the UART
  uint32_t resyncByteCount_ =
      0;                        // bytes discarded while looking for a sync byte
  uint32_t crcErrorCount_ = 0;  // frames with a length/CRC mismatch
  uint32_t goodFrameCount_ = 0; // frames that passed the CRC check
  uint8_t lastFrameType_ = 0;   // type of the most recently *valid* frame

  // Ring buffer snapshot of the most recently received raw bytes, so a
  // caller can print/inspect exactly what's arriving on the wire (stuck at
  // a fixed value ⇒ no real signal; varied but never matching a sync byte
  // ⇒ inversion/bit-order/framing problem; etc).
  static constexpr uint8_t RAW_BYTE_SNAPSHOT_LEN = 16;
  uint8_t rawByteSnapshot_[RAW_BYTE_SNAPSHOT_LEN] = {0};
  uint8_t rawByteSnapshotPos_ = 0;

  // ── Helpers ──
  static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte);
  static uint8_t calculateCRC(const uint8_t *data, size_t length);
  static int16_t crsfToNorm(uint16_t val);
  static void unpackChannels(const uint8_t *payload, uint16_t *channels);

  void processIncomingBytes();
  void handleFrame(const uint8_t *frame, uint8_t len);
  void decodeRcChannelsPacked(const uint8_t *payload);

  bool queueSerialFrame(const uint8_t *frame, size_t frameSize);
  void flushSerialWriteBuffer();

public:
  /**
   * @param serial The hardware serial port wired to the radio's module bay
   * CRSF pins.
   * @param rxPin RX pin to use, or -1 to leave the port's currently
   * configured/default pin mapping untouched. If rxPin == txPin, only the
   * RX signal is bound to that pin (single shared signal wire, as used by
   * e.g. the RadioMaster Pocket's nano module bay); TX is left unmapped so
   * we never drive the shared line ourselves and can't contend with the
   * handset's own driver. Telemetry transmission is therefore inert in
   * that configuration until proper direction-switching is implemented.
   * @param txPin TX pin to use, or -1 to leave the port's currently
   * configured/default pin mapping untouched.
   * @param baudRate CRSF baud rate. Must match the handset's configured
   * external-module baud rate exactly (visible in the radio's module
   * settings) -- 400000 is the widely-compatible default used by
   * OpenTX/EdgeTX's "CRSF" external module type; 420000 is the standard
   * receiver<->FC rate and is only used by some radios/protocols.
   * @param invert Set true if the physical signal is logic-inverted, as is
   * common on S.Port-style single-wire module bay pins.
   */
  explicit CRSFInput(HardwareSerial &serial, int8_t rxPin = -1,
                     int8_t txPin = -1, uint32_t baudRate = 400000,
                     bool invert = false);

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;

  // ── RC channel reception ──

  /**
   * @brief Registers a handler invoked every time a fresh
   * RC_CHANNELS_PACKED frame has been decoded. Multiple handlers may be
   * registered (e.g. one to forward over the FHSS radio link, another for
   * local logging/diagnostics).
   */
  void addRcChannelsHandler(
      std::function<
          void(const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &)>
          handler);

  /**
   * @brief Registers a handler invoked for every parsed CRSF frame,
   * regardless of type, with the raw frame type and payload. Use this to
   * support additional frame types in the future without modifying
   * CRSFInput.
   */
  void addRawFrameHandler(
      std::function<void(uint8_t frameType, const uint8_t *payload,
                         uint8_t payloadLen)>
          handler);

  /// @returns the most recently decoded RC channels packet.
  const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &getLastRcPacket() const;

  /// @returns the timestamp (Core::NowNs()) the last RC_CHANNELS_PACKED
  /// frame was received.
  int64_t getLastRcPacketTime() const;

  /// @returns true if a RC_CHANNELS_PACKED frame was received within the
  /// failsafe timeout window, i.e. the handset link is currently alive.
  bool isReceiving() const;

  // ── Diagnostics ──
  // Cheap counters for tracking down link issues (wrong pin, baud rate,
  // inversion, wiring, ...). Safe to read from another task/loop.

  /// @returns total number of raw bytes seen on the UART since boot. If this
  /// stays at 0 the UART is not receiving anything at all (wrong pin, wrong
  /// serial port, or nothing is being sent).
  uint32_t getRxByteCount() const { return rxByteCount_; }

  /// @returns number of bytes that were discarded while resynchronising to
  /// a valid CRSF sync byte. A high, ever-growing count relative to
  /// getRxByteCount() suggests garbage on the line (wrong baud rate,
  /// inversion, or bus contention on a shared/half-duplex wire).
  uint32_t getResyncByteCount() const { return resyncByteCount_; }

  /// @returns number of frames with a bad CRC (received a plausible length
  /// but the checksum didn't match). Non-zero counts here (with bytes
  /// actually arriving) usually mean a baud-rate mismatch or signal
  /// integrity problem rather than a purely logical bug.
  uint32_t getCrcErrorCount() const { return crcErrorCount_; }

  /// @returns number of frames that passed CRC validation, of any type.
  uint32_t getGoodFrameCount() const { return goodFrameCount_; }

  /// @returns the CRSF frame type of the most recently *valid* frame
  /// received (0 if none yet).
  uint8_t getLastFrameType() const { return lastFrameType_; }

  /// @brief Copies the most recently received raw bytes (in arrival order,
  /// oldest first) into `out`. Use this to eyeball exactly what's coming
  /// over the wire when nothing is decoding: a fixed/stuck value (0x00 or
  /// 0xFF) means no real signal is reaching the pin; varied values that
  /// never include a valid sync byte suggest inversion, wrong baud, or a
  /// framing/bit-order mismatch.
  /// @param out Buffer of at least RAW_BYTE_SNAPSHOT_LEN (16) bytes.
  void copyRawByteSnapshot(uint8_t *out) const {
    for (uint8_t i = 0; i < RAW_BYTE_SNAPSHOT_LEN; i++) {
      out[i] =
          rawByteSnapshot_[(rawByteSnapshotPos_ + i) % RAW_BYTE_SNAPSHOT_LEN];
    }
  }

  /// @returns the number of bytes copyRawByteSnapshot() will fill.
  static constexpr uint8_t getRawByteSnapshotLen() {
    return RAW_BYTE_SNAPSHOT_LEN;
  }

  // ── Telemetry transmission ──
  // The handset link is bidirectional: in addition to receiving RC
  // channels we can report telemetry back (battery, GPS, link stats, ...).
  // queueTelemetryFrame() is the generic low level building block; add new
  // convenience wrappers (like sendBatteryTelemetry()) as new telemetry
  // sources become available, without needing to touch the parsing code
  // above.

  /**
   * @brief Builds a standard CRSF frame of the given type wrapping
   * `payload` and queues it for (non-blocking) transmission back to the
   * handset.
   * @param frameType One of the CRSF_FRAMETYPE_* constants (or any other
   * valid CRSF frame type value).
   * @param payload Frame payload bytes (frame type specific layout).
   * @param payloadLen Number of bytes in `payload`. Must leave room for the
   * frame header and CRC (max CRSF_MAX_FRAME_LEN - 4 bytes).
   * @return false if the frame is too large or the output buffer is full.
   */
  bool queueTelemetryFrame(uint8_t frameType, const uint8_t *payload,
                           uint8_t payloadLen);

  /**
   * @brief Sends a CRSF battery-sensor telemetry frame back to the handset.
   * @param voltageV Battery voltage in volts.
   * @param currentA Battery current draw in amps.
   * @param capacityMah Consumed capacity in mAh.
   * @param percentRemaining Remaining battery capacity, 0-100 %.
   */
  bool sendBatteryTelemetry(float voltageV, float currentA,
                            uint32_t capacityMah, uint8_t percentRemaining);
};

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF_INPUT
#endif // EXVECRLINK_CRSFINPUT_HPP
