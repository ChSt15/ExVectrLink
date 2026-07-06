#ifndef EXVECRLINK_CRSF_HPP
#define EXVECRLINK_CRSF_HPP

#include <Arduino.h>

#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrPackets/Packets.hpp"

namespace VCTR::ExVectrLink {

enum class CRSFDuplex : uint8_t {
  Full = 0,
  Half = 1,
};

/**
 * @brief Single class handling both directions of CRSF protocol
 * communication (RC channels, telemetry, commands, ...) over a UART.
 *
 * Unlike CRSFOutput/CRSFInput, this is a passive object: it does not
 * register itself with the scheduler and has no callback/handler
 * registration. The caller is responsible for calling available()
 * periodically to pump the UART, and for calling write()/read() to
 * exchange packets.
 *
 * Packets are exchanged as the typed VCTR::packets::crsf::CRSFPacket_*
 * classes (see ExVectrPackets/crsf/packets/CRSFPackets.hpp) -- write()
 * serializes a packet into a real CRSF wire frame (sync/len/type/payload/
 * CRC) and queues it for transmission; read() looks at the next queued,
 * CRC-validated received frame and deserializes it into a packet of the
 * requested type if the frame type matches.
 *
 * In CRSFDuplex::Half, write()/writeFrame() always fail immediately -- this
 * class only ever receives on a half-duplex bus. Actually driving the
 * shared wire (direction switching, echo handling, etc.) is a driver/
 * hardware-level concern outside this class; if that's ever added, it
 * belongs behind the port abstraction below, not in the framing/queueing
 * logic that uses it.
 *
 * All actual UART access goes through portReadable()/portRead()/
 * portWritable()/portWrite() so a future implementation only needs to
 * change those four functions, not the framing/queueing logic that uses
 * them.
 */
class CRSF {
public:
  /// A received, CRC-validated packet plus when its last byte arrived.
  template <typename T> struct ReceivedPacket {
    T packet;
    /// Core::NowNs() timestamp captured when the frame finished arriving.
    int64_t receiveTime = 0;
  };

  /**
   * @param serial Serial port wired to the CRSF bus. Ownership stays with
   * the caller; the reference must outlive this object. Does NOT call
   * serial.begin() -- CRSF objects are typically constructed as globals
   * (before setup() runs), so call begin() yourself once it's safe to touch
   * the hardware (e.g. from taskInit()/setup()).
   * @param duplex Whether the port is a shared single-wire (Half) or
   * separate RX/TX (Full) bus.
   * @param baudRate Baud rate to configure the port with once begin() is
   * called.
   */
  explicit CRSF(HardwareSerial &serial, CRSFDuplex duplex = CRSFDuplex::Full,
               uint32_t baudRate = 420000);

  /// Calls serial.begin(baudRate). Must be called once, after it's safe to
  /// touch the hardware, before any other method.
  void begin();

  // ── TX ──

  /**
   * @brief Serializes `packet` into a complete CRSF wire frame and queues
   * it for (non-blocking) transmission.
   * @return false if the packet is too large, the send buffer is full, or
   * this object is in CRSFDuplex::Half (which only ever receives).
   */
  template <VCTR::packets::IsPacket T> bool write(const T &packet) {
    const size_t numBytes = packet.numBytes();
    if (numBytes == 0 || numBytes > CRSF_MAX_FRAME_LEN) {
      return false;
    }
    uint8_t buffer[CRSF_MAX_FRAME_LEN];
    packet.serialize(buffer);
    return writeFrame(buffer, numBytes);
  }

  /// @returns Core::NowNs() of the most recently transmitted frame, or 0 if
  /// none has been sent yet.
  int64_t getLastSendTime() const { return lastSendTime_; }

  // ── RX ──

  /**
   * @brief Pumps the UART: flushes any pending outgoing bytes and reads +
   * parses any newly arrived bytes into the received-frame queue.
   * @return the number of complete, CRC-valid received frames now queued.
   */
  size_t available();

  /// @returns the CRSF frame-type byte of the next queued received frame,
  /// or 0 if the queue is empty.
  uint8_t peekFrameType() const;

  /// Drops the next queued received frame without decoding it.
  void discardFrame();

  /**
   * @brief If the next queued received frame's type matches T, deserializes
   * it (with its receive timestamp) into `out`, removes it from the queue,
   * and returns true. Otherwise leaves the queue untouched and returns
   * false (so a different packet type can be tried, or the frame can be
   * inspected via peekFrameType()/discardFrame()).
   */
  template <VCTR::packets::IsPacket T> bool read(ReceivedPacket<T> &out) {
    if (rxQueue_.size() == 0) {
      return false;
    }
    T probe;
    if (rxQueue_[0].bytes[2] != static_cast<uint8_t>(probe.getPacketDataType())) {
      return false;
    }
    FrameSlot slot;
    rxQueue_.takeFront(slot);
    if (!out.packet.deserialize(slot.bytes)) {
      return false;
    }
    out.receiveTime = slot.receiveTime;
    return true;
  }

  CRSFDuplex getDuplex() const { return duplex_; }

private:
  static constexpr uint8_t CRSF_MAX_FRAME_LEN = 64;
  static constexpr uint8_t RX_QUEUE_SIZE = 8;
  static constexpr uint8_t IO_CHUNK_SIZE = 16;
  static constexpr uint16_t TX_BUFFER_SIZE = 256;

  struct FrameSlot {
    uint8_t bytes[CRSF_MAX_FRAME_LEN] = {0};
    uint8_t len = 0;
    int64_t receiveTime = 0;
  };

  HardwareSerial &serial_;
  CRSFDuplex duplex_;
  uint32_t baudRate_;

  // ── Port abstraction ──
  size_t portReadable();
  size_t portRead(uint8_t *data, size_t maxLen);
  size_t portWritable();
  size_t portWrite(const uint8_t *data, size_t len);

  // ── Frame parsing ──
  uint8_t rxBuf_[CRSF_MAX_FRAME_LEN];
  uint8_t rxBufPos_ = 0;
  void decodeByte(uint8_t b);
  void drainInput();
  Core::ListBuffer<FrameSlot, RX_QUEUE_SIZE> rxQueue_;

  // ── TX buffering ──
  Core::ListBuffer<uint8_t, TX_BUFFER_SIZE> txBuffer_;
  int64_t lastSendTime_ = 0;
  bool writeFrame(const uint8_t *frame, size_t len);
  void flushTx();

  static uint8_t crc8_d5(const uint8_t *data, size_t length);
};

} // namespace VCTR::ExVectrLink

#endif // EXVECRLINK_CRSF_HPP
