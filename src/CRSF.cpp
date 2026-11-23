#include "CRSF.hpp"

#include <cstring>

#if defined(ESP32)
#include "driver/gpio.h"
#include "rom/gpio.h" // gpio_matrix_in()/gpio_matrix_out()
#include "soc/uart_periph.h"
#endif

namespace VCTR::ExVectrLink {

CRSF::CRSF(HardwareSerial &serial, CRSFDuplex duplex, uint32_t baudRate,
          int8_t halfDuplexPin, uint8_t uartNum, bool invert)
    : serial_(serial), duplex_(duplex), baudRate_(baudRate),
      halfDuplexPin_(halfDuplexPin), uartNum_(uartNum), invert_(invert) {}

void CRSF::begin() { serial_.begin(baudRate_); }

void CRSF::beginHalfDuplex() {
  if (duplex_ != CRSFDuplex::Half) {
    return;
  }
  switchToRx();
}

// ─────────────────────────────────────────────────────────────────────────────
// Port abstraction. These are plain passthroughs to the HardwareSerial --
// direction switching for Half duplex lives in switchToRx()/switchToTx()
// below, kept separate since it operates on the GPIO matrix, not the byte
// stream.
// ─────────────────────────────────────────────────────────────────────────────

size_t CRSF::portReadable() { return serial_.available(); }

size_t CRSF::portRead(uint8_t *data, size_t maxLen) {
  // Caller always bounds maxLen by portReadable(), so these bytes are
  // already buffered and readBytes() returns immediately without hitting
  // its timeout.
  return serial_.readBytes(data, maxLen);
}

size_t CRSF::portWritable() { return serial_.availableForWrite(); }

size_t CRSF::portWrite(const uint8_t *data, size_t len) {
  return serial_.write(data, len);
}

// ─────────────────────────────────────────────────────────────────────────────
// Half-duplex direction switching (ESP32 GPIO matrix).
//
// Mirrors ExpressLRS's CRSFHandset::duplex_set_RX()/duplex_set_TX(): the
// UART peripheral itself is never reconfigured (no re-begin()), only which
// GPIO pin its RX/TX signals are routed to via the matrix, so switching is
// just a handful of register writes. Inversion is handled entirely here
// (not via HardwareSerial's own invert flag, which the caller must leave
// false for the shared pin -- see beginHalfDuplex()'s doc comment), since
// re-deriving it per direction is what makes repeated switching reliable.
// ─────────────────────────────────────────────────────────────────────────────

#if defined(ESP32)

void CRSF::switchToRx() {
  if (duplex_ != CRSFDuplex::Half || halfDuplexPin_ < 0) {
    return;
  }
  const auto pin = (gpio_num_t)halfDuplexPin_;
  const uint32_t rxSignal = UART_PERIPH_SIGNAL(uartNum_, SOC_UART_RX_PIN_IDX);
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  if (invert_) {
    gpio_matrix_in(pin, rxSignal, true);
    gpio_pulldown_en(pin);
    gpio_pullup_dis(pin);
  } else {
    gpio_matrix_in(pin, rxSignal, false);
    gpio_pullup_en(pin);
    gpio_pulldown_dis(pin);
  }
}

void CRSF::switchToTx() {
  if (duplex_ != CRSFDuplex::Half || halfDuplexPin_ < 0) {
    return;
  }
  const auto pin = (gpio_num_t)halfDuplexPin_;
  const uint32_t txSignal = UART_PERIPH_SIGNAL(uartNum_, SOC_UART_TX_PIN_IDX);
  const uint32_t rxSignal = UART_PERIPH_SIGNAL(uartNum_, SOC_UART_RX_PIN_IDX);
  // Route the RX input matrix slot to a constant level instead of the
  // shared pin so our own transmission is never looped back in as bogus
  // received bytes while we're driving the line.
  constexpr uint32_t MATRIX_CONST_LOW = 0x30;
  constexpr uint32_t MATRIX_CONST_HIGH = 0x38;
  gpio_set_pull_mode(pin, GPIO_FLOATING);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  if (invert_) {
    gpio_set_level(pin, 0); // idle level while the matrix attaches
    gpio_matrix_in(MATRIX_CONST_LOW, rxSignal, false);
    gpio_matrix_out(pin, txSignal, true, false);
  } else {
    gpio_set_level(pin, 1);
    gpio_matrix_in(MATRIX_CONST_HIGH, rxSignal, false);
    gpio_matrix_out(pin, txSignal, false, false);
  }
}

#else

void CRSF::switchToRx() {}
void CRSF::switchToTx() {}

#endif

// ─────────────────────────────────────────────────────────────────────────────
// CRC-8 / DVB-S2 (polynomial 0xD5, init 0x00). Covers Type + Payload of a
// standard CRSF frame.
// ─────────────────────────────────────────────────────────────────────────────

uint8_t CRSF::crc8_d5(const uint8_t *data, size_t length) { return CrsfParser::crc8(data, length); }

// ─────────────────────────────────────────────────────────────────────────────
// RX: byte-stream framing + received-frame queue
// ─────────────────────────────────────────────────────────────────────────────

void CRSF::decodeByte(uint8_t b) {
  parser_.feed(b, [this](const uint8_t *bytes, size_t totalLen) {
    FrameSlot slot;
    slot.len = static_cast<uint8_t>(totalLen);
    memcpy(slot.bytes, bytes, totalLen);
    slot.receiveTime = Core::NowNs();
    // Prefer freshest data over oldest if the caller isn't draining fast
    // enough.
    rxQueue_.placeBack(slot, /*overwrite=*/true);
    if (duplex_ == CRSFDuplex::Half) {
      // The bus master (RC) has just finished driving the line and is
      // now listening for exactly one reply -- if one is queued, this is
      // the only moment to send it.
      sendHalfDuplexReply();
    }
  });
}

void CRSF::drainInput() {
  uint8_t buf[IO_CHUNK_SIZE];
  size_t avail;
  while ((avail = portReadable()) > 0) {
    if (parser_.hardened()) {
      // A half-received frame with no byte behind it for kStalePartialNs cannot complete (a frame takes <1 ms on
      // the wire): drop it so it cannot swallow the start of the next one. Hardened (network TX) mode only.
      const int64_t now = Core::NowNs();
      if (parser_.hasPartial() && now - lastByteNs_ > kStalePartialNs) parser_.resetPartial();
      lastByteNs_ = now;
    }
    const size_t chunk = avail < sizeof(buf) ? avail : sizeof(buf);
    const size_t n = portRead(buf, chunk);
    for (size_t i = 0; i < n; i++) {
      decodeByte(buf[i]);
    }
  }
}

uint8_t CRSF::peekFrameType() const {
  if (rxQueue_.size() == 0) {
    return 0;
  }
  return rxQueue_[0].bytes[2];
}

void CRSF::discardFrame() {
  FrameSlot slot;
  rxQueue_.takeFront(slot);
}

// ─────────────────────────────────────────────────────────────────────────────
// TX: outgoing byte buffer
// ─────────────────────────────────────────────────────────────────────────────

bool CRSF::writeFrame(const uint8_t *frame, size_t len) {
  if (len == 0 || len > txBuffer_.sizeMax()) {
    return false;
  }
  if (duplex_ == CRSFDuplex::Half) {
    if (txBuffer_.size() > 0) halfRepliesReplaced_++;
    // Only one reply is ever in flight -- the bus master expects exactly
    // one frame back, sent the instant its own frame is fully received
    // (see sendHalfDuplexReply()). Replace any stale not-yet-sent frame
    // with the freshest one rather than appending, mirroring rxQueue_'s
    // overwrite-newest policy.
    txBuffer_.clear();
  } else if (txBuffer_.size() + len > txBuffer_.sizeMax()) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    txBuffer_.placeBack(frame[i]);
  }
  if (duplex_ == CRSFDuplex::Full) {
    // Half duplex only updates lastSendTime_ once the frame is actually
    // transmitted -- see sendHalfDuplexReply().
    lastSendTime_ = Core::NowNs();
  }
  flushTx();
  return true;
}

void CRSF::flushTx() {
  if (duplex_ == CRSFDuplex::Half) {
    // Half duplex TX only ever happens synchronously from
    // sendHalfDuplexReply(), immediately after receiving a frame -- the
    // shared pin isn't even routed to the UART's TX signal otherwise, so
    // writing here would go nowhere.
    return;
  }

  // Drain the outgoing buffer to the UART, but never hand portWrite() more
  // than the port can currently accept -- serial_.write() only blocks if
  // asked to write past availableForWrite(), so bounding every chunk by it
  // keeps this strictly non-blocking. Loop so a whole frame goes out in a
  // single call whenever there is room (rather than one IO_CHUNK_SIZE slice
  // per call); the loop is naturally bounded because each write shrinks the
  // port's free space until it hits 0, or the buffer empties.
  uint8_t buf[IO_CHUNK_SIZE];
  while (txBuffer_.size() > 0) {
    const size_t writable = portWritable();
    if (writable == 0) {
      break; // No room right now; the remainder flushes on the next call.
    }
    size_t chunk = txBuffer_.size();
    if (chunk > writable) {
      chunk = writable;
    }
    if (chunk > sizeof(buf)) {
      chunk = sizeof(buf);
    }
    for (size_t i = 0; i < chunk; i++) {
      buf[i] = txBuffer_[i];
    }
    const size_t written = portWrite(buf, chunk);
    if (written == 0) {
      break; // Defensive: don't spin if the port accepted nothing.
    }
    txBuffer_.removeFront(written);
  }
}

void CRSF::sendHalfDuplexReply() {
  if (txBuffer_.size() == 0) {
    return; // Nothing queued -- stay in RX, as required by the protocol.
  }

  uint8_t buf[CRSF_MAX_FRAME_LEN];
  size_t n = txBuffer_.size();
  if (n > sizeof(buf)) {
    n = sizeof(buf); // Defensive; writeFrame() never queues more than this.
  }
  for (size_t i = 0; i < n; i++) {
    buf[i] = txBuffer_[i];
  }
  txBuffer_.removeFront(n);

  halfRepliesSent_++;
  switchToTx();
  size_t offset = 0;
  while (offset < n) {
    const size_t written = portWrite(buf + offset, n - offset);
    if (written == 0) {
      break; // Defensive: don't spin if the port accepted nothing.
    }
    offset += written;
  }
  // Block until the frame has physically left the wire -- bounded to a
  // single (<=64 byte) frame's transmit time -- before releasing the line
  // back to the handset's own driver. The reply must be complete before
  // switching back to high-impedance RX, so this can't be deferred to a
  // later, non-blocking call the way Full duplex's flushTx() is.
  serial_.flush();
  lastSendTime_ = Core::NowNs();
  switchToRx();
}

// ─────────────────────────────────────────────────────────────────────────────
// Public pump
// ─────────────────────────────────────────────────────────────────────────────

size_t CRSF::available() {
  flushTx();
  drainInput();
  return rxQueue_.size();
}

} // namespace VCTR::ExVectrLink
