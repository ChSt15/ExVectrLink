#include "CRSF.hpp"

#include <cstring>

namespace VCTR::ExVectrLink {

CRSF::CRSF(HardwareSerial &serial, CRSFDuplex duplex, uint32_t baudRate)
    : serial_(serial), duplex_(duplex), baudRate_(baudRate) {}

void CRSF::begin() { serial_.begin(baudRate_); }

// ─────────────────────────────────────────────────────────────────────────────
// Port abstraction. writeFrame() never reaches portWrite() in Half duplex
// (see below), so these are plain passthroughs -- a future driver-level
// half-duplex implementation (direction pin, etc.) would live here.
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
// CRC-8 / DVB-S2 (polynomial 0xD5, init 0x00). Covers Type + Payload of a
// standard CRSF frame.
// ─────────────────────────────────────────────────────────────────────────────

uint8_t CRSF::crc8_d5(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// RX: byte-stream framing + received-frame queue
// ─────────────────────────────────────────────────────────────────────────────

void CRSF::decodeByte(uint8_t b) {
  if (rxBufPos_ == 0) {
    // Accept any plausible CRSF sync/address byte.
    if (b == 0xC8 || b == 0x00 || b == 0xEC || b == 0xEA || b == 0xEE) {
      rxBuf_[rxBufPos_++] = b;
    }
    return;
  }

  rxBuf_[rxBufPos_++] = b;

  if (rxBufPos_ == 2) {
    // rxBuf_[1] = frame_size; total frame = frame_size + 2 (sync + len).
    if (rxBuf_[1] < 2 || rxBuf_[1] > CRSF_MAX_FRAME_LEN - 2) {
      rxBufPos_ = 0; // invalid length, resync
    }
    return;
  }

  const uint8_t totalLen = rxBuf_[1] + 2;
  if (rxBufPos_ >= totalLen) {
    // Verify CRC over [type .. payload], i.e. bytes [2 .. totalLen-2].
    if (crc8_d5(&rxBuf_[2], rxBuf_[1] - 1) == rxBuf_[totalLen - 1]) {
      FrameSlot slot;
      slot.len = totalLen;
      memcpy(slot.bytes, rxBuf_, totalLen);
      slot.receiveTime = Core::NowNs();
      // Prefer freshest data over oldest if the caller isn't draining fast
      // enough.
      rxQueue_.placeBack(slot, /*overwrite=*/true);
    }
    rxBufPos_ = 0;
  }
}

void CRSF::drainInput() {
  uint8_t buf[IO_CHUNK_SIZE];
  size_t avail;
  while ((avail = portReadable()) > 0) {
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
  if (duplex_ == CRSFDuplex::Half) {
    // Half duplex: this class only receives. Actually driving the shared
    // wire is a driver/hardware-level concern, not handled here.
    return false;
  }
  if (txBuffer_.size() + len > txBuffer_.sizeMax()) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    txBuffer_.placeBack(frame[i]);
  }
  lastSendTime_ = Core::NowNs();
  flushTx();
  return true;
}

void CRSF::flushTx() {
  if (txBuffer_.size() == 0) {
    return;
  }
  const size_t writable = portWritable();
  if (writable == 0) {
    return;
  }

  uint8_t buf[IO_CHUNK_SIZE];
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
  if (written > 0) {
    txBuffer_.removeFront(written);
  }
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
