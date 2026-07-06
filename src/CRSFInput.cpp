#include "CRSFInput.hpp"

#ifdef ENABLE_CRSF_INPUT

#include <cstring>

namespace VCTR::ExVectrLink {

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

CRSFInput::CRSFInput(HardwareSerial &serial, int8_t rxPin, int8_t txPin,
                     uint32_t baudRate, bool invert)
    : Core::Task_Periodic("CRSFInput", TASK_INTERVAL_NS), serial_(serial),
      rxPin_(rxPin), txPin_(txPin), baudRate_(baudRate), invert_(invert) {
  Core::getSystemScheduler().addTask(*this);
}

// ─────────────────────────────────────────────────────────────────────────────
// Task lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void CRSFInput::taskInit() {
#if defined(ESP32)
  if (rxPin_ >= 0 && txPin_ >= 0 && rxPin_ == txPin_) {
    // Single shared half-duplex signal wire (e.g. a "nano" module bay's
    // S.Port-style pin). UART_MODE_RS485_HALF_DUPLEX turned out to gate
    // the receiver on our own transmit activity (its auto-RTS logic is
    // meant to drive an external transceiver's DE/RE pins, not to
    // tri-state our own pad) -- since we transmit telemetry rarely/never,
    // that left the receiver effectively permanently disabled.
    //
    // Instead, only attach the pad as UART RX (leave TX unmapped/floating)
    // so we purely listen and never drive the shared line ourselves. This
    // sacrifices telemetry transmission on this pin for now (queued
    // telemetry frames are simply never written to the wire -- see
    // flushSerialWriteBuffer()); proper bidirectional support would need
    // manual GPIO-matrix direction switching around each transmission,
    // the way ExpressLRS's CRSFHandset does it.
    serial_.begin(baudRate_, SERIAL_8N1, rxPin_, -1, invert_);
  } else if (rxPin_ >= 0 && txPin_ >= 0) {
    serial_.begin(baudRate_, SERIAL_8N1, rxPin_, txPin_, invert_);
  } else {
    serial_.begin(baudRate_);
  }
#else
  serial_.begin(baudRate_);
#endif
}

void CRSFInput::taskCheck() {
  // Wake up early to flush queued telemetry as soon as possible.
  if (serialWriteBuffer_.size() > 0) {
    setDeadline(Core::NowNs());
  }
}

void CRSFInput::taskThread() {
  processIncomingBytes();
  flushSerialWriteBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC-8 / DVB-S2  (polynomial 0xD5, init 0x00)
// ─────────────────────────────────────────────────────────────────────────────

uint8_t CRSFInput::crc8_dvb_s2(uint8_t crc, uint8_t byte) {
  crc ^= byte;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

uint8_t CRSFInput::calculateCRC(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; i++)
    crc = crc8_dvb_s2(crc, data[i]);
  return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Channel helpers
// ─────────────────────────────────────────────────────────────────────────────

int16_t CRSFInput::crsfToNorm(uint16_t val) {
  // Map from 172 … 1811  →  -1000 … 1000  (inverse of CRSFOutput::normToCrsf)
  float norm = ((float)(val - CRSF_CHANNEL_MIN) /
                (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) *
                   2000.0f -
               1000.0f;
  if (norm > 1000.0f)
    norm = 1000.0f;
  if (norm < -1000.0f)
    norm = -1000.0f;
  return (int16_t)norm;
}

/**
 * @brief Unpack 16 × 11-bit channel values (LSB-first) from the standard
 *        22-byte CRSF RC-channels payload. Inverse of the packing scheme
 *        used by CRSFOutput::packChannels.
 */
void CRSFInput::unpackChannels(const uint8_t *payload, uint16_t *channels) {
  uint32_t bitOffset = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    const uint32_t byteIdx = bitOffset / 8;
    const uint8_t bitIdx = bitOffset % 8;

    uint32_t val = ((uint32_t)payload[byteIdx] >> bitIdx) |
                   ((uint32_t)payload[byteIdx + 1] << (8 - bitIdx));
    if (bitIdx > 5) {
      val |= (uint32_t)payload[byteIdx + 2] << (16 - bitIdx);
    }

    channels[ch] = (uint16_t)(val & 0x7FF);
    bitOffset += 11;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Incoming CRSF frame parser (non-blocking)
// ─────────────────────────────────────────────────────────────────────────────

void CRSFInput::processIncomingBytes() {
  // Read at most one full frame's worth of bytes per call to stay non-blocking.
  uint8_t budget = CRSF_MAX_FRAME_LEN;
  while (serial_.available() > 0 && budget-- > 0) {
    uint8_t b = serial_.read();
    rxByteCount_++;
    rawByteSnapshot_[rawByteSnapshotPos_] = b;
    rawByteSnapshotPos_ =
        (uint8_t)((rawByteSnapshotPos_ + 1) % RAW_BYTE_SNAPSHOT_LEN);

    if (rxBufPos_ == 0) {
      // Expect a valid CRSF address/sync byte. RC_CHANNELS_PACKED frames
      // sent by a handset's module bay conventionally use either
      // CRSF_SYNC_BYTE or CRSF_ADDRESS_CRSF_TRANSMITTER; the receiver/radio
      // addresses are also accepted so extended-header frames addressed
      // directly to us are not dropped.
      if (b == CRSF_SYNC_BYTE || b == CRSF_ADDRESS_CRSF_RECEIVER ||
          b == CRSF_ADDRESS_RADIO_TRANSMITTER ||
          b == CRSF_ADDRESS_CRSF_TRANSMITTER) {
        rxBuf_[rxBufPos_++] = b;
      } else {
        resyncByteCount_++;
      }
      continue;
    }

    rxBuf_[rxBufPos_++] = b;

    if (rxBufPos_ == 2) {
      // frame[1] = frame_size; total = frame_size + 2 (sync + len)
      if (rxBuf_[1] < 2 || rxBuf_[1] > CRSF_MAX_FRAME_LEN - 2) {
        resyncByteCount_ += rxBufPos_;
        rxBufPos_ = 0; // invalid length, reset and resync
      }
      continue;
    }

    const uint8_t totalLen = rxBuf_[1] + 2;
    if (rxBufPos_ >= totalLen) {
      // Verify CRC (over type + payload, i.e. bytes [2 .. totalLen-2])
      const uint8_t crc = calculateCRC(&rxBuf_[2], rxBuf_[1] - 1);
      if (crc == rxBuf_[totalLen - 1]) {
        goodFrameCount_++;
        lastFrameType_ = rxBuf_[2];
        handleFrame(rxBuf_, totalLen);
      } else {
        crcErrorCount_++;
        resyncByteCount_ += totalLen;
      }
      rxBufPos_ = 0;
    }
  }
}

void CRSFInput::handleFrame(const uint8_t *frame, uint8_t len) {
  const uint8_t type = frame[2];
  const uint8_t *payload = &frame[3];
  const uint8_t payloadLen = (uint8_t)(len - 4); // sync, len, type, crc

  if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
      payloadLen >= CRSF_RC_CHANNELS_PAYLOAD_SIZE) {
    decodeRcChannelsPacked(payload);
  }

  // Let anything else (or everything, if a caller wants a second look) be
  // handled generically -- new frame types never require touching this file.
  rawFrameHandlers_.callHandlers(type, payload, payloadLen);
}

void CRSFInput::decodeRcChannelsPacked(const uint8_t *payload) {
  uint16_t channels[16];
  unpackChannels(payload, channels);

  VCTR::packets::radiolink::RadioLinkPacket_A4D4 packet;

  // Analogue channels (inverse of CRSFOutput::writeRcChannelsFrame mapping)
  packet.analogChannels[0] = crsfToNorm(channels[0]); // Roll
  packet.analogChannels[1] = crsfToNorm(channels[1]); // Pitch
  packet.analogChannels[3] = crsfToNorm(channels[2]); // Throttle
  packet.analogChannels[2] = crsfToNorm(channels[3]); // Yaw

  // Digital channels (inverse of CRSFOutput's mapping)
  auto digitalFromCrsf = [](uint16_t val) -> uint8_t {
    float norm = ((float)(val - CRSF_CHANNEL_MIN) /
                  (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) *
                 2.0f;
    if (norm < 0.0f)
      norm = 0.0f;
    if (norm > 2.0f)
      norm = 2.0f;
    return (uint8_t)(norm + 0.5f);
  };

  packet.digitalChannels[0] = digitalFromCrsf(channels[4]);
  packet.digitalChannels[1] = digitalFromCrsf(channels[5]);
  packet.digitalChannels[2] = digitalFromCrsf(channels[6]);
  packet.digitalChannels[3] = digitalFromCrsf(channels[7]);

  lastRcPacket_ = packet;
  lastRcPacketTime_ = Core::NowNs();

  rcChannelHandlers_.callHandlers(lastRcPacket_);
}

// ─────────────────────────────────────────────────────────────────────────────
// RC channel reception (public API)
// ─────────────────────────────────────────────────────────────────────────────

void CRSFInput::addRcChannelsHandler(
    std::function<void(const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &)>
        handler) {
  rcChannelHandlers_.addHandler(handler);
}

void CRSFInput::addRawFrameHandler(
    std::function<void(uint8_t, const uint8_t *, uint8_t)> handler) {
  rawFrameHandlers_.addHandler(handler);
}

const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &
CRSFInput::getLastRcPacket() const {
  return lastRcPacket_;
}

int64_t CRSFInput::getLastRcPacketTime() const { return lastRcPacketTime_; }

bool CRSFInput::isReceiving() const {
  return lastRcPacketTime_ != 0 &&
         (Core::NowNs() - lastRcPacketTime_) < FAILSAFE_TIMEOUT_NS;
}

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry transmission (public API)
// ─────────────────────────────────────────────────────────────────────────────

bool CRSFInput::queueTelemetryFrame(uint8_t frameType, const uint8_t *payload,
                                    uint8_t payloadLen) {
  if (payloadLen > CRSF_MAX_FRAME_LEN - 4) {
    return false;
  }

  uint8_t frame[CRSF_MAX_FRAME_LEN];
  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = (uint8_t)(payloadLen + 2); // type(1) + payload + crc(1)
  frame[2] = frameType;
  memcpy(&frame[3], payload, payloadLen);
  frame[3 + payloadLen] = calculateCRC(&frame[2], payloadLen + 1);

  return queueSerialFrame(frame, (size_t)payloadLen + 4);
}

bool CRSFInput::sendBatteryTelemetry(float voltageV, float currentA,
                                     uint32_t capacityMah,
                                     uint8_t percentRemaining) {
  // CRSF_FRAMETYPE_BATTERY_SENSOR payload layout (all big-endian):
  //  [0..1] voltage   uint16  0.1 V units
  //  [2..3] current   uint16  0.1 A units
  //  [4..6] capacity  uint24  mAh
  //  [7]    remaining uint8   %
  uint8_t payload[8];
  const uint16_t voltageRaw = (uint16_t)(voltageV * 10.0f + 0.5f);
  const uint16_t currentRaw = (uint16_t)(currentA * 10.0f + 0.5f);

  payload[0] = (uint8_t)(voltageRaw >> 8);
  payload[1] = (uint8_t)(voltageRaw & 0xFF);
  payload[2] = (uint8_t)(currentRaw >> 8);
  payload[3] = (uint8_t)(currentRaw & 0xFF);
  payload[4] = (uint8_t)((capacityMah >> 16) & 0xFF);
  payload[5] = (uint8_t)((capacityMah >> 8) & 0xFF);
  payload[6] = (uint8_t)(capacityMah & 0xFF);
  payload[7] = percentRemaining;

  return queueTelemetryFrame(CRSF_FRAMETYPE_BATTERY_SENSOR, payload,
                             sizeof(payload));
}

// ─────────────────────────────────────────────────────────────────────────────
// Serial write buffering
// ─────────────────────────────────────────────────────────────────────────────

bool CRSFInput::queueSerialFrame(const uint8_t *frame, size_t frameSize) {
  if (frameSize > serialWriteBuffer_.sizeMax() - serialWriteBuffer_.size()) {
    return false;
  }

  for (size_t i = 0; i < frameSize; i++) {
    serialWriteBuffer_.placeBack(frame[i]);
  }

  return true;
}

void CRSFInput::flushSerialWriteBuffer() {
  if (serialWriteBuffer_.size() == 0) {
    return;
  }

  const size_t writable = serial_.availableForWrite();
  if (writable == 0) {
    return;
  }

  uint8_t writeBuffer[SERIAL_WRITE_MAX_CHUNK];
  size_t chunkSize = serialWriteBuffer_.size();
  if (chunkSize > writable) {
    chunkSize = writable;
  }
  if (chunkSize > SERIAL_WRITE_MAX_CHUNK) {
    chunkSize = SERIAL_WRITE_MAX_CHUNK;
  }

  for (size_t i = 0; i < chunkSize; i++) {
    writeBuffer[i] = serialWriteBuffer_[i];
  }

  const size_t bytesWritten = serial_.write(writeBuffer, chunkSize);
  if (bytesWritten > 0) {
    serialWriteBuffer_.removeFront(bytesWritten);
  }
}

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF_INPUT
