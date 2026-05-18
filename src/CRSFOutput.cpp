#include "CRSFOutput.hpp"

#ifdef ENABLE_CRSF

#include "ExVectrCore.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"
#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

#include "DatalinkCommandManager.hpp"
#include "DiversitySetup.hpp"
#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "FhssSetup.hpp"
#include "WifiUpload.hpp"

namespace VCTR::ExVectrLink {

using namespace VCTR::packets::radiolink;

namespace {

template <typename TelemetryPacket>
void serializeTelemetryMessage(const TelemetryPacket &packet,
                               VCTR::Core::ListArray<uint8_t> &message) {
  const size_t packetSize = packet.numBytes();
  message.setSize(packetSize + 1);
  packet.serialize(message.getPtr());
  message[packetSize] = (uint8_t)VCTR::ExVectrLink::datalink::PacketTypes::Data;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

CRSFOutput::CRSFOutput()
    : Core::Task_Periodic("CRSFOutput", Core::MILLISECONDS * 10),
      packetManager_([](const VCTR::Core::ListArray<uint8_t> &) {}) {
  Core::getSystemScheduler().addTask(*this);
}

// ─────────────────────────────────────────────────────────────────────────────
// Task lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void CRSFOutput::taskInit() {
  // Configure the serial port for CRSF (420 000 baud, 8N1, not inverted)
  Serial.begin(CRSF_BAUD_RATE);

  // Register with the FHSS receive pipeline.
  // Only forward PacketTypes::Data packets, and strip the type byte so
  // packetManager_ receives clean ExVectrPackets data without corruption.
  datalink::getFhss().addReceiveHandler(
      [this](const VCTR::network::DataPacket &data) {
        if (data.payload.size() == 0)
          return;
        // Last byte is the PacketTypes tag added by DatalinkCommandManager.
        if (data.payload[data.payload.size() - 1] !=
            (uint8_t)datalink::PacketTypes::Data)
          return;
        // Build a view without the type byte.
        VCTR::Core::ListArray<uint8_t> stripped;
        stripped.setSize(data.payload.size() - 1);
        memcpy(stripped.getPtr(), data.payload.getPtr(),
               data.payload.size() - 1);
        packetManager_.receivePacketData(stripped);
      });

  // Packet-specific handlers
  packetManager_
      .addPacketReceiveHandler<VCTR::packets::radiolink::RadioLinkPacket_A4D4>(
          [this](const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &packet) {
            packet_ = packet;
            newRcPacket = true;
            lastPacketTime_ = Core::NowNs();
            // Wake up task immediately so we output the frame ASAP
            this->setDeadline(Core::NowNs());
          });

  packetManager_
      .addPacketReceiveHandler<VCTR::packets::telecoms::Telecommand_UpdateMode>(
          [](const VCTR::packets::telecoms::Telecommand_UpdateMode &) {
            WifiUpload::startWiFiUploadMode();
          });
}

void CRSFOutput::taskCheck() {
  if (serialWriteBuffer_.size() > 0) {
    setDeadline(Core::NowNs());
  }
}

void CRSFOutput::taskThread() {
  // Drain any incoming CRSF frames from the FC (bind command, etc.)
  processIncomingBytes();

  if (telemetryQueue_.size() == 0) {
    queueMessages();
  }

  if (telemetryQueue_.size() > 0 && !datalink::getFhss().isChannelBlocked()) {
    VCTR::Core::ListArray<uint8_t> nextMessage;
    if (telemetryQueue_.takeFront(nextMessage)) {
      network::DataPacket packet;
      packet.payload = nextMessage;
      if (!datalink::getFhss().transmitDataframe(packet)) {
        telemetryQueue_.placeFront(nextMessage);
      }
    }
  }

  if (newRcPacket) {
    writeRcChannelsFrame();
    newRcPacket = false;
  }

  // Send link statistics at a lower cadence (~100 ms)
  if (++linkStatsCounter_ >= LINK_STATS_INTERVAL) {
    linkStatsCounter_ = 0;
    writeLinkStatsFrame();
  }

  flushSerialWriteBuffer();
}

void CRSFOutput::queueMessages() {
  if (pendingBatteryTelemetryData_.size() > 0) {
    if (telemetryQueue_.placeBack(pendingBatteryTelemetryData_)) {
      pendingBatteryTelemetryData_.clear();
    } else {
      return;
    }
  }

  if (pendingGpsTelemetryData_.size() > 0) {
    if (telemetryQueue_.placeBack(pendingGpsTelemetryData_)) {
      pendingGpsTelemetryData_.clear();
    }
  }
}

bool CRSFOutput::queueSerialFrame(const uint8_t *frame, size_t frameSize) {
  if (frameSize > serialWriteBuffer_.sizeMax() - serialWriteBuffer_.size()) {
    return false;
  }

  for (size_t i = 0; i < frameSize; i++) {
    serialWriteBuffer_.placeBack(frame[i]);
  }

  return true;
}

void CRSFOutput::flushSerialWriteBuffer() {
  if (serialWriteBuffer_.size() == 0) {
    return;
  }

  const size_t writable = Serial.availableForWrite();
  if (writable == 0) {
    return;
  }

  uint8_t writeBuffer[SERIAL_WRITE_BUFFER_SIZE];
  size_t chunkSize = serialWriteBuffer_.size();
  if (chunkSize > writable) {
    chunkSize = writable;
  }
  if (chunkSize > sizeof(writeBuffer)) {
    chunkSize = sizeof(writeBuffer);
  }

  for (size_t i = 0; i < chunkSize; i++) {
    writeBuffer[i] = serialWriteBuffer_[i];
  }

  const size_t bytesWritten = Serial.write(writeBuffer, chunkSize);
  if (bytesWritten > 0) {
    serialWriteBuffer_.removeFront(bytesWritten);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC-8 / DVB-S2  (polynomial 0xD5, init 0x00)
// ─────────────────────────────────────────────────────────────────────────────

uint8_t CRSFOutput::crc8_dvb_s2(uint8_t crc, uint8_t byte) {
  crc ^= byte;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

uint8_t CRSFOutput::calculateCRC(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; i++)
    crc = crc8_dvb_s2(crc, data[i]);
  return crc;
}

uint16_t CRSFOutput::readU16BE(const uint8_t *data) {
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

// ─────────────────────────────────────────────────────────────────────────────
// Channel helpers
// ─────────────────────────────────────────────────────────────────────────────

uint16_t CRSFOutput::normToCrsf(int16_t val) {
  // Map from -1000 … 1000  →  172 … 1811  (same range as SBUS)
  return (uint16_t)((float)(val + 1000) / 2000.0f *
                        (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
                    CRSF_CHANNEL_MIN);
}

/**
 * @brief Pack 16 × 11-bit channel values into 22 bytes (LSB-first),
 *        the standard CRSF RC-channels payload encoding.
 */
void CRSFOutput::packChannels(const uint16_t *channels, uint8_t *payload) {
  memset(payload, 0, 22);

  uint32_t bitOffset = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    const uint32_t val = channels[ch] & 0x7FF;
    const uint32_t byteIdx = bitOffset / 8;
    const uint8_t bitIdx = bitOffset % 8;

    payload[byteIdx] |= (uint8_t)(val << bitIdx);
    payload[byteIdx + 1] |= (uint8_t)(val >> (8 - bitIdx));
    if (bitIdx > 5)
      payload[byteIdx + 2] |= (uint8_t)(val >> (16 - bitIdx));

    bitOffset += 11;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame builders
// ─────────────────────────────────────────────────────────────────────────────

void CRSFOutput::writeRcChannelsFrame() {
  packetFilter_.updateFromPacket(packet_, lastPacketTime_);
  const auto &f = packetFilter_.getFilteredPacket();

  uint16_t channels[16];
  // Default every channel to centre
  for (uint8_t i = 0; i < 16; i++)
    channels[i] = CRSF_CHANNEL_MID;

  // Analogue channels (same ordering as SBusTx)
  channels[0] = normToCrsf(f.analogChannels[0]); // Roll
  channels[1] = normToCrsf(f.analogChannels[1]); // Pitch
  channels[2] = normToCrsf(f.analogChannels[3]); // Throttle
  channels[3] = normToCrsf(f.analogChannels[2]); // Yaw

  // Digital channels (same mapping as SBusTx)
  channels[4] =
      (float)f.digitalChannels[0] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;
  channels[5] =
      (float)f.digitalChannels[1] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;
  channels[6] =
      (float)f.digitalChannels[2] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;
  channels[7] =
      (float)f.digitalChannels[3] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;

  // ── Build the 26-byte CRSF frame ──
  // [sync 0xC8] [len 24] [type 0x16] [payload 22B] [crc]
  uint8_t frame[26];
  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = 24; // type(1) + payload(22) + crc(1)
  frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
  packChannels(channels, &frame[3]);
  frame[25] = calculateCRC(&frame[2], 23); // CRC over type + payload

  queueSerialFrame(frame, sizeof(frame));
}

void CRSFOutput::writeLinkStatsFrame() {
  auto &diversity = datalink::getSx1280Diversity();
  const auto bestIdx = diversity.getCurrentBestLinkIndex();
  const auto *bestLink = diversity.getDiversityLink(bestIdx);

  // Always use local radio readings for uplink stats.
  const int16_t rssiRaw = bestLink ? bestLink->lastPacketRSSI() : 0;
  const int16_t snrRaw = bestLink ? bestLink->lastPacketSNR() : 0;
  const float lq = datalink::getFhss().getLinkQuality(); // 0.0 – 1.0

  const uint8_t uplinkRssi = (rssiRaw < 0) ? (uint8_t)(-rssiRaw) : (uint8_t)0;

  // TX power: use remote (TX side) power so the FC sees what the transmitter
  // is doing. Fall back to local power if no OTA telemetry received yet.
  const auto &remote = datalink::getRemoteLinkStats();
  // txPower in dBm
  const int8_t reportedTxDbm = remote.txPowerdBm;

  // Index power order is:
  // 0mW, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW.
  uint8_t txPowerIdx;
  if (reportedTxDbm <= 0) {
    txPowerIdx = 0; // 0 mW
  } else if (reportedTxDbm <= 10) {
    txPowerIdx = 1; // 10 mW
  } else if (reportedTxDbm <= 14) {
    txPowerIdx = 2; // 25 mW
  } else if (reportedTxDbm <= 17) {
    txPowerIdx = 8; // 50 mW
  } else if (reportedTxDbm <= 20) {
    txPowerIdx = 3; // 100 mW
  } else if (reportedTxDbm <= 24) {
    txPowerIdx = 7; // 250 mW
  } else if (reportedTxDbm <= 27) {
    txPowerIdx = 4; // 500 mW
  } else if (reportedTxDbm <= 30) {
    txPowerIdx = 5; // 1000 mW
  } else if (reportedTxDbm <= 33) {
    txPowerIdx = 6; // 2000 mW
  } else {
    txPowerIdx = 0;
  }

  // ── Build the 14-byte CRSF link-statistics frame ──
  // [sync 0xC8] [len 12] [type 0x14] [payload 10B] [crc]
  uint8_t frame[14];
  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = 12; // type(1) + payload(10) + crc(1)
  frame[2] = CRSF_FRAMETYPE_LINK_STATISTICS;

  frame[3] = uplinkRssi;              // uplink RSSI ant 1 (positive magnitude)
  frame[4] = 0;                       // uplink RSSI ant 2 (N/A single-ant)
  frame[5] = (uint8_t)(lq * 100.0f);  // uplink link quality 0-100 %
  frame[6] = (uint8_t)(int8_t)snrRaw; // uplink SNR (signed)
  frame[7] = (uint8_t)bestIdx;        // active antenna index
  frame[8] = 4;                       // RF mode (generic)
  frame[9] = txPowerIdx;              // uplink TX power index (remote TX side)
  frame[10] = 0;                      // downlink RSSI (N/A on RX side)
  frame[11] = 0;                      // downlink link quality (N/A on RX side)
  frame[12] = 0;                      // downlink SNR (N/A on RX side)

  frame[13] = calculateCRC(&frame[2], 11); // CRC over type + payload

  queueSerialFrame(frame, sizeof(frame));
}

// ─────────────────────────────────────────────────────────────────────────────
// Incoming CRSF frame parser (non-blocking, handles FC → RX frames)
// ─────────────────────────────────────────────────────────────────────────────

void CRSFOutput::processIncomingBytes() {
  // Read at most ~64 bytes per call to stay well under 1 ms
  uint8_t budget = CRSF_MAX_FRAME_LEN;
  while (Serial.available() > 0 && budget-- > 0) {
    uint8_t b = Serial.read();

    if (rxBufPos_ == 0) {
      // Expect a valid CRSF address/sync byte
      if (b == 0xC8 || b == 0xEC || b == 0xEA) {
        rxBuf_[rxBufPos_++] = b;
      }
      continue;
    }

    rxBuf_[rxBufPos_++] = b;

    if (rxBufPos_ == 2) {
      // frame[1] = frame_size; total = frame_size + 2
      if (rxBuf_[1] < 2 || rxBuf_[1] > CRSF_MAX_FRAME_LEN - 2) {
        rxBufPos_ = 0; // invalid length, reset
      }
      continue;
    }

    // Check if we have a complete frame
    const uint8_t totalLen = rxBuf_[1] + 2; // sync + len + payload
    if (rxBufPos_ >= totalLen) {
      // Verify CRC (over type + payload, i.e. bytes [2 .. totalLen-2])
      uint8_t crc = calculateCRC(&rxBuf_[2], rxBuf_[1] - 1);
      if (crc == rxBuf_[totalLen - 1]) {
        handleCrsfFrame(rxBuf_, totalLen);
        switch (rxBuf_[2]) {
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
          forwardBatteryTelemetry(rxBuf_, totalLen);
          break;
        case CRSF_FRAMETYPE_GPS:
          forwardGpsTelemetry(rxBuf_, totalLen);
          break;
        default:
          break;
        }
      }
      rxBufPos_ = 0;
    }
  }
}

void CRSFOutput::forwardBatteryTelemetry(const uint8_t *frame, uint8_t len) {
  if (len < 12) {
    return;
  }

  const uint8_t *payload = &frame[3];
  VCTR::packets::telecoms::Telemetry_Battery telemetry;

  // CRSF battery sensor: voltage in 100 mV units (0.1 V), current in 100 mA
  // units (0.1 A)
  telemetry.voltage = 0.1f * (float)readU16BE(payload);
  telemetry.current = 0.1f * (float)readU16BE(payload + 2);
  serializeTelemetryMessage(telemetry, pendingBatteryTelemetryData_);
}

void CRSFOutput::forwardGpsTelemetry(const uint8_t *frame, uint8_t len) {
  if (len < 19) {
    return;
  }

  // CRSF GPS payload layout (all big-endian):
  //  [0..3]  latitude      int32  degrees * 1e7  (skipped, too large to
  //  forward) [4..7]  longitude     int32  degrees * 1e7  (skipped) [8..9]
  //  speed         uint16 km/h [10..11] ground_course uint16 degrees * 100
  //  (skipped) [12..13] altitude     uint16 metres + 1000 offset [14]
  //  satellites    uint8
  const uint8_t *payload = &frame[3];
  VCTR::packets::telecoms::Telemetry_GNSS telemetry;

  telemetry.speed = readU16BE(payload + 8);
  telemetry.satellites = payload[14];
  serializeTelemetryMessage(telemetry, pendingGpsTelemetryData_);
}

void CRSFOutput::handleCrsfFrame(const uint8_t *frame, uint8_t len) {
  const uint8_t type = frame[2];

  if (type == CRSF_FRAMETYPE_COMMAND && len >= 7) {
    // Extended-header bind command (sent by BetaFlight "Bind" button)
    // frame[3]=dest, frame[4]=orig, frame[5]=cmd, frame[6]=subcmd
    const uint8_t cmd = frame[5];
    const uint8_t subcmd = frame[6];
    if (cmd == CRSF_COMMAND_SUBCMD_RX &&
        subcmd == CRSF_COMMAND_SUBCMD_RX_BIND) {
      WifiUpload::startWiFiUploadMode();
    }
  }
}

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF
