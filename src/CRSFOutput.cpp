#include "CRSFOutput.hpp"

#ifdef ENABLE_CRSF

#include "ExVectrCore.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"
#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

#include "DatalinkCommandManager.hpp"
#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "FhssSetup.hpp"
#include "WifiUpload.hpp"

namespace VCTR::ExVectrLink {

using namespace VCTR::packets::radiolink;

namespace {

const VCTR::network::datalink::Sx1280_DirectI &getLinkStatsRadio() {
#if defined(HAS_DIVERSITY)
  return VCTR::ExVectrLink::datalink::getSx1280Diversity();
#else
  return VCTR::ExVectrLink::datalink::getDatalinkRadioA();
#endif
}

template <typename TelemetryPacket>
void serializeTelemetryMessage(const TelemetryPacket &packet,
                               VCTR::Core::ListArray<uint8_t> &message) {
  const size_t packetSize = packet.numBytes();
  message.setSize(packetSize + 1);
  packet.serialize(message.getPtr());
  message[packetSize] = (uint8_t)VCTR::ExVectrLink::datalink::PacketTypes::Data;
}

uint16_t rcPacketRateHz = 0;
uint16_t rcPacketsThisWindow = 0;
uint16_t crsfTxFailureRateHz = 0;
uint16_t crsfTxFailuresThisWindow = 0;
int64_t crsfStatsWindowStartNs = 0;

void updateCrsfStatsWindow(int64_t nowNs) {
  if (crsfStatsWindowStartNs == 0) {
    crsfStatsWindowStartNs = nowNs;
    return;
  }

  while (nowNs - crsfStatsWindowStartNs >= Core::SECONDS) {
    rcPacketRateHz = rcPacketsThisWindow;
    rcPacketsThisWindow = 0;
    crsfTxFailureRateHz = crsfTxFailuresThisWindow;
    crsfTxFailuresThisWindow = 0;
    crsfStatsWindowStartNs += Core::SECONDS;
  }
}

void noteRcPacketReceived(int64_t nowNs) {
  updateCrsfStatsWindow(nowNs);
  if (rcPacketsThisWindow < 0xFFFFu) {
    rcPacketsThisWindow++;
  }
}

void noteCrsfTxFailure(int64_t nowNs) {
  updateCrsfStatsWindow(nowNs);
  if (crsfTxFailuresThisWindow < 0xFFFFu) {
    crsfTxFailuresThisWindow++;
  }
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
            const int64_t nowNs = Core::NowNs();
            packet_ = packet;
            newRcPacket = true;
            lastPacketTime_ = nowNs;
            noteRcPacketReceived(nowNs);
            // Wake up task immediately so we output the frame ASAP
            this->setDeadline(nowNs);
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
  const int64_t nowNs = Core::NowNs();
  updateCrsfStatsWindow(nowNs);

  // Hard budget: this task must not block the scheduler for more than 500 µs
  // so that FHSS can wake up on time.  Individual operations are normally
  // well under that limit; the guard exists to cap any unexpected stall from
  // USB-CDC mutex contention or similar platform overhead.
  const int64_t budgetEnd = nowNs + 500 * Core::MICROSECONDS;

  // Drain any incoming CRSF frames from the FC (bind command, etc.)
  processIncomingBytes();

  if (Core::NowNs() < budgetEnd && telemetryQueue_.size() == 0) {
    queueMessages();
  }

  if (Core::NowNs() < budgetEnd && telemetryQueue_.size() > 0 &&
      !datalink::getFhss().isChannelBlocked()) {
    VCTR::Core::ListArray<uint8_t> nextMessage;
    if (telemetryQueue_.takeFront(nextMessage)) {
      network::DataPacket packet;
      packet.payload = nextMessage;
      if (!datalink::getFhss().transmitDataframe(packet)) {
        telemetryQueue_.placeFront(nextMessage);
      }
    }
  }

  if (Core::NowNs() < budgetEnd && newRcPacket) {
    writeRcChannelsFrame();
    newRcPacket = false;
  }

  // Send link statistics at a lower cadence (~100 ms)
  if (Core::NowNs() < budgetEnd) {
    if (++linkStatsCounter_ >= LINK_STATS_INTERVAL) {
      linkStatsCounter_ = 0;
      writeLinkStatsFrame();
    }
  }

  if (Core::NowNs() < budgetEnd) {
    flushSerialWriteBuffer();
  }
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
    noteCrsfTxFailure(Core::NowNs());
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

// ─────────────────────────────────────────────────────────────────────────────
// Channel helpers
// ─────────────────────────────────────────────────────────────────────────────

uint16_t CRSFOutput::normToCrsf(int16_t val) {
  // Map from -1000 … 1000  →  172 … 1811  (same range as SBUS)
  return (uint16_t)((float)(val + 1000) / 2000.0f *
                        (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
                    CRSF_CHANNEL_MIN);
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame builders
// ─────────────────────────────────────────────────────────────────────────────

void CRSFOutput::writeRcChannelsFrame() {
  packetFilter_.updateFromPacket(packet_, lastPacketTime_);
  const auto &f = packetFilter_.getFilteredPacket();

  VCTR::packets::crsf::CRSFPacket_RcChannelsPacked pkt;
  // Default every channel to centre
  for (uint8_t i = 0; i < 16; i++)
    pkt.channels[i] = CRSF_CHANNEL_MID;

  // Analogue channels (same ordering as SBusTx)
  pkt.channels[0] = normToCrsf(f.analogChannels[0]); // Roll
  pkt.channels[1] = normToCrsf(f.analogChannels[1]); // Pitch
  pkt.channels[2] = normToCrsf(f.analogChannels[3]); // Throttle
  pkt.channels[3] = normToCrsf(f.analogChannels[2]); // Yaw

  // Digital channels (same mapping as SBusTx)
  pkt.channels[4] =
      (float)f.digitalChannels[0] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;
  pkt.channels[5] =
      (float)f.digitalChannels[1] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;
  pkt.channels[6] =
      (float)f.digitalChannels[2] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;
  pkt.channels[7] =
      (float)f.digitalChannels[3] / 2 * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
      CRSF_CHANNEL_MIN;

  uint8_t frame[26];
  pkt.serialize(frame);
  queueSerialFrame(frame, pkt.numBytes());
}

void CRSFOutput::writeLinkStatsFrame() {
  const auto &radio = getLinkStatsRadio();

  // Always use local radio readings for uplink stats.
  const int16_t rssiRaw = radio.getPacketRSSI();
  const int16_t snrRaw = radio.getPacketSNR();
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

  VCTR::packets::crsf::CRSFPacket_LinkStatistics pkt;
  pkt.upRssiAnt1 = uplinkRssi;              // uplink RSSI ant 1 (positive magnitude)
  pkt.upRssiAnt2 = 0;                       // uplink RSSI ant 2 (N/A single-ant)
  pkt.upLinkQuality = (uint8_t)(lq * 100.0f); // uplink link quality 0-100 %
  pkt.upSnr = (int8_t)snrRaw;               // uplink SNR (signed)
  pkt.activeAntenna = 0;                    // active antenna index (no diversity)
  pkt.rfProfile = 4;                        // RF mode (generic)
  pkt.upRfPower = txPowerIdx;               // uplink TX power index (remote TX side)
  pkt.downRssi = 0;                         // downlink RSSI (N/A on RX side)
  pkt.downLinkQuality = (uint8_t)(lq * 100.0f); // repurpose unused downlink LQ
  pkt.downSnr = 0;                          // downlink SNR (N/A on RX side)

  uint8_t frame[14];
  pkt.serialize(frame);
  queueSerialFrame(frame, pkt.numBytes());
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
        handleCrsfFrame(rxBuf_);
        switch (rxBuf_[2]) {
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
          forwardBatteryTelemetry(rxBuf_);
          break;
        case CRSF_FRAMETYPE_GPS:
          forwardGpsTelemetry(rxBuf_);
          break;
        default:
          break;
        }
      }
      rxBufPos_ = 0;
    }
  }
}

void CRSFOutput::forwardBatteryTelemetry(const uint8_t *frame) {
  VCTR::packets::crsf::CRSFPacket_BatterySensor crsfBattery;
  if (!crsfBattery.deserialize(frame)) {
    return;
  }

  VCTR::packets::telecoms::Telemetry_Battery telemetry;
  telemetry.voltage = crsfBattery.voltage;
  telemetry.current = crsfBattery.current;
  serializeTelemetryMessage(telemetry, pendingBatteryTelemetryData_);
}

void CRSFOutput::forwardGpsTelemetry(const uint8_t *frame) {
  VCTR::packets::crsf::CRSFPacket_GPS crsfGps;
  if (!crsfGps.deserialize(frame)) {
    return;
  }

  VCTR::packets::telecoms::Telemetry_GNSS telemetry;
  telemetry.speed = (uint16_t)crsfGps.groundspeedKmh;
  telemetry.satellites = crsfGps.satellites;
  serializeTelemetryMessage(telemetry, pendingGpsTelemetryData_);
}

void CRSFOutput::handleCrsfFrame(const uint8_t *frame) {
  const uint8_t type = frame[2];

  if (type == CRSF_FRAMETYPE_COMMAND) {
    // Extended-header bind command (sent by BetaFlight "Bind" button)
    VCTR::packets::crsf::CRSFPacket_CommandRxBind bindCmd;
    if (bindCmd.deserialize(frame)) {
      WifiUpload::startWiFiUploadMode();
    }
  }
}

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF
