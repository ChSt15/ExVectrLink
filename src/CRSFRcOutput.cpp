#include "CRSFRcOutput.hpp"

#ifdef ENABLE_CRSF

#include <cstring>

#include "ExVectrCore.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"
#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "FhssSetup.hpp"
#include "WifiUpload.hpp"

namespace VCTR::ExVectrLink {

namespace {

// Local radio to read uplink RSSI/SNR from -- the aggregate diversity view
// when two radios are present, otherwise radio A. Same selection CRSFOutput
// and CRSFRcInput use for their own link-stats frames.
const VCTR::network::datalink::Sx1280_DirectI &getLinkStatsRadio() {
#if defined(HAS_DIVERSITY)
  return VCTR::ExVectrLink::datalink::getSx1280Diversity();
#else
  return VCTR::ExVectrLink::datalink::getDatalinkRadioA();
#endif
}

// 1-second sliding-window counters, same approach as CRSFOutput's original
// implementation (which these replace -- see getCrsfRcPacketRateHz() /
// getCrsfTxFailureRateHz() below).
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

uint16_t getCrsfRcPacketRateHz() { return rcPacketRateHz; }
uint16_t getCrsfTxFailureRateHz() { return crsfTxFailureRateHz; }

CRSFRcOutput::CRSFRcOutput(HardwareSerial &serial)
    : Core::Task_Periodic("CRSFRcOutput", Core::MILLISECONDS * 10),
      crsf_(serial, CRSFDuplex::Full, CRSF_BAUD_RATE),
      packetManager_([](const VCTR::Core::ListArray<uint8_t> &) {}) {
  Core::getSystemScheduler().addTask(*this);
}

void CRSFRcOutput::taskInit() {
  // Deferred from the constructor: crsf_ is a member of a global object
  // constructed before setup() runs, so touching the hardware only happens
  // once the scheduler actually initializes this task.
  crsf_.begin();

  // Only forward PacketTypes::Data packets, and strip the type byte so
  // packetManager_ receives clean ExVectrPackets data without corruption.
  datalink::getFhss().addReceiveHandler(
      [this](const VCTR::network::DataPacket &data) {
        if (data.payload.size() == 0)
          return;
        if (data.payload[data.payload.size() - 1] !=
            (uint8_t)datalink::PacketTypes::Data)
          return;
        VCTR::Core::ListArray<uint8_t> stripped;
        stripped.setSize(data.payload.size() - 1);
        memcpy(stripped.getPtr(), data.payload.getPtr(),
               data.payload.size() - 1);
        packetManager_.receivePacketData(stripped);
      });

  packetManager_.addPacketReceiveHandler<RcLinkPacket>(
      [this](const RcLinkPacket &packet) {
        const int64_t nowNs = Core::NowNs();
        rcAccumulator_.updateFromPacket(packet, nowNs);
        newRcPacket_ = true;
        lastPacketTime_ = nowNs;
        noteRcPacketReceived(nowNs);
        // Wake up the task immediately so we output the frame ASAP.
        this->setDeadline(nowNs);
      });

  packetManager_
      .addPacketReceiveHandler<VCTR::packets::telecoms::Telecommand_UpdateMode>(
          [](const VCTR::packets::telecoms::Telecommand_UpdateMode &) {
            WifiUpload::startWiFiUploadMode();
          });

  registerCrsfTelemetryForwarding();
}

void CRSFRcOutput::taskCheck() {
  // Wake early to drain any bytes that couldn't be flushed last cycle
  // because the FC's UART was momentarily full, so they go out as soon as
  // the port has room rather than waiting a whole task period. Mirrors
  // SerialTelecoms::taskCheck().
  if (crsf_.pendingTxBytes() > 0) {
    setDeadline(Core::NowNs());
  }
}

void CRSFRcOutput::taskThread() {
  const int64_t nowNs = Core::NowNs();
  updateCrsfStatsWindow(nowNs);

  // Pump the port: flushes any outgoing bytes that didn't fit on the wire
  // last time, and parses any newly arrived bytes from the FC.
  crsf_.available();

  // Drain every queued frame: try each registered telemetry conversion in
  // turn (matches CRSFRcInput's incoming-frame loop), discard anything none
  // of them recognise so it can never wedge behind an unsupported type.
  while (crsf_.peekFrameType() != 0) {
    bool handled = false;
    for (size_t i = 0; i < crsfTelemetryHandlers_.size(); i++) {
      if (crsfTelemetryHandlers_[i](crsf_)) {
        handled = true;
        break;
      }
    }
    if (!handled) {
      crsf_.discardFrame();
    }
  }

  if (newRcPacket_) {
    writeRcChannelsFrame();
    newRcPacket_ = false;
  }

  // Send link stats on their own timer, independent of RC-packet arrival:
  // the FC's LQ/RSSI/SNR readouts only update when it receives a 0x14 frame.
  if (nowNs - lastLinkStatsSendNs_ >= LINK_STATS_INTERVAL_NS) {
    lastLinkStatsSendNs_ = nowNs;
    writeLinkStatsFrame();
  }
}

template <VCTR::packets::IsPacket CrsfT, VCTR::packets::IsPacket ExVectrT>
void CRSFRcOutput::forwardCrsfTelemetry(
    datalink::OtaSendKey key, std::function<ExVectrT(const CrsfT &)> convert) {
  crsfTelemetryHandlers_.append([this, key, convert](CRSF &crsf) -> bool {
    CRSF::ReceivedPacket<CrsfT> received;
    if (!crsf.read(received)) {
      return false;
    }
    sendOverFhss(key, convert(received.packet));
    return true;
  });
}

template <VCTR::packets::IsPacket CrsfT>
void CRSFRcOutput::forwardCrsfTelemetry(
    std::function<void(const CrsfT &)> handle) {
  crsfTelemetryHandlers_.append([handle](CRSF &crsf) -> bool {
    CRSF::ReceivedPacket<CrsfT> received;
    if (!crsf.read(received)) {
      return false;
    }
    handle(received.packet);
    return true;
  });
}

template <VCTR::packets::IsPacket T>
void CRSFRcOutput::sendOverFhss(datalink::OtaSendKey key, const T &packet) {
  VCTR::network::DataPacket otaPkt;
  otaPkt.payload.setSize(packet.numBytes() + 1);
  packet.serialize(otaPkt.payload.getPtr());
  otaPkt.payload[packet.numBytes()] =
      (uint8_t)ExVectrLink::datalink::PacketTypes::Data;
  ExVectrLink::datalink::queueOtaPacket(key, otaPkt);
}

void CRSFRcOutput::registerCrsfTelemetryForwarding() {
  // Every FC -> ground telemetry type is wired up the same way: read it off
  // the FC's CRSF wire, convert to the matching ExVectrPackets type, queue
  // it for the FHSS downlink under its own OtaSendKey. Adding a new
  // telemetry source is one more forwardCrsfTelemetry<>() call (plus a new
  // OtaSendKey value) -- nothing else here needs to change.
  forwardCrsfTelemetry<VCTR::packets::crsf::CRSFPacket_BatterySensor,
                      VCTR::packets::telecoms::Telemetry_Battery>(
      datalink::OtaSendKey::CrsfBatterySensor,
      [](const VCTR::packets::crsf::CRSFPacket_BatterySensor &c) {
        VCTR::packets::telecoms::Telemetry_Battery t;
        t.voltage = c.voltage;
        t.current = c.current;
        return t;
      });

  // GPS is manually split into two OTA sends: latitude+longitude alone
  // already fill an OTA frame, so the remaining fields (speed, satellites,
  // altitude) go out as a separate "stats" packet under its own key.
  forwardCrsfTelemetry<VCTR::packets::crsf::CRSFPacket_GPS>(
      [this](const VCTR::packets::crsf::CRSFPacket_GPS &c) {
        VCTR::packets::telecoms::Telemetry_GNSS stats;
        stats.speed = (uint16_t)c.groundspeedKmh;
        stats.satellites = c.satellites;
        stats.altitudeM = (int16_t)c.altitudeM;
        sendOverFhss(datalink::OtaSendKey::CrsfGpsStats, stats);

        VCTR::packets::telecoms::Telemetry_GNSSPosition position;
        position.latitude = c.latitude;
        position.longitude = c.longitude;
        sendOverFhss(datalink::OtaSendKey::CrsfGpsPosition, position);
      });
}

uint16_t CRSFRcOutput::normToCrsf(int16_t val) {
  // Map from -1000 … 1000  →  172 … 1811  (same range as SBUS)
  return (uint16_t)((float)(val + 1000) / 2000.0f *
                        (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
                    CRSF_CHANNEL_MIN);
}

void CRSFRcOutput::writeRcChannelsFrame() {
  // Collapse the accumulated per-channel state into one full packet for the
  // glitch filter (the LinkType tag is inert -- it is never serialized).
  VCTR::packets::radiolink::RadioLinkPacket<
      kRcPriorityAnalog + kRcMuxAnalog, kRcPriorityDigital + kRcMuxDigital,
      VCTR::packets::radiolink::RadioLinkTypes::A4D0>
      full;
  rcAccumulator_.fillPacket(full);
  packetFilter_.updateFromPacket(full, lastPacketTime_);
  const auto &f = packetFilter_.getFilteredPacket();

  VCTR::packets::crsf::CRSFPacket_RcChannelsPacked pkt;
  // Default every channel to centre
  for (uint8_t i = 0; i < 16; i++)
    pkt.channels[i] = CRSF_CHANNEL_MID;

  // Analogue channels (same ordering as SBusTx/CRSFOutput)
  pkt.channels[0] = normToCrsf(f.analogChannels[0]); // Roll
  pkt.channels[1] = normToCrsf(f.analogChannels[1]); // Pitch
  pkt.channels[2] = normToCrsf(f.analogChannels[3]); // Throttle
  pkt.channels[3] = normToCrsf(f.analogChannels[2]); // Yaw

  // Digital channels (same mapping as SBusTx/CRSFOutput): 0 = aux1
  // (priority), 1-5 = aux2-aux6 (multiplexed) -> CRSF ch 4-9.
  for (size_t i = 0; i < kRcPriorityDigital + kRcMuxDigital; i++) {
    pkt.channels[4 + i] = (float)f.digitalChannels[i] / 2 *
                              (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
                          CRSF_CHANNEL_MIN;
  }

  if (!crsf_.write(pkt)) {
    noteCrsfTxFailure(Core::NowNs());
  }
}

void CRSFRcOutput::writeLinkStatsFrame() {
  const auto &radio = getLinkStatsRadio();

  // Uplink (ground TX -> this RX -> FC): the RX receives the ground link
  // directly, so uplink RSSI/SNR/LQ come from our own local radio readings.
  const int16_t rssiRaw = radio.getPacketRSSI();
  const int16_t snrRaw = radio.getPacketSNR();
  const float lq = datalink::getFhss().getLinkQuality(); // 0.0 – 1.0

  const uint8_t uplinkRssi = (rssiRaw < 0) ? (uint8_t)(-rssiRaw) : (uint8_t)0;

  // TX power: report the remote (ground TX) power so the FC sees what the
  // transmitter is actually doing, falling back to 0 if no OTA link telemetry
  // has been received yet. Mapped to the CRSF Link Statistics power-index
  // enum: 0mW, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW.
  const int8_t reportedTxDbm = datalink::getRemoteLinkStats().txPowerdBm;
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
  pkt.upRssiAnt1 = uplinkRssi;
  pkt.upRssiAnt2 = 0;                       // single antenna
  pkt.upLinkQuality = (uint8_t)(lq * 100.0f);
  pkt.upSnr = (int8_t)snrRaw;
  pkt.activeAntenna = 0;
  pkt.rfProfile = 4;                        // RF mode (generic)
  pkt.upRfPower = txPowerIdx;
  pkt.downRssi = 0;                         // downlink (FC->RX) N/A here
  pkt.downLinkQuality = (uint8_t)(lq * 100.0f);
  pkt.downSnr = 0;

  if (!crsf_.write(pkt)) {
    noteCrsfTxFailure(Core::NowNs());
  }
}

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF
