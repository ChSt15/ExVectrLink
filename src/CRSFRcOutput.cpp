#include "CRSFRcOutput.hpp"

#ifdef ENABLE_CRSF

#include <cstring>

#include "ExVectrCore.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"
#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "FhssSetup.hpp"
#include "WifiUpload.hpp"

namespace VCTR::ExVectrLink {

namespace {

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

  packetManager_
      .addPacketReceiveHandler<VCTR::packets::radiolink::RadioLinkPacket_A4D4>(
          [this](const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &packet) {
            const int64_t nowNs = Core::NowNs();
            packet_ = packet;
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
}

template <VCTR::packets::IsPacket CrsfT, VCTR::packets::IsPacket ExVectrT>
void CRSFRcOutput::forwardCrsfTelemetry(
    std::function<ExVectrT(const CrsfT &)> convert) {
  crsfTelemetryHandlers_.append([this, convert](CRSF &crsf) -> bool {
    CRSF::ReceivedPacket<CrsfT> received;
    if (!crsf.read(received)) {
      return false;
    }
    sendOverFhss(convert(received.packet));
    return true;
  });
}

template <VCTR::packets::IsPacket T>
void CRSFRcOutput::sendOverFhss(const T &packet) {
  VCTR::network::DataPacket otaPkt;
  otaPkt.payload.setSize(packet.numBytes() + 1);
  packet.serialize(otaPkt.payload.getPtr());
  otaPkt.payload[packet.numBytes()] =
      (uint8_t)ExVectrLink::datalink::PacketTypes::Data;
  ExVectrLink::datalink::getFhss().transmitDataframe(otaPkt);
}

void CRSFRcOutput::registerCrsfTelemetryForwarding() {
  // Every FC -> ground telemetry type is wired up the same way: read it off
  // the FC's CRSF wire, convert to the matching ExVectrPackets type, send
  // over FHSS. Adding a new telemetry source is one more
  // forwardCrsfTelemetry<>() call -- nothing else here needs to change.
  forwardCrsfTelemetry<VCTR::packets::crsf::CRSFPacket_BatterySensor,
                      VCTR::packets::telecoms::Telemetry_Battery>(
      [](const VCTR::packets::crsf::CRSFPacket_BatterySensor &c) {
        VCTR::packets::telecoms::Telemetry_Battery t;
        t.voltage = c.voltage;
        t.current = c.current;
        return t;
      });

  forwardCrsfTelemetry<VCTR::packets::crsf::CRSFPacket_GPS,
                      VCTR::packets::telecoms::Telemetry_GNSS>(
      [](const VCTR::packets::crsf::CRSFPacket_GPS &c) {
        VCTR::packets::telecoms::Telemetry_GNSS t;
        t.speed = (uint16_t)c.groundspeedKmh;
        t.satellites = c.satellites;
        return t;
      });
}

uint16_t CRSFRcOutput::normToCrsf(int16_t val) {
  // Map from -1000 … 1000  →  172 … 1811  (same range as SBUS)
  return (uint16_t)((float)(val + 1000) / 2000.0f *
                        (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) +
                    CRSF_CHANNEL_MIN);
}

void CRSFRcOutput::writeRcChannelsFrame() {
  packetFilter_.updateFromPacket(packet_, lastPacketTime_);
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

  // Digital channels (same mapping as SBusTx/CRSFOutput)
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

  if (!crsf_.write(pkt)) {
    noteCrsfTxFailure(Core::NowNs());
  }
}

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF
