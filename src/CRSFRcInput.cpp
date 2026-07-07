#include "CRSFRcInput.hpp"

#ifdef ENABLE_CRSF_INPUT

#include <cstring>

#include "ExVectrCore.hpp"

#include "DatalinkCommandManager.hpp"
#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrPackets/crsf/CRSFTypes.hpp"
#include "FhssSetup.hpp"

namespace VCTR::ExVectrLink {

namespace {
constexpr uint16_t CRSF_CHANNEL_MIN = 172;
constexpr uint16_t CRSF_CHANNEL_MAX = 1811;

// Maps a dBm value to the CRSF Link Statistics power-index enum: 0mW,
// 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW. Same table as
// CRSFOutput::writeLinkStatsFrame() uses for the FC-facing side.
uint8_t crsfPowerIndex(int8_t dbm) {
  if (dbm <= 0) {
    return 0; // 0 mW
  } else if (dbm <= 10) {
    return 1; // 10 mW
  } else if (dbm <= 14) {
    return 2; // 25 mW
  } else if (dbm <= 17) {
    return 8; // 50 mW
  } else if (dbm <= 20) {
    return 3; // 100 mW
  } else if (dbm <= 24) {
    return 7; // 250 mW
  } else if (dbm <= 27) {
    return 4; // 500 mW
  } else if (dbm <= 30) {
    return 5; // 1000 mW
  } else if (dbm <= 33) {
    return 6; // 2000 mW
  }
  return 0;
}

// CRSF device addresses relevant to the parameter/command protocol (see
// extern/CRSF details.md, "Device Addresses").
constexpr uint8_t CRSF_ADDRESS_BROADCAST = 0x00;
constexpr uint8_t CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA; // the radio/handset
constexpr uint8_t CRSF_ADDRESS_CRSF_TRANSMITTER =
    0xEE; // R/C Transmitter Module / Crossfire Tx -- this device.
constexpr uint8_t CRSF_ADDRESS_ELRS_LUA = 0xEF; // the ELRS handset Lua script

// Chooses the destination address to answer a parameter request to. Mirrors
// ExpressLRS's TXModuleEndpoint "dodgy hack": once the ELRS Lua recognises us
// as a native ELRS TX it addresses its reads/writes from 0xEF, but EdgeTX
// only routes a reply back to the script if it is addressed to the radio
// (0xEA) -- so remap 0xEF -> 0xEA. Any other origin is answered directly.
uint8_t replyDestForOrigin(uint8_t origin) {
  return (origin == CRSF_ADDRESS_ELRS_LUA) ? CRSF_ADDRESS_RADIO_TRANSMITTER
                                           : origin;
}

} // namespace

CRSFRcInput::CRSFRcInput(HardwareSerial &serial, int8_t rxPin, int8_t txPin,
                         uint32_t baudRate, bool invert, uint8_t uartNum)
    : Core::Task_Periodic("CRSFRcInput", 2 * Core::MILLISECONDS),
      serial_(serial), rxPin_(rxPin), txPin_(txPin), baudRate_(baudRate),
      invert_(invert), uartNum_(uartNum),
      crsf_(serial, CRSFDuplex::Half, baudRate, rxPin, uartNum, invert),
      telemetryPacketManager_([](const VCTR::Core::ListArray<uint8_t> &) {}) {
  Core::getSystemScheduler().addTask(*this);
}

void CRSFRcInput::taskInit() {
  // crsf_ deliberately never has begin() called on it: it only knows the
  // generic 1-argument HardwareSerial::begin(baud), which has no way to
  // express the pin-binding workaround below. We configure the port
  // directly instead; crsf_ is only used for CRSF framing/queueing from
  // here on.
#if defined(ESP32)
  if (rxPin_ >= 0 && txPin_ >= 0 && rxPin_ == txPin_) {
    // Single shared half-duplex signal wire (e.g. a "nano" module bay's
    // S.Port-style pin). Binding both RX and TX to the same pin previously
    // triggered the ESP32's automatic RS485-half-duplex hardware mode,
    // whose auto-RTS logic gated the receiver on our own transmit activity
    // and left it effectively permanently disabled. Only attach the pad as
    // UART RX (leave TX unmapped/floating) at begin() time -- crsf_ takes
    // over from there, rerouting the pad to TX via the GPIO matrix for the
    // duration of each reply (see CRSF::switchToRx()/switchToTx()).
    //
    // invert is intentionally NOT passed here (always false): inversion is
    // instead handled entirely inside CRSF's direction switching, which is
    // what makes it come out correct across repeated RX<->TX switches --
    // mirrors ExpressLRS's CRSFHandset::Begin()/duplex_set_RX()/
    // duplex_set_TX().
    serial_.begin(baudRate_, SERIAL_8N1, rxPin_, -1, false);
    crsf_.beginHalfDuplex();
  } else if (rxPin_ >= 0 && txPin_ >= 0) {
    serial_.begin(baudRate_, SERIAL_8N1, rxPin_, txPin_, invert_);
  } else {
    serial_.begin(baudRate_);
  }
#else
  serial_.begin(baudRate_);
#endif

  registerTelemetryForwarding();
}

void CRSFRcInput::taskCheck() {
  // Wake up as soon as bytes arrive rather than waiting for the next
  // periodic tick -- the handset expects its telemetry reply immediately
  // after its own frame finishes, so latency here directly matters (see
  // CRSF::sendHalfDuplexReply(), which fires synchronously inside the next
  // taskThread()'s crsf_.available() call).
  if (serial_.available() > 0) {
    setDeadline(Core::NowNs());
  }
}

void CRSFRcInput::taskThread() {
  // Queue fresh link stats before pumping the port, so if a reply goes out
  // this tick it carries current numbers rather than lagging by one frame.
  const int64_t nowNs = Core::NowNs();
  if (nowNs - lastLinkStatsSendNs_ >= LINK_STATS_INTERVAL_NS) {
    lastLinkStatsSendNs_ = nowNs;
    writeLinkStatsFrame();
  }

  crsf_.available();

  // Drain every queued frame each tick: forward RC channels, discard
  // anything else so it can never wedge behind an unrelated frame type.
  while (crsf_.peekFrameType() != 0) {
    const uint8_t frameType = crsf_.peekFrameType();

    if (frameType == static_cast<uint8_t>(
                          VCTR::packets::crsf::CRSFFrameType::RcChannelsPacked)) {
      CRSF::ReceivedPacket<VCTR::packets::crsf::CRSFPacket_RcChannelsPacked>
          received;
      if (crsf_.read(received)) {
        forwardRcChannels(received.packet);
        continue;
      }
    } else if (frameType == static_cast<uint8_t>(
                                 VCTR::packets::crsf::CRSFFrameType::ParameterPing)) {
      CRSF::ReceivedPacket<VCTR::packets::crsf::CRSFPacket_ParameterPing>
          received;
      if (crsf_.read(received)) {
        handleParameterPing(received.packet);
        continue;
      }
    } else if (frameType == static_cast<uint8_t>(
                                 VCTR::packets::crsf::CRSFFrameType::ParameterRead)) {
      CRSF::ReceivedPacket<VCTR::packets::crsf::CRSFPacket_ParameterRead>
          received;
      if (crsf_.read(received)) {
        handleParameterRead(received.packet);
        continue;
      }
    } else if (frameType == static_cast<uint8_t>(
                                 VCTR::packets::crsf::CRSFFrameType::ParameterWrite)) {
      CRSF::ReceivedPacket<VCTR::packets::crsf::CRSFPacket_ParameterWrite>
          received;
      if (crsf_.read(received)) {
        handleParameterWrite(received.packet);
        continue;
      }
    }
    crsf_.discardFrame();
  }
}

void CRSFRcInput::writeLinkStatsFrame() {
  const auto &radio = ExVectrLink::datalink::getDatalinkRadioA();
  const auto &fhss = ExVectrLink::datalink::getFhss();
  const auto &remote = ExVectrLink::datalink::getRemoteLinkStats();

  VCTR::packets::crsf::CRSFPacket_LinkStatistics pkt;

  // Uplink (this TX module -> FC): the FC is the one receiving it, so its
  // self-reported OTA stats (relayed back to us over the radio link) are
  // the correct source -- mirrors CRSFOutput::writeLinkStatsFrame()'s
  // reasoning, from the other end of the link.
  pkt.upRssiAnt1 = remote.valid ? (uint8_t)(-remote.rssi) : 0;
  pkt.upRssiAnt2 = 0;
  pkt.upLinkQuality = remote.valid ? remote.linkQuality : 0;
  pkt.upSnr = remote.valid ? remote.snr : 0;
  pkt.activeAntenna = remote.valid ? remote.antenna : 0;
  pkt.rfProfile = 4;
  pkt.upRfPower =
      crsfPowerIndex(ExVectrLink::datalink::getLocalTxPowerdBm());

  // Downlink (FC -> this TX module): received directly, so use our own
  // local radio readings.
  const int16_t rssiRaw = radio.getPacketRSSI();
  pkt.downRssi = (rssiRaw < 0) ? (uint8_t)(-rssiRaw) : 0;
  pkt.downLinkQuality = (uint8_t)(fhss.getLinkQuality() * 100.0f);
  pkt.downSnr = (int8_t)radio.getPacketSNR();

  sendTelemetry(pkt);
}

void CRSFRcInput::handleParameterPing(
    const VCTR::packets::crsf::CRSFPacket_ParameterPing &ping) {
  // Only reply if this ping is for us (or everyone) -- mirrors ExpressLRS's
  // device.reply-to-ping behaviour (see extern/CRSF details.md, "0x28
  // Parameter Ping Devices").
  if (ping.destAddress != CRSF_ADDRESS_BROADCAST &&
      ping.destAddress != CRSF_ADDRESS_CRSF_TRANSMITTER) {
    return;
  }
  VCTR::packets::crsf::CRSFPacket_ParameterDeviceInfo info;
  info.destAddress = replyDestForOrigin(ping.origAddress);
  sendTelemetry(info);
}

void CRSFRcInput::handleParameterRead(
    const VCTR::packets::crsf::CRSFPacket_ParameterRead &req) {
  if (req.parameterNumber != kBindParameterNumber) {
    // Only one parameter exposed -- see extern/CRSF details.md's Chunks
    // note: hosts fall back to requesting from parameter 1 onward when the
    // device doesn't answer for parameter 0 (root folder).
    return;
  }
  // Reading a COMMAND parameter resets it to idle -- mirrors ExpressLRS's
  // CRSFEndpoint::parameterUpdateReq() (READ of a CRSF_COMMAND, chunk 0,
  // sets step=lcsIdle, info=""), so re-opening the menu always shows the
  // Bind command ready rather than stuck mid-run.
  if (req.chunkNumber == 0) {
    bindStep_ = VCTR::packets::crsf::CRSFCommandStep::Ready;
    bindInfo_[0] = '\0';
  }
  sendBindEntry(bindStep_, bindInfo_, replyDestForOrigin(req.origAddress));
}

void CRSFRcInput::handleParameterWrite(
    const VCTR::packets::crsf::CRSFPacket_ParameterWrite &req) {
  using VCTR::packets::crsf::CRSFCommandStep;

  const uint8_t replyDest = replyDestForOrigin(req.origAddress);

  // Parameter 0 is the ELRS Lua's special "link statistics request": it
  // writes param 0 roughly once a second expecting a 0x2E ELRS status frame
  // back to drive the good/bad packet counter and the connection icon in its
  // title bar. Mirrors ExpressLRS's TXModuleEndpoint special-case.
  if (req.parameterNumber == 0) {
    sendElrsStatus(replyDest);
    return;
  }

  if (req.parameterNumber != kBindParameterNumber) {
    return;
  }

  const int64_t nowNs = Core::NowNs();

  if (req.commandStatus < static_cast<uint8_t>(CRSFCommandStep::Cancel)) {
    // Click (and anything short of Cancel/Poll) executes immediately, same
    // as ExpressLRS's handleSimpleSendCmd() for its "Bind" command -- no
    // confirmation step. Unlike a real RF bind, "executing" here just means
    // relaying a one-shot trigger to the RX so it enters WiFi upload mode.
    lastBindClickNs_ = nowNs;
    bindStep_ = CRSFCommandStep::Progress;
    strncpy(bindInfo_, "WiFi Upload...", sizeof(bindInfo_) - 1);
    bindInfo_[sizeof(bindInfo_) - 1] = '\0';
    triggerRxWifiUpload();
    sendBindEntry(bindStep_, bindInfo_, replyDest);
  } else if (req.commandStatus == static_cast<uint8_t>(CRSFCommandStep::Cancel) ||
             (nowNs - lastBindClickNs_) > kBindStatusTimeoutNs) {
    bindStep_ = CRSFCommandStep::Ready;
    bindInfo_[0] = '\0';
    sendBindEntry(bindStep_, bindInfo_, replyDest);
  }
  // else: still executing (Poll) and not timed out yet -- send nothing, same
  // as ExpressLRS: the handset keeps polling until it gets an Idle reply or
  // its own local timeout expires.
}

void CRSFRcInput::sendElrsStatus(uint8_t destAddress) {
  const auto &fhss = ExVectrLink::datalink::getFhss();
  const auto &remote = ExVectrLink::datalink::getRemoteLinkStats();

  VCTR::packets::crsf::CRSFPacket_ElrsStatus pkt;
  pkt.destAddress = destAddress;

  // Present downlink link quality as a good/bad split out of 100 -- the ELRS
  // Lua just prints "bad/good" verbatim in its title bar, so this reads as a
  // familiar percentage there without needing real per-frame CRSF counters.
  int lq = (int)(fhss.getLinkQuality() * 100.0f + 0.5f);
  if (lq < 0) {
    lq = 0;
  } else if (lq > 100) {
    lq = 100;
  }
  pkt.pktsGood = (uint16_t)lq;
  pkt.pktsBad = (uint8_t)(100 - lq);
  // Bit 0 = LUA_FLAG_CONNECTED -> the "C" (vs "-") icon. Treat "connected" as
  // having a currently-valid link telemetry read from the far end.
  pkt.flags = remote.valid ? 0x01 : 0x00;
  pkt.info[0] = '\0';

  sendTelemetry(pkt);
}

void CRSFRcInput::sendBindEntry(VCTR::packets::crsf::CRSFCommandStep status,
                                const char *info, uint8_t destAddress) {
  VCTR::packets::crsf::CRSFPacket_ParameterSettingsEntryCommand entry;
  entry.destAddress = destAddress;
  entry.parameterNumber = kBindParameterNumber;
  entry.status = static_cast<uint8_t>(status);
  strncpy(entry.info, info, sizeof(entry.info) - 1);
  entry.info[sizeof(entry.info) - 1] = '\0';
  sendTelemetry(entry);
}

void CRSFRcInput::triggerRxWifiUpload() {
  // One-shot trigger relayed over the FHSS OTA link -- the RX side already
  // enters WiFi upload mode on receiving this (see DatalinkCommandManager's
  // PacketTypes::UploadMode handling). Queued (rather than a direct
  // transmitDataframe()) so it shares the single-slot channel with the rest
  // of the OTA senders and gets retried if the channel is momentarily busy;
  // safe to coalesce since UploadModeTrigger only ever carries this one
  // message.
  VCTR::network::DataPacket pkt;
  pkt.payload.append(
      (uint8_t)ExVectrLink::datalink::PacketTypes::UploadMode);
  ExVectrLink::datalink::queueOtaPacket(
      ExVectrLink::datalink::OtaSendKey::UploadModeTrigger, pkt);
}

void CRSFRcInput::registerTelemetryForwarding() {
  // Every FC -> handset telemetry type is wired up the same way: decode it
  // out of the FHSS downlink, convert to the matching CRSF frame, queue it.
  // Adding a new telemetry source is one more forwardTelemetry<>() call --
  // nothing else here needs to change.
  forwardTelemetry<VCTR::packets::telecoms::Telemetry_Battery,
                   VCTR::packets::crsf::CRSFPacket_BatterySensor>(
      [](const VCTR::packets::telecoms::Telemetry_Battery &t) {
        VCTR::packets::crsf::CRSFPacket_BatterySensor pkt;
        pkt.voltage = t.voltage;
        pkt.current = t.current;
        // Telemetry_Battery doesn't carry these (kept compact for the OTA
        // link) -- report unknown rather than a misleading fixed number.
        pkt.capacityUsedMah = 0;
        pkt.remainingPercent = 0;
        return pkt;
      });

  // GPS arrives as two independently-timed OTA packets (see
  // CRSFRcOutput::registerCrsfTelemetryForwarding() -- position doesn't fit
  // in the same frame as the rest of the GPS fields), so instead of
  // forwardTelemetry<>()'s stateless 1:1 conversion, each one updates its
  // own fields of a persistent gpsCache_ and re-sends the combined packet,
  // always carrying whichever of position/stats was most recently heard.
  telemetryPacketManager_
      .addPacketReceiveHandler<VCTR::packets::telecoms::Telemetry_GNSS>(
          [this](const VCTR::packets::telecoms::Telemetry_GNSS &t) {
            gpsCache_.groundspeedKmh = (float)t.speed;
            gpsCache_.satellites = t.satellites;
            gpsCache_.altitudeM = (float)t.altitudeM;
            sendTelemetry(gpsCache_);
          });

  telemetryPacketManager_.addPacketReceiveHandler<
      VCTR::packets::telecoms::Telemetry_GNSSPosition>(
      [this](const VCTR::packets::telecoms::Telemetry_GNSSPosition &t) {
        gpsCache_.latitude = t.latitude;
        gpsCache_.longitude = t.longitude;
        sendTelemetry(gpsCache_);
      });

  // Feed every FHSS downlink PacketTypes::Data payload into the telemetry
  // PacketManager above; each registered forwardTelemetry<>() handler
  // tries its own deserialize() and only the matching type fires. Mirrors
  // CRSFRcOutput's uplink registration for RC channels/telecommands.
  ExVectrLink::datalink::getFhss().addReceiveHandler(
      [this](const VCTR::network::DataPacket &data) {
        if (data.payload.size() == 0) {
          return;
        }
        if (data.payload[data.payload.size() - 1] !=
            (uint8_t)ExVectrLink::datalink::PacketTypes::Data) {
          return;
        }
        VCTR::Core::ListArray<uint8_t> stripped;
        stripped.setSize(data.payload.size() - 1);
        memcpy(stripped.getPtr(), data.payload.getPtr(),
               data.payload.size() - 1);
        telemetryPacketManager_.receivePacketData(stripped);
      });
}

int16_t CRSFRcInput::crsfToNorm(uint16_t val) {
  // Map from 172 … 1811  →  -1000 … 1000  (inverse of CRSFRcOutput::normToCrsf)
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

uint8_t CRSFRcInput::digitalFromCrsf(uint16_t val) {
  float norm = ((float)(val - CRSF_CHANNEL_MIN) /
                (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) *
               2.0f;
  if (norm < 0.0f)
    norm = 0.0f;
  if (norm > 2.0f)
    norm = 2.0f;
  return (uint8_t)(norm + 0.5f);
}

void CRSFRcInput::forwardRcChannels(
    const VCTR::packets::crsf::CRSFPacket_RcChannelsPacked &pkt) {
  VCTR::packets::radiolink::RadioLinkPacket_A4D4 packet;

  // Analogue channels (inverse of CRSFRcOutput::writeRcChannelsFrame mapping)
  packet.analogChannels[0] = crsfToNorm(pkt.channels[0]); // Roll
  packet.analogChannels[1] = crsfToNorm(pkt.channels[1]); // Pitch
  packet.analogChannels[3] = crsfToNorm(pkt.channels[2]); // Throttle
  packet.analogChannels[2] = crsfToNorm(pkt.channels[3]); // Yaw

  // Digital channels (inverse of CRSFRcOutput's mapping)
  packet.digitalChannels[0] = digitalFromCrsf(pkt.channels[4]);
  packet.digitalChannels[1] = digitalFromCrsf(pkt.channels[5]);
  packet.digitalChannels[2] = digitalFromCrsf(pkt.channels[6]);
  packet.digitalChannels[3] = digitalFromCrsf(pkt.channels[7]);

  lastRcPacket_ = packet;
  lastRcPacketTime_ = Core::NowNs();
  rcFrameCount_++;

  // Forward straight to the FHSS uplink, tagged the same way SerialTelecoms'
  // PacketData handler does it (see DatalinkCommandManager.cpp), so the
  // receive side (CRSFRcOutput) doesn't need to know or care whether the RC
  // data came from CRSF or a serial host app. Queued (not sent directly) so
  // it shares the single-slot channel with link stats/telemetry instead of
  // racing to overwrite whichever of them is currently pending.
  VCTR::network::DataPacket otaPkt;
  otaPkt.payload.setSize(packet.numBytes() + 1);
  packet.serialize(otaPkt.payload.getPtr());
  otaPkt.payload[packet.numBytes()] =
      (uint8_t)ExVectrLink::datalink::PacketTypes::Data;
  ExVectrLink::datalink::queueOtaPacket(
      ExVectrLink::datalink::OtaSendKey::RcChannels, otaPkt);
}

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF_INPUT
