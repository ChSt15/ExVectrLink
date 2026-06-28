#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/datalink/LinkSettings.hpp"
#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "ExVectrLink/serial/SerialTelecomPackets.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "DynamicPowerSetup.hpp"
#include "FhssSetup.hpp"

#if defined(ENABLE_CRSF)
#include "CRSFOutput.hpp"
#endif

#include "DatalinkCommandManager.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::ExVectrLink::packets;

uint8_t packetNumber = 0;

int32_t rxDesyncCount = 0;
uint16_t rxPacketRateHz = 0;
uint8_t rxPacketQuality = 0;
uint16_t rxCrsfTxFailures = 0;

/// Decoded far-end link stats from the last received OTA LinkInfo packet.
static RemoteLinkStats remoteStats;

const RemoteLinkStats &getRemoteLinkStats() { return remoteStats; }

static uint8_t quantizeLinkQualityPercent(float lq);

static uint16_t getLocalRcPacketRateHz() {
#if defined(ENABLE_CRSF)
  return VCTR::ExVectrLink::getCrsfRcPacketRateHz();
#else
  return 0;
#endif
}

static uint8_t getLocalPacketQualityPercent() {
  return quantizeLinkQualityPercent(getFhss().getPacketQuality());
}

static uint16_t getLocalCrsfTxFailureRateHz() {
#if defined(ENABLE_CRSF)
  return VCTR::ExVectrLink::getCrsfTxFailureRateHz();
#else
  return 0;
#endif
}

static const VCTR::network::datalink::Sx1280_DirectI &getLinkStatsRadio() {
#if defined(HAS_DIVERSITY)
  return getSx1280Diversity();
#else
  return getDatalinkRadioA();
#endif
}

int8_t getLocalTxPowerdBm() { return getDatalinkRadioA().getTxPower(); }

static void applyLocalTxPowerToRadios() {
  const int8_t txPowerDbm = getDynamicPower().getPower();
  getDatalinkRadioA().setTxPower(txPowerDbm);
#if defined(HAS_DIVERSITY)
  getDatalinkRadioB().setTxPower(txPowerDbm);
#endif
}

VCTR::ExVectrLink::SerialTelecoms *serialTelecomsPtr = nullptr;

VCTR::Core::StaticCallback_Subscriber<const char *> logSubscriber;

SerialPacket_LinkInfo linkInfoPacket;

// ---------------------------------------------------------------------------
// Helpers: encode / decode the compact 5-byte OTA LinkInfo payload
//
// Bit layout (28 bits used, 12 spare):
//   Byte 0  [7:5] antenna  (3 bits, 0-7)
//           [4:0] LQ[6:2]  (upper 5 bits of 7-bit quality field)
//   Byte 1  [7:6] LQ[1:0]  (lower 2 bits of 7-bit quality field)
//           [5:1] txPower  (5 bits, value 0-31 → 1-32 dBm)
//           [0]   spare
//   Byte 2  [7:0] RSSI magnitude (abs dBm, 0-127)
//   Byte 3  [7:3] SNR     (5 bits, 0-30 → -15 to +15 dB in 1 dB steps)
//           [2:0] spare
//   Byte 4  [0]   dynamicPower
//           [7:1] spare
// ---------------------------------------------------------------------------

static uint8_t quantizeLinkQualityPercent(float lq) {
  float clamped = lq < 0.0f ? 0.0f : (lq > 1.0f ? 1.0f : lq);
  return (uint8_t)(clamped * 100.0f + 0.5f);
}

static void encodeLinkInfo(uint8_t antenna, float lq, int8_t txPowerdBm,
                           int16_t rssi, float snr, bool dynamicPower,
                           uint8_t out[5]) {
  uint8_t antVal = antenna & 0x07;
  uint8_t lqVal = quantizeLinkQualityPercent(lq);
  int8_t pwr = txPowerdBm < 1 ? 1 : (txPowerdBm > 32 ? 32 : txPowerdBm);
  uint8_t pwrVal = (uint8_t)(pwr - 1) & 0x1F;
  int16_t mag = rssi < 0 ? -rssi : rssi;
  uint8_t rssiVal = (uint8_t)(mag > 127 ? 127 : mag);
  float snrC = snr < -15.0f ? -15.0f : (snr > 15.0f ? 15.0f : snr);
  uint8_t snrVal = (uint8_t)(snrC + 15.0f + 0.5f) & 0x1F;

  out[0] = (uint8_t)((antVal << 5) | (lqVal >> 2));
  out[1] = (uint8_t)(((lqVal & 0x03) << 6) | (pwrVal << 1));
  out[2] = rssiVal;
  out[3] = (uint8_t)(snrVal << 3);
  out[4] = dynamicPower ? 0x01 : 0x00;
}

static void decodeLinkInfo(const uint8_t in[5], uint8_t &antenna, float &lq,
                           int8_t &txPowerdBm, int8_t &rssi, float &snr,
                           bool &dynamicPower) {
  antenna = (in[0] >> 5) & 0x07;
  uint8_t lqRaw = (uint8_t)(((in[0] & 0x1F) << 2) | ((in[1] >> 6) & 0x03));
  lq = (float)lqRaw / 100.0f;
  txPowerdBm = (int8_t)(((in[1] >> 1) & 0x1F) + 1);
  rssi = -(int8_t)(in[2] & 0x7F);
  uint8_t snrRaw = (in[3] >> 3) & 0x1F;
  snr = (float)snrRaw - 15.0f;
  dynamicPower = (in[4] & 0x01) != 0;
}

static void encodeRxState(uint16_t desyncCount, uint16_t packetRateHz,
                          uint8_t packetQuality, uint16_t crsfTxFailures,
                          uint8_t out[8]) {
  out[0] = (uint8_t)(desyncCount & 0xFF);
  out[1] = (uint8_t)((desyncCount >> 8) & 0xFF);
  out[2] = (uint8_t)(packetRateHz & 0xFF);
  out[3] = (uint8_t)((packetRateHz >> 8) & 0xFF);
  out[4] = packetQuality;
  out[5] = (uint8_t)(crsfTxFailures & 0xFF);
  out[6] = (uint8_t)((crsfTxFailures >> 8) & 0xFF);
  out[7] = 0;
}

static void decodeRxState(const uint8_t in[8], uint16_t &desyncCount,
                          uint16_t &packetRateHz, uint8_t &packetQuality,
                          uint16_t &crsfTxFailures) {
  desyncCount = (uint16_t)in[0] | ((uint16_t)in[1] << 8);
  packetRateHz = (uint16_t)in[2] | ((uint16_t)in[3] << 8);
  packetQuality = in[4];
  crsfTxFailures = (uint16_t)in[5] | ((uint16_t)in[6] << 8);
}

static uint8_t getLocalTxAntennaIndex() {
#if defined(HAS_DIVERSITY)
  return (uint8_t)getSx1280Diversity().getCurrentTxLinkIndex();
#else
  return 0;
#endif
}

class LinkInfoSender : public VCTR::Core::Task_Periodic {

  int64_t lastRxStatePacketSendTime = 0;

public:
  // Run at 4 Hz (every 250 ms).
  LinkInfoSender() : Task_Periodic("LinkInfoSender", 200 * Core::MILLISECONDS) {
    VCTR::Core::getSystemScheduler().addTask(*this);
  }

  void taskThread() override {

    if (serialTelecomsPtr == nullptr) {
      return;
    }

    if (Core::NowNs() - lastRxStatePacketSendTime > 1000 * Core::MILLISECONDS) {
      lastRxStatePacketSendTime = Core::NowNs();
      sendRxStatePacket();
    } else {
      sendLinkTelemetryPacket();
    }
  }

  void sendLinkTelemetryPacket() {
    const auto &radio = getLinkStatsRadio();
    auto &dynamicPower = getDynamicPower();

    // --- Fill local side stats ---
    linkInfoPacket.local.rssi = (int8_t)radio.getPacketRSSI();
    linkInfoPacket.local.snr = (int8_t)radio.getPacketSNR();
    linkInfoPacket.local.antenna = getLocalTxAntennaIndex();
    const uint8_t localLinkQuality =
        quantizeLinkQualityPercent(getFhss().getLinkQuality());
    linkInfoPacket.local.linkQuality = localLinkQuality;
    linkInfoPacket.local.lossRate = (uint8_t)(100 - localLinkQuality);
    linkInfoPacket.local.packetQuality = getLocalPacketQualityPercent();
    const int8_t localTxPowerdBm = getLocalTxPowerdBm();
    linkInfoPacket.local.txPower = localTxPowerdBm;
    linkInfoPacket.local.dynamicPower = dynamicPower.isDynamicPowerEnabled();
    linkInfoPacket.local.desyncCount = getFhss().getDesyncCounter();
    linkInfoPacket.local.packetRate = getLocalRcPacketRateHz();
    linkInfoPacket.local.crsfTxFailures = getLocalCrsfTxFailureRateHz();

    // --- Fill remote side stats (from last received OTA LinkTelemetry) ---
    linkInfoPacket.remote.rssi = remoteStats.rssi;
    linkInfoPacket.remote.snr = remoteStats.snr;
    linkInfoPacket.remote.antenna = remoteStats.antenna;
    linkInfoPacket.remote.linkQuality = remoteStats.linkQuality;
    linkInfoPacket.remote.lossRate = (uint8_t)(100 - remoteStats.linkQuality);
    linkInfoPacket.remote.packetQuality = rxPacketQuality;
    linkInfoPacket.remote.txPower = remoteStats.txPowerdBm;
    linkInfoPacket.remote.dynamicPower = remoteStats.dynamicPower;
    linkInfoPacket.remote.desyncCount = rxDesyncCount;
    linkInfoPacket.remote.packetRate = rxPacketRateHz;
    linkInfoPacket.remote.crsfTxFailures = rxCrsfTxFailures;

    linkInfoPacket.remoteValid = remoteStats.valid;

    if (getFhss().getFhssState() != FHSSState::Synced) {
      // If not synced, override link quality to 0 and mark remote as invalid.
      linkInfoPacket.remote.linkQuality = 0;
      linkInfoPacket.remote.packetQuality = 0;
      linkInfoPacket.remoteValid = false;
    }

    // Send combined stats to the serial host.
    serialTelecomsPtr->sendSerialPacket(linkInfoPacket);

    // Also broadcast our link stats to the other end over-the-air.
    uint8_t payload[8];
    encodeLinkInfo(linkInfoPacket.local.antenna, getFhss().getLinkQuality(),
                   localTxPowerdBm, linkInfoPacket.local.rssi,
                   linkInfoPacket.local.snr, linkInfoPacket.local.dynamicPower,
                   payload);

    network::DataPacket otaPkt;
    for (size_t i = 0; i < 8; i++) {
      otaPkt.payload.append(payload[i]);
    }
    otaPkt.payload.append((uint8_t)PacketTypes::LinkTelemetry);
    getFhss().transmitDataframe(otaPkt);
  }

  void sendRxStatePacket() {

    uint8_t payload[8];
    encodeRxState(getFhss().getDesyncCounter(), getLocalRcPacketRateHz(),
                  getLocalPacketQualityPercent(), getLocalCrsfTxFailureRateHz(),
                  payload);

    network::DataPacket otaPkt;
    for (size_t i = 0; i < 8; i++) {
      otaPkt.payload.append(payload[i]);
    }
    otaPkt.payload.append((uint8_t)PacketTypes::RxState);
    getFhss().transmitDataframe(otaPkt);
  }

} linkInfoSender;

void printToSerial(const char *const &message) {
  if (serialTelecomsPtr != nullptr) {
    SerialPacket_Print packet;
    strncpy(packet.message, message, sizeof(packet.message) - 1);
    packet.length = strnlen(packet.message, sizeof(packet.message) - 1);
    serialTelecomsPtr->sendSerialPacket(packet);
  }
}

void initCommandManager(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms) {

  serialTelecomsPtr = &serialTelecoms;

  // logSubscriber.subscribe(VCTR::Core::getDebugTopic());
  // logSubscriber.setCallbackFunction(printToSerial);

  serialTelecoms.addSerialPacketHandler(
      SerialPacketType::PacketData,
      [&serialTelecoms](const Core::ListArray<uint8_t> &data) {
        if (data.size() > 0) {
          network::DataPacket packet(data);
          // Append the type byte so the far end knows this is a data packet.
          packet.payload.append((uint8_t)PacketTypes::Data);
          getFhss().transmitDataframe(packet);
        }
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetModulationPreset>(
      [&serialTelecoms](const SerialPacket_SetModulationPreset &packet) {
        const auto &modParams = modulationPresets[packet.presetIndex];
        getDatalinkRadioA().setCodingRate(modParams.codingRate);
        getDatalinkRadioA().setSpreadingFactor(modParams.spreadingFactor);
        getDatalinkRadioA().setBandwidth(modParams.bandwidth);

#if defined(HAS_DIVERSITY)
        getDatalinkRadioB().setCodingRate(modParams.codingRate);
        getDatalinkRadioB().setSpreadingFactor(modParams.spreadingFactor);
        getDatalinkRadioB().setBandwidth(modParams.bandwidth);
#endif
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetPowerParams>(
      [](const SerialPacket_SetPowerParams &packet) {
        auto &dynamicPower = getDynamicPower();
        const uint8_t requestedMaxPower =
            (packet.txPower == 0) ? 255 : packet.txPower;
        const uint8_t requestedDynMaxPower =
            (packet.maxDynPower == 0) ? requestedMaxPower : packet.maxDynPower;
        const uint8_t requestedDynMinPower = (packet.minDynPower == 0)
                                                 ? DynamicPower::kPowerLevels[0]
                                                 : packet.minDynPower;

        dynamicPower.setMaxPower(requestedMaxPower);
        dynamicPower.setEnableDynamicPower(packet.enableDynamicPower);

        dynamicPower.setDynMaxPower(requestedDynMaxPower);
        dynamicPower.setDynMinPower(requestedDynMinPower);

        if (!packet.enableDynamicPower) {
          dynamicPower.setPower(requestedMaxPower);
        }

        applyLocalTxPowerToRadios();
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetLinkChannel>(
      [&serialTelecoms](const SerialPacket_SetLinkChannel &packet) {
        // getFhss().enableFhss(false);
        // getSx1280Diversity().setChannel(packet.channelIndex);
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetEnableFhss>(
      [&serialTelecoms](const SerialPacket_SetEnableFhss &packet) {
        // getFhss().enableFhss(packet.enable);
        getFhss().setFhssKey(packet.seqKey);
      });

  // Receiving info from far end over FHSS
  getFhss().addReceiveHandler([&serialTelecoms](
                                  const VCTR::network::DataPacket &packet) {
    // Serial.printf("Received %d byte FHSS packet with payload: ",
    //               packet.payload.size());
    // for (size_t i = 0; i < packet.payload.size(); i++) {
    //   Serial.print(packet.payload[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();
    // return;

    if (packet.payload.size() == 0) {
      return;
    }
    // Last byte is always the packet type.
    uint8_t pktType = packet.payload[packet.payload.size() - 1];

    if (pktType == (uint8_t)PacketTypes::Data) {
      // Strip the type byte and forward raw data to the serial host.
      serialTelecoms.sendSerialPacket(SerialPacketType::PacketData,
                                      packet.payload.getPtr(),
                                      packet.payload.size() - 1);

    } else if (pktType == (uint8_t)PacketTypes::LinkTelemetry) {
      // Decode the 5-byte OTA link telemetry and forward to serial host.
      // if (packet.payload.size() < 8)
      //   return;
      // Decode into remoteStats cache; the next LinkInfoSender tick will
      // bundle this into the combined SerialPacket_LinkInfo sent to the
      // host.
      float lq;
      float snrF;
      int8_t remRssi;
      int8_t remTxPwr;
      uint8_t remAnt;
      bool remDynamicPower;
      decodeLinkInfo(packet.payload.getPtr(), remAnt, lq, remTxPwr, remRssi,
                     snrF, remDynamicPower);
      const uint8_t remLqPercent = quantizeLinkQualityPercent(lq);

      updateDynamicPowerFromRemoteLinkInfo(remRssi, (int8_t)snrF, remLqPercent);

      remoteStats.antenna = remAnt;
      remoteStats.linkQuality = remLqPercent;
      remoteStats.txPowerdBm = remTxPwr;
      remoteStats.rssi = remRssi;
      remoteStats.snr = (int8_t)snrF;
      remoteStats.valid = true;
      remoteStats.dynamicPower = remDynamicPower;

    } else if (pktType == (uint8_t)PacketTypes::RxState) {

      // Decode the 8-byte OTA RxState packet which carries the far end's
      // desync count plus rate/quality diagnostics that do not fit in the
      // compact LinkTelemetry packet.
      uint16_t remDesyncCount;
      uint16_t remPacketRateHz;
      uint8_t remPacketQuality;
      uint16_t remCrsfTxFailures;
      decodeRxState(packet.payload.getPtr(), remDesyncCount, remPacketRateHz,
                    remPacketQuality, remCrsfTxFailures);

      rxDesyncCount = remDesyncCount;
      rxPacketRateHz = remPacketRateHz;
      rxPacketQuality = remPacketQuality;
      rxCrsfTxFailures = remCrsfTxFailures;
    }
    // Other types are silently ignored for now.
  });

  // getFhss().addChannelBlockedChangeHandler(
  //     [&serialTelecoms](bool blocked, uint8_t maxPacketSize) {
  //       SerialPacket_ChannelState packet{maxPacketSize, blocked};
  //       serialTelecoms.sendSerialPacket(packet);
  //       serialTelecoms.forcePacketSendNow();
  //     });
}

} // namespace VCTR::ExVectrLink::datalink