#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/datalink/LinkSettings.hpp"
#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "ExVectrLink/serial/SerialTelecomPackets.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "FhssSetup.hpp"

#include "DatalinkCommandManager.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::ExVectrLink::packets;

uint8_t packetNumber = 0;

/// Current TX power in dBm, tracked from SerialPacket_SetTxPower so it can be
/// reported in outgoing OTA LinkInfo packets.
static int8_t currentTxPowerdBm = 20;

/// Decoded far-end link stats from the last received OTA LinkInfo packet.
static RemoteLinkStats remoteStats;

int8_t getLocalTxPowerdBm() { return currentTxPowerdBm; }
const RemoteLinkStats &getRemoteLinkStats() { return remoteStats; }

VCTR::ExVectrLink::SerialTelecoms *serialTelecomsPtr = nullptr;

VCTR::Core::StaticCallback_Subscriber<const char *> logSubscriber;

SerialPacket_LinkInfo linkInfoPacket;

// ---------------------------------------------------------------------------
// Helpers: encode / decode the compact 4-byte OTA LinkInfo payload
//
// Bit layout (27 bits used, 5 spare):
//   Byte 0  [7:5] antenna  (3 bits, 0-7)
//           [4:0] LQ[6:2]  (upper 5 bits of 7-bit quality field)
//   Byte 1  [7:6] LQ[1:0]  (lower 2 bits of 7-bit quality field)
//           [5:1] txPower  (5 bits, value 0-31 → 1-32 dBm)
//           [0]   spare
//   Byte 2  [7:0] RSSI magnitude (abs dBm, 0-127)
//   Byte 3  [7:4] SNR     (4 bits, 0-15 → -15 to +15 dB in 2 dB steps)
//           [3:0] spare
// ---------------------------------------------------------------------------

static void encodeLinkInfo(uint8_t antenna, float lq, int8_t txPowerdBm,
                           int16_t rssi, float snr, uint8_t out[4]) {
  uint8_t antVal = antenna & 0x07;
  uint8_t lqVal = (uint8_t)(lq * 127.0f + 0.5f) & 0x7F;
  int8_t pwr = txPowerdBm < 1 ? 1 : (txPowerdBm > 32 ? 32 : txPowerdBm);
  uint8_t pwrVal = (uint8_t)(pwr - 1) & 0x1F;
  int16_t mag = rssi < 0 ? -rssi : rssi;
  uint8_t rssiVal = (uint8_t)(mag > 127 ? 127 : mag);
  float snrC = snr < -15.0f ? -15.0f : (snr > 15.0f ? 15.0f : snr);
  uint8_t snrVal = (uint8_t)((snrC + 15.0f) / 2.0f + 0.5f) & 0x0F;

  out[0] = (uint8_t)((antVal << 5) | (lqVal >> 2));
  out[1] = (uint8_t)(((lqVal & 0x03) << 6) | (pwrVal << 1));
  out[2] = rssiVal;
  out[3] = (uint8_t)(snrVal << 4);
}

static void decodeLinkInfo(const uint8_t in[4], uint8_t &antenna, float &lq,
                           int8_t &txPowerdBm, int8_t &rssi, float &snr) {
  antenna = (in[0] >> 5) & 0x07;
  uint8_t lqRaw = (uint8_t)(((in[0] & 0x1F) << 2) | ((in[1] >> 6) & 0x03));
  lq = (float)lqRaw / 127.0f;
  txPowerdBm = (int8_t)(((in[1] >> 1) & 0x1F) + 1);
  rssi = -(int8_t)(in[2] & 0x7F);
  uint8_t snrRaw = (in[3] >> 4) & 0x0F;
  snr = (float)snrRaw * 2.0f - 15.0f;
}

class LinkInfoSender : public VCTR::Core::Task_Periodic {
public:
  // Run at 4 Hz (every 250 ms).
  LinkInfoSender() : Task_Periodic("LinkInfoSender", 250 * Core::MILLISECONDS) {
    VCTR::Core::getSystemScheduler().addTask(*this);
  }

  void taskThread() override {

    if (serialTelecomsPtr == nullptr) {
      return;
    }

    auto bestLinkIndex = getSx1280Diversity().getCurrentBestLinkIndex();
    auto &bestLink = *getSx1280Diversity().getDiversityLink(bestLinkIndex);

    // --- Fill local side stats ---
    linkInfoPacket.local.rssi = (int8_t)bestLink.lastPacketRSSI();
    linkInfoPacket.local.snr = (int8_t)getFhss().getSnr();
    linkInfoPacket.local.antenna = (uint8_t)bestLinkIndex;
    linkInfoPacket.local.lossRate =
        (uint8_t)((1.0f - getFhss().getLinkQuality()) * 100);
    linkInfoPacket.local.linkQuality =
        (uint8_t)(getFhss().getLinkQuality() * 100);
    linkInfoPacket.local.txPower = currentTxPowerdBm;

    // --- Fill remote side stats (from last received OTA LinkTelemetry) ---
    linkInfoPacket.remote.rssi = remoteStats.rssi;
    linkInfoPacket.remote.snr = remoteStats.snr;
    linkInfoPacket.remote.antenna = remoteStats.antenna;
    linkInfoPacket.remote.linkQuality = remoteStats.linkQuality;
    linkInfoPacket.remote.lossRate = (uint8_t)(100 - remoteStats.linkQuality);
    linkInfoPacket.remote.txPower = remoteStats.txPowerdBm;
    linkInfoPacket.remoteValid = remoteStats.valid;

    if (getFhss().getFhssState() != FHSSState::Synced) {
      // If not synced, override link quality to 0 and mark remote as invalid.
      linkInfoPacket.remote.linkQuality = 0;
    }

    // Send combined stats to the serial host.
    serialTelecomsPtr->sendSerialPacket(linkInfoPacket);

    // Also broadcast our link stats to the other end over-the-air.
    uint8_t payload[4];
    encodeLinkInfo(linkInfoPacket.local.antenna, getFhss().getLinkQuality(),
                   currentTxPowerdBm, linkInfoPacket.local.rssi,
                   linkInfoPacket.local.snr, payload);

    network::DataPacket otaPkt;
    for (size_t i = 0; i < 4; i++) {
      otaPkt.payload.append(payload[i]);
    }
    otaPkt.payload.append((uint8_t)PacketTypes::LinkTelemetry);
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

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetTxPower>(
      [&serialTelecoms](const SerialPacket_SetTxPower &packet) {
        currentTxPowerdBm = packet.txPower;
        getDatalinkRadioA().setTxPower(packet.txPower);
#if defined(HAS_DIVERSITY)
        getDatalinkRadioB().setTxPower(packet.txPower);
#endif
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
  getFhss().addReceiveHandler(
      [&serialTelecoms](const VCTR::network::DataPacket &packet) {
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
          // Decode the 4-byte OTA link telemetry and forward to serial host.
          if (packet.payload.size() < 5)
            return;
          uint8_t rawBytes[4] = {packet.payload[0], packet.payload[1],
                                 packet.payload[2], packet.payload[3]};
          // Decode into remoteStats cache; the next LinkInfoSender tick will
          // bundle this into the combined SerialPacket_LinkInfo sent to the
          // host.
          float lq;
          float snrF;
          int8_t remRssi;
          int8_t remTxPwr;
          uint8_t remAnt;
          decodeLinkInfo(rawBytes, remAnt, lq, remTxPwr, remRssi, snrF);

          remoteStats.antenna = remAnt;
          remoteStats.linkQuality = (uint8_t)(lq * 100);
          remoteStats.txPowerdBm = remTxPwr;
          remoteStats.rssi = remRssi;
          remoteStats.snr = (int8_t)snrF;
          remoteStats.valid = true;
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