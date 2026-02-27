#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/SerialTelecomPackets.hpp"

#include "DiversitySetup.hpp"

#include "DatalinkCommandManager.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::network::datalink;

VCTR::network::DataPacket dataSendBuffer;

void initCommandManager(
    SerialTelecoms::ExVectrLinkSerialTelecoms &serialTelecoms) {

  // Sending Data Command
  serialTelecoms.addSerialPacketHandler(
      SerialTelecoms::SerialPacketType::PacketData,
      [&serialTelecoms](const Core::ListArray<uint8_t> &data) {
        for (size_t i = 0; i < data.size(); i++) {
          dataSendBuffer.payload.append(data[i]);
        }
      });

  serialTelecoms.addSerialPacketHandler(
      SerialTelecoms::SerialPacketType::PacketDataClear,
      [&serialTelecoms](const Core::ListArray<uint8_t> &data) {
        dataSendBuffer.payload.clear();
      });

  serialTelecoms.addSerialPacketHandler(
      SerialTelecoms::SerialPacketType::DataSend,
      [&serialTelecoms](const Core::ListArray<uint8_t> &data) {
        if (dataSendBuffer.payload.size() > 0) {
          getSx1280Diversity().transmitDataframe(dataSendBuffer);
          dataSendBuffer.payload.clear();
        }
      });

  // Send Link Info response
  serialTelecoms.addSerialPacketHandler(
      SerialTelecoms::SerialPacketType::LinkInfo,
      [&serialTelecoms](const Core::ListArray<uint8_t> &data) {
        SerialTelecoms::packets::SerialPacket_LinkInfo linkInfo;
        auto bestLinkIndex = getSx1280Diversity().getCurrentBestLinkIndex();
        auto bestLink = *getSx1280Diversity().getDiversityLink(bestLinkIndex);
        linkInfo.antenna = bestLinkIndex;
        linkInfo.rssi = bestLink.lastPacketRSSI();
        linkInfo.snr = bestLink.lastPacketSNR();
        linkInfo.linkQuality = (linkInfo.rssi + 120) * 2;
        linkInfo.dualLinkMode = false;
        linkInfo.linkSpeed = 0;
        Core::ListArray<uint8_t> serializedLinkInfo;
        linkInfo.serialize(serializedLinkInfo.getPtr());
        serialTelecoms.sendSerialPacket(
            SerialTelecoms::SerialPacketType::LinkInfo, serializedLinkInfo);
      });
}

} // namespace VCTR::ExVectrLink::datalink