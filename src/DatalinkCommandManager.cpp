#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/datalink/LinkSettings.hpp"
#include "ExVectrLink/serial/SerialTelecomPackets.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "FhssSetup.hpp"

#include "DatalinkCommandManager.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::ExVectrLink::packets;

uint8_t packetNumber = 0;

void initCommandManager(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms) {

  serialTelecoms.addSerialPacketHandler(
      SerialPacketType::PacketData,
      [&serialTelecoms](const Core::ListArray<uint8_t> &data) {
        if (data.size() > 0 && !getFhss().isChannelBlocked()) {
          Core::ListArray<uint8_t> payload;
          payload.setSize(data.size() + 1);
          payload[0] = packetNumber++;
          mempcpy(payload.getPtr() + 1, data.getPtr(), data.size());
          network::DataPacket packet(payload);
          getFhss().transmitDataframe(packet);
        }
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetModulationPreset>(
      [&serialTelecoms](const SerialPacket_SetModulationPreset &packet) {
        const auto &modParams = modulationPresets[packet.presetIndex];
        getDatalinkRadioA().setCodingRate(modParams.codingRate);
        getDatalinkRadioA().setSpreadingFactor(modParams.spreadingFactor);
        getDatalinkRadioA().setBandwidth(modParams.bandwidth);

        getDatalinkRadioB().setCodingRate(modParams.codingRate);
        getDatalinkRadioB().setSpreadingFactor(modParams.spreadingFactor);
        getDatalinkRadioB().setBandwidth(modParams.bandwidth);
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetTxPower>(
      [&serialTelecoms](const SerialPacket_SetTxPower &packet) {
        getDatalinkRadioA().setTxPower(packet.txPower);
        getDatalinkRadioB().setTxPower(packet.txPower);
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetLinkChannel>(
      [&serialTelecoms](const SerialPacket_SetLinkChannel &packet) {
        getFhss().enableFhss(false);
        getSx1280Diversity().setChannel(packet.channelIndex);
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetEnableFhss>(
      [&serialTelecoms](const SerialPacket_SetEnableFhss &packet) {
        getFhss().enableFhss(packet.enable);
        getFhss().setFhssKey(packet.seqKey);
      });

  // Sending info
  getFhss().addReceiveHandler(
      [&serialTelecoms](const VCTR::network::DataPacket &packet) {
        Core::ListArray<uint8_t> copy;
        copy.setSize(packet.payload.size() - 1);
        mempcpy(copy.getPtr(), packet.payload.getPtr() + 1, copy.size());
        serialTelecoms.sendSerialPacket(SerialPacketType::PacketData, copy);

        auto bestLinkIndex = getSx1280Diversity().getCurrentBestLinkIndex();
        auto &bestLink = *getSx1280Diversity().getDiversityLink(bestLinkIndex);
        SerialPacket_LinkInfo linkInfoPacket{bestLink.lastPacketRSSI(),
                                             bestLink.lastPacketSNR(),
                                             bestLinkIndex, 0, false};
        serialTelecoms.sendSerialPacket(linkInfoPacket);

        serialTelecoms.forcePacketSendNow(10 * Core::MILLISECONDS);
      });
}

} // namespace VCTR::ExVectrLink::datalink