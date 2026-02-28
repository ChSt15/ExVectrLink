#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrLink/serial/SerialTelecoms.hpp"

#include "Config.hpp"
#include "Hardware.hpp"

#include "DatalinkSetup.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

namespace VCTR::ExVectrLink {

#ifdef TEST_RADIO_DATALINK_THROUGHPUT

using namespace VCTR::network::datalink;

auto bandwidth = SX1280_BW::BW_800KHz;
auto codingRate = SX1280_CR::LI_4_8;
auto spreadingFactor = SX1280_SF::SF_9;
auto frequency = 2445000000;

/**
 * The RadioSelfTest will use one radio to send a packet and the other radio to
 * receive this packet. Once a packet is received, its immediately sent back.
 * The RadioSelfTest addionally tests the latency and datathroughput from one
 * end to the other.
 */
class RadioThroughputTestTx : public Core::Task_Periodic {
private:
  VCTR::network::datalink::Datalink_SX1280 &radio;

  uint8_t packetCount = 0;

  const size_t packetSize = 18;

public:
  RadioThroughputTestTx(VCTR::network::datalink::Datalink_SX1280 &radio)
      : Task_Periodic("Radio Throughput test tx", 1 * Core::SECONDS),
        radio(radio) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {
    LOG_MSG("Setting up radio self test task\n");
    radio.enableTxRx(true);
    radio.cadBeforeSend(false);
    radio.setTxPower(0);

    radio.setBandwidth(bandwidth);
    radio.setCodingRate(codingRate);
    radio.setFrequency(frequency);
    radio.setSpreadingFactor(spreadingFactor);
  }

  void taskCheck() override {
    if (!radio.isChannelBlocked()) {
      setRelease(Core::NOW());
    }
  }

  void taskThread() override {
    network::DataPacket packet;
    packet.payload.append(packetCount);
    for (size_t i = 0; i < packetSize; i++) {
      packet.payload.append(i);
    }
    radio.transmitDataframe(packet);
  }
};

class RadioThroughputTestRx : public Core::Task_Periodic {
private:
  VCTR::network::datalink::Datalink_SX1280 &radio;

  uint8_t lastPacketCount = 0;

  size_t totalPacketsLost = 0;

  size_t totalBytesReceived = 0;
  size_t totalPacketsReceived = 0;

  int64_t lastLoopTime = 0;

public:
  RadioThroughputTestRx(VCTR::network::datalink::Datalink_SX1280 &radio)
      : Task_Periodic("Radio Throughput test rx", 1 * Core::SECONDS),
        radio(radio) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {
    LOG_MSG("Setting up radio self test task\n");
    radio.enableTxRx(true);
    radio.cadBeforeSend(false);

    radio.setBandwidth(bandwidth);
    radio.setCodingRate(codingRate);
    radio.setFrequency(frequency);
    radio.setSpreadingFactor(spreadingFactor);

    radio.addReceiveHandler([this](const VCTR::network::DataPacket &packet) {
      handlePacket(packet);
    });
  }

  void handlePacket(const VCTR::network::DataPacket &packet) {
    auto time = Core::NOW();
    auto currentPacket = packet.payload[0];
    bool packetOk = true;
    for (size_t i = 1; i < packet.payload.size(); i++) {
      if (packet.payload[i] != i - 1) {
        packetOk = false;
        break;
      }
    }

    if (packetOk) {
      totalBytesReceived += packet.payload.size();
      totalPacketsReceived++;
    } else {
      auto numPacketsLost = (currentPacket - lastPacketCount) % 256;
      totalPacketsLost += numPacketsLost;
    }
  }

  void taskCheck() override {}

  void taskThread() override {

    float dTime = (Core::NOW() - lastLoopTime) / (float)Core::SECONDS;
    lastLoopTime = Core::NOW();
    float throughputBytesPerSecond = totalBytesReceived / dTime;
    LOG_MSG("\nRadio Throughput Test Report: \nTotal Packets Received: %d, "
            "\nTotal Packets Lost: %d, \nTotal Bytes Received: %d, "
            "\nThroughput (Bytes/s): %f\n",
            totalPacketsReceived, totalPacketsLost, totalBytesReceived,
            throughputBytesPerSecond);
    Serial.print("Radio Throughput Test Report: Total Packets Received: ");

    // Reset counters for next report
    totalBytesReceived = 0;
    totalPacketsReceived = 0;
    totalPacketsLost = 0;
  }
};

RadioThroughputTestTx
    radioThroughputTestTx(VCTR::ExVectrLink::datalink::getDatalinkRadioA());
RadioThroughputTestRx
    radioThroughputTestRx(VCTR::ExVectrLink::datalink::getDatalinkRadioB());

#endif // TEST_RADIO_DATALINK_THROUGHPUT

} // namespace VCTR::ExVectrLink