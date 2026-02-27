#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrLink/SerialTelecoms.hpp"

#include "Config.hpp"
#include "Hardware.hpp"

#include "DatalinkSetup.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

namespace VCTR::ExVectrLink {

#ifdef TEST_RADIO_DATALINK_PINGPONG

using namespace VCTR::network::datalink;

/**
 * The RadioSelfTest will use one radio to send a packet and the other radio to
 * receive this packet. Once a packet is received, its immediately sent back.
 * The RadioSelfTest addionally tests the latency and datathroughput from one
 * end to the other.
 */
class RadioSelfTest : public Core::Task_Periodic {
private:
  VCTR::network::datalink::Datalink_SX1280 &radioA;
  VCTR::network::datalink::Datalink_SX1280 &radioB;

  int64_t packetSentRadioATime = 0;
  int64_t packetSentRadioBTime = 0;

  const size_t packetSize = 20;

public:
  size_t packetCount = 0;
  size_t failed = 0;
  size_t totalBytesSent = 0;
  int64_t totalLatency = 0;

  size_t restarts = -1;

  RadioSelfTest(VCTR::network::datalink::Datalink_SX1280 &radioA,
                VCTR::network::datalink::Datalink_SX1280 &radioB)
      : Task_Periodic("RadioSelfTest", 1 * Core::SECONDS), radioA(radioA),
        radioB(radioB) {
    Core::getSystemScheduler().addTask(*this);
  }

  void addPacketLatency(int64_t latency, bool success) {
    packetCount++;
    totalBytesSent += packetSize;
    totalLatency += latency;
    if (!success) {
      failed++;
    }
  }

  void taskInit() override {
    LOG_MSG("Setting up radio self test task\n");
    radioA.enableTxRx(true);
    radioB.enableTxRx(true);
    radioA.cadBeforeSend(true);
    radioB.cadBeforeSend(true);

    radioA.setBandwidth(SX1280_BW::BW_800KHz);
    radioB.setBandwidth(SX1280_BW::BW_800KHz);

    radioA.setCodingRate(SX1280_CR::LI_4_8);
    radioB.setCodingRate(SX1280_CR::LI_4_8);

    radioA.setFrequency(2445000000);
    radioB.setFrequency(2445000000);

    radioA.setSpreadingFactor(SX1280_SF::SF_8);
    radioB.setSpreadingFactor(SX1280_SF::SF_8);

    radioA.addReceiveHandler([this](const VCTR::network::DataPacket &packet) {
      auto time = Core::NOW();
      auto latency = time - packetSentRadioBTime;
      packetSentRadioATime = time;
      radioA.transmitDataframe(packet);
      bool packetOk = true;
      for (size_t i = 0; i < packet.payload.size(); i++) {
        if (packet.payload[i] != i) {
          packetOk = false;
          break;
        }
      }
      addPacketLatency(latency, packetOk);
      setRelease(Core::NOW() + 1 * Core::SECONDS);
    });
    radioB.addReceiveHandler([this](const VCTR::network::DataPacket &packet) {
      auto time = Core::NOW();
      auto latency = time - packetSentRadioATime;
      packetSentRadioBTime = time;
      radioB.transmitDataframe(packet);
      bool packetOk = true;
      for (size_t i = 0; i < packet.payload.size(); i++) {
        if (packet.payload[i] != i) {
          packetOk = false;
          break;
        }
      }
      addPacketLatency(latency, packetOk);
      setRelease(Core::NOW() + 1 * Core::SECONDS);
    });
  }

  void taskThread() override {
    LOG_MSG("No packet received for 1 second. Sending packet on radio A\n");
    VCTR::network::DataPacket packet;
    for (size_t i = 0; i < packetSize; i++) {
      packet.payload.append(i);
    }
    radioB.transmitDataframe(packet);
    restarts++;
  }
};
RadioSelfTest radioSelfTest(VCTR::ExVectrLink::datalink::getDatalinkRadioA(),
                            VCTR::ExVectrLink::datalink::getDatalinkRadioB());

class RadioSelfTestReporter : public Core::Task_Periodic {
private:
  RadioSelfTest &radioSelfTest;

  int64_t lastReportTime = 0;

public:
  RadioSelfTestReporter(RadioSelfTest &radioSelfTest)
      : Task_Periodic("RadioSelfTestReporter", 1 * Core::SECONDS),
        radioSelfTest(radioSelfTest) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskThread() override {
    float dTime = (Core::NOW() - lastReportTime) / (float)Core::SECONDS;
    lastReportTime = Core::NOW();
    float bytesPerSecond = radioSelfTest.totalBytesSent / dTime;
    float latencyPerPacket =
        radioSelfTest.packetCount > 0
            ? radioSelfTest.totalLatency / radioSelfTest.packetCount
            : 0;
    LOG_MSG("\nRadio Self Test Report: \n\tTotal Packets: %d, \n\tFailed: %d, "
            "\n\tTotal "
            "Bytes Sent: %d, \n\tAverage Latency: %.2f ms, \n\tRestarts: %d, "
            "\n\tBytes per "
            "Second: %.2f\n",
            radioSelfTest.packetCount, radioSelfTest.failed,
            radioSelfTest.totalBytesSent,
            (double)latencyPerPacket / Core::MILLISECONDS,
            radioSelfTest.restarts, (double)bytesPerSecond);
    radioSelfTest.packetCount = 0;
    radioSelfTest.failed = 0;
    radioSelfTest.totalBytesSent = 0;
    radioSelfTest.totalLatency = 0;
    radioSelfTest.restarts = 0;
  }
};
RadioSelfTestReporter radioSelfTestReporter(radioSelfTest);

#endif // TEST_RADIO_DATALINK_SELFTEST

} // namespace VCTR::ExVectrLink