#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"

#include "Config.hpp"
#include "Hardware.hpp"

#include "DatalinkSetup.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

namespace VCTR::ExVectrLink {

#ifdef TEST_RADIO_DATALINK_SIMPLE

using namespace VCTR::network::datalink;

auto bandwidth = SX1280_BW::BW_800KHz;
auto codingRate = SX1280_CR::LI_4_8;
auto spreadingFactor = SX1280_SF::SF_7;
auto frequency = 2445000000;

class RadioSimpleTest : public Core::Task_Periodic {
private:
  VCTR::network::datalink::Datalink_SX1280_V2 &radioA;
  VCTR::network::datalink::Datalink_SX1280_V2 &radioB;

  uint8_t packetCount = 0;

  const size_t packetSize = 3;

public:
  RadioSimpleTest(VCTR::network::datalink::Datalink_SX1280_V2 &radioA,
                  VCTR::network::datalink::Datalink_SX1280_V2 &radioB)
      : Task_Periodic("Radio Simple test", 2 * Core::SECONDS,
                      5 * Core::SECONDS),
        radioA(radioA), radioB(radioB) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {
    LOG_MSG("Setting up radio simple test task\n");

    auto setupRadio = [](VCTR::network::datalink::Datalink_SX1280_V2 &radio) {
      radio.setEnableTxRx(true);
      radio.setTxPower(0);
      radio.setBandwidth(bandwidth);
      radio.setCodingRate(codingRate);
      radio.setFrequency(frequency);
      radio.setSpreadingFactor(spreadingFactor);
    };

    setupRadio(radioA);
    setupRadio(radioB);

    radioB.addReceiveHandler([this](const VCTR::network::DataPacket &packet) {
      // Simply print that we received a data packet with correct or incorrect
      // data.
      bool packetOk = true;
      for (size_t i = 0; i < packet.payload.size() - 1; i++) {
        if (packet.payload[i] != i) {
          packetOk = false;
          break;
        }
      }
      LOG_MSG("Received packet %d on radio B. Start: %.4f. Packet is %s\n",
              packetCount, (double)packet.timestamp / Core::MILLISECONDS,
              packetOk ? "OK" : "NOT OK");
    });
  }

  void taskCheck() override {}

  void taskThread() override {
    network::DataPacket packet;
    // packet.payload.append(packetCount);
    for (size_t i = 0; i < packetSize; i++) {
      packet.payload.append(i);
    }

    int secondInterval = (Core::NowNs()) / Core::SECONDS;

    packet.timestamp = secondInterval * Core::SECONDS + 10 * Core::MILLISECONDS;

    LOG_MSG("Transmitting packet %d from radio A at time %.4f\n", packetCount,
            (double)packet.timestamp / Core::MILLISECONDS);
    radioA.transmitDataframe(packet);
    packetCount++;
  }
};

RadioSimpleTest
    radioSimpleTest(VCTR::ExVectrLink::datalink::getDatalinkRadioA(),
                    VCTR::ExVectrLink::datalink::getDatalinkRadioB());

#endif // TEST_RADIO_DATALINK_SIMPLE

} // namespace VCTR::ExVectrLink