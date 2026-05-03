#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "ExVectrLink/serial/SerialTelecoms.hpp"

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"

#include "Hardware.hpp"

#include "DatalinkCommandManager.hpp"
#include "SX1280.hpp"
#include "WifiUpload.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "DynamicPowerSetup.hpp"
#include "FhssSetup.hpp"

#include "sbus.h"

#if defined(ENABLE_CRSF)
#include "CRSFOutput.hpp"
#endif

using namespace VCTR;
using namespace VCTR::ExVectrLink;

#if defined(ESP32)
HardwareSerial linkSerialPort(1);
Platform::BusUART serialBus(linkSerialPort);
#else
Platform::BusUART serialBus(Serial);
#endif
SerialTelecoms serialTelecoms(serialBus);

#if defined(ESP32)
class TempMonitor : public Core::Task_Periodic {
public:
  TempMonitor() : Core::Task_Periodic("TempMonitor", 5 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    int8_t temp = temperatureRead();
    serialTelecoms.sendSerialPacket(
        VCTR::ExVectrLink::packets::SerialPacket_DeviceTemperature{temp});
  }
};
TempMonitor tempMonitor;
#endif

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
class SyncedMonitor : public Core::Task_Periodic {
private:
  VCTR::ExVectrLink::datalink::FHSSState lastState =
      VCTR::ExVectrLink::datalink::FHSSState::Searching;

public:
  SyncedMonitor()
      : Core::Task_Periodic("SyncedMonitor", 500 * Core::MILLISECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    auto fhssState = VCTR::ExVectrLink::datalink::getFhss().getFhssState();

    VCTR::ExVectrLink::packets::SerialPacket_FhssSyncState packet{
        fhssState == VCTR::ExVectrLink::datalink::FHSSState::Synced};
    serialTelecoms.sendSerialPacket(packet);
  }
};
SyncedMonitor syncedMonitor;
#endif

#if defined(ENABLE_SBUS)
VCTR::packets::PacketManager
    packetManager([](const VCTR::Core::ListArray<uint8_t> &) {});

namespace VCTR::ExVectrLink {
using namespace VCTR::packets::radiolink;

class SBusTx : public Core::Task_Periodic {
private:
  bfs::SbusTx sbus_tx;
  RadioLinkPacket_A4D4 packet;
  RadioLinkPacketFilter<4, 4, RadioLinkTypes::A4D4> packetFilter;
  bool dataNew = false;

  int64_t lastPacketTime = 0;

public:
  SBusTx()
      : Core::Task_Periodic("SBusTx", Core::MILLISECONDS * 5),
        sbus_tx(&Serial, true, true) {

    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {
    Serial.begin(200000, SERIAL_8E2, SerialMode::SERIAL_FULL, 1, true);
    // sbus_tx.Begin();
    VCTR::ExVectrLink::datalink::getFhss().addReceiveHandler(
        [this](const VCTR::network::DataPacket &data) {
          packetManager.receivePacketData(data.payload);
          // if (data.payload.size() == packet.numBytes()) {

          //   if (packet.deserialize(data.payload.getPtr())) {
          //     dataNew = true;
          //     lastPacketTime = Core::NOW();
          //   }
          // }
        });

    packetManager.addPacketReceiveHandler<
        VCTR::packets::radiolink::RadioLinkPacket_A4D4>(
        [this](const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &packet) {
          this->packet = packet;
          dataNew = true;
          lastPacketTime = Core::NOW();
          this->setDeadline(Core::NOW());
        });

    packetManager.addPacketReceiveHandler<
        VCTR::packets::telecoms::Telecommand_UpdateMode>(
        [this](const VCTR::packets::telecoms::Telecommand_UpdateMode &packet) {
          WifiUpload::startWiFiUploadMode();
        });

    // VCTR::ExVectrLink::datalink::getFhss()
    //     .addPacketReceiveHandler<
    //         VCTR::packets::radiolink::RadioLinkPacket_A4D4>(
    //         [this](
    //             const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &packet)
    //             {
    //           this->packet = packet;
    //           dataNew = true;
    //           packetCount++;
    //           lastPacketTime = Core::NOW();
    //         });
  }

  void taskThread() override {
    if (Core::NOW() - lastPacketTime < Core::MILLISECONDS * 200) {
      bfs::SbusData sbusData;
      sbusData.lost_frame = false;
      sbusData.failsafe = false;

      packetFilter.updateFromPacket(packet, lastPacketTime);
      auto &packetFiltered = packetFilter.getFilteredPacket();

      auto normToSbusMap = [](int16_t val) {
        // Map from -1000 - 1000 to 172 - 1811;
        return (float)(val + 1000) / 2000.0f * (1811 - 172) + 172;
      };

      for (int i = 0; i < bfs::SbusData::NUM_CH; i++) {
        sbusData.ch[i] = 1000;
      }

      // sbusData.ch[0] = packet.analogChannels[0] / 2 + 1500;
      sbusData.ch[0] = normToSbusMap(packetFiltered.analogChannels[0]);
      sbusData.ch[1] = normToSbusMap(packetFiltered.analogChannels[1]);
      sbusData.ch[2] = normToSbusMap(packetFiltered.analogChannels[3]);
      sbusData.ch[3] = normToSbusMap(packetFiltered.analogChannels[2]);
      sbusData.ch[4] = !packetFiltered.digitalChannels[1] ? 1811 : 172;
      sbusData.ch[8] = packetFiltered.digitalChannels[2] ? 1811 : 172;
      sbusData.ch[6] = packetFiltered.digitalChannels[3] ? 1811 : 172;
      sbusData.ch[7] = packetFiltered.digitalChannels[0] ? 1811 : 172;

      sbusData.ch17 = false;
      sbusData.ch18 = false;

      sbus_tx.data(sbusData);
      sbus_tx.Write();
      // Serial.printf("Received packet: A: %d %d %d %d D: %d %d %d %d\n",
      //               packetFiltered.analogChannels[0],
      //               packetFiltered.analogChannels[1],
      //               packetFiltered.analogChannels[2],
      //               packetFiltered.analogChannels[3],
      //               packetFiltered.digitalChannels[0],
      //               packetFiltered.digitalChannels[1],
      //               packetFiltered.digitalChannels[2],
      //               packetFiltered.digitalChannels[3]);
    } else {
      // If we haven't received a packet in a while, send failsafe
      bfs::SbusData sbusData;
      sbusData.lost_frame = true;
      sbusData.failsafe = true;
      for (int i = 0; i < bfs::SbusData::NUM_CH; i++) {
        sbusData.ch[i] = 1000;
      }
      sbusData.ch17 = false;
      sbusData.ch18 = false;

      sbus_tx.data(sbusData);
      sbus_tx.Write();
    }
    dataNew = false;
  }
};
SBusTx sbusTx;
} // namespace VCTR::ExVectrLink
#elif defined(ENABLE_CRSF)
VCTR::ExVectrLink::CRSFOutput crsfOutput;
#endif

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
class ErrorSender : public Core::Task_Periodic {
public:
  ErrorSender() : Core::Task_Periodic("ErrorSender", 0.1001 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    auto error = !radio::radioDriversInitialized();
    // network::DataPacket packet;
    // packet.payload.setSize(5);
    // datalink::getFhss().transmitDataframe(packet);

    if (error) {
      radio::initRadioDrivers();
      auto error2 = !radio::radioDriversInitialized();
      String message =
          error2
              ? "Error: Radios failed to initialize. Retrying in 1s\n"
              : "Error: Radios failed to initialize. Retried and succeeded\n";
      LOG_MSG(message.c_str());
      serialTelecoms.sendSerialPacket(VCTR::ExVectrLink::packets::Error,
                                      message.c_str(), message.length() + 1);
    } else {
#ifdef ENABLE_DEBUG_OUTPUT
      Serial.println("All good!");
      uint16_t irqFlags =
          VCTR::ExVectrLink::datalink::getDatalinkRadioA().getRemainIrqFlags();
      String irqStr;
      // if (irqFlags & IRQ_TX_DONE)
      //   irqStr += "TX_DONE ";
      // if (irqFlags & IRQ_RX_DONE)
      //   irqStr += "RX_DONE ";
      // if (irqFlags & IRQ_SYNCWORD_VALID)
      //   irqStr += "SW_VALID ";
      // if (irqFlags & IRQ_SYNCWORD_ERROR)
      //   irqStr += "SW_ERR ";
      // if (irqFlags & IRQ_HEADER_VALID)
      //   irqStr += "HDR_VALID ";
      // if (irqFlags & IRQ_HEADER_ERROR)
      //   irqStr += "HDR_ERR ";
      // if (irqFlags & IRQ_CRC_ERROR)
      //   irqStr += "CRC_ERR ";
      // if (irqFlags & IRQ_PREAMBLE_DETECTED)
      //   irqStr += "PREAMBLE ";
      // if (irqFlags & IRQ_RX_TX_TIMEOUT)
      //   irqStr += "TIMEOUT ";
      // if (irqFlags & IRQ_CAD_DONE)
      //   irqStr += "CAD_DONE ";
      // if (irqFlags & IRQ_CAD_ACTIVITY_DETECTED)
      //   irqStr += "CAD_ACTIVE ";
      // if (irqStr.isEmpty())
      //   irqStr = "none";
      String syncState;

      switch (datalink::getFhss().getFhssState()) {
      case datalink::FHSSState::Searching:
        syncState = "Searching";
        break;
      case datalink::FHSSState::Syncing:
        syncState = "Syncing";
        break;
      case datalink::FHSSState::Synced:
        syncState = "Synced";
      }

      auto &diversity = VCTR::ExVectrLink::datalink::getSx1280Diversity();
      auto bestLinkIndex = diversity.getCurrentBestLinkIndex();
      auto bestLink = diversity.getDiversityLink(bestLinkIndex);
      auto &radioA = VCTR::ExVectrLink::datalink::getDatalinkRadioA();
#if defined(HAS_DIVERSITY)
      auto &radioB = VCTR::ExVectrLink::datalink::getDatalinkRadioB();
#endif

      auto now = Core::NOW();

      String message =
          String() + syncState + " A: " +
          String(
              (double)VCTR::ExVectrLink::datalink::getFhss().getTimingOffset() /
              Core::MILLISECONDS) +
          " I: " +
          String((double)VCTR::ExVectrLink::datalink::getFhss()
                     .getTrueSlotInterval() /
                 Core::MICROSECONDS) +
          " RSSI:" + bestLink->lastPacketRSSI() +
          " SNR:" + bestLink->lastPacketSNR() + " LQ:" +
          String(VCTR::ExVectrLink::datalink::getFhss().getLinkQuality() * 100,
                 0) +
          " S:" + radioA.getState() + "|" + radioB.getState() + //
          // " L:" +
          // String((double)(now - radioA.getRxIdleStartTime()) /
          //        Core::MILLISECONDS) +
          // " L:" + (double)radioA.getLastRunTime() / Core::SECONDS //
          " A:" + bestLinkIndex + //
                                  // " R:" + radioA.lastPacketRSSI() //
#if defined(HAS_DIVERSITY)
      // + "|" + radioB.lastPacketRSSI() //
#endif
          +"\n";
      // LOG_MSG(message.c_str());
      VCTR::ExVectrLink::datalink::getDatalinkRadioA().clearRemainIrqFlags();
      serialTelecoms.sendSerialPacket(VCTR::ExVectrLink::packets::Error,
                                      message.c_str(), message.length() + 1);
#endif
    }
  }
};
ErrorSender errorSender;
#endif

// ISR-captured state for DIO1 interrupt timestamps
volatile int32_t dio1MicrosA = 0;
volatile bool dio1FiredA = false;
#if defined(HAS_DIVERSITY)
volatile int32_t dio1MicrosB = 0;
volatile bool dio1FiredB = false;
#endif

void IRAM_ATTR onRadioADio1() {
  dio1MicrosA = micros();
  dio1FiredA = true;
}

#if defined(HAS_DIVERSITY)
void IRAM_ATTR onRadioBDio1() {
  dio1MicrosB = micros();
  dio1FiredB = true;
}
#endif

void setup() {

  // delay(5000);

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
  Serial.begin(115200);
#endif

  pinMode(HardwareConfig::button, INPUT_PULLUP);

  // Serial.begin(115200);
#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
  linkSerialPort.begin(1000000, SERIAL_8N1, HardwareConfig::serialRx,
                       HardwareConfig::serialTx);
#endif

  delay(500);

  // Check button independently – works even when Serial is not yet open
  if (digitalRead(HardwareConfig::button) == LOW) {
    WifiUpload::startWiFiUploadMode(true);
    WifiUpload::initWifiUpload(serialTelecoms);
    return;
  }

#ifdef ENABLE_FORCEONLYWIFI
  WifiUpload::startWiFiUploadMode(true);
  return;
#endif

  uint8_t count = 0;
  auto startTime = millis();
  while (Serial.available() && millis() - startTime < 1000) {
    auto byte = Serial.read();
    if (byte == 'r') {
      count++;
    } else {
      count = 0;
    }
    if (count >= 5) {
      WifiUpload::startWiFiUploadMode(true);
      WifiUpload::initWifiUpload(serialTelecoms);
      return;
    }
  }

  pinMode(HardwareConfig::sx1280ADio1, INPUT);
#if defined(HAS_DIVERSITY)
  pinMode(HardwareConfig::sx1280BDio1, INPUT);
#endif

  // Setup DIO1 interrupts for precise timestamping
  attachInterrupt(digitalPinToInterrupt(HardwareConfig::sx1280ADio1),
                  onRadioADio1, RISING);
#if defined(HAS_DIVERSITY)
  attachInterrupt(digitalPinToInterrupt(HardwareConfig::sx1280BDio1),
                  onRadioBDio1, RISING);
#endif

#ifndef TARGET_EXVECTRCONTROLLER
  VCTR::ExVectrLink::datalink::getFhss().setIsRxSide(true);
#else
  VCTR::ExVectrLink::datalink::getFhss().setIsRxSide(false);
  VCTR::ExVectrLink::datalink::getSx1280Diversity().setDesignatedTxLink(
      VCTR::ExVectrLink::datalink::getDatalinkRadioA());
#endif

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
  Core::initialise();
#endif

  ExVectrLink::radio::initRadioDrivers();

  ExVectrLink::datalink::initCommandManager(serialTelecoms);

  WifiUpload::initWifiUpload(serialTelecoms);

  auto &radioA = ExVectrLink::datalink::getDatalinkRadioA();
#if defined(HAS_DIVERSITY)
  auto &radioB = ExVectrLink::datalink::getDatalinkRadioB();
#endif

  auto radioSetup = [](VCTR::network::datalink::Datalink_SX1280_V2 &radio) {
    radio.setBandwidth(VCTR::network::datalink::BW_800KHz);
    radio.setCodingRate(VCTR::network::datalink::LI_4_8);
    radio.setSpreadingFactor(VCTR::network::datalink::SF_7);
    radio.setPacketMode(VCTR::network::datalink::SX1280_PacketMode::Limited);
    radio.setFixedPacketLength(11);
    radio.setEnableAutoRx(false);
    radio.setChannel(0);
    radio.setEnableTxRx(true);
    radio.setTxMaxPower(HardwareConfig::sx1280AMaxPower);
    radio.setPAdbm(HardwareConfig::sx1280APAdbm);
    radio.setTxPower(20);
  };

  radioSetup(radioA);
#if defined(HAS_DIVERSITY)
  radioSetup(radioB);
#endif

  ExVectrLink::datalink::getDynamicPower().setMaxPower(
      HardwareConfig::sx1280AMaxPower);
  ExVectrLink::datalink::getDynamicPower().setPower(20);
  ExVectrLink::datalink::setupDynamicPower();

#if defined(ENABLE_SBUS) || defined(ENABLE_CRSF)
  Core::getSystemScheduler().removeTask(serialTelecoms);
#endif
}

void processDio1Interrupts() {
  if (dio1FiredA &&
      ExVectrLink::datalink::getDatalinkRadioA().getInitialised()) {
    // Atomically read ISR-captured micros and clear flag
    noInterrupts();
    int32_t isrMicros = dio1MicrosA;
    dio1FiredA = false;
    interrupts();
    // Reconstruct Core::NOW()-compatible timestamp by subtracting elapsed time
    int32_t elapsed = (int32_t)micros() - isrMicros;
    int64_t timestamp = Core::NOW() - int64_t(elapsed) * Core::MICROSECONDS;
    ExVectrLink::datalink::getDatalinkRadioA().notifyDio1Irq(timestamp);
  }
#if defined(HAS_DIVERSITY)
  if (dio1FiredB &&
      ExVectrLink::datalink::getDatalinkRadioB().getInitialised()) {
    noInterrupts();
    int32_t isrMicros = dio1MicrosB;
    dio1FiredB = false;
    interrupts();
    int32_t elapsed = (int32_t)micros() - isrMicros;
    int64_t timestamp = Core::NOW() - int64_t(elapsed) * Core::MICROSECONDS;
    ExVectrLink::datalink::getDatalinkRadioB().notifyDio1Irq(timestamp);
  }
#endif
}

void loop() {

  WifiUpload::loopWifiUpload();

  processDio1Interrupts();

  Core::getSystemScheduler().tick();
}