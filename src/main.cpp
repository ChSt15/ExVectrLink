#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrLink/serial/SerialTelecoms.hpp"

#include "Hardware.hpp"

#include "DatalinkCommandManager.hpp"
#include "SX1280.hpp"
#include "WifiUpload.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "FhssSetup.hpp"

using namespace VCTR;
using namespace VCTR::ExVectrLink;

HardwareSerial linkSerialPort(1);
Platform::BusUART serialBus(linkSerialPort);
SerialTelecoms serialTelecoms(serialBus);

class TempMonitor : public Core::Task_Periodic {
public:
  TempMonitor() : Core::Task_Periodic("TempMonitor", 5 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    int8_t temp = temperatureRead();
    serialTelecoms.sendSerialPacket(
        packets::SerialPacket_DeviceTemperature{temp});
  }
};
TempMonitor tempMonitor;

class ErrorSender : public Core::Task_Periodic {
public:
  ErrorSender() : Core::Task_Periodic("ErrorSender", 1 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    auto error = !radio::radioDriversInitialized();
    // network::DataPacket packet;
    // packet.payload.setSize(5);
    // datalink::getFhss().transmitDataframe(packet);
    if (error) {
      serialTelecoms.sendSerialPacket(packets::Error,
                                      "Failed to initialize radios", 28);
    }
  }
};
ErrorSender errorSender;

void setup() {

  pinMode(HardwareConfig::button, INPUT_PULLUP);

  Serial.begin(115200);
  linkSerialPort.begin(460800, SERIAL_8N1, HardwareConfig::serialRx,
                       HardwareConfig::serialTx);

  setCpuFrequencyMhz(160);

  delay(500);
  uint8_t count = 0;
  auto startTime = millis();
  while (Serial.available() && millis() - startTime < 500) {
    auto byte = Serial.read();
    if (byte == 'r') {
      count++;
    } else {
      count = 0;
    }
    if (count >= 5) {
      WifiUpload::startWiFiUploadMode(true);
      break;
    }
  }

  Core::initialise();

  ExVectrLink::radio::initRadioDrivers();
  ExVectrLink::datalink::initCommandManager(serialTelecoms);

  WifiUpload::initWifiUpload(serialTelecoms);

  auto &radioA = ExVectrLink::datalink::getDatalinkRadioA();
  auto &radioB = ExVectrLink::datalink::getDatalinkRadioB();

  auto radioSetup = [](VCTR::network::datalink::Datalink_SX1280 &radio) {
    radio.setBandwidth(VCTR::network::datalink::BW_800KHz);
    radio.setCodingRate(VCTR::network::datalink::LI_4_6);
    radio.setSpreadingFactor(VCTR::network::datalink::SF_6);
    radio.setChannel(0);
    radio.enableTxRx(true);
    radio.cadBeforeSend(true);
    radio.setTxPower(12);
  };

  radioSetup(radioA);
  radioSetup(radioB);

  // Core::getSystemScheduler().removeTask(serialTelecoms);
}

void loop() {

  WifiUpload::loopWifiUpload();

  Core::getSystemScheduler().tick();
}