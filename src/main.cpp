#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrLink/SerialTelecoms.hpp"

#include "Hardware.hpp"

#include "DatalinkCommandManager.hpp"
#include "SX1280.hpp"
#include "WifiUpload.hpp"

#include "DatalinkSetup.hpp"

using namespace VCTR;

Platform::BusUART serialBus(Serial);
SerialTelecoms::ExVectrLinkSerialTelecoms serialTelecoms(serialBus);

class TempMonitor : public Core::Task_Periodic {
public:
  TempMonitor() : Core::Task_Periodic("TempMonitor", 5 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    int8_t temp = temperatureRead();
    serialTelecoms.sendSerialPacket(
        SerialTelecoms::SerialPacketType::DeviceTemperature, &temp, 1);
  }
};
TempMonitor tempMonitor;

void setup() {

  pinMode(HardwareConfig::button, INPUT_PULLUP);

  Serial.begin(115200);

  setCpuFrequencyMhz(80);

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

  WifiUpload::initWifiUpload(serialTelecoms);

  // Core::getSystemScheduler().removeTask(serialTelecoms);
}

void loop() {

  WifiUpload::loopWifiUpload();

  Core::getSystemScheduler().tick();
}