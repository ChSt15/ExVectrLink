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

using namespace VCTR;
using namespace VCTR::ExVectrLink;

HardwareSerial linkSerialPort(0);
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

void setup() {

  pinMode(HardwareConfig::button, INPUT_PULLUP);

  Serial.begin(115200);
  linkSerialPort.begin(115200, SERIAL_8N1, HardwareConfig::serialRx,
                       HardwareConfig::serialTx);

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

  ExVectrLink::datalink::getDatalinkRadioA().setPAdbm(
      HardwareConfig::sx1280APAdbm);
  ExVectrLink::datalink::getDatalinkRadioB().setPAdbm(
      HardwareConfig::sx1280BPAdbm);
  ExVectrLink::datalink::getDatalinkRadioA().setTxMaxPower(
      HardwareConfig::sx1280AMaxPower);
  ExVectrLink::datalink::getDatalinkRadioB().setTxMaxPower(
      HardwareConfig::sx1280BMaxPower);

  // Core::getSystemScheduler().removeTask(serialTelecoms);
}

void loop() {

  WifiUpload::loopWifiUpload();

  Core::getSystemScheduler().tick();
}