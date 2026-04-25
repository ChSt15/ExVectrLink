#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/pin_gpio.hpp"

#include "Hardware.hpp"

#include "ExVectrNetwork/datalink/sx1280/sx12xxAL/src/SX128XLT.h"
#include "ExVectrNetwork/datalink/sx1280/sx12xxAL/src/SX128XLT_Definitions.h"

#include "SX1280.hpp"

namespace VCTR::ExVectrLink::radio {

using namespace VCTR::network::datalink;

Platform::PinGPIO sx1280ANss(HardwareConfig::sx1280ANss);
Platform::PinGPIO sx1280ARst(HardwareConfig::sx1280ARst);
Platform::PinGPIO sx1280ADio1(HardwareConfig::sx1280ADio1);
Platform::PinGPIO sx1280ABusy(HardwareConfig::sx1280ABusy);
Platform::PinGPIO sx1280ARxEn(HardwareConfig::sx1280ARxEn);
Platform::PinGPIO sx1280ATxEn(HardwareConfig::sx1280ATxEn);

#if defined(HAS_DIVERSITY)
Platform::PinGPIO sx1280BNss(HardwareConfig::sx1280BNss);
Platform::PinGPIO sx1280BRst(HardwareConfig::sx1280BRst);
Platform::PinGPIO sx1280BDio1(HardwareConfig::sx1280BDio1);
Platform::PinGPIO sx1280BBusy(HardwareConfig::sx1280BBusy);
Platform::PinGPIO sx1280BRxEn(HardwareConfig::sx1280BRxEn);
Platform::PinGPIO sx1280BTxEn(HardwareConfig::sx1280BTxEn);
#endif

Platform::BusSPIDevice sx1280ASpiBus(SPI, sx1280ANss, true);

#if defined(HAS_DIVERSITY)
Platform::BusSPIDevice sx1280BSpiBus(SPI, sx1280BNss, true);
#endif

bool radiosInitialized = false;

SX128XLT &getRadioDriverA() {
  static SX128XLT sx1280ADriver(sx1280ASpiBus, DEVICE_SX1281, sx1280ANss,
                                sx1280ARst, sx1280ABusy, sx1280ADio1,
                                sx1280ARxEn, sx1280ATxEn);
  return sx1280ADriver;
}

#if defined(HAS_DIVERSITY)
SX128XLT &getRadioDriverB() {
  static SX128XLT sx1280BDriver(sx1280BSpiBus, DEVICE_SX1281, sx1280BNss,
                                sx1280BRst, sx1280BBusy, sx1280BDio1,
                                sx1280BRxEn, sx1280BTxEn);
  return sx1280BDriver;
}
#endif

bool radioDriversInitialized() { return radiosInitialized; }

void initRadioDrivers() {

  sx1280ANss.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  sx1280ARst.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  sx1280ADio1.init(HAL::GPIO_IOMODE_t::IOMODE_INPUT);
  sx1280ABusy.init(HAL::GPIO_IOMODE_t::IOMODE_INPUT);
  sx1280ARxEn.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  sx1280ATxEn.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);

#if defined(HAS_DIVERSITY)
  sx1280BNss.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  sx1280BRst.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  sx1280BDio1.init(HAL::GPIO_IOMODE_t::IOMODE_INPUT);
  sx1280BBusy.init(HAL::GPIO_IOMODE_t::IOMODE_INPUT);
  sx1280BRxEn.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  sx1280BTxEn.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
#endif

  sx1280ANss.setPinValue(HIGH);
  sx1280ARst.setPinValue(HIGH);
  sx1280ARxEn.setPinValue(LOW);
  sx1280ATxEn.setPinValue(LOW);

#if defined(HAS_DIVERSITY)
  sx1280BNss.setPinValue(HIGH);
  sx1280BRst.setPinValue(HIGH);
  sx1280BRxEn.setPinValue(LOW);
  sx1280BTxEn.setPinValue(LOW);
#endif
#if defined(ESP32)
  SPI.begin(HardwareConfig::spiSck, HardwareConfig::spiMiso,
            HardwareConfig::spiMosi);
#else
  SPI.begin(); // ESP8266 uses default SPI pins
#endif

  // Shared reset pulse for both radios.
  // Some SuperD boards appear to have reset coupling, so avoid driving
  // opposite levels on RST_A and RST_B.
  sx1280ARst.setPinValue(LOW);
#if defined(HAS_DIVERSITY)
  sx1280BRst.setPinValue(LOW);
#endif
  Core::delay(50 * Core::MILLISECONDS);

  sx1280ARst.setPinValue(HIGH);
#if defined(HAS_DIVERSITY)
  sx1280BRst.setPinValue(HIGH);
#endif
  // Core::delay(50 * Core::MILLISECONDS);

  int64_t startTime = Core::NOW();
  int64_t lastCheckTime = startTime;
  bool failed = false;
  while (sx1280ABusy.getPinValue()
#if defined(HAS_DIVERSITY)
         || sx1280BBusy.getPinValue()
#endif
  ) {
    lastCheckTime = Core::NOW();
    if (Core::NOW() - startTime > 1 * Core::MILLISECONDS) {
      LOG_MSG("ERROR: Radios did not become ready after reset\n");
      failed = true;
      break;
    }
  }
  if (!failed) {
    LOG_MSG("Radios ready after %lld us\n",
            (lastCheckTime - startTime) / Core::MICROSECONDS);
  }

  getRadioDriverA().setMode(MODE_STDBY_RC);

#if defined(HAS_DIVERSITY)
  getRadioDriverB().setMode(MODE_STDBY_RC);
#endif

  radiosInitialized = true;

  if (getRadioDriverA().checkDevice()) {
    LOG_MSG("Radio A initialized successfully\n");
  } else {
    LOG_MSG("Failed to initialize Radio A!\n");
    radiosInitialized = false;
  }
#if defined(HAS_DIVERSITY)
  if (getRadioDriverB().checkDevice()) {
    LOG_MSG("Radio B initialized successfully\n");
  } else {
    LOG_MSG("Failed to initialize Radio B!\n");
    radiosInitialized = false;
  }
#endif
}

} // namespace VCTR::ExVectrLink::radio