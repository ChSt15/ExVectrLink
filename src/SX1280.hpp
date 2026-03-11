#ifndef EXVECTRLINK_SX1280_HPP
#define EXVECTRLINK_SX1280_HPP

#include <Arduino.h>

#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/pin_gpio.hpp"

#include "ExVectrNetwork/datalink/sx1280/sx12xxAL/src/SX128XLT.h"

#include "Hardware.hpp"

namespace VCTR::ExVectrLink::radio {

VCTR::network::datalink::SX128XLT &getRadioDriverA();
VCTR::network::datalink::SX128XLT &getRadioDriverB();

bool radioDriversInitialized();

void initRadioDrivers();

void checkBusy();

} // namespace VCTR::ExVectrLink::radio

#endif // EXVECTRLINK_SX1280_HPP