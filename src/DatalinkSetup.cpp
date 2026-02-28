#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/pin_gpio.hpp"

#include "Hardware.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"
#include "ExVectrNetwork/datalink/sx1280/sx12xxAL/src/SX128XLT.h"

#include "SX1280.hpp"

#include "DatalinkSetup.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::ExVectrLink::radio;
using namespace VCTR::network::datalink;

VCTR::network::datalink::Datalink_SX1280 &getDatalinkRadioA() {
  static Datalink_SX1280 datalinkRadioA(getRadioDriverA());
  return datalinkRadioA;
}
VCTR::network::datalink::Datalink_SX1280 &getDatalinkRadioB() {
  static Datalink_SX1280 datalinkRadioB(getRadioDriverB());
  return datalinkRadioB;
}

} // namespace VCTR::ExVectrLink::datalink