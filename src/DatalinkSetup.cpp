#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/pin_gpio.hpp"

#include "Hardware.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280_Direct.hpp"

#include "SX1280.hpp"

#include "DatalinkSetup.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::ExVectrLink::radio;
using namespace VCTR::network::datalink;

VCTR::network::datalink::Sx1280_Direct &getDatalinkRadioA() {
  static Sx1280_Direct datalinkRadioA(getRadioDriverA());
  return datalinkRadioA;
}

#if defined(HAS_DIVERSITY)
VCTR::network::datalink::Sx1280_Direct &getDatalinkRadioB() {
  static Sx1280_Direct datalinkRadioB(getRadioDriverB());
  return datalinkRadioB;
}
#endif

} // namespace VCTR::ExVectrLink::datalink