#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/pin_gpio.hpp"

#include "Hardware.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

#include "SX1280.hpp"

#include "DatalinkSetup.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::ExVectrLink::radio;
using namespace VCTR::network::datalink;

VCTR::network::datalink::Datalink_SX1280_V2 &getDatalinkRadioA() {
  static Datalink_SX1280_V2 datalinkRadioA(getRadioDriverA());
  return datalinkRadioA;
}

#if defined(HAS_DIVERSITY)
VCTR::network::datalink::Datalink_SX1280_V2 &getDatalinkRadioB() {
  static Datalink_SX1280_V2 datalinkRadioB(getRadioDriverB());
  return datalinkRadioB;
}
#endif

} // namespace VCTR::ExVectrLink::datalink