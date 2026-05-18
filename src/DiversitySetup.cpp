#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/pin_gpio.hpp"

#include "Hardware.hpp"

#include "ExVectrLink/datalink/Sx1280Diversity.hpp"

#include "DatalinkSetup.hpp"

#include "DiversitySetup.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::ExVectrLink::datalink::Sx1280Diversity &getSx1280Diversity() {
  static Sx1280Diversity diversity({
      &getDatalinkRadioB(),
#if defined(HAS_DIVERSITY)
  //&getDatalinkRadioB()
#endif
  });
  return diversity;
}

} // namespace VCTR::ExVectrLink::datalink