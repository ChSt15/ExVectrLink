#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/datalink/Sx1280Diversity.hpp"

#include "DiversitySetup.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::ExVectrLink::datalink::Sx1280Diversity &getSx1280Diversity() {
#if defined(HAS_DIVERSITY)
  static Sx1280Diversity diversity(
      {&getDatalinkRadioA(), &getDatalinkRadioB()});
#else
  static Sx1280Diversity diversity({&getDatalinkRadioA()});
#endif
  return diversity;
}

} // namespace VCTR::ExVectrLink::datalink