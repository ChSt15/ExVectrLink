#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/datalink/FHSS.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"

#include "FhssSetup.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::ExVectrLink::datalink::FHSS &getFhss() {
#if defined(HAS_DIVERSITY)
  static FHSS fhss(getSx1280Diversity());
#else
  static FHSS fhss(getSx1280Diversity());
#endif
  return fhss;
}

} // namespace VCTR::ExVectrLink::datalink