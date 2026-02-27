#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/datalink/FHSS.hpp"

#include "DiversitySetup.hpp"

#include "FhssSetup.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::ExVectrLink::datalink::FHSS &getFhss() {
  static FHSS fhss(getSx1280Diversity());
  return fhss;
}

} // namespace VCTR::ExVectrLink::datalink