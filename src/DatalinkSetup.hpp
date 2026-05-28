#ifndef EXVECRLINK_DATALINKSETUP_HPP
#define EXVECRLINK_DATALINKSETUP_HPP

#include "Hardware.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280_Direct.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::network::datalink::Sx1280_Direct &getDatalinkRadioA();

#if defined(HAS_DIVERSITY)
VCTR::network::datalink::Sx1280_Direct &getDatalinkRadioB();
#endif

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKSETUP_HPP