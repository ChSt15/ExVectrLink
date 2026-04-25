#ifndef EXVECRLINK_DATALINKSETUP_HPP
#define EXVECRLINK_DATALINKSETUP_HPP

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::network::datalink::Datalink_SX1280_V2 &getDatalinkRadioA();

#if defined(HAS_DIVERSITY)
VCTR::network::datalink::Datalink_SX1280_V2 &getDatalinkRadioB();
#endif

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKSETUP_HPP