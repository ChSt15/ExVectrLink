#ifndef EXVECRLINK_DATALINKSETUP_HPP
#define EXVECRLINK_DATALINKSETUP_HPP

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::network::datalink::Datalink_SX1280 &getDatalinkRadioA();
VCTR::network::datalink::Datalink_SX1280 &getDatalinkRadioB();

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKSETUP_HPP