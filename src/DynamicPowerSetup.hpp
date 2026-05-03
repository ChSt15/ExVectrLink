#ifndef EXVECRLINK_DYNAMICPOWERSETUP_HPP
#define EXVECRLINK_DYNAMICPOWERSETUP_HPP

#include "ExVectrLink/datalink/DynamicPower.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::ExVectrLink::datalink::DynamicPower &getDynamicPower();

void applyDynamicPowerToRadios();

void updateDynamicPowerFromRemoteLinkInfo(int8_t remoteRssi, int8_t remoteSnr,
                                          uint8_t remoteLq);

void setupDynamicPower();

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DYNAMICPOWERSETUP_HPP