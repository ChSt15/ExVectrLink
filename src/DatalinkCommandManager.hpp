#ifndef EXVECRLINK_DATALINKCOMMANDMANAGER_HPP
#define EXVECRLINK_DATALINKCOMMANDMANAGER_HPP

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

#include "ExVectrLink/SerialTelecoms.hpp"

namespace VCTR::ExVectrLink::datalink {

extern void
initCommandManager(SerialTelecoms::ExVectrLinkSerialTelecoms &serialTelecoms);

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKCOMMANDMANAGER_HPP