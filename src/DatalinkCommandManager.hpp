#ifndef EXVECRLINK_DATALINKCOMMANDMANAGER_HPP
#define EXVECRLINK_DATALINKCOMMANDMANAGER_HPP

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

#include "ExVectrLink/serial/SerialTelecoms.hpp"

namespace VCTR::ExVectrLink::datalink {

extern void
initCommandManager(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms);

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKCOMMANDMANAGER_HPP