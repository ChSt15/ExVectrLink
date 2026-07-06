#ifndef EXVECRLINK_HARDWARE_HPP
#define EXVECRLINK_HARDWARE_HPP

#if defined(TARGET_GENERICDIVERSITY_ELRS)
#include "targets/GenericDiversityELRS.hpp"
#endif

#if defined(TARGET_EXVECTRCONTROLLER)
#include "targets/ExVectrControllerLink.hpp"
#endif

#if defined(TARGET_EXVECTRLINKTXMODULE)
#include "targets/ExVectrLinkTxModule.hpp"
#endif

#if defined(TARGET_GENERICESP8285)
#include "targets/GenericEsp8266.hpp"
#endif

#if defined(TARGET_GENERICESP8285WITHPA)
#include "targets/GenericEsp8266WithPA.hpp"
#endif

#endif // EXVECRLINK_HARDWARE_HPP