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

#if defined(TARGET_EXVECTRLINKTXMODULEDIVERSITY)
#include "targets/ExVectrLinkTxModuleDiversity.hpp"
#endif

#if defined(TARGET_GENERICESP8285)
#include "targets/GenericEsp8266.hpp"
#endif

#if defined(TARGET_GENERICESP8285WITHPA)
#include "targets/GenericEsp8266WithPA.hpp"
#endif

// Which end of the link this build is: the handset-side module / controller
// (IS_TX_SIDE) or the receiver in the aircraft. Anything that differs between
// the two ends -- the RX running at a fixed transmit power, the TX carrying
// the CRSF configuration menu -- keys off this rather than re-listing targets.
#if defined(TARGET_EXVECTRCONTROLLER) ||                                       \
    defined(TARGET_EXVECTRLINKTXMODULE) ||                                     \
    defined(TARGET_EXVECTRLINKTXMODULEDIVERSITY)
#define IS_TX_SIDE
#endif

#endif // EXVECRLINK_HARDWARE_HPP