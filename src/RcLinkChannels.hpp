#ifndef EXVECRLINK_RCLINKCHANNELS_HPP
#define EXVECRLINK_RCLINKCHANNELS_HPP

#include "ExVectrPackets/radiolink/packets/RadioLinkMuxPackets.hpp"

namespace VCTR::ExVectrLink {

/**
 * The RC channel configuration carried over the FHSS link, shared by the TX
 * side (CRSFRcInput -> commander) and RX side (CRSFRcOutput -> accumulator).
 * Both ends MUST use these aliases -- the template parameters are part of
 * the wire format.
 *
 * Priority (every packet):     analog 0-3  = roll/pitch/yaw/throttle
 *                                            (CRSF ch 0-3)
 *                              digital 0   = aux1 / arm (CRSF ch 4)
 * Multiplexed (1 per packet):  digital 1-5 = aux2-aux6 (CRSF ch 5-9),
 *                              so each refreshes every 5th RC packet.
 */
inline constexpr size_t kRcPriorityAnalog = 4;
inline constexpr size_t kRcPriorityDigital = 1;
inline constexpr size_t kRcMuxAnalog = 0;
inline constexpr size_t kRcMuxDigital = 5;

using RcLinkPacket = VCTR::packets::radiolink::RadioLinkMuxPacket<
    kRcPriorityAnalog, kRcPriorityDigital, kRcMuxAnalog, kRcMuxDigital>;
using RcLinkCommander = VCTR::packets::radiolink::RadioLinkMuxCommander<
    kRcPriorityAnalog, kRcPriorityDigital, kRcMuxAnalog, kRcMuxDigital>;
using RcLinkAccumulator = VCTR::packets::radiolink::RadioLinkMuxAccumulator<
    kRcPriorityAnalog, kRcPriorityDigital, kRcMuxAnalog, kRcMuxDigital>;

} // namespace VCTR::ExVectrLink

#endif // EXVECRLINK_RCLINKCHANNELS_HPP
