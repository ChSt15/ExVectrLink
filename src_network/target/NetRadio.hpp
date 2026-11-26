#pragma once
// Radio glue shared by the network-mode firmwares (TX module node 2, RX module node 3). Arduino-dependent.
//
// Brings up the SX1280(s) through the same src/ objects as the FPV firmware (SX1280.cpp, DatalinkSetup.cpp,
// DiversitySetup.cpp, FhssSetup.cpp), configures the air profile (build-time choice: v1 by default, or the v2-trial with
// -DEXVECTRLINK_NET_PROFILE_V2_TRIAL, docs/air-link-optimisation.md section 8; v1 = LoRa SF5/BW1600/CR4/5, 12-symbol
// preamble, CRC on, implicit header, fixed 255 bytes; FHSS 20 ms slots, 1:1 roles, 2 slots per hop, own link
// version and hop key) and connects FHSS to the NetAirAccess (pull hook for TX slots, receive handler into the
// mailbox). DIO1 ISRs only record a timestamp, exactly as in src/main.cpp.
#include <stdint.h>

#include "ExVectrLink/datalink/FHSS.hpp"
#include "ExVectrLink/datalink/Sx1280Diversity.hpp"
#include "ExVectrNetwork/TimeSourceI.hpp"

#include "NetAirAccess.hpp"
#include "NetPower.hpp"

namespace VCTR::ExVectrLink::netmode {

/// The stack's clock: Core::NowNs(), the same clock FHSS and the radio timestamps use.
class CoreTime : public network::TimeSourceI {
public:
  int64_t nowNs() override;
};

/// Power at every boot of a network-mode module (10 mW legal limit, user 2026-11-20). It is the DEFAULT, no longer a
/// hard cap: the Router's Lua menu may raise the power in RAM up to the board maximum (src_network/NetPower.hpp).
constexpr int8_t kNetTxPowerCapDbm = kNetTxPowerDefaultDbm;

/// Resets and configures the radios, sets the FHSS role and profile and hooks `access` in. `txPowerDbm` is the
/// start power (clamped by the board maximum in the driver). Call once from setup().
void netRadioSetup(bool isRxSide, NetAirAccess &access, int8_t txPowerDbm);

/// Main loop: hands DIO1 timestamps to the drivers and runs the Core scheduler (FHSS, WiFi task).
void netRadioLoop();

/// Sets the TX power of every radio, clamped to 0..the board maximum (HardwareConfig::sx1280AMaxPower).
void netRadioSetTxPower(int8_t dbm);

/// FHSS state, power and antennas for the air access (call every loop).
AirLinkStatus netRadioStatus(int8_t txPowerDbm);

/// Expected packets per second for this side under air profile v1 (25).
uint16_t netExpectedRxPerSecond(bool isRxSide);

/// True while the FHSS has left Searching (for the sync failsafe).
bool netRadioLeftSearching();

datalink::FHSS &netFhss();
datalink::Sx1280Diversity &netDiversity();

} // namespace VCTR::ExVectrLink::netmode
