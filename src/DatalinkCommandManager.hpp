#ifndef EXVECRLINK_DATALINKCOMMANDMANAGER_HPP
#define EXVECRLINK_DATALINKCOMMANDMANAGER_HPP

#include <cstdint>

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

#include "ExVectrLink/serial/SerialTelecoms.hpp"

namespace VCTR::ExVectrLink::datalink {

/// Decoded far-end link telemetry received via OTA LinkInfo packets.
struct RemoteLinkStats {
  uint8_t antenna = 0;
  uint8_t linkQuality = 0; ///< 0-100 %
  int8_t txPowerdBm = 0;   ///< dBm, 1-32
  bool dynamicPower = false;
  int8_t rssi = 0; ///< dBm (negative)
  int8_t snr = 0;  ///< dB (-15 to +15)
  bool valid = false;
};

extern void
initCommandManager(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms);

/// Returns the TX power (dBm) currently configured on this node.
int8_t getLocalTxPowerdBm();

/// Returns the most recently decoded far-end link stats (from OTA LinkInfo).
const RemoteLinkStats &getRemoteLinkStats();

/// Returns the far end's packet quality (0-100 %), from the OTA RxState
/// packet. Kept separate from RemoteLinkStats since it arrives via a
/// different OTA packet type (RxState vs LinkTelemetry).
uint8_t getRemotePacketQuality();

/// Returns the far end's CRSF TX-failure rate (frames/s that couldn't be
/// written to its flight controller because the send buffer was full),
/// from the OTA RxState packet. On the RX side this is 0 when CRSF frames
/// are flowing to the FC and climbs when the FC UART never accepts bytes.
uint16_t getRemoteCrsfTxFailureRateHz();

/// Returns the far end's RC packet rate (RC frames/s it received over the
/// radio and fed to its CRSF output), from the OTA RxState packet. >0
/// confirms the RX is decoding RC and building CRSF frames at all.
uint16_t getRemoteRcPacketRateHz();

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKCOMMANDMANAGER_HPP