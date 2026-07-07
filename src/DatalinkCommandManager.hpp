#ifndef EXVECRLINK_DATALINKCOMMANDMANAGER_HPP
#define EXVECRLINK_DATALINKCOMMANDMANAGER_HPP

#include <cstdint>

#include "ExVectrNetwork/DataPacket.hpp"
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

/// Identifies an independent OTA sender for queueOtaPacket()'s coalescing
/// queue -- each key gets its own slot in the queue, always updated with
/// the freshest data rather than falling behind, and shares the underlying
/// single-slot FHSS channel fairly with every other key. Add a new value
/// here for each new independent thing that needs to be sent over the air,
/// including each individually-sent piece of one logically larger packet
/// that's been manually split because it doesn't fit in a single OTA frame.
enum class OtaSendKey : uint8_t {
  RcChannels,        ///< CRSFRcInput: handset RC channels, ground -> FC.
  LinkTelemetry,     ///< DatalinkCommandManager: compact link stats.
  RxState,           ///< DatalinkCommandManager: desync/rate diagnostics.
  CrsfBatterySensor, ///< CRSFRcOutput: FC battery telemetry relay.
  CrsfGpsStats,    ///< CRSFRcOutput: FC GPS speed/satellites/altitude relay.
  CrsfGpsPosition, ///< CRSFRcOutput: FC GPS latitude/longitude relay --
                   ///< manually split from CrsfGpsStats since both
                   ///< together don't fit in one OTA frame.
  UploadModeTrigger, ///< CRSFRcInput: one-shot "enter WiFi upload mode"
                     ///< command relayed to the far end (see PacketTypes::
                     ///< UploadMode). Only ever carries this single message,
                     ///< so coalescing repeated clicks is harmless.
};

extern void
initCommandManager(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms);

/// Queues `packet` for transmission over the FHSS OTA link, coalesced by
/// `key` (see OtaSendKey/OtaPacketQueue) so independent senders share the
/// single-slot channel without racing to overwrite each other's pending
/// frame. Use this instead of calling getFhss().transmitDataframe()
/// directly for anything that represents an ongoing state (as opposed to a
/// one-off message, which should still go straight to transmitDataframe()
/// since coalescing would silently drop it if another update for the same
/// key arrives before it's sent).
/// @return false only if `key` is new and the internal queue is full.
bool queueOtaPacket(OtaSendKey key, const VCTR::network::DataPacket &packet);

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