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
  RxState,           ///< DatalinkCommandManager: desync/runtime/FHSS-timing
                     ///< debug diagnostics (see encodeRxState).
  CrsfBatterySensor, ///< CRSFRcOutput: FC battery telemetry relay.
  CrsfGpsStats,    ///< CRSFRcOutput: FC GPS speed/satellites/altitude relay.
  CrsfGpsPosition, ///< CRSFRcOutput: FC GPS latitude/longitude relay --
                   ///< manually split from CrsfGpsStats since both
                   ///< together don't fit in one OTA frame.
  UploadModeTrigger, ///< CRSFRcInput: one-shot "enter WiFi upload mode"
                     ///< command relayed to the far end (see PacketTypes::
                     ///< UploadMode). Only ever carries this single message,
                     ///< so coalescing repeated clicks is harmless.
  SlotStats,         ///< DatalinkCommandManager: FHSS missed-slot counter
                     ///< (see PacketTypes::SlotStats), sent once per second.
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

/// Returns the far end's packet quality (0-100 %). No longer carried in the
/// RxState packet (dropped for the debug fields below); currently always 0.
uint8_t getRemotePacketQuality();

/// Returns the far end's CRSF TX-failure rate. No longer carried in the
/// RxState packet (see getRemotePacketQuality); currently always 0.
uint16_t getRemoteCrsfTxFailureRateHz();

/// Returns the far end's RC packet rate. No longer carried in the RxState
/// packet (see getRemotePacketQuality); currently always 0.
uint16_t getRemoteRcPacketRateHz();

// ── Far-end (RX-side) debug diagnostics, from the OTA RxState packet ──
// Shown on the TX-side Lua config menu. All are the values the far end last
// reported; 0 until an RxState packet has been received from it.

/// Far end's FHSS desync counter (wrapping 16-bit).
uint16_t getRemoteDesyncCount();
/// Far end's runtime since boot, in whole minutes (wrapping 16-bit).
uint16_t getRemoteRuntimeMinutes();
/// Far end's current FHSS slot timing offset, in microseconds (signed).
int16_t getRemoteFhssOffsetUs();
/// Far end's current FHSS slot-interval clock-drift correction, in
/// nanoseconds (signed; saturates at the int16 range).
int16_t getRemoteIntervalOffsetNs();
/// Far end's cumulative FHSS missed-slot (scheduling-latency) counter, from
/// the OTA SlotStats packet.
uint32_t getRemoteMissedSlots();

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKCOMMANDMANAGER_HPP