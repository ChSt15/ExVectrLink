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
  /// Far end's current FHSS sync latency compensation (see
  /// FHSS::getSyncLatencyCompensation()), microseconds, 0-2000 in 50us
  /// steps -- true readback of the live value, not just the last commanded
  /// one, so it still reflects reality if a write was lost or the far end
  /// hasn't applied it yet.
  uint16_t syncOffsetUs = 0;
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
  TestBlockTrigger,  ///< CRSFRcInput: one-shot "block for 500ms" bench-test
                     ///< command relayed to the far end (see PacketTypes::
                     ///< TestBlock).
  TestForceResyncTrigger, ///< CRSFRcInput: one-shot "desync and force
                          ///< Searching" bench-test command relayed to the
                          ///< far end (see PacketTypes::TestForceResync).
  SyncOffsetTrigger, ///< CRSFRcInput: one-shot "set FHSS sync latency
                     ///< compensation" command relayed to the far end (see
                     ///< PacketTypes::SetSyncOffset).
  ResetRcGapTrigger, ///< CRSFRcInput: one-shot "reset max CRSF RC-frame gap
                     ///< statistic" command relayed to the far end (see
                     ///< PacketTypes::ResetRcGapStat).
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

/// Same as queueOtaPacket(), but a new (not-yet-queued) entry is inserted at
/// the FRONT of the shared queue instead of the back, so `key` jumps ahead of
/// any backlog of lower-priority keys already waiting. For senders whose data
/// goes stale quickly and must not be held up behind that backlog.
/// @return false only if `key` is new and the internal queue is full.
bool queueOtaPacketFront(OtaSendKey key,
                         const VCTR::network::DataPacket &packet);

/// Same as queueOtaPacketFront(), but a new entry is inserted at the SECOND
/// position: whatever was already first in line still goes out next, then
/// `key`, then the rest of the backlog (see OtaPacketQueue::
/// updateQueueSecond()). Guarantees at most one other packet ever goes out
/// ahead of `key` without starving the entry at the front -- used by the TX
/// side's RC channels and the link statistics telemetry, so a burst of
/// queued telemetry-relay packets can no longer delay them by several slots.
/// @return false only if `key` is new and the internal queue is full.
bool queueOtaPacketSecond(OtaSendKey key,
                          const VCTR::network::DataPacket &packet);

/// Returns the TX power (dBm) currently configured on this node.
int8_t getLocalTxPowerdBm();

/// Returns the most recently decoded far-end link stats (from OTA LinkInfo).
const RemoteLinkStats &getRemoteLinkStats();

/// Returns the far end's CRSF TX-failure rate. No longer carried in the
/// RxState packet (dropped for the debug fields below); currently always 0.
uint16_t getRemoteCrsfTxFailureRateHz();

/// Returns the far end's RC packet rate. No longer carried in the RxState
/// packet (see getRemoteCrsfTxFailureRateHz); currently always 0.
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
/// Far end's max observed gap between successive CRSF RC-channel frames it
/// wrote to its CRSF port (see getCrsfMaxRcFrameGapUs()), microseconds, from
/// the OTA SlotStats packet.
uint32_t getRemoteMaxRcGapUs();
/// Zeros the locally-cached copy of getRemoteMaxRcGapUs(). Used by the Lua
/// "RX Gap Reset" command so the menu shows the reset immediately instead of
/// the stale value until the far end's next SlotStats report arrives.
void resetRemoteMaxRcGapCache();

/// Far end's current FHSS sync latency compensation, microseconds (0-2000,
/// 50us resolution) -- see RemoteLinkStats::syncOffsetUs. 0 until a
/// LinkTelemetry packet has been received from it.
uint16_t getRemoteSyncOffsetUs();

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DATALINKCOMMANDMANAGER_HPP