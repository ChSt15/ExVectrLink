#include <Arduino.h>

#include "ExVectrCore/print.hpp"

#include "ExVectrLink/datalink/LinkSettings.hpp"
#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "ExVectrLink/serial/SerialTelecomPackets.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "DynamicPowerSetup.hpp"
#include "FhssSetup.hpp"
#include "OtaPacketQueue.hpp"
#include "WifiUpload.hpp"

#if defined(ENABLE_CRSF)
#include "CRSFRcOutput.hpp"
#endif

#include "DatalinkCommandManager.hpp"

namespace VCTR::ExVectrLink::datalink {

using namespace VCTR::ExVectrLink::packets;

uint8_t packetNumber = 0;

int32_t rxDesyncCount = 0;
// Debug diagnostics decoded from the far end's RxState packet (see
// encodeRxState/decodeRxState) and surfaced on the TX-side Lua menu.
uint16_t rxRuntimeMinutes = 0;
int16_t rxFhssOffsetUs = 0;
int16_t rxIntervalOffsetNs = 0;
uint32_t rxMissedSlots = 0;
uint32_t rxMaxRcGapUs = 0;
// No longer carried in the RxState packet (dropped to make room for the debug
// fields above -- the OTA frame only holds 8 data bytes). Kept so existing
// consumers still compile; they now read a constant 0.
uint16_t rxPacketRateHz = 0;
uint16_t rxCrsfTxFailures = 0;

/// Decoded far-end link stats from the last received OTA LinkInfo packet.
static RemoteLinkStats remoteStats;
/// Core::NowNs() timestamp of the last OTA packet received from the far end,
/// of ANY type (Data, LinkTelemetry, RxState, ...) -- not just LinkTelemetry,
/// since e.g. RC channel Data packets flow far more often and are just as
/// good evidence the link is still alive.
static int64_t lastRemotePacketNs = 0;

// How long without ANY OTA packet from the far end before it's considered
// gone. Mirrors ExpressLRS's tx_main.cpp UpdateConnectDisconnectStatus(),
// which declares the link down after missing RX_LOSS_CNT telemetry packets
// in a row -- but keyed off any packet here, since LinkTelemetry specifically
// is only sent by LinkInfoSender roughly every 200ms (with an occasional
// ~1s gap for the interleaved RxState packet), while e.g. RC channel Data
// packets normally arrive far more frequently and are just as valid a signal
// that the far end is still there.
static constexpr int64_t REMOTE_LINK_STATS_TIMEOUT_NS =
    1000 * Core::MILLISECONDS;

const RemoteLinkStats &getRemoteLinkStats() {
  // Lazily invalidate a stale cache: without this, remoteStats.valid stayed
  // true forever after the first OTA packet, so consumers (e.g. CRSFRcInput's
  // CRSF Link Statistics to the handset) kept reporting the last-known-good
  // LQ/RSSI/SNR indefinitely even after the far end actually went away --
  // exactly the "telemetry lost but nothing shows it" gap ExpressLRS's own
  // connectionState timeout exists to avoid.
  if (remoteStats.valid &&
      Core::NowNs() - lastRemotePacketNs > REMOTE_LINK_STATS_TIMEOUT_NS) {
    remoteStats.valid = false;
    remoteStats.linkQuality = 0;
  }
  return remoteStats;
}

uint16_t getRemoteCrsfTxFailureRateHz() { return rxCrsfTxFailures; }

uint16_t getRemoteRcPacketRateHz() { return rxPacketRateHz; }

uint16_t getRemoteDesyncCount() { return (uint16_t)rxDesyncCount; }

uint16_t getRemoteRuntimeMinutes() { return rxRuntimeMinutes; }

int16_t getRemoteFhssOffsetUs() { return rxFhssOffsetUs; }

int16_t getRemoteIntervalOffsetNs() { return rxIntervalOffsetNs; }

uint32_t getRemoteMissedSlots() { return rxMissedSlots; }

uint32_t getRemoteMaxRcGapUs() { return rxMaxRcGapUs; }

void resetRemoteMaxRcGapCache() { rxMaxRcGapUs = 0; }

uint16_t getRemoteSyncOffsetUs() { return remoteStats.syncOffsetUs; }

// ---------------------------------------------------------------------------
// Shared outgoing OTA queue -- see OtaPacketQueue.hpp / OtaSendKey. Every
// independent state-like sender (RC channels, link stats, telemetry relay,
// ...) queues through here instead of calling getFhss().transmitDataframe()
// directly, so they share the single-slot channel instead of racing to
// overwrite each other's pending frame.
// ---------------------------------------------------------------------------

static OtaPacketQueue<8> otaSendQueue;

// Defined below (needs encodeLinkInfo & friends); used by OtaQueuePump as
// the empty-queue filler.
static VCTR::network::DataPacket buildLinkTelemetryOtaPacket();

bool queueOtaPacket(OtaSendKey key, const VCTR::network::DataPacket &packet) {
  return otaSendQueue.updateQueue((uint8_t)key, packet);
}

bool queueOtaPacketFront(OtaSendKey key,
                         const VCTR::network::DataPacket &packet) {
  return otaSendQueue.updateQueueFront((uint8_t)key, packet);
}

bool queueOtaPacketSecond(OtaSendKey key,
                          const VCTR::network::DataPacket &packet) {
  return otaSendQueue.updateQueueSecond((uint8_t)key, packet);
}

class OtaQueuePump : public VCTR::Core::Task_Periodic {
public:
  OtaQueuePump() : Task_Periodic("OtaQueuePump", 2 * Core::MILLISECONDS) {
    VCTR::Core::getSystemScheduler().addTask(*this);
  }

  void taskCheck() override {
    // Wake early the moment there's both something queued and a free slot
    // to put it in, rather than waiting for the next periodic tick.
    if (!otaSendQueue.isEmpty() && !getFhss().isChannelBlocked()) {
      setDeadline(Core::NowNs());
    }
  }

  void taskThread() override {
    if (getFhss().isChannelBlocked()) {
      return; // Previous frame hasn't gone out over the air yet.
    }
    uint8_t key;
    network::DataPacket packet;
    if (!otaSendQueue.dequeue(key, packet)) {
      // Nothing queued: fill the free slot with a fresh link-statistics
      // packet instead of letting an empty frame go out, so the link
      // always carries data when there is any to carry.
      getFhss().transmitDataframe(buildLinkTelemetryOtaPacket());
      return;
    }
    if (!getFhss().transmitDataframe(packet)) {
      // Shouldn't normally happen right after isChannelBlocked() said
      // clear, but don't lose the packet if it does.
      otaSendQueue.updateQueue(key, packet);
    }
  }
} otaQueuePump;

static uint8_t quantizeLinkQualityPercent(float lq);

static uint16_t getLocalRcPacketRateHz() {
#if defined(ENABLE_CRSF)
  return VCTR::ExVectrLink::getCrsfRcPacketRateHz();
#else
  return 0;
#endif
}

static uint16_t getLocalCrsfTxFailureRateHz() {
#if defined(ENABLE_CRSF)
  return VCTR::ExVectrLink::getCrsfTxFailureRateHz();
#else
  return 0;
#endif
}

static uint32_t getLocalMaxRcFrameGapUs() {
#if defined(ENABLE_CRSF)
  return VCTR::ExVectrLink::getCrsfMaxRcFrameGapUs();
#else
  return 0;
#endif
}

static const VCTR::network::datalink::Sx1280_DirectI &getLinkStatsRadio() {
#if defined(HAS_DIVERSITY)
  return getSx1280Diversity();
#else
  return getDatalinkRadioA();
#endif
}

int8_t getLocalTxPowerdBm() { return getDatalinkRadioA().getTxPower(); }

static void applyLocalTxPowerToRadios() {
  const int8_t txPowerDbm = getDynamicPower().getPower();
  getDatalinkRadioA().setTxPower(txPowerDbm);
#if defined(HAS_DIVERSITY)
  getDatalinkRadioB().setTxPower(txPowerDbm);
#endif
}

VCTR::ExVectrLink::SerialTelecoms *serialTelecomsPtr = nullptr;

VCTR::Core::StaticCallback_Subscriber<const char *> logSubscriber;

SerialPacket_LinkInfo linkInfoPacket;

// ---------------------------------------------------------------------------
// Helpers: encode / decode the compact 5-byte OTA LinkInfo payload
//
// Bit layout (28 bits used, 12 spare):
//   Byte 0  [7:5] antenna  (3 bits, 0-7)
//           [4:0] LQ[6:2]  (upper 5 bits of 7-bit quality field)
//   Byte 1  [7:6] LQ[1:0]  (lower 2 bits of 7-bit quality field)
//           [5:1] txPower  (5 bits, value 0-31 → 1-32 dBm)
//           [0]   spare
//   Byte 2  [7:0] RSSI magnitude (abs dBm, 0-127)
//   Byte 3  [7:3] SNR     (5 bits, 0-30 → -15 to +15 dB in 1 dB steps)
//           [2:0] spare
//   Byte 4  [0]   dynamicPower
//           [7:1] spare
// ---------------------------------------------------------------------------

static uint8_t quantizeLinkQualityPercent(float lq) {
  float clamped = lq < 0.0f ? 0.0f : (lq > 1.0f ? 1.0f : lq);
  return (uint8_t)(clamped * 100.0f + 0.5f);
}

// Byte 4 layout: [0] dynamicPower, [6:1] sync offset step (0-40, 50us each,
// 0-2000us), [7] spare.
static uint8_t syncOffsetUsToStep(uint16_t syncOffsetUs) {
  uint16_t clamped = syncOffsetUs > 2000 ? 2000 : syncOffsetUs;
  uint8_t step = (uint8_t)((clamped + 25) / 50); // round to nearest step
  return step > 40 ? 40 : step;
}

static void encodeLinkInfo(uint8_t antenna, float lq, int8_t txPowerdBm,
                           int16_t rssi, float snr, bool dynamicPower,
                           uint16_t syncOffsetUs, uint8_t out[5]) {
  uint8_t antVal = antenna & 0x07;
  uint8_t lqVal = quantizeLinkQualityPercent(lq);
  int8_t pwr = txPowerdBm < 1 ? 1 : (txPowerdBm > 32 ? 32 : txPowerdBm);
  uint8_t pwrVal = (uint8_t)(pwr - 1) & 0x1F;
  int16_t mag = rssi < 0 ? -rssi : rssi;
  uint8_t rssiVal = (uint8_t)(mag > 127 ? 127 : mag);
  float snrC = snr < -15.0f ? -15.0f : (snr > 15.0f ? 15.0f : snr);
  uint8_t snrVal = (uint8_t)(snrC + 15.0f + 0.5f) & 0x1F;
  uint8_t syncStep = syncOffsetUsToStep(syncOffsetUs);

  out[0] = (uint8_t)((antVal << 5) | (lqVal >> 2));
  out[1] = (uint8_t)(((lqVal & 0x03) << 6) | (pwrVal << 1));
  out[2] = rssiVal;
  out[3] = (uint8_t)(snrVal << 3);
  out[4] = (uint8_t)((dynamicPower ? 0x01 : 0x00) | (syncStep << 1));
}

static void decodeLinkInfo(const uint8_t in[5], uint8_t &antenna, float &lq,
                           int8_t &txPowerdBm, int8_t &rssi, float &snr,
                           bool &dynamicPower, uint16_t &syncOffsetUs) {
  antenna = (in[0] >> 5) & 0x07;
  uint8_t lqRaw = (uint8_t)(((in[0] & 0x1F) << 2) | ((in[1] >> 6) & 0x03));
  lq = (float)lqRaw / 100.0f;
  txPowerdBm = (int8_t)(((in[1] >> 1) & 0x1F) + 1);
  rssi = -(int8_t)(in[2] & 0x7F);
  uint8_t snrRaw = (in[3] >> 3) & 0x1F;
  snr = (float)snrRaw - 15.0f;
  dynamicPower = (in[4] & 0x01) != 0;
  uint8_t syncStep = (in[4] >> 1) & 0x3F;
  if (syncStep > 40) {
    syncStep = 40;
  }
  syncOffsetUs = (uint16_t)syncStep * 50;
}

// The 8-byte OTA RxState payload -- far-end (RX-side) debug diagnostics shown
// on the TX Lua menu. All four fields are 16-bit, filling the frame exactly:
//   [0..1] desync count      [2..3] runtime (minutes)
//   [4..5] FHSS offset (us)  [6..7] FHSS interval correction (ns)
// FHSS offset / interval are signed; they saturate at the int16 range (the
// interval correction can in theory reach ~1e5 ns, but is a few hundred ns in
// normal operation).
static int16_t clampToI16(int64_t v) {
  if (v > INT16_MAX)
    return INT16_MAX;
  if (v < INT16_MIN)
    return INT16_MIN;
  return (int16_t)v;
}

static void encodeRxState(uint16_t desyncCount, uint16_t runtimeMinutes,
                          int16_t fhssOffsetUs, int16_t intervalOffsetNs,
                          uint8_t out[8]) {
  out[0] = (uint8_t)(desyncCount & 0xFF);
  out[1] = (uint8_t)((desyncCount >> 8) & 0xFF);
  out[2] = (uint8_t)(runtimeMinutes & 0xFF);
  out[3] = (uint8_t)((runtimeMinutes >> 8) & 0xFF);
  out[4] = (uint8_t)((uint16_t)fhssOffsetUs & 0xFF);
  out[5] = (uint8_t)(((uint16_t)fhssOffsetUs >> 8) & 0xFF);
  out[6] = (uint8_t)((uint16_t)intervalOffsetNs & 0xFF);
  out[7] = (uint8_t)(((uint16_t)intervalOffsetNs >> 8) & 0xFF);
}

static void decodeRxState(const uint8_t in[8], uint16_t &desyncCount,
                          uint16_t &runtimeMinutes, int16_t &fhssOffsetUs,
                          int16_t &intervalOffsetNs) {
  desyncCount = (uint16_t)in[0] | ((uint16_t)in[1] << 8);
  runtimeMinutes = (uint16_t)in[2] | ((uint16_t)in[3] << 8);
  fhssOffsetUs = (int16_t)((uint16_t)in[4] | ((uint16_t)in[5] << 8));
  intervalOffsetNs = (int16_t)((uint16_t)in[6] | ((uint16_t)in[7] << 8));
}

// SlotStats OTA payload -- the far end's cumulative FHSS missed-slot counter
// (scheduling latency, not RF loss) in bytes 0-3, plus its max observed gap
// between successive CRSF RC-channel frame writes (microseconds -- see
// getCrsfMaxRcFrameGapUs()) in bytes 4-7, filling the same 8-data +
// 1-type-byte length every other OTA packet uses (the receive path reads the
// type from the last byte).
static void encodeSlotStats(uint32_t missedSlots, uint32_t maxRcGapUs,
                            uint8_t out[8]) {
  out[0] = (uint8_t)(missedSlots & 0xFF);
  out[1] = (uint8_t)((missedSlots >> 8) & 0xFF);
  out[2] = (uint8_t)((missedSlots >> 16) & 0xFF);
  out[3] = (uint8_t)((missedSlots >> 24) & 0xFF);
  out[4] = (uint8_t)(maxRcGapUs & 0xFF);
  out[5] = (uint8_t)((maxRcGapUs >> 8) & 0xFF);
  out[6] = (uint8_t)((maxRcGapUs >> 16) & 0xFF);
  out[7] = (uint8_t)((maxRcGapUs >> 24) & 0xFF);
}

static void decodeSlotStats(const uint8_t in[8], uint32_t &missedSlots,
                            uint32_t &maxRcGapUs) {
  missedSlots = (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
                ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
  maxRcGapUs = (uint32_t)in[4] | ((uint32_t)in[5] << 8) |
               ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
}

static uint8_t getLocalTxAntennaIndex() {
#if defined(HAS_DIVERSITY)
  return (uint8_t)getSx1280Diversity().getCurrentTxLinkIndex();
#else
  return 0;
#endif
}

/// Builds an OTA LinkTelemetry packet from the current local link state.
/// Shared by LinkInfoSender's fixed 2 Hz high-priority send and
/// OtaQueuePump's empty-queue filler.
static VCTR::network::DataPacket buildLinkTelemetryOtaPacket() {
  const auto &radio = getLinkStatsRadio();

  uint8_t payload[8] = {};
  encodeLinkInfo(getLocalTxAntennaIndex(), getFhss().getLinkQuality(),
                 getLocalTxPowerdBm(), radio.getPacketRSSI(),
                 (float)radio.getPacketSNR(),
                 getDynamicPower().isDynamicPowerEnabled(),
                 (uint16_t)(getFhss().getSyncLatencyCompensation() /
                            Core::MICROSECONDS),
                 payload);

  network::DataPacket otaPkt;
  for (size_t i = 0; i < 8; i++) {
    otaPkt.payload.append(payload[i]);
  }
  otaPkt.payload.append((uint8_t)PacketTypes::LinkTelemetry);
  return otaPkt;
}

class LinkInfoSender : public VCTR::Core::Task_Periodic {

  int64_t lastLinkTelemetrySendTime = 0;
  int64_t lastRxStatePacketSendTime = 0;

  // LinkTelemetry carries the RSSI/SNR/LQ that the far end's dynamic-power
  // loop (see updateDynamicPowerFromRemoteLinkInfo()) depends on, so it's
  // timed independently of -- and sent at a fixed 2 Hz regardless of --
  // RxState/SlotStats below, and queued via queueOtaPacketSecond() so at
  // most one packet of the backlog of lower-priority OTA traffic (CRSF
  // sensor relay, ...) ever goes out ahead of it instead of it going stale
  // behind all of it -- second rather than front so it can't itself starve
  // the TX side's RC channels, which claim the same position. On top of
  // this baseline, OtaQueuePump sends an extra LinkTelemetry whenever the
  // OTA queue runs empty, so idle slots still carry fresh link stats.
  static constexpr int64_t kLinkTelemetryIntervalNs = 500 * Core::MILLISECONDS;
  static constexpr int64_t kRxStateIntervalNs = 1000 * Core::MILLISECONDS;

public:
  // Ticks well inside the 500 ms LinkTelemetry period so each stream's
  // deadline below is hit with low jitter.
  LinkInfoSender() : Task_Periodic("LinkInfoSender", 50 * Core::MILLISECONDS) {
    VCTR::Core::getSystemScheduler().addTask(*this);
  }

  void taskThread() override {

    if (serialTelecomsPtr == nullptr) {
      return;
    }

    const int64_t now = Core::NowNs();

    if (now - lastLinkTelemetrySendTime > kLinkTelemetryIntervalNs) {
      lastLinkTelemetrySendTime = now;
      sendLinkTelemetryPacket();
    }

    if (now - lastRxStatePacketSendTime > kRxStateIntervalNs) {
      lastRxStatePacketSendTime = now;
      sendRxStatePacket();
      sendSlotStatsPacket();
    }
  }

  void sendLinkTelemetryPacket() {
    const auto &radio = getLinkStatsRadio();
    auto &dynamicPower = getDynamicPower();

    // --- Fill local side stats ---
    linkInfoPacket.local.rssi = (int8_t)radio.getPacketRSSI();
    linkInfoPacket.local.snr = (int8_t)radio.getPacketSNR();
    linkInfoPacket.local.antenna = getLocalTxAntennaIndex();
    const uint8_t localLinkQuality =
        quantizeLinkQualityPercent(getFhss().getLinkQuality());
    linkInfoPacket.local.linkQuality = localLinkQuality;
    linkInfoPacket.local.lossRate = (uint8_t)(100 - localLinkQuality);
    // Same value as linkQuality -- kept only for serial wire-format
    // compatibility with existing hosts.
    linkInfoPacket.local.packetQuality = localLinkQuality;
    const int8_t localTxPowerdBm = getLocalTxPowerdBm();
    linkInfoPacket.local.txPower = localTxPowerdBm;
    linkInfoPacket.local.dynamicPower = dynamicPower.isDynamicPowerEnabled();
    linkInfoPacket.local.desyncCount = getFhss().getDesyncCounter();
    linkInfoPacket.local.packetRate = getLocalRcPacketRateHz();
    linkInfoPacket.local.crsfTxFailures = getLocalCrsfTxFailureRateHz();

    // --- Fill remote side stats (from last received OTA LinkTelemetry) ---
    linkInfoPacket.remote.rssi = remoteStats.rssi;
    linkInfoPacket.remote.snr = remoteStats.snr;
    linkInfoPacket.remote.antenna = remoteStats.antenna;
    linkInfoPacket.remote.linkQuality = remoteStats.linkQuality;
    linkInfoPacket.remote.lossRate = (uint8_t)(100 - remoteStats.linkQuality);
    linkInfoPacket.remote.packetQuality = remoteStats.linkQuality;
    linkInfoPacket.remote.txPower = remoteStats.txPowerdBm;
    linkInfoPacket.remote.dynamicPower = remoteStats.dynamicPower;
    linkInfoPacket.remote.desyncCount = rxDesyncCount;
    linkInfoPacket.remote.packetRate = rxPacketRateHz;
    linkInfoPacket.remote.crsfTxFailures = rxCrsfTxFailures;

    linkInfoPacket.remoteValid = remoteStats.valid;

    if (getFhss().getFhssState() != FHSSState::Synced) {
      // If not synced, override link quality to 0 and mark remote as invalid.
      linkInfoPacket.remote.linkQuality = 0;
      linkInfoPacket.remote.packetQuality = 0;
      linkInfoPacket.remoteValid = false;
    }

    // Send combined stats to the serial host.
    serialTelecomsPtr->sendSerialPacket(linkInfoPacket);

    // Also broadcast our link stats to the other end over-the-air.
    queueOtaPacketSecond(OtaSendKey::LinkTelemetry,
                         buildLinkTelemetryOtaPacket());
  }

  void sendRxStatePacket() {

    const uint16_t runtimeMinutes = (uint16_t)((millis() / 60000UL) & 0xFFFF);
    const int16_t fhssOffsetUs = clampToI16(getFhss().getTimingOffset() / 1000);
    const int16_t intervalOffsetNs = clampToI16(getFhss().getIntervalCorrection());

    uint8_t payload[8];
    encodeRxState((uint16_t)getFhss().getDesyncCounter(), runtimeMinutes,
                  fhssOffsetUs, intervalOffsetNs, payload);

    network::DataPacket otaPkt;
    for (size_t i = 0; i < 8; i++) {
      otaPkt.payload.append(payload[i]);
    }
    otaPkt.payload.append((uint8_t)PacketTypes::RxState);
    queueOtaPacket(OtaSendKey::RxState, otaPkt);
  }

  void sendSlotStatsPacket() {
    uint8_t payload[8];
    encodeSlotStats(getFhss().getMissedSlotCounter(),
                    getLocalMaxRcFrameGapUs(), payload);

    network::DataPacket otaPkt;
    for (size_t i = 0; i < 8; i++) {
      otaPkt.payload.append(payload[i]);
    }
    otaPkt.payload.append((uint8_t)PacketTypes::SlotStats);
    queueOtaPacket(OtaSendKey::SlotStats, otaPkt);
  }

} linkInfoSender;

void printToSerial(const char *const &message) {
  if (serialTelecomsPtr != nullptr) {
    SerialPacket_Print packet;
    strncpy(packet.message, message, sizeof(packet.message) - 1);
    packet.length = strnlen(packet.message, sizeof(packet.message) - 1);
    serialTelecomsPtr->sendSerialPacket(packet);
  }
}

void initCommandManager(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms) {

  serialTelecomsPtr = &serialTelecoms;

  // logSubscriber.subscribe(VCTR::Core::getDebugTopic());
  // logSubscriber.setCallbackFunction(printToSerial);

  serialTelecoms.addSerialPacketHandler(
      SerialPacketType::PacketData,
      [&serialTelecoms](const Core::ListArray<uint8_t> &data) {
        if (data.size() > 0) {
          network::DataPacket packet(data);
          // Append the type byte so the far end knows this is a data packet.
          packet.payload.append((uint8_t)PacketTypes::Data);
          getFhss().transmitDataframe(packet);
        }
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetModulationPreset>(
      [&serialTelecoms](const SerialPacket_SetModulationPreset &packet) {
        const auto &modParams = modulationPresets[packet.presetIndex];
        getDatalinkRadioA().setCodingRate(modParams.codingRate);
        getDatalinkRadioA().setSpreadingFactor(modParams.spreadingFactor);
        getDatalinkRadioA().setBandwidth(modParams.bandwidth);

#if defined(HAS_DIVERSITY)
        getDatalinkRadioB().setCodingRate(modParams.codingRate);
        getDatalinkRadioB().setSpreadingFactor(modParams.spreadingFactor);
        getDatalinkRadioB().setBandwidth(modParams.bandwidth);
#endif
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetPowerParams>(
      [](const SerialPacket_SetPowerParams &packet) {
        auto &dynamicPower = getDynamicPower();
        const uint8_t requestedMaxPower =
            (packet.txPower == 0) ? 255 : packet.txPower;
        const uint8_t requestedDynMaxPower =
            (packet.maxDynPower == 0) ? requestedMaxPower : packet.maxDynPower;
        const uint8_t requestedDynMinPower = (packet.minDynPower == 0)
                                                 ? DynamicPower::kPowerLevels[0]
                                                 : packet.minDynPower;

        dynamicPower.setMaxPower(requestedMaxPower);
        dynamicPower.setEnableDynamicPower(packet.enableDynamicPower);

        dynamicPower.setDynMaxPower(requestedDynMaxPower);
        dynamicPower.setDynMinPower(requestedDynMinPower);

        if (!packet.enableDynamicPower) {
          dynamicPower.setPower(requestedMaxPower);
        }

        applyLocalTxPowerToRadios();
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetLinkChannel>(
      [&serialTelecoms](const SerialPacket_SetLinkChannel &packet) {
        // getFhss().enableFhss(false);
        // getSx1280Diversity().setChannel(packet.channelIndex);
      });

  serialTelecoms.addSerialPacketHandler<SerialPacket_SetEnableFhss>(
      [&serialTelecoms](const SerialPacket_SetEnableFhss &packet) {
        // getFhss().enableFhss(packet.enable);
        getFhss().setFhssKey(packet.seqKey);
      });

  // Any valid FHSS packet -- with or without payload data -- proves the far
  // end is still transmitting. The receive handler below only fires for
  // data-carrying packets, so empty keep-alive frames alone would let
  // remoteStats time out (see REMOTE_LINK_STATS_TIMEOUT_NS) and the CRSF
  // link stats sent to the handset would report uplink LQ/RSSI of 0 --
  // read as "connection lost" -- despite perfect slot reception.
  getFhss().addSlotHandler([](datalink::FHSSSlotEvent event, bool isRxSlot,
                              bool receivedPacket, datalink::FHSSState) {
    if (event == datalink::FHSSSlotEvent::SlotEnd && isRxSlot &&
        receivedPacket) {
      lastRemotePacketNs = Core::NowNs();
    }
  });

  // Receiving info from far end over FHSS
  getFhss().addReceiveHandler([&serialTelecoms](
                                  const VCTR::network::DataPacket &packet) {
    // Serial.printf("Received %d byte FHSS packet with payload: ",
    //               packet.payload.size());
    // for (size_t i = 0; i < packet.payload.size(); i++) {
    //   Serial.print(packet.payload[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();
    // return;

    if (packet.payload.size() == 0) {
      return;
    }
    // Any packet from the far end (RC channels, telemetry, link stats, ...)
    // is evidence the link is still alive -- see REMOTE_LINK_STATS_TIMEOUT_NS.
    lastRemotePacketNs = Core::NowNs();

    // Last byte is always the packet type.
    uint8_t pktType = packet.payload[packet.payload.size() - 1];

    if (pktType == (uint8_t)PacketTypes::Data) {
      // Strip the type byte and forward raw data to the serial host.
      serialTelecoms.sendSerialPacket(SerialPacketType::PacketData,
                                      packet.payload.getPtr(),
                                      packet.payload.size() - 1);

    } else if (pktType == (uint8_t)PacketTypes::LinkTelemetry) {
      // Decode the 5-byte OTA link telemetry and forward to serial host.
      // if (packet.payload.size() < 8)
      //   return;
      // Decode into remoteStats cache; the next LinkInfoSender tick will
      // bundle this into the combined SerialPacket_LinkInfo sent to the
      // host.
      float lq;
      float snrF;
      int8_t remRssi;
      int8_t remTxPwr;
      uint8_t remAnt;
      bool remDynamicPower;
      uint16_t remSyncOffsetUs;
      decodeLinkInfo(packet.payload.getPtr(), remAnt, lq, remTxPwr, remRssi,
                     snrF, remDynamicPower, remSyncOffsetUs);
      const uint8_t remLqPercent = quantizeLinkQualityPercent(lq);

      updateDynamicPowerFromRemoteLinkInfo(remRssi, (int8_t)snrF, remLqPercent);

      remoteStats.antenna = remAnt;
      remoteStats.linkQuality = remLqPercent;
      remoteStats.txPowerdBm = remTxPwr;
      remoteStats.rssi = remRssi;
      remoteStats.snr = (int8_t)snrF;
      remoteStats.valid = true;
      remoteStats.dynamicPower = remDynamicPower;
      remoteStats.syncOffsetUs = remSyncOffsetUs;

    } else if (pktType == (uint8_t)PacketTypes::RxState) {

      // Decode the 8-byte OTA RxState packet: the far end's desync count plus
      // the runtime / FHSS timing diagnostics shown on the TX Lua menu.
      uint16_t remDesyncCount;
      uint16_t remRuntimeMinutes;
      int16_t remFhssOffsetUs;
      int16_t remIntervalOffsetNs;
      decodeRxState(packet.payload.getPtr(), remDesyncCount, remRuntimeMinutes,
                    remFhssOffsetUs, remIntervalOffsetNs);

      rxDesyncCount = remDesyncCount;
      rxRuntimeMinutes = remRuntimeMinutes;
      rxFhssOffsetUs = remFhssOffsetUs;
      rxIntervalOffsetNs = remIntervalOffsetNs;
    } else if (pktType == (uint8_t)PacketTypes::SlotStats) {
      // Far end's FHSS missed-slot (scheduling-latency) counter and its max
      // CRSF RC-frame output gap.
      uint32_t remMissedSlots;
      uint32_t remMaxRcGapUs;
      decodeSlotStats(packet.payload.getPtr(), remMissedSlots, remMaxRcGapUs);
      rxMissedSlots = remMissedSlots;
      rxMaxRcGapUs = remMaxRcGapUs;
    } else if (pktType == (uint8_t)PacketTypes::UploadMode) {
      // OTA upload mode packet -- forward to serial host.
      VCTR::WifiUpload::startWiFiUploadMode();
    } else if (pktType == (uint8_t)PacketTypes::TestBlock) {
      // Deliberately blocks right here, inside the FHSS receive-handler
      // callback (called from FHSS::taskThread()), so the stall looks like a
      // real scheduling hiccup on this side -- e.g. testing whether the
      // in-flight-reception grace window and the missed-slot catch-up path
      // recover cleanly afterwards.
      delay(500);
    } else if (pktType == (uint8_t)PacketTypes::TestForceResync) {
      getFhss().triggerFullResyncTest();
    } else if (pktType == (uint8_t)PacketTypes::SetSyncOffset) {
      // 2-byte little-endian microseconds, 0-2000 (see PacketTypes::
      // SetSyncOffset). Applied straight to this side's own FHSS -- meant
      // for the RX side; harmless (unused) if received on the TX side.
      if (packet.payload.size() >= 3) {
        const uint16_t us = (uint16_t)packet.payload[0] |
                            ((uint16_t)packet.payload[1] << 8);
        const uint16_t clampedUs = us > 2000 ? 2000 : us;
        getFhss().setSyncLatencyCompensation(clampedUs * Core::MICROSECONDS);
      }
    } else if (pktType == (uint8_t)PacketTypes::ResetRcGapStat) {
      // Reset the max CRSF RC-frame output gap statistic -- meant for the RX
      // side; harmless no-op on a build without a CRSF output.
#if defined(ENABLE_CRSF)
      VCTR::ExVectrLink::resetCrsfMaxRcFrameGap();
#endif
    }
    // Other types are silently ignored for now.
  });

  // getFhss().addChannelBlockedChangeHandler(
  //     [&serialTelecoms](bool blocked, uint8_t maxPacketSize) {
  //       SerialPacket_ChannelState packet{maxPacketSize, blocked};
  //       serialTelecoms.sendSerialPacket(packet);
  //       serialTelecoms.forcePacketSendNow();
  //     });
}

} // namespace VCTR::ExVectrLink::datalink