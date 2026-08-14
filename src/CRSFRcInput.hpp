#ifndef EXVECRLINK_CRSFRCINPUT_HPP
#define EXVECRLINK_CRSFRCINPUT_HPP

#ifdef ENABLE_CRSF_INPUT

#include <Arduino.h>

#include <functional>

#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

#include "CRSF.hpp"
#include "RcLinkChannels.hpp"

namespace VCTR::ExVectrLink {

/**
 * @brief Receives CRSF RC_CHANNELS_PACKED frames from a handset's external
 * module bay, built on the generic CRSF class, and forwards them over the
 * FHSS radio link as RcLinkPacket (RadioLinkMuxPacket) packets: sticks and
 * aux1 in every packet, aux2-aux6 multiplexed one per packet (see
 * RcLinkChannels.hpp for the channel layout).
 *
 * This is the first step of moving CRSFInput's responsibilities onto the
 * new CRSF system: only RC-channel input is handled here. Telemetry-out and
 * the diagnostic counters/raw-byte snapshot are not ported yet -- see
 * CRSFInput for the previous, still-complete implementation of those.
 *
 * Runs the underlying CRSF object in CRSFDuplex::Half. On a shared
 * single-wire module bay pin (rxPin == txPin), the port is initially bound
 * with only RX attached (see taskInit()); CRSF then reroutes the pin to
 * UART TX for the duration of a single reply frame immediately after each
 * RC_CHANNELS_PACKED frame is received, via the GPIO matrix (see
 * CRSF::switchToRx()/switchToTx()), and back to RX once it's sent. Queue
 * telemetry to send back to the handset with sendTelemetry().
 *
 * Also relays FC -> handset telemetry: any VCTR::packets::telecoms
 * Telemetry_* packet arriving over the FHSS downlink (tagged
 * PacketTypes::Data, same as CRSFOutput sends when it decodes them off the
 * FC's own CRSF wire) is converted to the matching CRSF telemetry frame and
 * queued via sendTelemetry() -- see registerTelemetryForwarding(). Adding a
 * new telemetry type to relay is a single forwardTelemetry<>() call there,
 * not a new hand-rolled decode/convert/send path.
 */
class CRSFRcInput : public Core::Task_Periodic {
private:
  HardwareSerial &serial_;
  int8_t rxPin_;
  int8_t txPin_;
  uint32_t baudRate_;
  bool invert_;
  uint8_t uartNum_;

  CRSF crsf_;

  /// Holds the live value of every RC channel and rotates the multiplexed
  /// aux slot; forwardRcChannels() refreshes it from each CRSF frame and
  /// sends its next packet.
  RcLinkCommander rcCommander_{};
  RcLinkPacket lastRcPacket_{};
  int64_t lastRcPacketTime_ = 0;
  uint32_t rcFrameCount_ = 0;

  // Link-stats telemetry cadence (sent back to the handset).
  static constexpr int64_t LINK_STATS_INTERVAL_NS = 100 * Core::MILLISECONDS;
  int64_t lastLinkStatsSendNs_ = 0;
  void writeLinkStatsFrame();

  static int16_t crsfToNorm(uint16_t val);
  static uint8_t digitalFromCrsf(uint16_t val);

  void forwardRcChannels(
      const VCTR::packets::crsf::CRSFPacket_RcChannelsPacked &pkt);

  // ── CRSF parameter/command protocol (handset Lua config menu) ──
  // Exposes two CRSF COMMAND parameters the standard ELRS/EdgeTX Lua config
  // screen can list and execute:
  //   "TX Upload Mode" -- puts THIS module into WiFi firmware-upload mode
  //                       (WifiUpload::startWiFiUploadMode()); terminal, it
  //                       tears down all tasks including CRSF.
  //   "RX Upload Mode" -- relays a one-shot OTA trigger to the far end (see
  //                       DatalinkCommandManager's PacketTypes::UploadMode
  //                       handling) so the RX enters WiFi upload mode.
  // See TXModuleParameters.cpp in extern/ExpressLRS-master for the reference
  // command-parameter state machine this mirrors (handleSimpleSendCmd()).
  // Actions live at the end of the menu, after the read-only stats and the
  // settings (see the parameter-number assignments below).
  static constexpr uint8_t kTxUploadParameterNumber = 11;
  static constexpr uint8_t kRxUploadParameterNumber = 12;
  // Mirrors ExpressLRS's handleSimpleSendCmd() 2 s auto-revert-to-idle so a
  // lost/never-polled-again command doesn't leave the Lua entry stuck
  // showing "executing" forever.
  static constexpr int64_t kCommandStatusTimeoutNs = 2000 * Core::MILLISECONDS;

  // Per-command state for the COMMAND parameters above -- each drives its own
  // idle/executing lifecycle independently.
  struct UploadCommand {
    VCTR::packets::crsf::CRSFCommandStep step =
        VCTR::packets::crsf::CRSFCommandStep::Ready;
    char info[32] = "";
    int64_t lastClickNs = 0;
  };
  UploadCommand txUpload_;
  UploadCommand rxUpload_;

  // ── Bench-test COMMAND parameters (see FHSS::triggerCounterDesyncTest() /
  // triggerFullResyncTest() and DatalinkCommandManager's PacketTypes::
  // TestBlock/TestForceResync handling) ──
  //   "RX Block Test"  -- relays an OTA trigger that blocks the far end's
  //                       FHSS task for 500 ms, to exercise the in-flight-
  //                       reception grace window / catch-up path under a
  //                       real scheduling stall.
  //   "TX Desync Test" -- local-only: shifts THIS side's own channel and
  //                       slot/role counters, so the far end sees a suddenly
  //                       shifted-but-still-valid trailer -- exercises the
  //                       resync-on-agreement path in FHSS::receivePacket()
  //                       without a full reacquisition.
  //   "RX Force Search" -- relays an OTA trigger that does the same
  //                       perturbation on the far end AND forces it straight
  //                       to FHSSState::Searching, bypassing the multi-second
  //                       desync timeout, for repeatable reacquisition tests.
  static constexpr uint8_t kBlockTestParameterNumber = 13;
  static constexpr uint8_t kDesyncTestParameterNumber = 14;
  static constexpr uint8_t kForceResyncParameterNumber = 15;
  UploadCommand blockTest_;
  UploadCommand desyncTest_;
  UploadCommand forceResync_;
  void triggerRxBlockTest();
  void triggerRxForceResync();

  // Runs `action` once, a fixed delay after trigger() is called, then parks
  // itself (release = Core::END_OF_TIME, so the scheduler never selects it)
  // until triggered again. Used to defer the three bench-test actions above:
  // without this, the disruptive event (block/desync/force-search) happens
  // the instant the Lua command is clicked, with no time to back out of the
  // menu and switch to the telemetry page first to actually watch it happen.
  class DelayedTestAction : public Core::Scheduler::Task {
  public:
    explicit DelayedTestAction(const char *name) : Core::Scheduler::Task(name) {
      setRelease(Core::END_OF_TIME);
      Core::getSystemScheduler().addTask(*this);
    }

    void trigger(int64_t delayNs, std::function<void()> action) {
      action_ = std::move(action);
      const int64_t runAt = Core::NowNs() + delayNs;
      setDeadline(runAt);
      setRelease(runAt);
    }

    void taskThread() override {
      if (action_) {
        auto action = std::move(action_);
        action_ = nullptr;
        action();
      }
      // Park until triggered again -- see class doc comment.
      setRelease(Core::END_OF_TIME);
    }

  private:
    std::function<void()> action_;
  };

  static constexpr int64_t kTestActionDelayNs = 10 * Core::SECONDS;
  DelayedTestAction blockTestDelay_{"RxBlockTestDelay"};
  DelayedTestAction desyncTestDelay_{"TxDesyncTestDelay"};
  DelayedTestAction forceResyncDelay_{"RxForceResyncDelay"};

  // ── RX Sync Offset tuning (see FHSS::setSyncLatencyCompensation() /
  // PacketTypes::SetSyncOffset) ──
  // "Sync Offset" is a read-only INFO field showing the RX side's current
  // value, sourced ONLY from its OTA LinkTelemetry reports (getRemote
  // SyncOffsetUs()) -- never a locally-cached "last commanded" value -- so
  // it always reflects what the RX is actually using, not what was merely
  // requested. The two buttons nudge it by +/-100us: each click reads that
  // same live remote value, adds/subtracts one step, and relays the new
  // absolute target immediately (no bench-test-style delay -- this is meant
  // for real-time tuning while watching the telemetry page, so instant
  // feedback is the point).
  static constexpr uint8_t kSyncOffsetInfoParameterNumber = 16;
  static constexpr uint8_t kSyncOffsetIncParameterNumber = 17;
  static constexpr uint8_t kSyncOffsetDecParameterNumber = 18;
  static constexpr uint16_t kSyncOffsetStepUs = 100;
  static constexpr uint16_t kSyncOffsetMaxUs = 2000;
  UploadCommand syncOffsetInc_;
  UploadCommand syncOffsetDec_;
  void sendSyncOffsetInfoEntry(uint8_t destAddress);
  void adjustRemoteSyncOffset(int16_t deltaUs);

  // ── RX RC-frame output gap monitoring (see getCrsfMaxRcFrameGapUs() /
  // PacketTypes::ResetRcGapStat) ──
  // "RX RC Gap" is a read-only INFO field showing the RX side's max observed
  // time between successive RC-channel frames it wrote to its CRSF port,
  // reported back inside its 1 Hz OTA SlotStats packet (getRemoteMaxRcGapUs()).
  // "RX Gap Reset" relays a one-shot OTA trigger that zeros the statistic on
  // the RX so a fresh measurement can start -- applied immediately (no
  // bench-test-style delay), like the sync-offset buttons; it also zeros the
  // local cache so the menu reads 0 right away rather than showing the stale
  // value until the next SlotStats report.
  static constexpr uint8_t kRxRcGapParameterNumber = 19;
  static constexpr uint8_t kRxRcGapResetParameterNumber = 20;
  UploadCommand rcGapReset_;
  void sendRxRcGapEntry(uint8_t destAddress);
  void triggerRxRcGapReset();

  // Read-only INFO parameter showing this TX module's own chip temperature.
  // Sampled fresh each time the field is actually read (menu opened/
  // reloaded) rather than on a timer -- the CRSF parameter protocol has no
  // way to push an unsolicited update to a plain (non-command) field, so a
  // background timer would just go stale between reads anyway.
  static constexpr uint8_t kTempParameterNumber = 1;
  void sendTempEntry(uint8_t destAddress);

  // TEXT_SELECTION controls for VCTR::ExVectrLink::datalink::DynamicPower --
  // enable/disable, plus the min/max power the dynamic-power algorithm is
  // allowed to pick within. Mirrors ExpressLRS's "Dynamic" TX-power selection
  // (TXModuleParameters.cpp's luaDynamicPower). The offered options are
  // always filtered down to DynamicPower::kPowerLevels entries within this
  // radio's actual hardware ceiling/floor (DynamicPower::getMaxPower()/
  // getMinPower()) -- see buildAllowedPowerOptions() in the .cpp -- so the
  // menu can never offer a level the hardware isn't capable of.
  static constexpr uint8_t kDynPowerEnableParameterNumber = 8;
  static constexpr uint8_t kDynMinPowerParameterNumber = 9;
  static constexpr uint8_t kDynMaxPowerParameterNumber = 10;
  void sendDynPowerEnableEntry(uint8_t destAddress);
  void sendDynMinPowerEntry(uint8_t destAddress);
  void sendDynMaxPowerEntry(uint8_t destAddress);

  // TEXT_SELECTION control for which antenna this module transmits on, one
  // entry per Sx1280Diversity::TxAntennaMode plus one per antenna:
  //   "Best SNR" (the default) -- per-slot choice by Sx1280Diversity, which
  //               follows whichever antenna is receiving the far end best.
  //   "Cycle"    -- one antenna per transmitted packet, wrapping
  //               (TxAntennaMode::RoundRobin), so no single antenna's fade
  //               can take out a run of consecutive packets.
  //   "Ant N"    -- every transmission pinned to that one antenna.
  // The per-antenna options are built from the radios this build actually has
  // (see datalink::getTxAntennaCount()), so a single-radio build offers
  // "Best SNR", "Cycle" and "Ant 1" (the first two being equivalent there).
  static constexpr uint8_t kTxAntennaParameterNumber = 21;
  // Option indices in the list sendTxAntennaEntry() builds. The fixed
  // antennas follow the two mode options, so option kTxAntennaOptionFirstFixed
  // is antenna 0.
  static constexpr uint8_t kTxAntennaOptionBestSnr = 0;
  static constexpr uint8_t kTxAntennaOptionCycle = 1;
  static constexpr uint8_t kTxAntennaOptionFirstFixed = 2;
  void sendTxAntennaEntry(uint8_t destAddress);
  /// Maps an option index from the list above back to the stored txAntenna
  /// value it means (see RFSettings::txAntenna).
  static uint8_t txAntennaForOption(uint8_t option);

  // Highest parameter number this module answers for. The handset only ever
  // requests parameters 1..parametersTotal, so a newly added entry stays
  // invisible in the Lua menu until this is bumped -- see the deviceInfo reply
  // in handleParameterPing().
  static constexpr uint8_t kParametersTotal = 21;

  // Read-only INFO parameters showing the far-end (RX-side) debug diagnostics
  // decoded from its OTA RxState packet (see DatalinkCommandManager's
  // getRemote*() accessors). Like the temperature entry, each is sampled
  // fresh when the menu actually reads it.
  static constexpr uint8_t kRuntimeParameterNumber = 2;
  static constexpr uint8_t kDesyncParameterNumber = 3;
  static constexpr uint8_t kFhssOffsetParameterNumber = 4;
  static constexpr uint8_t kIntervalOffsetParameterNumber = 5;
  // FHSS missed-slot (scheduling-latency) counters -- TX Missed is this
  // module's own (getFhss().getMissedSlotCounter()); RX Missed is the far
  // end's, decoded from its OTA SlotStats packet (getRemoteMissedSlots()).
  static constexpr uint8_t kTxMissedParameterNumber = 6;
  static constexpr uint8_t kRxMissedParameterNumber = 7;
  void sendRuntimeEntry(uint8_t destAddress);
  void sendDesyncEntry(uint8_t destAddress);
  void sendFhssOffsetEntry(uint8_t destAddress);
  void sendIntervalOffsetEntry(uint8_t destAddress);
  void sendTxMissedEntry(uint8_t destAddress);
  void sendRxMissedEntry(uint8_t destAddress);

  void handleParameterPing(
      const VCTR::packets::crsf::CRSFPacket_ParameterPing &ping);
  void handleParameterRead(
      const VCTR::packets::crsf::CRSFPacket_ParameterRead &req);
  void handleParameterWrite(
      const VCTR::packets::crsf::CRSFPacket_ParameterWrite &req);
  // Serializes one COMMAND parameter entry (`name` + current step/info of
  // `cmd`) back to the handset.
  void sendCommandEntry(uint8_t parameterNumber, const char *name,
                        const UploadCommand &cmd, uint8_t destAddress);
  // Runs the click/execute/auto-revert state machine for a COMMAND parameter
  // write, invoking `action` on click. `runningInfo` is the status text shown
  // while executing.
  void handleUploadCommandWrite(UploadCommand &cmd, uint8_t parameterNumber,
                                const char *name, const char *runningInfo,
                                const std::function<void()> &action,
                                uint8_t commandStatus, uint8_t replyDest);
  // Sends a CRSF 0x2E ELRS-status frame (good/bad packet counter + connection
  // flag) in response to the ELRS Lua's periodic parameter-0 link-stat poll.
  void sendElrsStatus(uint8_t destAddress);
  void triggerRxWifiUpload();

  // ── FC -> handset telemetry relay ──
  // Decodes VCTR::packets::telecoms Telemetry_* packets out of the FHSS
  // downlink's raw PacketTypes::Data payloads; see
  // registerTelemetryForwarding() for what's currently wired up.
  VCTR::packets::PacketManager telemetryPacketManager_;
  void registerTelemetryForwarding();

  // GPS arrives as two independently-timed packets (stats and position --
  // see registerTelemetryForwarding()); this holds the latest of each so
  // every send to the handset carries the freshest known values for both,
  // even though only one half updates at a time.
  VCTR::packets::crsf::CRSFPacket_GPS gpsCache_{};

  /**
   * @brief Registers a conversion from an incoming ExVectrPackets telemetry
   * type to a CRSF telemetry frame: whenever `SourceT` is decoded out of
   * the FHSS downlink, `convert` is applied and the result queued via
   * sendTelemetry(). This is the one place new telemetry sources need to
   * hook into -- no other code needs to change.
   */
  template <VCTR::packets::IsPacket SourceT, VCTR::packets::IsPacket CrsfT>
  void forwardTelemetry(std::function<CrsfT(const SourceT &)> convert) {
    telemetryPacketManager_.addPacketReceiveHandler<SourceT>(
        [this, convert](const SourceT &packet) {
          sendTelemetry(convert(packet));
        });
  }

public:
  /**
   * @param serial Serial port wired to the handset's external module bay
   * CRSF pins.
   * @param rxPin RX pin to use, or -1 to leave the port's default pin
   * mapping untouched. If rxPin == txPin, only RX is bound (shared
   * single-wire bay pin, as used by e.g. the RadioMaster Pocket's nano
   * module bay); see the class doc comment.
   * @param txPin TX pin to use, or -1 to leave the port's default pin
   * mapping untouched.
   * @param baudRate CRSF baud rate. Must match the handset's configured
   * external-module baud rate exactly.
   * @param invert Set true if the physical signal is logic-inverted, as is
   * common on S.Port-style single-wire module bay pins.
   * @param uartNum The ESP32 UART peripheral number `serial` wraps (e.g.
   * HardwareSerial(1) -> 1). Only used (and required) when rxPin == txPin,
   * to drive the GPIO-matrix direction switching for the shared pin.
   */
  explicit CRSFRcInput(HardwareSerial &serial, int8_t rxPin = -1,
                       int8_t txPin = -1, uint32_t baudRate = 400000,
                       bool invert = false, uint8_t uartNum = 0);

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;

  /**
   * @brief Queues a CRSF packet to be sent back to the handset as
   * telemetry. On the shared half-duplex pin this is only ever actually
   * put on the wire the instant the next RC_CHANNELS_PACKED frame is fully
   * received (see CRSF::sendHalfDuplexReply()); only the single freshest
   * queued packet is kept, so calling this faster than frames arrive just
   * replaces the pending one.
   * @return false if the packet is too large to fit in one CRSF frame.
   */
  template <VCTR::packets::IsPacket T> bool sendTelemetry(const T &packet) {
    return crsf_.write(packet);
  }

  // ── Diagnostics ──
  // Minimal visibility for bench testing; see CRSFInput for the fuller
  // byte-level diagnostics (raw snapshot, CRC error/resync counters) not
  // ported here yet.

  /// @returns the most recently sent RC channels packet (priority channels
  /// plus whichever multiplexed aux channel that packet carried).
  const RcLinkPacket &getLastRcPacket() const { return lastRcPacket_; }

  /// @returns the timestamp (Core::NowNs()) the last RC_CHANNELS_PACKED
  /// frame was received.
  int64_t getLastRcPacketTime() const { return lastRcPacketTime_; }

  /// @returns total number of RC_CHANNELS_PACKED frames forwarded since
  /// boot (free-running; callers wanting a rate should diff this
  /// themselves over time, e.g. once per second).
  uint32_t getRcFrameCount() const { return rcFrameCount_; }
};

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF_INPUT
#endif // EXVECRLINK_CRSFRCINPUT_HPP
