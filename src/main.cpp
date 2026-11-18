#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrLink/datalink/PacketTypes.hpp"

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"

#include "Hardware.hpp"

#include "DatalinkCommandManager.hpp"
#include "SX1280.hpp"
#include "WifiUpload.hpp"

#include "DatalinkSetup.hpp"
#include "DynamicPowerSetup.hpp"
#include "FhssSetup.hpp"
#include "RFSetup.hpp"
#include "Settings.hpp"

#include "sbus.h"

#if defined(ENABLE_CRSF)
#include "CRSFRcOutput.hpp"
#endif

#if defined(ENABLE_CRSF_INPUT)
#include "CRSFRcInput.hpp"
#endif

using namespace VCTR;
using namespace VCTR::ExVectrLink;

#if defined(ESP32)
// UART used by CRSFRcInput on the TX module (see below).
HardwareSerial linkSerialPort(1);
#endif

#if defined(ENABLE_SBUS)
VCTR::packets::PacketManager
    packetManager([](const VCTR::Core::ListArray<uint8_t> &) {});

namespace VCTR::ExVectrLink {
using namespace VCTR::packets::radiolink;

class SBusTx : public Core::Task_Periodic {
private:
  bfs::SbusTx sbus_tx;
  RadioLinkPacket_A4D4 packet;
  RadioLinkPacketFilter<4, 4, RadioLinkTypes::A4D4> packetFilter;
  bool dataNew = false;

  int64_t lastPacketTime = 0;

public:
  SBusTx()
      : Core::Task_Periodic("SBusTx", Core::MILLISECONDS * 5),
        sbus_tx(&Serial, true, true) {

    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {
    Serial.begin(200000, SERIAL_8E2, SerialMode::SERIAL_FULL, 1, true);
    // sbus_tx.Begin();
    VCTR::ExVectrLink::datalink::getFhss().addReceiveHandler(
        [this](const VCTR::network::DataPacket &data) {
          packetManager.receivePacketData(data.payload);
          // if (data.payload.size() == packet.numBytes()) {

          //   if (packet.deserialize(data.payload.getPtr())) {
          //     dataNew = true;
          //     lastPacketTime = Core::NowNs();
          //   }
          // }
        });

    packetManager.addPacketReceiveHandler<
        VCTR::packets::radiolink::RadioLinkPacket_A4D4>(
        [this](const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &packet) {
          this->packet = packet;
          dataNew = true;
          lastPacketTime = Core::NowNs();
          this->setDeadline(Core::NowNs());
        });

    packetManager.addPacketReceiveHandler<
        VCTR::packets::telecoms::Telecommand_UpdateMode>(
        [this](const VCTR::packets::telecoms::Telecommand_UpdateMode &packet) {
          WifiUpload::startWiFiUploadMode();
        });

    // VCTR::ExVectrLink::datalink::getFhss()
    //     .addPacketReceiveHandler<
    //         VCTR::packets::radiolink::RadioLinkPacket_A4D4>(
    //         [this](
    //             const VCTR::packets::radiolink::RadioLinkPacket_A4D4 &packet)
    //             {
    //           this->packet = packet;
    //           dataNew = true;
    //           packetCount++;
    //           lastPacketTime = Core::NowNs();
    //         });
  }

  void taskThread() override {
    if (Core::NowNs() - lastPacketTime < Core::MILLISECONDS * 200) {
      bfs::SbusData sbusData;
      sbusData.lost_frame = false;
      sbusData.failsafe = false;

      packetFilter.updateFromPacket(packet, lastPacketTime);
      auto &packetFiltered = packetFilter.getFilteredPacket();

      auto normToSbusMap = [](int16_t val) {
        // Map from -1000 - 1000 to 172 - 1811;
        return (float)(val + 1000) / 2000.0f * (1811 - 172) + 172;
      };

      for (int i = 0; i < bfs::SbusData::NUM_CH; i++) {
        sbusData.ch[i] = 1000;
      }

      // sbusData.ch[0] = packet.analogChannels[0] / 2 + 1500;
      sbusData.ch[0] = normToSbusMap(packetFiltered.analogChannels[0]);
      sbusData.ch[1] = normToSbusMap(packetFiltered.analogChannels[1]);
      sbusData.ch[2] = normToSbusMap(packetFiltered.analogChannels[3]);
      sbusData.ch[3] = normToSbusMap(packetFiltered.analogChannels[2]);
      sbusData.ch[4] = !packetFiltered.digitalChannels[1] ? 1811 : 172;
      sbusData.ch[8] = packetFiltered.digitalChannels[2] ? 1811 : 172;
      sbusData.ch[6] = packetFiltered.digitalChannels[3] ? 1811 : 172;
      sbusData.ch[7] = packetFiltered.digitalChannels[0] ? 1811 : 172;

      sbusData.ch17 = false;
      sbusData.ch18 = false;

      sbus_tx.data(sbusData);
      sbus_tx.Write();
      // Serial.printf("Received packet: A: %d %d %d %d D: %d %d %d %d\n",
      //               packetFiltered.analogChannels[0],
      //               packetFiltered.analogChannels[1],
      //               packetFiltered.analogChannels[2],
      //               packetFiltered.analogChannels[3],
      //               packetFiltered.digitalChannels[0],
      //               packetFiltered.digitalChannels[1],
      //               packetFiltered.digitalChannels[2],
      //               packetFiltered.digitalChannels[3]);
    } else {
      // If we haven't received a packet in a while, send failsafe
      bfs::SbusData sbusData;
      sbusData.lost_frame = true;
      sbusData.failsafe = true;
      for (int i = 0; i < bfs::SbusData::NUM_CH; i++) {
        sbusData.ch[i] = 1000;
      }
      sbusData.ch17 = false;
      sbusData.ch18 = false;

      sbus_tx.data(sbusData);
      sbus_tx.Write();
    }
    dataNew = false;
  }
};
SBusTx sbusTx;
} // namespace VCTR::ExVectrLink
#elif defined(ENABLE_CRSF)
// USB CDC `Serial` port, hijacked here for CRSF framing to the flight
// controller (separate RX/TX lines internally -- full duplex).
VCTR::ExVectrLink::CRSFRcOutput crsfRcOutput(Serial);
#elif defined(ENABLE_CRSF_INPUT)
// 400000 baud matches the handset's external-module CRSF baud rate setting
// (OpenTX/EdgeTX's "CRSF" module type default). This MUST match whatever
// baud rate is configured in the radio's module settings, or every frame
// will fail CRC checks.
//
// invert=true: getting a handful of CRC-passing good frames (not just
// resync failures) during connector insertion transients is strong
// evidence this polarity is correct -- a wrong polarity would essentially
// never produce a CRC match by chance. The "goes silent once fully seated"
// symptom looks like a physical/mechanical connection problem rather than
// a polarity one -- see notes at the CRSFRcInput instantiation site.
// uartNum=1 matches `HardwareSerial linkSerialPort(1)` above -- needed so
// CRSFRcInput/CRSF can drive the GPIO matrix directly for half-duplex
// direction switching on the shared serialRx/serialTx pin.
VCTR::ExVectrLink::CRSFRcInput crsfRcInput(linkSerialPort,
                                           HardwareConfig::serialRx,
                                           HardwareConfig::serialTx, 400000,
                                           true, 1);
#endif

/**
 * @brief Failsafe recovery for the RX (flight-controller-facing) side: if
 * the radio never gets past FHSSState::Searching within 60 s of boot, drop
 * into WiFi upload mode automatically so a bad config/firmware can be fixed
 * without needing physical access to the button.
 *
 * Only a *never-synced-since-boot* timeout triggers this -- Syncing counts
 * the same as Synced (either one permanently cancels the failsafe), and
 * losing sync again later (after having synced at least once) never
 * re-triggers it.
 */
class SyncFailsafeMonitor : public Core::Task_Periodic {
private:
  static constexpr int64_t TIMEOUT_NS = 1 * Core::MINUTES;

  int64_t bootTimeNs_ = 0;
  bool everSynced_ = false;
  bool triggered_ = false;

public:
  SyncFailsafeMonitor()
      : Core::Task_Periodic("SyncFailsafeMonitor", 1 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override { bootTimeNs_ = Core::NowNs(); }

  void taskThread() override {
    if (triggered_ || everSynced_) {
      return;
    }

    if (ExVectrLink::datalink::getFhss().getFhssState() !=
        ExVectrLink::datalink::FHSSState::Searching) {
      // Reached Syncing or Synced at least once -- cancels the failsafe
      // permanently, even if sync is lost again later.
      everSynced_ = true;
      return;
    }

    if (Core::NowNs() - bootTimeNs_ >= TIMEOUT_NS) {
      triggered_ = true;
      WifiUpload::startWiFiUploadMode();
    }
  }
};
SyncFailsafeMonitor syncFailsafeMonitor;

// SBUS and CRSF (output) both hijack the USB `Serial` port for their own
// protocol framing, so debug printing is only safe when neither is active
// (plain/test builds, and CRSF_INPUT builds -- CRSFRcInput talks over
// linkSerialPort, leaving `Serial` free).
#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
// Defined further down with the DIO1 ISRs; declared here so DebugMonitor can
// report the per-radio interrupt rate.
extern volatile uint32_t dio1CountA;
#if defined(HAS_DIVERSITY)
extern volatile uint32_t dio1CountB;
#endif

/**
 * @brief Prints radio link stats (and, if CRSFRcInput is enabled, the
 * current RC channels + reception rate) to the USB Serial port once per
 * second.
 * (CRSFRcInput does not yet expose the byte-level UART diagnostics
 * CRSFInput used to -- raw byte snapshot, resync/CRC-error counters -- see
 * CRSFInput for that reference implementation.)
 */
class DebugMonitor : public Core::Task_Periodic {
private:
  uint32_t lastRxPacketCount_ = 0;
  uint32_t lastTxPacketCount_ = 0;
  uint32_t lastDio1CountA_ = 0;
#if defined(HAS_DIVERSITY)
  uint32_t lastDio1CountB_ = 0;
#endif
#if defined(ENABLE_CRSF_INPUT)
  uint32_t lastRcFrameCount_ = 0;
#endif

public:
  DebugMonitor() : Core::Task_Periodic("DebugMonitor", 1 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override { Serial.begin(115200); }

  void taskThread() override {
    auto &radioA = ExVectrLink::datalink::getDatalinkRadioA();
    auto &fhss = ExVectrLink::datalink::getFhss();

    const uint32_t rxCount = radioA.getRxPacketCount();
    const uint32_t txCount = radioA.getTxPacketCount();
    const uint32_t rxRate = rxCount - lastRxPacketCount_;
    const uint32_t txRate = txCount - lastTxPacketCount_;
    lastRxPacketCount_ = rxCount;
    lastTxPacketCount_ = txCount;

    Serial.printf("[%9.3f s] LQ:%3.0f%% | RX pkts:%lu (%lu Hz) | TX "
                  "pkts:%lu (%lu Hz)\n",
                  (double)Core::NowNs() / (double)Core::SECONDS,
                  (double)(fhss.getLinkQuality() * 100.0f),
                  (unsigned long)rxCount, (unsigned long)rxRate,
                  (unsigned long)txCount, (unsigned long)txRate);

    // Local = this board's own radio. Remote = the other end of the RF
    // link (received via OTA LinkTelemetry/RxState packets) -- whichever
    // physical side (RX or TX module) this firmware is running on, these
    // two lines together cover both ends of the link.
    const auto &remote = ExVectrLink::datalink::getRemoteLinkStats();
    Serial.printf("             Local  LQ:%3.0f%% RSSI:%4d SNR:%3d "
                  "TXpwr:%3d dBm\n",
                  (double)(fhss.getLinkQuality() * 100.0f),
                  radioA.getPacketRSSI(), radioA.getPacketSNR(),
                  ExVectrLink::datalink::getLocalTxPowerdBm());
    Serial.printf("             Remote LQ:%3d%% RSSI:%4d SNR:%3d "
                  "TXpwr:%3d dBm%s\n",
                  remote.linkQuality, remote.rssi, remote.snr,
                  remote.txPowerdBm, remote.valid ? "" : " (no data yet)");
    // DIO1 edge rate per radio, alongside the FHSS sync offset it feeds.
    // A radio carrying packets at 0 edges/s is running on poll-time
    // timestamps, which shows up here as a sync offset wandering by tens to
    // hundreds of microseconds instead of staying inside ~+/-10 us.
    const uint32_t dio1A = dio1CountA;
    const uint32_t dio1RateA = dio1A - lastDio1CountA_;
    lastDio1CountA_ = dio1A;
#if defined(HAS_DIVERSITY)
    const uint32_t dio1B = dio1CountB;
    const uint32_t dio1RateB = dio1B - lastDio1CountB_;
    lastDio1CountB_ = dio1B;
    Serial.printf("             DIO1 edges: A %lu /s B %lu /s | sync offset "
                  "%7.1f us\n",
                  (unsigned long)dio1RateA, (unsigned long)dio1RateB,
                  (double)fhss.getTimingOffset() / (double)Core::MICROSECONDS);
#else
    Serial.printf("             DIO1 edges: A %lu /s | sync offset %7.1f us\n",
                  (unsigned long)dio1RateA,
                  (double)fhss.getTimingOffset() / (double)Core::MICROSECONDS);
#endif

    // Remote (RX-side) CRSF -> flight controller health.
    //  RC rate  >0  => RX is decoding RC and building CRSF frames.
    //  fail rate 0  => those frames are being written to the FC UART OK
    //                  (so if the FC still sees nothing, look past the RX's
    //                  UART: wiring / baud / inversion / FC CRSF config).
    //  fail rate >0 => RX builds frames but the FC serial port won't accept
    //                  the bytes (buffer full / port not draining).
    Serial.printf("             Remote CRSF: RC rate %u /s | CRSF->FC fail "
                  "rate %u /s\n",
                  ExVectrLink::datalink::getRemoteRcPacketRateHz(),
                  ExVectrLink::datalink::getRemoteCrsfTxFailureRateHz());

#if defined(ENABLE_CRSF_INPUT)
    const auto &rc = crsfRcInput.getLastRcPacket();
    const uint32_t rcFrameCount = crsfRcInput.getRcFrameCount();
    const uint32_t rcRate = rcFrameCount - lastRcFrameCount_;
    lastRcFrameCount_ = rcFrameCount;
    const bool rcReceiving =
        (Core::NowNs() - crsfRcInput.getLastRcPacketTime()) <
        300 * Core::MILLISECONDS;

    Serial.printf("             CRSF CH1:%5d CH2:%5d CH3:%5d CH4:%5d | "
                  "Rate:%lu Hz | %s\n",
                  rc.analogChannels[0], rc.analogChannels[1],
                  rc.analogChannels[2], rc.analogChannels[3],
                  (unsigned long)rcRate, rcReceiving ? "RX OK" : "NO SIGNAL");
#endif
  }
};
DebugMonitor debugMonitor;
#endif

// ISR-captured state for DIO1 interrupt timestamps
volatile uint32_t dio1MicrosA = 0;
volatile bool dio1FiredA = false;
// Free-running edge counters, reported by DebugMonitor. A radio whose DIO1
// rate sits at 0 while it is demonstrably moving packets has a dead interrupt
// line (wrong/unavailable pin), which is not fatal -- fetchIrqFlags() falls
// back to a poll-time Core::NowNs() -- but stamps every RX_DONE late by the
// main-loop latency and so jitters the FHSS sync offset badly.
volatile uint32_t dio1CountA = 0;
#if defined(HAS_DIVERSITY)
volatile uint32_t dio1MicrosB = 0;
volatile bool dio1FiredB = false;
volatile uint32_t dio1CountB = 0;
#endif

void IRAM_ATTR onRadioADio1() {
  dio1MicrosA = micros();
  dio1FiredA = true;
  dio1CountA++;
}

#if defined(HAS_DIVERSITY)
void IRAM_ATTR onRadioBDio1() {
  dio1MicrosB = micros();
  dio1FiredB = true;
  dio1CountB++;
}
#endif

// Atomically drains one radio's ISR capture and converts it to a system
// timestamp (ns). Returns 0 if no edge is pending. Shared by the main-loop
// transfer (processDio1Interrupts) and the drivers' pull-time drain
// (setDio1TimestampSource) so an RX_DONE edge that fires between the two is
// never stamped with a late Core::NowNs() fallback.
int64_t drainDio1Timestamp(volatile bool &fired, volatile uint32_t &isrTime) {
  if (!fired) {
    return 0;
  }
  noInterrupts();
  uint32_t isrMicros = isrTime;
  fired = false;
  interrupts();
  // Unsigned subtraction keeps the elapsed time correct even when micros()
  // wraps roughly every 71 minutes.
  uint32_t elapsedMicros = micros() - isrMicros;
  return Core::NowNs() - int64_t(elapsedMicros) * Core::MICROSECONDS;
}

int64_t drainDio1TimestampA() {
  return drainDio1Timestamp(dio1FiredA, dio1MicrosA);
}

#if defined(HAS_DIVERSITY)
int64_t drainDio1TimestampB() {
  return drainDio1Timestamp(dio1FiredB, dio1MicrosB);
}
#endif

void setup() {

  // delay(5000);

  pinMode(HardwareConfig::button, INPUT_PULLUP);

  delay(500);

  // Check button independently – works even when Serial is not yet open
  if (digitalRead(HardwareConfig::button) == LOW) {
    WifiUpload::startWiFiUploadMode(true);
    WifiUpload::initWifiUpload();
    return;
  }

#ifdef ENABLE_FORCEONLYWIFI
  WifiUpload::startWiFiUploadMode(true);
  return;
#endif

  uint8_t count = 0;
  auto startTime = millis();
  while (Serial.available() && millis() - startTime < 1000) {
    auto byte = Serial.read();
    if (byte == 'r') {
      count++;
    } else {
      count = 0;
    }
    if (count >= 5) {
      WifiUpload::startWiFiUploadMode(true);
      WifiUpload::initWifiUpload();
      return;
    }
  }

  pinMode(HardwareConfig::sx1280ADio1, INPUT);
#if defined(HAS_DIVERSITY)
  pinMode(HardwareConfig::sx1280BDio1, INPUT);
#endif

  // Setup DIO1 interrupts for precise timestamping
  attachInterrupt(digitalPinToInterrupt(HardwareConfig::sx1280ADio1),
                  onRadioADio1, RISING);
#if defined(HAS_DIVERSITY)
  attachInterrupt(digitalPinToInterrupt(HardwareConfig::sx1280BDio1),
                  onRadioBDio1, RISING);
#endif

  // Keyed off IS_TX_SIDE (Hardware.hpp) rather than re-listing targets here --
  // a new handset-side target that is missing from such a list silently comes
  // up as a receiver: it never transmits, and it keys its sync word/CRC with
  // the wrong side bit, so the link never forms at all.
#if defined(IS_TX_SIDE)
  VCTR::ExVectrLink::datalink::getFhss().setIsRxSide(false);
#else
  VCTR::ExVectrLink::datalink::getFhss().setIsRxSide(true);
#endif

  // Unlike ENABLE_SBUS/ENABLE_CRSF (which hijack the USB `Serial` port for
  // their own protocol framing), CRSFRcInput talks over a separate
  // `linkSerialPort`, leaving `Serial` free -- so Core::initialise() (which
  // wires up Core::printM/D/W/E to Serial) is safe and should still run for
  // CRSF_INPUT builds too. Skipping it here previously silenced all
  // library debug/error/warning output.
#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
  Core::initialise();
#endif

  ExVectrLink::radio::initRadioDrivers();

  ExVectrLink::datalink::initCommandManager();

  // Note: unlike the old CRSFInput, CRSFRcInput forwards decoded RC
  // channels to the FHSS uplink internally -- no handler wiring needed here.

  WifiUpload::initWifiUpload();

  auto &radioA = ExVectrLink::datalink::getDatalinkRadioA();
#if defined(HAS_DIVERSITY)
  auto &radioB = ExVectrLink::datalink::getDatalinkRadioB();
#endif

  auto radioSetup = [](VCTR::network::datalink::Sx1280_Direct &radio) {
    radio.setBandwidth(VCTR::network::datalink::BW_800KHz);
    radio.setCodingRate(VCTR::network::datalink::LI_4_8);
    radio.setSpreadingFactor(VCTR::network::datalink::SF_7);
    radio.setPacketMode(VCTR::network::datalink::SX1280_PacketMode::Limited);
    radio.setFixedPacketLength(11);
    radio.setChannel(0);
    radio.setTxMaxPower(HardwareConfig::sx1280AMaxPower);
    radio.setPAdbm(HardwareConfig::sx1280APAdbm);
    radio.setTxPower(20);
  };

  radioSetup(radioA);
  radioA.setDio1TimestampSource(drainDio1TimestampA);
#if defined(HAS_DIVERSITY)
  radioSetup(radioB);
  radioB.setDio1TimestampSource(drainDio1TimestampB);
#endif

  ExVectrLink::datalink::getDynamicPower().setMaxPower(
      HardwareConfig::sx1280AMaxPower);
  ExVectrLink::datalink::getDynamicPower().setMinPower(10);
  // The hardware limits above are fixed by the board; the transmit power and
  // TX antenna come from EEPROM (falling back to the built-in defaults on a
  // device that has never had them saved, and to a fixed power on the RX side
  // -- see applyRfSettings()). Must run after the hardware limits, which clamp
  // whatever is loaded.
  ExVectrLink::setupSettings();
  ExVectrLink::datalink::applyRfSettings();
  ExVectrLink::datalink::setupDynamicPower();
}

void processDio1Interrupts() {
  // force=true: with DIO1 masked to RX_DONE only, every edge is a packet
  // end, so the latest edge is always the right one. Keep-first latching
  // would let a stale timestamp (from an edge whose packet was already
  // consumed via the pull-time drain) block the next packet's edge.
  if (ExVectrLink::datalink::getDatalinkRadioA().isConfigured()) {
    int64_t timestamp = drainDio1TimestampA();
    if (timestamp != 0) {
      ExVectrLink::datalink::getDatalinkRadioA().notifyDio1Irq(timestamp,
                                                               true);
    }
  }
#if defined(HAS_DIVERSITY)
  if (ExVectrLink::datalink::getDatalinkRadioB().isConfigured()) {
    int64_t timestamp = drainDio1TimestampB();
    if (timestamp != 0) {
      ExVectrLink::datalink::getDatalinkRadioB().notifyDio1Irq(timestamp,
                                                               true);
    }
  }
#endif
}

void loop() {

  WifiUpload::loopWifiUpload();

  processDio1Interrupts();

  Core::getSystemScheduler().tick();
}