#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrCore.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "ExVectrLink/serial/SerialTelecoms.hpp"

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
HardwareSerial linkSerialPort(1);
Platform::BusUART serialBus(linkSerialPort);
#else
Platform::BusUART serialBus(Serial);
#endif
SerialTelecoms serialTelecoms(serialBus);

#if defined(ESP32)
class TempMonitor : public Core::Task_Periodic {
public:
  TempMonitor() : Core::Task_Periodic("TempMonitor", 5 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    int8_t temp = temperatureRead();
    serialTelecoms.sendSerialPacket(
        VCTR::ExVectrLink::packets::SerialPacket_DeviceTemperature{temp});
  }
};
TempMonitor tempMonitor;
#endif

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF) &&                          \
    !defined(ENABLE_CRSF_INPUT)
class SyncedMonitor : public Core::Task_Periodic {
private:
  VCTR::ExVectrLink::datalink::FHSSState lastState =
      VCTR::ExVectrLink::datalink::FHSSState::Searching;

public:
  SyncedMonitor()
      : Core::Task_Periodic("SyncedMonitor", 500 * Core::MILLISECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    auto fhssState = VCTR::ExVectrLink::datalink::getFhss().getFhssState();

    Serial.println("FHSS Sync State: " + String((int)fhssState));

    VCTR::ExVectrLink::packets::SerialPacket_FhssSyncState packet{
        fhssState == VCTR::ExVectrLink::datalink::FHSSState::Synced};
    serialTelecoms.sendSerialPacket(packet);
  }
};
SyncedMonitor syncedMonitor;
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

    Serial.printf("[%9.3f s] LQ:%3.0f%% PQ:%3.0f%% | RX pkts:%lu (%lu Hz) | TX "
                  "pkts:%lu (%lu Hz)\n",
                  (double)Core::NowNs() / (double)Core::SECONDS,
                  (double)(fhss.getLinkQuality() * 100.0f),
                  (double)(fhss.getPacketQuality() * 100.0f),
                  (unsigned long)rxCount, (unsigned long)rxRate,
                  (unsigned long)txCount, (unsigned long)txRate);

    // Local = this board's own radio. Remote = the other end of the RF
    // link (received via OTA LinkTelemetry/RxState packets) -- whichever
    // physical side (RX or TX module) this firmware is running on, these
    // two lines together cover both ends of the link.
    const auto &remote = ExVectrLink::datalink::getRemoteLinkStats();
    Serial.printf("             Local  LQ:%3.0f%% PQ:%3.0f%% RSSI:%4d SNR:%3d "
                  "TXpwr:%3d dBm\n",
                  (double)(fhss.getLinkQuality() * 100.0f),
                  (double)(fhss.getPacketQuality() * 100.0f),
                  radioA.getPacketRSSI(), radioA.getPacketSNR(),
                  ExVectrLink::datalink::getLocalTxPowerdBm());
    Serial.printf("             Remote LQ:%3d%% PQ:%3d%% RSSI:%4d SNR:%3d "
                  "TXpwr:%3d dBm%s\n",
                  remote.linkQuality,
                  ExVectrLink::datalink::getRemotePacketQuality(), remote.rssi,
                  remote.snr, remote.txPowerdBm,
                  remote.valid ? "" : " (no data yet)");
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

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF) &&                          \
    !defined(ENABLE_CRSF_INPUT)
class ErrorSender : public Core::Task_Periodic {
public:
  ErrorSender() : Core::Task_Periodic("ErrorSender", 0.1001 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override {
    auto error = !radio::radioDriversInitialized();
    // network::DataPacket packet;
    // packet.payload.setSize(5);
    // datalink::getFhss().transmitDataframe(packet);

    if (error) {
      radio::initRadioDrivers();
      auto error2 = !radio::radioDriversInitialized();
      String message =
          error2
              ? "Error: Radios failed to initialize. Retrying in 1s\n"
              : "Error: Radios failed to initialize. Retried and succeeded\n";
      LOG_MSG(message.c_str());
      serialTelecoms.sendSerialPacket(VCTR::ExVectrLink::packets::Error,
                                      message.c_str(), message.length() + 1);
    } else {
#ifdef ENABLE_DEBUG_OUTPUT
      Serial.println("All good!");
      String irqStr;
      // if (irqFlags & IRQ_TX_DONE)
      //   irqStr += "TX_DONE ";
      // if (irqFlags & IRQ_RX_DONE)
      //   irqStr += "RX_DONE ";
      // if (irqFlags & IRQ_SYNCWORD_VALID)
      //   irqStr += "SW_VALID ";
      // if (irqFlags & IRQ_SYNCWORD_ERROR)
      //   irqStr += "SW_ERR ";
      // if (irqFlags & IRQ_HEADER_VALID)
      //   irqStr += "HDR_VALID ";
      // if (irqFlags & IRQ_HEADER_ERROR)
      //   irqStr += "HDR_ERR ";
      // if (irqFlags & IRQ_CRC_ERROR)
      //   irqStr += "CRC_ERR ";
      // if (irqFlags & IRQ_PREAMBLE_DETECTED)
      //   irqStr += "PREAMBLE ";
      // if (irqFlags & IRQ_RX_TX_TIMEOUT)
      //   irqStr += "TIMEOUT ";
      // if (irqFlags & IRQ_CAD_DONE)
      //   irqStr += "CAD_DONE ";
      // if (irqFlags & IRQ_CAD_ACTIVITY_DETECTED)
      //   irqStr += "CAD_ACTIVE ";
      // if (irqStr.isEmpty())
      //   irqStr = "none";
      String syncState;

      switch (datalink::getFhss().getFhssState()) {
      case datalink::FHSSState::Searching:
        syncState = "Searching";
        break;
      case datalink::FHSSState::Syncing:
        syncState = "Syncing";
        break;
      case datalink::FHSSState::Synced:
        syncState = "Synced";
      }

      auto &radioA = VCTR::ExVectrLink::datalink::getDatalinkRadioA();

      String message =
          String() + syncState + " A: " +
          String(
              (double)VCTR::ExVectrLink::datalink::getFhss().getTimingOffset() /
              Core::MILLISECONDS) +
          " I: " +
          String((double)VCTR::ExVectrLink::datalink::getFhss()
                     .getTrueSlotInterval() /
                 Core::MICROSECONDS) +
          " RSSI:" + radioA.getPacketRSSI() + " SNR:" + radioA.getPacketSNR() +
          " LQ:" +
          String(VCTR::ExVectrLink::datalink::getFhss().getLinkQuality() * 100,
                 0) +
          "\n";
      serialTelecoms.sendSerialPacket(VCTR::ExVectrLink::packets::Error,
                                      message.c_str(), message.length() + 1);
#endif
    }
  }
};
ErrorSender errorSender;
#endif

// ISR-captured state for DIO1 interrupt timestamps
volatile uint32_t dio1MicrosA = 0;
volatile bool dio1FiredA = false;
#if defined(HAS_DIVERSITY)
volatile uint32_t dio1MicrosB = 0;
volatile bool dio1FiredB = false;
#endif

void IRAM_ATTR onRadioADio1() {
  dio1MicrosA = micros();
  dio1FiredA = true;
}

#if defined(HAS_DIVERSITY)
void IRAM_ATTR onRadioBDio1() {
  dio1MicrosB = micros();
  dio1FiredB = true;
}
#endif

void setup() {

  // delay(5000);

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF) &&                          \
    !defined(ENABLE_CRSF_INPUT)
  Serial.begin(115200);
#endif

  pinMode(HardwareConfig::button, INPUT_PULLUP);

  // Serial.begin(115200);
#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF) &&                          \
    !defined(ENABLE_CRSF_INPUT)
  linkSerialPort.begin(1000000, SERIAL_8N1, HardwareConfig::serialRx,
                       HardwareConfig::serialTx);
#endif

  delay(500);

  // Check button independently – works even when Serial is not yet open
  if (digitalRead(HardwareConfig::button) == LOW) {
    WifiUpload::startWiFiUploadMode(true);
    WifiUpload::initWifiUpload(serialTelecoms);
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
      WifiUpload::initWifiUpload(serialTelecoms);
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

#if !defined(TARGET_EXVECTRCONTROLLER) && !defined(TARGET_EXVECTRLINKTXMODULE)
  VCTR::ExVectrLink::datalink::getFhss().setIsRxSide(true);
#else
  VCTR::ExVectrLink::datalink::getFhss().setIsRxSide(false);
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

  ExVectrLink::datalink::initCommandManager(serialTelecoms);

  // Note: unlike the old CRSFInput, CRSFRcInput forwards decoded RC
  // channels to the FHSS uplink internally -- no handler wiring needed here.

  WifiUpload::initWifiUpload(serialTelecoms);

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
#if defined(HAS_DIVERSITY)
  radioSetup(radioB);
#endif

  ExVectrLink::datalink::getDynamicPower().setMaxPower(
      HardwareConfig::sx1280AMaxPower);
  ExVectrLink::datalink::getDynamicPower().setMinPower(10);
  ExVectrLink::datalink::getDynamicPower().setDynMaxPower(20);
  ExVectrLink::datalink::getDynamicPower().setPower(20);
  ExVectrLink::datalink::setupDynamicPower();

#if defined(ENABLE_SBUS) || defined(ENABLE_CRSF) || defined(ENABLE_CRSF_INPUT)
  Core::getSystemScheduler().removeTask(serialTelecoms);
  serialTelecoms.setDisabled();
#endif
}

void processDio1Interrupts() {
  if (dio1FiredA && ExVectrLink::datalink::getDatalinkRadioA().isConfigured()) {
    // Atomically read ISR-captured micros and clear flag
    noInterrupts();
    uint32_t isrMicros = dio1MicrosA;
    dio1FiredA = false;
    interrupts();
    // Unsigned subtraction keeps the elapsed time correct even when micros()
    // wraps roughly every 71 minutes.
    uint32_t elapsedMicros = micros() - isrMicros;
    int64_t timestamp =
        Core::NowNs() - int64_t(elapsedMicros) * Core::MICROSECONDS;
    ExVectrLink::datalink::getDatalinkRadioA().notifyDio1Irq(timestamp);
  }
#if defined(HAS_DIVERSITY)
  if (dio1FiredB && ExVectrLink::datalink::getDatalinkRadioB().isConfigured()) {
    noInterrupts();
    uint32_t isrMicros = dio1MicrosB;
    dio1FiredB = false;
    interrupts();
    uint32_t elapsedMicros = micros() - isrMicros;
    int64_t timestamp =
        Core::NowNs() - int64_t(elapsedMicros) * Core::MICROSECONDS;
    ExVectrLink::datalink::getDatalinkRadioB().notifyDio1Irq(timestamp);
  }
#endif
}

void loop() {

  WifiUpload::loopWifiUpload();

  processDio1Interrupts();

  Core::getSystemScheduler().tick();
}