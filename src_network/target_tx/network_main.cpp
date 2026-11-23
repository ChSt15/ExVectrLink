// ExVectrNetwork-mode TX module firmware (env exvectrlink_tx_module_network): node 2, router, sync master.
//
// USB CDC `Serial` carries the COBS/CRC network stream to the ground station and nothing else: no debug text, no
// DebugMonitor, no Core::initialise() (its print hooks would write to Serial). Logs are LOG_TEXT packets.
// The CRSF half-duplex port to the handset is the same one the FPV firmware uses (400 kBaud, inverted, shared pin),
// driven through src/CRSF.cpp.
// The air side is the FHSS air link (air profile v1, src_network/target/NetRadio.cpp) with diversity on both radios.
// ONE manual power setting for the whole network (no dynamic power): 10 dBm (10 mW legal limit) at every boot, raised
// temporarily (RAM only) from the Lua menu up to the board maximum, applied to both radios and broadcast to the ship
// radio node as LINK_CONFIG (src_network/NetPower.hpp). The initial TX antenna comes from the FPV EEPROM settings
// (read only); the Lua menu (src_network/NetLuaMenu.hpp) changes it in RAM only.
// WiFi upload (over-the-air firmware update) as in FPV: boot button held at power-up (link never starts), button
// held 3 s while running (WiFi next to the link), and the 60 s sync failsafe (link stopped). USB flashing works as
// before.
//
// Compiled only by the network env. The FPV envs never see this directory (platformio.ini build_src_filter).
#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "CRSF.hpp"
#include "CrsfParser.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "Hardware.hpp"
#include "Settings.hpp"
#include "WifiUpload.hpp"

#include "NetAirAccess.hpp"
#include "NetLuaMenu.hpp"
#include "NetPower.hpp"
#include "NetRadio.hpp"
#include "NetworkRouter.hpp"
#include "UploadTriggers.hpp"

#if !defined(TARGET_EXVECTRLINKTXMODULEDIVERSITY)
#error "The network-mode TX firmware is only defined for the diversity TX module pin map"
#endif

using namespace VCTR;
using namespace VCTR::ExVectrLink;
using namespace VCTR::ExVectrLink::netmode;

namespace {

constexpr uint32_t kCrsfBaud = 400000;
constexpr uint8_t CRSF_ADDRESS_BROADCAST = 0x00;
constexpr uint8_t CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA; // the handset
constexpr uint8_t CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE;  // this module
constexpr uint8_t CRSF_ADDRESS_ELRS_LUA = 0xEF;

/// USB CDC as a non-blocking byte stream. Writes never exceed what the CDC buffer accepts, and the transmit
/// timeout is 0 so a missing host can never stall the loop.
class UsbStream : public network::ByteStreamI {
public:
  size_t readable() override { return static_cast<size_t>(Serial.available()); }
  size_t read(uint8_t *data, size_t len) override {
    size_t i = 0;
    while (i < len && Serial.available() > 0) {
      const int c = Serial.read();
      if (c < 0) break;
      data[i++] = static_cast<uint8_t>(c);
    }
    return i;
  }
  size_t writable() override {
    if (!Serial) return 0;
    const int n = Serial.availableForWrite();
    return n > 0 ? static_cast<size_t>(n) : 0;
  }
  size_t write(const uint8_t *data, size_t len) override { return Serial.write(data, len); }
};

/// A raw, already framed CRSF frame, so CRSF::write() (which takes typed packets) can send any frame.
class RawCrsfFrame {
public:
  explicit RawCrsfFrame(const CrsfFrame &f) : f_(f) {}
  size_t getPacketType() const { return 0; }
  size_t getPacketDataType() const { return f_.bytes[2]; }
  size_t numBytes() const { return f_.len; }
  void serialize(uint8_t *buffer) const { memcpy(buffer, f_.bytes, f_.len); }
  bool deserialize(const uint8_t *) { return false; }

private:
  const CrsfFrame &f_;
};

CoreTime coreTime;
UsbStream usbStream;
network::StreamFrameLink<1024> usbLink(usbStream, coreTime);

/// Admits frames to USB only while a host is attached and the CDC buffer is not backed up (<= 256 bytes still
/// waiting in the TX ring), so stale frames are dropped, never held back for later.
class UsbGate : public LinkGateI {
public:
  bool admits() override { return Serial && usbLink.txPending() <= 256; }
};
UsbGate usbGate;

NetAirAccess::Link airLink;
NetAirAccess airAccess(airLink, coreTime, /*isMaster=*/true, netExpectedRxPerSecond(false));
NetworkRouter router(coreTime, usbLink, &usbGate, airLink, airAccess);

HardwareSerial linkSerialPort(1);
CRSF crsf(linkSerialPort, CRSFDuplex::Half, kCrsfBaud, HardwareConfig::serialRx, 1, true);

NetTxPowerSetting txPower(HardwareConfig::sx1280AMaxPower); // RAM only, boots at 10 dBm, ceiling = board maximum
uint8_t txAntennaSel = 0; // 0 Auto, 1 A, 2 B (RAM only)
NetLuaMenu luaMenu(HardwareConfig::sx1280AMaxPower);
UploadTriggers *triggers = nullptr;
int64_t routerRebootAtNs = 0; // Lua "Reboot router" confirmed: restart at this time (0 = none)

bool hostWasConnected = false;
bool linkStopped = false; // WiFi upload mode with the link stopped (until reset)

/// The single power variable: applied to both radios and broadcast as LINK_CONFIG. Only variable writes (no radio
/// command), so it is safe in the main loop.
void applyPower() {
  netRadioSetTxPower(txPower.get());
  router.setConfigPower(txPower.get());
}

void setTxAntennaSelection(uint8_t sel) {
  auto &div = netDiversity();
  const size_t idx = sel == 0 ? datalink::Sx1280Diversity::kAutoTxLink : static_cast<size_t>(sel - 1);
  if (div.setDesignatedTxLinkIndex(idx)) {
    txAntennaSel = sel;
  } else {
    div.setDesignatedTxLinkIndex(datalink::Sx1280Diversity::kAutoTxLink);
    txAntennaSel = 0;
  }
}

/// Initial TX antenna from the FPV settings record (read only; default Auto on a fresh device).
void applyStoredRfSettings() {
  setupSettings();
  const RFSettings &rf = getSettings().get<RFSettings>();
  setTxAntennaSelection(rf.txAntenna == RFSettings::kDiversityTxAntenna ? 0 : static_cast<uint8_t>(rf.txAntenna + 1));
}

/// What the Lua menu does to the firmware.
class LuaActions : public LuaMenuActions {
public:
  void setTxPower(int8_t dbm) override {
    txPower.set(dbm);
    applyPower();
  }
  void setTxAntenna(uint8_t selection) override { setTxAntennaSelection(selection); }
  void rebootRouter() override { routerRebootAtNs = coreTime.nowNs() + 500000000LL; } // the reply leaves first
  void rebootShipNode() override { router.requestShipReboot(); }
};
LuaActions luaActions;

/// Snapshot for the Lua menu (one localAirStats() call, bounded).
LuaMenuStatus luaStatus() {
  LuaMenuStatus st;
  const LocalAirStats local = airAccess.localAirStats();
  st.txPowerDbm = txPower.get();
  st.txAntennaSel = txAntennaSel;
  st.airState = local.airState;
  st.rxLq = local.rxLq;
  const auto &port = router.linkStatsPort();
  st.shipValid = router.node().isPresent(NetworkRouter::kRxModule) && port.hasValue() &&
                 coreTime.nowNs() - port.rxNs() < CrsfTelemetryScheduler::kStaleNs;
  st.shipPowerDbm = st.shipValid ? port.value().tx_power : 0;
  return st;
}

/// One Lua parameter frame (ping, read or write), at most one per main-loop pass, never blocks.
void handleLuaFrame(uint8_t type) {
  using namespace VCTR::packets::crsf;
  const int64_t now = coreTime.nowNs();
  CrsfFrame reply;
  if (type == static_cast<uint8_t>(CRSFFrameType::ParameterPing)) {
    // The handset pings (~250/s) and sends no channels until the module answers with its device info.
    CRSF::ReceivedPacket<CRSFPacket_ParameterPing> ping;
    if (!crsf.read(ping) || !NetLuaMenu::pingIsForUs(ping.packet.destAddress)) return;
    CRSFPacket_ParameterDeviceInfo info;
    info.destAddress = NetLuaMenu::replyDest(ping.packet.origAddress);
    strncpy(info.deviceName, "ExVectr Router", sizeof(info.deviceName) - 1);
    info.parametersTotal = NetLuaMenu::kParamCount;
    crsf.write(info);
  } else if (type == static_cast<uint8_t>(CRSFFrameType::ParameterRead)) {
    CRSF::ReceivedPacket<CRSFPacket_ParameterRead> rd;
    if (!crsf.read(rd)) return;
    if (luaMenu.buildRead(rd.packet.parameterNumber, rd.packet.origAddress, luaStatus(), now, reply))
      crsf.write(RawCrsfFrame(reply));
  } else if (type == static_cast<uint8_t>(CRSFFrameType::ParameterWrite)) {
    CRSF::ReceivedPacket<CRSFPacket_ParameterWrite> wr;
    if (!crsf.read(wr)) return;
    if (wr.packet.parameterNumber == 0) {
      // The ELRS Lua writes parameter 0 about once a second and expects the 0x2E status (title bar counter, icon).
      const LuaMenuStatus st = luaStatus();
      CRSFPacket_ElrsStatus es;
      es.destAddress = NetLuaMenu::replyDest(wr.packet.origAddress);
      es.pktsGood = st.rxLq;
      es.pktsBad = static_cast<uint8_t>(100 - st.rxLq);
      es.flags = st.shipValid ? 0x01 : 0x00;
      es.info[0] = '\0';
      crsf.write(es);
      return;
    }
    if (luaMenu.handleWrite(wr.packet.parameterNumber, wr.packet.commandStatus, wr.packet.origAddress, luaStatus(),
                            now, luaActions, reply))
      crsf.write(RawCrsfFrame(reply));
  }
}

/// RC-stream diagnostics (docs/network-mode.md "RC stream diagnostics"). No schema field has room for these, so
/// they leave as LOG_TEXT lines at 1 Hz (dropped silently while no host is attached). Instrumentation only.
class RcDiag {
public:
  void update(int64_t now) {
    const CrsfParser::Stats &st = crsf.parserStats();
    if (st.framesRc != lastRc_) { lastRc_ = st.framesRc; lastRcNs_ = now; frozen_ = false; }
    const uint32_t anyNow = st.framesRc + st.framesLua + st.framesOther;
    if (anyNow != lastAny_) { lastAny_ = anyNow; lastAnyNs_ = now; }
    // RC silent for 200 ms: freeze the last raw bytes once per silence (kept until RC returns).
    if (!frozen_ && lastRcNs_ != 0 && now - lastRcNs_ >= 200000000LL) {
      crsf.copyRecentBytes(frozenBytes_);
      frozenAtMs_ = static_cast<uint32_t>(now / 1000000LL);
      frozenBytesAtSilence_ = st.bytesIn;
      frozen_ = true;
    }
    if (now < nextLogNs_) return;
    nextLogNs_ = now + 1000000000LL;
    char line[200];
    const long rcAge = lastRcNs_ ? static_cast<long>((now - lastRcNs_) / 1000000LL) : -1;
    const long anyAge = lastAnyNs_ ? static_cast<long>((now - lastAnyNs_) / 1000000LL) : -1;
    snprintf(line, sizeof line,
             "RCDIAG b=%lu rc=%lu lua=%lu oth=%lu crc=%lu len=%lu drop=%lu z0=%lu resc=%lu stale=%lu "
             "rcAge=%ld anyAge=%ld off=%lu sf=%lu rep=%lu/%lu",
             (unsigned long)st.bytesIn, (unsigned long)st.framesRc, (unsigned long)st.framesLua,
             (unsigned long)st.framesOther, (unsigned long)st.crcFail, (unsigned long)st.badLen,
             (unsigned long)st.droppedHunting, (unsigned long)st.zeroStarts, (unsigned long)st.rescans,
             (unsigned long)st.staleResets, rcAge, anyAge, (unsigned long)router.stats.rcFramesOffered,
             (unsigned long)router.stats.rcSendFailed, (unsigned long)crsf.halfRepliesSent(),
             (unsigned long)crsf.halfRepliesReplaced());
    router.log(Packets::Severity::kInfo, line);
    if (frozen_) {
      int n = snprintf(line, sizeof line, "RCFROZEN atMs=%lu bytesIn=%lu last16:", (unsigned long)frozenAtMs_,
                       (unsigned long)frozenBytesAtSilence_);
      for (uint8_t i = 0; i < CrsfParser::kRecentBytes && n > 0 && n < (int)sizeof line - 4; i++)
        n += snprintf(line + n, sizeof line - n, " %02X", frozenBytes_[i]);
      router.log(Packets::Severity::kWarning, line);
    }
  }

private:
  uint32_t lastRc_ = 0, lastAny_ = 0;
  int64_t lastRcNs_ = 0, lastAnyNs_ = 0, nextLogNs_ = 0;
  bool frozen_ = false;
  uint8_t frozenBytes_[CrsfParser::kRecentBytes] = {0};
  uint32_t frozenAtMs_ = 0, frozenBytesAtSilence_ = 0;
};
RcDiag rcDiag;

void enterUploadMode(const char *why) {
  router.log(Packets::Severity::kWarning, why);
  usbLink.poll();
  linkStopped = true;
  WifiUpload::startWiFiUploadMode(); // removes every scheduler task except WiFi: FHSS stops
}

/// LINK_REBOOT: the ACK left 400 ms ago (router.poll() + usbLink.poll()); clean software restart.
void rebootNow(const char *why) {
  router.log(Packets::Severity::kWarning, why);
  usbLink.poll();
  delay(20); // let the USB log line drain
  ESP.restart();
}

} // namespace

void setup() {
  pinMode(HardwareConfig::button, INPUT_PULLUP);
  delay(500);

  Serial.setTxTimeoutMs(0);
  Serial.begin(115200); // baud rate is irrelevant on CDC

  // Boot button held: WiFi upload only, the link never starts (as FPV).
  if (digitalRead(HardwareConfig::button) == LOW) {
    WifiUpload::startWiFiUploadMode(true);
    WifiUpload::initWifiUpload();
    linkStopped = true;
    return;
  }

  // Single shared half-duplex pin: attach as UART RX only, CRSF reroutes it to TX for each reply.
  linkSerialPort.begin(kCrsfBaud, SERIAL_8N1, HardwareConfig::serialRx, -1, false);
  crsf.beginHalfDuplex();
  crsf.setHardenedParser(true); // network mode only: no 0x00 start byte, rescan after a CRC/length failure

  WifiUpload::initWifiUpload();
  applyStoredRfSettings();
  netRadioSetup(/*isRxSide=*/false, airAccess, txPower.get());
  applyPower();

  static UploadTriggers t(coreTime.nowNs(), /*useUartTrigger=*/false);
  triggers = &t;
}

void loop() {
  WifiUpload::loopWifiUpload();
  if (linkStopped) {
    Core::getSystemScheduler().tick(); // WiFi task only
    return;
  }

  netRadioLoop(); // DIO1 timestamps + scheduler (FHSS slots)

  usbLink.poll();

  // Host attach edge: announce ourselves once as LOG_TEXT (a dropped frame while no host is attached is fine).
  const bool host = static_cast<bool>(Serial);
  if (host && !hostWasConnected) router.log(Packets::Severity::kInfo, "Router up");
  hostWasConnected = host;

  crsf.available();
  rcDiag.update(coreTime.nowNs());
  bool paramHandled = false;
  // queuedFrames(), not peekFrameType() != 0: a CRC-valid type-0 frame at the front must be dropped below, not end
  // (or latch) this loop.
  while (crsf.queuedFrames() > 0) {
    if (crsf.peekFrameType() ==
        static_cast<uint8_t>(VCTR::packets::crsf::CRSFFrameType::RcChannelsPacked)) {
      CRSF::ReceivedPacket<VCTR::packets::crsf::CRSFPacket_RcChannelsPacked> rx;
      if (crsf.read(rx)) {
        router.onCrsfRcFrame(rx.packet.channels);
        // Queue the reply that goes out after the handset's next frame (one telemetry frame per handset frame).
        CrsfFrame reply;
        if (router.nextHandsetReply(reply)) crsf.write(RawCrsfFrame(reply));
        continue;
      }
      continue; // read() took the frame out of the queue even though it did not decode
    }
    const uint8_t t = crsf.peekFrameType();
    if (t == static_cast<uint8_t>(VCTR::packets::crsf::CRSFFrameType::ParameterPing) ||
        t == static_cast<uint8_t>(VCTR::packets::crsf::CRSFFrameType::ParameterRead) ||
        t == static_cast<uint8_t>(VCTR::packets::crsf::CRSFFrameType::ParameterWrite)) {
      if (paramHandled) break; // at most one Lua frame per pass (docs/rf-timing-audit.md); the rest wait in the queue
      paramHandled = true;
      handleLuaFrame(t);
      continue;
    }
    crsf.discardFrame(); // anything else is not handled in network mode
  }

  airAccess.process(); // received air packet -> frames -> router
  airAccess.setStatus(netRadioStatus(txPower.get())); // LINK_STATS tx_power and the CRSF power index = what is in use

  router.poll();
  usbLink.poll(); // flush what the router just queued

  if (router.rebootCommandDue() && !linkStopped) rebootNow("reboot command (LINK_REBOOT)"); // ACK left above
  if (routerRebootAtNs != 0 && coreTime.nowNs() >= routerRebootAtNs && !linkStopped)
    rebootNow("reboot router (Lua menu)");

  if (triggers) {
    if (router.uploadCommandDue()) triggers->requestCommand(); // ACK left in router.poll() + usbLink.poll() above
    const auto r = triggers->update(coreTime.nowNs(), netRadioLeftSearching(), false);
    if (r == UploadTriggers::Reason::SyncFailsafe) enterUploadMode("sync failsafe: no RX module for 60 s, WiFi upload");
    else if (r == UploadTriggers::Reason::Command) enterUploadMode("upload command, WiFi upload");
  }
}
