// ExVectrNetwork-mode TX module firmware (env exvectrlink_tx_module_network): node 2, router, sync master.
//
// USB CDC `Serial` carries the COBS/CRC network stream to the ground station and nothing else: no debug text, no
// DebugMonitor, no Core::initialise() (its print hooks would write to Serial). Logs are LOG_TEXT packets.
// The CRSF half-duplex port to the handset is the same one the FPV firmware uses (400 kBaud, inverted, shared pin),
// driven through src/CRSF.cpp.
// The air side is the FHSS air link (air profile v1, src_network/target/NetRadio.cpp) with diversity on both radios
// and dynamic power fed by the vehicle receiver's LINK_STATS (node 3). Power limits and the TX antenna come from the
// same EEPROM settings as the FPV firmware (read only here; there is no Lua menu in this firmware).
// WiFi upload (over-the-air firmware update) as in FPV: boot button held at power-up (link never starts), button
// held 3 s while running (WiFi next to the link), and the 60 s sync failsafe (link stopped). USB flashing works as
// before.
//
// Compiled only by the network env. The FPV envs never see this directory (platformio.ini build_src_filter).
#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "CRSF.hpp"
#include "ExVectrLink/datalink/DynamicPower.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"
#include "Hardware.hpp"
#include "Settings.hpp"
#include "WifiUpload.hpp"

#include "NetAirAccess.hpp"
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

datalink::DynamicPower dynamicPower;
NetPowerPolicy powerPolicy;
UploadTriggers *triggers = nullptr;

bool hostWasConnected = false;
bool linkStopped = false; // WiFi upload mode with the link stopped (until reset)

void applyPower() { netRadioSetTxPower(dynamicPower.getPower()); }

/// Power limits and TX antenna from the FPV settings record (read only; defaults on a fresh device).
void applyStoredRfSettings() {
  setupSettings();
  const RFSettings &rf = getSettings().get<RFSettings>();
  dynamicPower.setMaxPower(HardwareConfig::sx1280AMaxPower);
  dynamicPower.setMinPower(10);
  dynamicPower.setDynMinPower(rf.dynMinPowerDbm);
  dynamicPower.setDynMaxPower(rf.dynMaxPowerDbm);
  dynamicPower.setEnableDynamicPower(rf.dynamicPowerEnabled);
  dynamicPower.setPower(dynamicPower.getDynMaxPower()); // start at the ceiling, as FPV
  auto &div = netDiversity();
  const size_t idx = rf.txAntenna == RFSettings::kDiversityTxAntenna ? datalink::Sx1280Diversity::kAutoTxLink
                                                                     : static_cast<size_t>(rf.txAntenna);
  if (!div.setDesignatedTxLinkIndex(idx)) div.setDesignatedTxLinkIndex(datalink::Sx1280Diversity::kAutoTxLink);
}

void enterUploadMode(const char *why) {
  router.log(Packets::Severity::kWarning, why);
  usbLink.poll();
  linkStopped = true;
  WifiUpload::startWiFiUploadMode(); // removes every scheduler task except WiFi: FHSS stops
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

  WifiUpload::initWifiUpload();
  applyStoredRfSettings();
  netRadioSetup(/*isRxSide=*/false, airAccess, dynamicPower.getPower());
  applyPower();

  // Per RX slot: power one step up after 2 lost RX slots in a row or while node 3's LINK_STATS is stale.
  netFhss().addSlotHandler([](datalink::FHSSSlotEvent ev, bool isRxSlot, bool received, datalink::FHSSState) {
    if (ev != datalink::FHSSSlotEvent::SlotEnd || !isRxSlot) return;
    const auto &port = router.linkStatsPort();
    const bool fresh = NetPowerPolicy::remoteFresh(port.hasValue(), port.rxNs(), coreTime.nowNs());
    if (powerPolicy.onRxSlotEnd(received, fresh) && dynamicPower.isDynamicPowerEnabled()) {
      dynamicPower.incPower();
    }
    applyPower();
  });

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
  if (host && !hostWasConnected) router.log(Packets::Severity::kInfo, "tx network router up");
  hostWasConnected = host;

  crsf.available();
  while (crsf.peekFrameType() != 0) {
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
    }
    crsf.discardFrame(); // Lua parameter traffic etc. is not handled in network mode
  }

  airAccess.process(); // received air packet -> frames -> router
  airAccess.setStatus(netRadioStatus(dynamicPower.getPower()));

  // Dynamic power from every new LINK_STATS of the vehicle receiver (node 3).
  const auto &port = router.linkStatsPort();
  if (port.hasValue() && powerPolicy.newRemoteStats(port.rxNs()) && dynamicPower.isDynamicPowerEnabled()) {
    const auto &ls = port.value();
    dynamicPower.update(true, ls.rssi, ls.snr, ls.rx_lq);
    applyPower();
  }

  router.poll();
  usbLink.poll(); // flush what the router just queued

  if (triggers) {
    if (router.uploadCommandDue()) triggers->requestCommand(); // ACK left in router.poll() + usbLink.poll() above
    const auto r = triggers->update(coreTime.nowNs(), netRadioLeftSearching(), false);
    if (r == UploadTriggers::Reason::SyncFailsafe) enterUploadMode("sync failsafe: no RX module for 60 s, WiFi upload");
    else if (r == UploadTriggers::Reason::Command) enterUploadMode("upload command, WiFi upload");
  }
}
