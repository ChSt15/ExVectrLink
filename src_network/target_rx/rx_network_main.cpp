// ExVectrNetwork-mode vehicle receiver firmware (env exvectrlink_rx_network_esp8285, and the ESP32 diversity
// variant): node 3, router, sync follower. Contains only the network stack: no CRSF, no FPV OTA application layer.
//
// UART: `Serial` (UART0, the board's RX/TX pads, GPIO 3/1) at 460800 8N1 carries the COBS/CRC network stream to the
// vehicle (node 1) and nothing else. On the ESP8285 UART0 is the only full UART and is shared with debug output, so
// this firmware prints nothing (no Core::initialise(), no DebugMonitor). Only the boot ROM prints at power-up
// (74880 baud garbage on the line; the host resynchronises at the next frame delimiter), and the ArduinoOTA progress
// callbacks print while in WiFi upload mode, when the link is stopped anyway.
// Air: the FHSS air link, air profile v1. Transmit power: 10 dBm (10 mW legal limit) at every boot, then the Router's
// LINK_CONFIG value (clamped to 0..board maximum, RAM only), back to 10 dBm if LINK_CONFIG stops for 10 s
// (src_network/NetPower.hpp). No dynamic power.
// WiFi upload (over-the-air firmware update, the receiver cannot be flashed by UART without soldering):
//   - boot button held at power-up: WiFi only, the link never starts;
//   - button held 3 s while running: WiFi next to the link (as FPV);
//   - 60 s sync failsafe: the air link never left Searching within 60 s of boot (link stopped);
//   - stale UART: no valid frame from the UART host within 60 s of boot (link stopped);
//   - network upload command: REQUEST action LINK_UPLOAD_MODE (10) with target 3, always honoured (LinkUploadCommand).
#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "Hardware.hpp"
#include "WifiUpload.hpp"

#include "NetAirAccess.hpp"
#include "NetPower.hpp"
#include "NetRadio.hpp"
#include "RxRouter.hpp"

#if defined(IS_TX_SIDE)
#error "The network-mode receiver firmware needs a receiver pin map"
#endif

using namespace VCTR;
using namespace VCTR::ExVectrLink;
using namespace VCTR::ExVectrLink::netmode;

namespace {

constexpr uint32_t kUartBaud = 460800;
// Power at boot: 10 dBm (10 mW legal limit), then the runtime follower of LINK_CONFIG (src_network/NetPower.hpp).
constexpr int8_t kRxPowerDbm = kNetTxPowerCapDbm;

/// UART0 as a non-blocking byte stream: never writes more than the hardware FIFO / driver buffer accepts.
class UartStream : public network::ByteStreamI {
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
    const int n = Serial.availableForWrite();
    return n > 0 ? static_cast<size_t>(n) : 0;
  }
  size_t write(const uint8_t *data, size_t len) override { return Serial.write(data, len); }
};

CoreTime coreTime;
UartStream uartStream;
network::StreamFrameLink<1024> uartLink(uartStream, coreTime);

NetAirAccess::Link airLink;
NetAirAccess airAccess(airLink, coreTime, /*isMaster=*/false, netExpectedRxPerSecond(true));
RxRouter router(coreTime, uartLink, airLink, airAccess, HardwareConfig::sx1280AMaxPower);

int8_t currentPower = clampNetPower(kRxPowerDbm, HardwareConfig::sx1280AMaxPower);

/// The power the ship node should use now; applied to the radio on change (a variable write, no radio command, so
/// it is safe in the main loop and takes effect at the next own TX slot).
void applyDesiredPower() {
  const int8_t want = router.desiredPowerDbm();
  if (want == currentPower) return;
  currentPower = want;
  netRadioSetTxPower(currentPower);
}
UploadTriggers *triggers = nullptr;
bool linkStopped = false;

/// AIRDIAG line at 1 Hz (FHSS slot-path counters, docs/air-link-optimisation.md "AIRDIAG"). Read in the main loop only.
AirDiagReporter airDiag;
void airDiagUpdate() {
  char line[240];
  const size_t n = airDiag.poll(coreTime.nowNs(), netAirDiagInput(), line, sizeof line);
  if (n == 0) return;
  router.log(Packets::Severity::kInfo, line);
  netFhss().resetSlotDiagWindow();
}

void enterUploadMode(const char *why) {
  router.log(Packets::Severity::kWarning, why); // queued for the air; may not get out before the link stops
  linkStopped = true;
  WifiUpload::startWiFiUploadMode(); // removes every scheduler task except WiFi: FHSS stops
}

/// LINK_REBOOT (REQUEST action 11, target 3): clean software restart. In flight this stops RC for a few seconds
/// (failsafe); the senders guard it. No log line: it could not leave before the restart.
void rebootNow() { ESP.restart(); }

} // namespace

void setup() {
  pinMode(HardwareConfig::button, INPUT_PULLUP);
  delay(500);

  if (digitalRead(HardwareConfig::button) == LOW) {
    WifiUpload::startWiFiUploadMode(true);
    WifiUpload::initWifiUpload();
    linkStopped = true;
    return;
  }

#if defined(ESP8266)
  Serial.setRxBufferSize(1024);
#endif
  Serial.begin(kUartBaud, SERIAL_8N1);

  WifiUpload::initWifiUpload();

  netRadioSetup(/*isRxSide=*/true, airAccess, currentPower);
  netRadioSetTxPower(currentPower);

  static UploadTriggers t(coreTime.nowNs(), /*useUartTrigger=*/true);
  triggers = &t;
}

void loop() {
  WifiUpload::loopWifiUpload();
  if (linkStopped) {
    Core::getSystemScheduler().tick(); // WiFi task only
    return;
  }

  netRadioLoop(); // DIO1 timestamps + scheduler (FHSS slots)

  uartLink.poll();
  airAccess.process(); // received air packet -> frames -> router (-> UART, LINK_CONFIG -> power follower)
  applyDesiredPower();
  airAccess.setStatus(netRadioStatus(currentPower)); // LINK_STATS tx_power = what this radio really uses
  router.poll();
  airDiagUpdate();
  uartLink.poll(); // flush what the router just queued

  if (router.rebootCommandDue() && !linkStopped) rebootNow(); // ACK was sent 400 ms earlier (across the air)

  if (triggers) {
    if (router.uploadCommandDue()) triggers->requestCommand(); // ACK was sent 400 ms earlier
    const auto r = triggers->update(coreTime.nowNs(), netRadioLeftSearching(), router.uartHostSeen());
    if (r == UploadTriggers::Reason::SyncFailsafe) enterUploadMode("sync failsafe: no TX module for 60 s, WiFi upload");
    else if (r == UploadTriggers::Reason::UartStale) enterUploadMode("no UART host for 60 s, WiFi upload");
    else if (r == UploadTriggers::Reason::Command) enterUploadMode("upload command, WiFi upload");
  }
}
