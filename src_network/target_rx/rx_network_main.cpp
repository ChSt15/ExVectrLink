// ExVectrNetwork-mode vehicle receiver firmware (env exvectrlink_rx_network_esp8285, and the ESP32 diversity
// variant): node 3, router, sync follower. Contains only the network stack: no CRSF, no FPV OTA application layer.
//
// UART: `Serial` (UART0, the board's RX/TX pads, GPIO 3/1) at 460800 8N1 carries the COBS/CRC network stream to the
// vehicle (node 1) and nothing else. On the ESP8285 UART0 is the only full UART and is shared with debug output, so
// this firmware prints nothing (no Core::initialise(), no DebugMonitor). Only the boot ROM prints at power-up
// (74880 baud garbage on the line; the host resynchronises at the next frame delimiter), and the ArduinoOTA progress
// callbacks print while in WiFi upload mode, when the link is stopped anyway.
// Air: the FHSS air link, air profile v1, fixed transmit power (20 dBm clamped to the board maximum), as the FPV RX.
// WiFi upload (over-the-air firmware update, the receiver cannot be flashed by UART without soldering):
//   - boot button held at power-up: WiFi only, the link never starts;
//   - button held 3 s while running: WiFi next to the link (as FPV);
//   - 60 s sync failsafe: the air link never left Searching within 60 s of boot (link stopped);
//   - stale UART: no valid frame from the UART host within 60 s of boot (link stopped);
//   - network upload command: REQUEST action LINK_UPLOAD_MODE (10) with target 3, always honoured (LinkUploadCommand).
#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrLink/datalink/DynamicPower.hpp"
#include "Hardware.hpp"
#include "WifiUpload.hpp"

#include "NetAirAccess.hpp"
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
constexpr int8_t kRxPowerDbm = 20; // as the FPV receiver; the driver clamps it to the board maximum

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
RxRouter router(coreTime, uartLink, airLink, airAccess);

datalink::DynamicPower power; // fixed power: only used for the board limit
UploadTriggers *triggers = nullptr;
bool linkStopped = false;

void enterUploadMode(const char *why) {
  router.log(Packets::Severity::kWarning, why); // queued for the air; may not get out before the link stops
  linkStopped = true;
  WifiUpload::startWiFiUploadMode(); // removes every scheduler task except WiFi: FHSS stops
}

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

  power.setMaxPower(HardwareConfig::sx1280AMaxPower);
  power.setMinPower(10);
  power.setEnableDynamicPower(false);
  power.setDynMinPower(kRxPowerDbm);
  power.setDynMaxPower(kRxPowerDbm);
  power.setPower(kRxPowerDbm);

  netRadioSetup(/*isRxSide=*/true, airAccess, power.getPower());
  netRadioSetTxPower(power.getPower());

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
  airAccess.process(); // received air packet -> frames -> router (-> UART)
  airAccess.setStatus(netRadioStatus(power.getPower()));
  router.poll();
  uartLink.poll(); // flush what the router just queued

  if (triggers) {
    if (router.uploadCommandDue()) triggers->requestCommand(); // ACK was sent 400 ms earlier
    const auto r = triggers->update(coreTime.nowNs(), netRadioLeftSearching(), router.uartHostSeen());
    if (r == UploadTriggers::Reason::SyncFailsafe) enterUploadMode("sync failsafe: no TX module for 60 s, WiFi upload");
    else if (r == UploadTriggers::Reason::UartStale) enterUploadMode("no UART host for 60 s, WiFi upload");
    else if (r == UploadTriggers::Reason::Command) enterUploadMode("upload command, WiFi upload");
  }
}
