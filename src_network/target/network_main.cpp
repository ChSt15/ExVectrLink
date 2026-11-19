// ExVectrNetwork-mode TX module firmware (env exvectrlink_tx_module_network): node 2, router.
//
// USB CDC `Serial` carries the COBS/CRC network stream to the ground station and nothing else: no debug text, no
// DebugMonitor, no Core::initialise() (its print hooks would write to Serial). Logs are LOG_TEXT packets.
// The CRSF half-duplex port to the handset is the same one the FPV firmware uses (400 kBaud, inverted, shared
// pin), driven through src/CRSF.cpp.
// The air side is a stub (src_network/AirStub.hpp). There is no WiFi, no FHSS, no radio use in this firmware.
//
// Compiled only by the network env. The FPV envs never see this directory (platformio.ini build_src_filter).
#include <Arduino.h>
#include <esp_timer.h>

#include "ExVectrArduinoPlatform.hpp" // pulls in the platform clock that CRSF.cpp needs (Core::NowNs)

#include "CRSF.hpp"
#include "Hardware.hpp"
#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"

#include "AirStub.hpp"
#include "NetworkRouter.hpp"

#if !defined(TARGET_EXVECTRLINKTXMODULEDIVERSITY)
#error "The network-mode TX firmware is only defined for the diversity TX module pin map"
#endif

using namespace VCTR;
using namespace VCTR::ExVectrLink;
using namespace VCTR::ExVectrLink::netmode;

namespace {

constexpr uint32_t kCrsfBaud = 400000;

/// Time source for the stack: monotonic nanoseconds from the 64-bit microsecond timer.
class EspTime : public network::TimeSourceI {
public:
  int64_t nowNs() override { return esp_timer_get_time() * 1000LL; }
};

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

EspTime espTime;
UsbStream usbStream;
network::StreamFrameLink<1024> usbLink(usbStream, espTime);

/// Admits frames to USB only while a host is attached and the CDC buffer is not backed up (<= 256 bytes still
/// waiting in the TX ring), so stale frames are dropped, never held back for later.
class UsbGate : public LinkGateI {
public:
  bool admits() override { return Serial && usbLink.txPending() <= 256; }
};
UsbGate usbGate;

AirSide air;
NetworkRouter router(espTime, usbLink, &usbGate, air.link, air.stats);

HardwareSerial linkSerialPort(1);
CRSF crsf(linkSerialPort, CRSFDuplex::Half, kCrsfBaud, HardwareConfig::serialRx, 1, true);

bool hostWasConnected = false;

// The SX1280s are not used in this build. Keep both deselected and their RF switches off.
void parkRadios() {
  pinMode(HardwareConfig::sx1280ANss, OUTPUT);
  digitalWrite(HardwareConfig::sx1280ANss, HIGH);
  pinMode(HardwareConfig::sx1280ARxEn, OUTPUT);
  digitalWrite(HardwareConfig::sx1280ARxEn, LOW);
  pinMode(HardwareConfig::sx1280ATxEn, OUTPUT);
  digitalWrite(HardwareConfig::sx1280ATxEn, LOW);
#if defined(HAS_DIVERSITY)
  pinMode(HardwareConfig::sx1280BNss, OUTPUT);
  digitalWrite(HardwareConfig::sx1280BNss, HIGH);
  pinMode(HardwareConfig::sx1280BRxEn, OUTPUT);
  digitalWrite(HardwareConfig::sx1280BRxEn, LOW);
  pinMode(HardwareConfig::sx1280BTxEn, OUTPUT);
  digitalWrite(HardwareConfig::sx1280BTxEn, LOW);
#endif
}

} // namespace

void setup() {
  parkRadios();

  Serial.setTxTimeoutMs(0);
  Serial.begin(115200); // baud rate is irrelevant on CDC

  // Single shared half-duplex pin: attach as UART RX only, CRSF reroutes it to TX for each reply.
  // Same setup as CRSFRcInput::taskInit() (docs/crsf.md).
  linkSerialPort.begin(kCrsfBaud, SERIAL_8N1, HardwareConfig::serialRx, -1, false);
  crsf.beginHalfDuplex();
}

void loop() {
  const int64_t now = espTime.nowNs();

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

  router.poll();
  air.access.poll(now);
  usbLink.poll(); // flush what the router just queued
}
