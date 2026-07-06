#include "CRSFRcInput.hpp"

#ifdef ENABLE_CRSF_INPUT

#include "ExVectrCore.hpp"

#include "ExVectrLink/datalink/PacketTypes.hpp"
#include "FhssSetup.hpp"

namespace VCTR::ExVectrLink {

namespace {
constexpr uint16_t CRSF_CHANNEL_MIN = 172;
constexpr uint16_t CRSF_CHANNEL_MAX = 1811;
} // namespace

CRSFRcInput::CRSFRcInput(HardwareSerial &serial, int8_t rxPin, int8_t txPin,
                         uint32_t baudRate, bool invert)
    : Core::Task_Periodic("CRSFRcInput", 2 * Core::MILLISECONDS),
      serial_(serial), rxPin_(rxPin), txPin_(txPin), baudRate_(baudRate),
      invert_(invert), crsf_(serial, CRSFDuplex::Half, baudRate) {
  Core::getSystemScheduler().addTask(*this);
}

void CRSFRcInput::taskInit() {
  // crsf_ deliberately never has begin() called on it: it only knows the
  // generic 1-argument HardwareSerial::begin(baud), which has no way to
  // express the pin-binding workaround below. We configure the port
  // directly instead; crsf_ is only used for CRSF framing/queueing from
  // here on.
#if defined(ESP32)
  if (rxPin_ >= 0 && txPin_ >= 0 && rxPin_ == txPin_) {
    // Single shared half-duplex signal wire (e.g. a "nano" module bay's
    // S.Port-style pin). Binding both RX and TX to the same pin previously
    // triggered the ESP32's automatic RS485-half-duplex hardware mode,
    // whose auto-RTS logic gated the receiver on our own transmit activity
    // and left it effectively permanently disabled. Only attach the pad as
    // UART RX (leave TX unmapped/floating) so we purely listen and never
    // drive the shared line ourselves -- this class never calls
    // crsf_.write() in Half duplex mode anyway (see CRSF::writeFrame()).
    serial_.begin(baudRate_, SERIAL_8N1, rxPin_, -1, invert_);
  } else if (rxPin_ >= 0 && txPin_ >= 0) {
    serial_.begin(baudRate_, SERIAL_8N1, rxPin_, txPin_, invert_);
  } else {
    serial_.begin(baudRate_);
  }
#else
  serial_.begin(baudRate_);
#endif
}

void CRSFRcInput::taskThread() {
  crsf_.available();

  // Drain every queued frame each tick: forward RC channels, discard
  // anything else so it can never wedge behind an unrelated frame type.
  while (crsf_.peekFrameType() != 0) {
    if (crsf_.peekFrameType() == static_cast<uint8_t>(
                                     VCTR::packets::crsf::CRSFFrameType::RcChannelsPacked)) {
      CRSF::ReceivedPacket<VCTR::packets::crsf::CRSFPacket_RcChannelsPacked>
          received;
      if (crsf_.read(received)) {
        forwardRcChannels(received.packet);
        continue;
      }
    }
    crsf_.discardFrame();
  }
}

int16_t CRSFRcInput::crsfToNorm(uint16_t val) {
  // Map from 172 … 1811  →  -1000 … 1000  (inverse of CRSFRcOutput::normToCrsf)
  float norm = ((float)(val - CRSF_CHANNEL_MIN) /
                (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) *
                   2000.0f -
               1000.0f;
  if (norm > 1000.0f)
    norm = 1000.0f;
  if (norm < -1000.0f)
    norm = -1000.0f;
  return (int16_t)norm;
}

uint8_t CRSFRcInput::digitalFromCrsf(uint16_t val) {
  float norm = ((float)(val - CRSF_CHANNEL_MIN) /
                (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) *
               2.0f;
  if (norm < 0.0f)
    norm = 0.0f;
  if (norm > 2.0f)
    norm = 2.0f;
  return (uint8_t)(norm + 0.5f);
}

void CRSFRcInput::forwardRcChannels(
    const VCTR::packets::crsf::CRSFPacket_RcChannelsPacked &pkt) {
  VCTR::packets::radiolink::RadioLinkPacket_A4D4 packet;

  // Analogue channels (inverse of CRSFRcOutput::writeRcChannelsFrame mapping)
  packet.analogChannels[0] = crsfToNorm(pkt.channels[0]); // Roll
  packet.analogChannels[1] = crsfToNorm(pkt.channels[1]); // Pitch
  packet.analogChannels[3] = crsfToNorm(pkt.channels[2]); // Throttle
  packet.analogChannels[2] = crsfToNorm(pkt.channels[3]); // Yaw

  // Digital channels (inverse of CRSFRcOutput's mapping)
  packet.digitalChannels[0] = digitalFromCrsf(pkt.channels[4]);
  packet.digitalChannels[1] = digitalFromCrsf(pkt.channels[5]);
  packet.digitalChannels[2] = digitalFromCrsf(pkt.channels[6]);
  packet.digitalChannels[3] = digitalFromCrsf(pkt.channels[7]);

  lastRcPacket_ = packet;
  lastRcPacketTime_ = Core::NowNs();
  rcFrameCount_++;

  // Forward straight to the FHSS uplink, tagged the same way SerialTelecoms'
  // PacketData handler does it (see DatalinkCommandManager.cpp), so the
  // receive side (CRSFRcOutput) doesn't need to know or care whether the RC
  // data came from CRSF or a serial host app.
  VCTR::network::DataPacket otaPkt;
  otaPkt.payload.setSize(packet.numBytes() + 1);
  packet.serialize(otaPkt.payload.getPtr());
  otaPkt.payload[packet.numBytes()] =
      (uint8_t)ExVectrLink::datalink::PacketTypes::Data;
  ExVectrLink::datalink::getFhss().transmitDataframe(otaPkt);
}

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF_INPUT
