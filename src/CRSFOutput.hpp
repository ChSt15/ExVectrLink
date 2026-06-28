#ifndef EXVECRLINK_CRSFOUTPUT_HPP
#define EXVECRLINK_CRSFOUTPUT_HPP

#ifdef ENABLE_CRSF

#include <Arduino.h>

#include "ExVectrCore.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"
#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

namespace VCTR::ExVectrLink {

uint16_t getCrsfRcPacketRateHz();
uint16_t getCrsfTxFailureRateHz();

/**
 * @brief Outputs CRSF protocol frames (RC channels + link statistics) over a
 * HardwareSerial port. Designed as a non-blocking periodic task (<1ms per
 * execution). Converts internal RadioLinkPacket_A4D4 data to standard CRSF
 * frames suitable for flight controllers.
 */
class CRSFOutput : public Core::Task_Periodic {
private:
  // ── CRSF protocol constants ──
  static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;
  static constexpr uint8_t CRSF_FRAMETYPE_GPS = 0x02;
  static constexpr uint8_t CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08;
  static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
  static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;
  static constexpr uint16_t CRSF_CHANNEL_MIN = 172;
  static constexpr uint16_t CRSF_CHANNEL_MID = 992;
  static constexpr uint16_t CRSF_CHANNEL_MAX = 1811;
  static constexpr uint32_t CRSF_BAUD_RATE = 420000;
  static constexpr uint8_t TELEMETRY_QUEUE_SIZE = 4;
  static constexpr uint16_t SERIAL_WRITE_BUFFER_SIZE = 256;
  static constexpr uint8_t SERIAL_WRITE_MAX_CHUNK = 32;

  // ── CRSF incoming frame parsing (for bind → upload mode) ──
  static constexpr uint8_t CRSF_FRAMETYPE_COMMAND = 0x32;
  static constexpr uint8_t CRSF_COMMAND_SUBCMD_RX = 0x10;
  static constexpr uint8_t CRSF_COMMAND_SUBCMD_RX_BIND = 0x01;
  static constexpr uint8_t CRSF_MAX_FRAME_LEN = 64;
  uint8_t rxBuf_[CRSF_MAX_FRAME_LEN];
  uint8_t rxBufPos_ = 0;
  void processIncomingBytes();
  void handleCrsfFrame(const uint8_t *frame, uint8_t len);

  // ── Packet handling ──
  VCTR::packets::PacketManager packetManager_;
  VCTR::packets::radiolink::RadioLinkPacket_A4D4 packet_;
  VCTR::packets::radiolink::RadioLinkPacketFilter<
      4, 4, VCTR::packets::radiolink::RadioLinkTypes::A4D4>
      packetFilter_;
  Core::ListArray<uint8_t> pendingBatteryTelemetryData_;
  Core::ListArray<uint8_t> pendingGpsTelemetryData_;
  Core::ListBuffer<Core::ListArray<uint8_t>, TELEMETRY_QUEUE_SIZE>
      telemetryQueue_;
  Core::ListBuffer<uint8_t, SERIAL_WRITE_BUFFER_SIZE> serialWriteBuffer_;
  int64_t lastPacketTime_ = 0;

  bool newRcPacket = false;

  // Link-stats cadence counter (send every N-th taskThread call)
  uint8_t linkStatsCounter_ = 0;
  static constexpr uint8_t LINK_STATS_INTERVAL =
      10; // every ~100 ms at 10 ms period

  // ── Helpers ──
  static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte);
  static uint8_t calculateCRC(const uint8_t *data, size_t length);
  static uint16_t readU16BE(const uint8_t *data);
  static uint16_t normToCrsf(int16_t val);

  void packChannels(const uint16_t *channels, uint8_t *payload);
  bool queueSerialFrame(const uint8_t *frame, size_t frameSize);
  void flushSerialWriteBuffer();
  void writeRcChannelsFrame();
  void writeLinkStatsFrame();
  void queueMessages();
  void forwardBatteryTelemetry(const uint8_t *frame, uint8_t len);
  void forwardGpsTelemetry(const uint8_t *frame, uint8_t len);

public:
  CRSFOutput();
  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;
};

} // namespace VCTR::ExVectrLink

#endif // ENABLE_CRSF
#endif // EXVECRLINK_CRSFOUTPUT_HPP
