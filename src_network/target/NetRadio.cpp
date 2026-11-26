#include <Arduino.h>

#include "ExVectrArduinoPlatform.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrLink/datalink/AirProfile.hpp"
#include "ExVectrLink/datalink/FhssMath.hpp"

#include "DatalinkSetup.hpp"
#include "DiversitySetup.hpp"
#include "FhssSetup.hpp"
#include "Hardware.hpp"
#include "SX1280.hpp"

#include "NetRadio.hpp"

namespace VCTR::ExVectrLink::netmode {

using datalink::AirProfile;
using datalink::kNetworkProfileV1;
namespace sx = VCTR::network::datalink;

int64_t CoreTime::nowNs() { return Core::NowNs(); }

namespace {

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

// Same as src/main.cpp drainDio1Timestamp(): atomically take one ISR capture and convert it to Core time.
int64_t drainDio1Timestamp(volatile bool &fired, volatile uint32_t &isrTime) {
  if (!fired) return 0;
  noInterrupts();
  uint32_t isrMicros = isrTime;
  fired = false;
  interrupts();
  uint32_t elapsedMicros = micros() - isrMicros;
  return Core::NowNs() - int64_t(elapsedMicros) * Core::MICROSECONDS;
}
int64_t drainDio1TimestampA() { return drainDio1Timestamp(dio1FiredA, dio1MicrosA); }
#if defined(HAS_DIVERSITY)
int64_t drainDio1TimestampB() { return drainDio1Timestamp(dio1FiredB, dio1MicrosB); }
#endif

NetAirAccess *gAccess = nullptr;

// The air profile in use: chosen at build time (kNetworkProfileActive, v1 unless -DEXVECTRLINK_NET_PROFILE_V2_TRIAL).
// A profile that does not fit its slot (netSlotTiming) is a build mistake; fall back to v1 instead of running it.
const AirProfile &activeProfile() {
  return datalink::netSlotTiming(datalink::kNetworkProfileActive).fitsWithGrace ? datalink::kNetworkProfileActive
                                                                               : kNetworkProfileV1;
}

sx::SX1280_SF sfOf(const AirProfile &p) {
  switch (p.lora.sf) {
  case 6: return sx::SF_6;
  case 7: return sx::SF_7;
  default: return sx::SF_5;
  }
}

void configureRadio(sx::Sx1280_Direct &radio, int maxPower, int paDbm, int8_t power) {
  const auto &p = activeProfile();
  radio.setMaxFrameLength(p.fixedFrameLength); // 255: shared FIFO base 0 (before configureRadio)
  radio.setSpreadingFactor(sfOf(p));           // BW1600 and CR 4/5 are the same in every network profile so far
  radio.setBandwidth(sx::BW_1600KHz);
  radio.setCodingRate(sx::CR_4_5);
  radio.setPacketMode(sx::SX1280_PacketMode::Fixed); // implicit header, exactly fixedFrameLength bytes, no length prefix
  radio.setFixedPacketLength(p.fixedFrameLength);
  radio.setChannel(0);
  radio.setTxMaxPower(maxPower); // board maximum = the runtime ceiling; the power in use is set separately
  radio.setPAdbm(paDbm);
  radio.setTxPower(clampNetPower(power, maxPower));
}

} // namespace

datalink::FHSS &netFhss() { return datalink::getFhss(); }
datalink::Sx1280Diversity &netDiversity() { return datalink::getSx1280Diversity(); }

uint16_t netExpectedRxPerSecond(bool isRxSide) {
  return static_cast<uint16_t>(datalink::fhss::expectedRxPacketsPerSecond(
      isRxSide, activeProfile().rolePeriod, activeProfile().slotIntervalNs));
}

void netRadioSetup(bool isRxSide, NetAirAccess &access, int8_t txPowerDbm) {
  gAccess = &access;

  pinMode(HardwareConfig::sx1280ADio1, INPUT);
  attachInterrupt(digitalPinToInterrupt(HardwareConfig::sx1280ADio1), onRadioADio1, RISING);
#if defined(HAS_DIVERSITY)
  pinMode(HardwareConfig::sx1280BDio1, INPUT);
  attachInterrupt(digitalPinToInterrupt(HardwareConfig::sx1280BDio1), onRadioBDio1, RISING);
#endif

  auto &fhss = datalink::getFhss();
  fhss.setIsRxSide(isRxSide);
  fhss.applyProfile(activeProfile());
  fhss.setTxPayloadProvider(&access);

  radio::initRadioDrivers();

  auto &radioA = datalink::getDatalinkRadioA();
  configureRadio(radioA, HardwareConfig::sx1280AMaxPower, HardwareConfig::sx1280APAdbm, txPowerDbm);
  radioA.setDio1TimestampSource(drainDio1TimestampA);
#if defined(HAS_DIVERSITY)
  auto &radioB = datalink::getDatalinkRadioB();
  configureRadio(radioB, HardwareConfig::sx1280AMaxPower, HardwareConfig::sx1280APAdbm, txPowerDbm);
  radioB.setDio1TimestampSource(drainDio1TimestampB);
#endif

  // Every frame that passed the FHSS trailer check: copy the payload (253 bytes, trailer removed) into the mailbox.
  // Runs inside the FHSS slot task, so it does nothing else.
  fhss.addReceiveHandler([](const VCTR::network::DataPacket &p) {
    if (!gAccess) return;
    auto &div = datalink::getSx1280Diversity();
    gAccess->onRadioPacket(p.payload.getPtr(), p.payload.size(), p.timestamp, div.getPacketRSSI(),
                           div.getPacketSNR());
  });
}

void netRadioLoop() {
  if (datalink::getDatalinkRadioA().isConfigured()) {
    int64_t t = drainDio1TimestampA();
    if (t != 0) datalink::getDatalinkRadioA().notifyDio1Irq(t, true);
  }
#if defined(HAS_DIVERSITY)
  if (datalink::getDatalinkRadioB().isConfigured()) {
    int64_t t = drainDio1TimestampB();
    if (t != 0) datalink::getDatalinkRadioB().notifyDio1Irq(t, true);
  }
#endif
  Core::getSystemScheduler().tick();
}

void netRadioSetTxPower(int8_t dbm) {
  dbm = clampNetPower(dbm, HardwareConfig::sx1280AMaxPower);
  datalink::getDatalinkRadioA().setTxPower(dbm);
#if defined(HAS_DIVERSITY)
  datalink::getDatalinkRadioB().setTxPower(dbm);
#endif
}

bool netRadioLeftSearching() {
  return datalink::getFhss().getFhssState() != datalink::FHSSState::Searching;
}

AirLinkStatus netRadioStatus(int8_t txPowerDbm) {
  AirLinkStatus s;
  switch (datalink::getFhss().getFhssState()) {
  case datalink::FHSSState::Searching:
    s.state = Packets::AirState::kSearching;
    break;
  case datalink::FHSSState::Syncing:
    s.state = Packets::AirState::kSyncing;
    break;
  case datalink::FHSSState::Synced:
    s.state = Packets::AirState::kSynced;
    break;
  }
  s.txPowerDbm = txPowerDbm;
  auto &div = datalink::getSx1280Diversity();
  s.rxAntenna = static_cast<uint8_t>(div.getCurrentBestLinkIndex() & 0x0F);
  s.txAntenna = static_cast<uint8_t>(div.getCurrentTxLinkIndex() & 0x0F);
  return s;
}

} // namespace VCTR::ExVectrLink::netmode
