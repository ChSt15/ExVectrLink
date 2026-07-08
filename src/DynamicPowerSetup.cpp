#include "DynamicPowerSetup.hpp"

#include "DatalinkCommandManager.hpp"
#include "DatalinkSetup.hpp"
#include "FhssSetup.hpp"

namespace VCTR::ExVectrLink::datalink {

VCTR::ExVectrLink::datalink::DynamicPower &getDynamicPower() {
  static DynamicPower dynamicPower;
  return dynamicPower;
}

void applyDynamicPowerToRadios() {
  int8_t powerDbm = getDynamicPower().getPower();
  getDatalinkRadioA().setTxPower(powerDbm);
#if defined(HAS_DIVERSITY)
  getDatalinkRadioB().setTxPower(powerDbm);
#endif
}

void updateDynamicPowerFromRemoteLinkInfo(int8_t remoteRssi, int8_t remoteSnr,
                                          uint8_t remoteLq) {
  auto &dynamicPower = getDynamicPower();
  dynamicPower.update(true, remoteRssi, remoteSnr, remoteLq);
  applyDynamicPowerToRadios();
}

void setupDynamicPower() {
  static bool setupDone = false;
  if (setupDone) {
    return;
  }
  setupDone = true;

  // Apply the current configured dynamic-power value immediately.
  applyDynamicPowerToRadios();

  getFhss().addSlotHandler([](FHSSSlotEvent slotEvent, bool isRxSlot,
                              bool receivedPacket, FHSSState) {
    if (slotEvent != FHSSSlotEvent::SlotEnd || !isRxSlot) {
      return;
    }

    static uint8_t consecutiveLostPackets = 0;
    if (receivedPacket) {
      consecutiveLostPackets = 0;
    } else {
      consecutiveLostPackets++;
    }

    // "Far end link info gone stale" is deliberately keyed off
    // getRemoteLinkStats().valid (any OTA packet within the last 1000 ms --
    // see its doc comment) rather than a separately-tracked "last
    // LinkTelemetry packet" timestamp: LinkInfoSender substitutes an
    // RxState packet for LinkTelemetry roughly once every 1.2 s, creating a
    // routine ~400 ms gap between LinkTelemetry packets on an otherwise
    // perfectly healthy link. A tighter, LinkTelemetry-only staleness check
    // here previously false-triggered on that gap every cycle -- and with
    // no rate limit on this per-slot incPower() (unlike decPower()'s 1 s
    // cooldown), a handful of slot-ends during each false-positive window
    // was enough to saturate power to max almost immediately.
    if ((consecutiveLostPackets >= 2 || !getRemoteLinkStats().valid) &&
        getDynamicPower().isDynamicPowerEnabled()) {
      getDynamicPower().incPower();
    }
    applyDynamicPowerToRadios();
  });
}

} // namespace VCTR::ExVectrLink::datalink