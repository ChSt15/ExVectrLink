#include "DynamicPowerSetup.hpp"

#include "DatalinkSetup.hpp"
#include "FhssSetup.hpp"

namespace VCTR::ExVectrLink::datalink {

static int64_t lastReceivedLinkInfoPacketTime = 0;

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
  lastReceivedLinkInfoPacketTime = Core::NowNs();
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

    if ((consecutiveLostPackets >= 2 ||
         Core::NowNs() - lastReceivedLinkInfoPacketTime >
             300 * Core::MILLISECONDS) &&
        getDynamicPower().isDynamicPowerEnabled()) {
      getDynamicPower().incPower();
    }
    applyDynamicPowerToRadios();
  });
}

} // namespace VCTR::ExVectrLink::datalink