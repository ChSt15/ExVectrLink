#include "RFSetup.hpp"

#include "DiversitySetup.hpp"
#include "DynamicPowerSetup.hpp"
#include "Hardware.hpp"
#include "Settings.hpp"

namespace VCTR::ExVectrLink::datalink {

namespace {

#if !defined(IS_TX_SIDE)
/// @brief What the receiver always transmits at. The RX has no menu to change
/// its power with and no reason to vary it: the TX side is the end that has to
/// reach a moving aircraft, while the RX's own uplink budget is fixed, so it
/// runs the dynamic-power algorithm off at this level rather than at whatever
/// happens to be stored.
constexpr uint8_t kRxFixedPowerDbm = 20;
#endif

/// @brief Points the radios at whatever a stored txAntenna value means: one of
/// the two algorithmic modes, or a fixed antenna index.
/// @return false if `antenna` names an antenna this build has no radio for,
/// in which case nothing is changed.
bool applyTxAntenna(uint8_t antenna) {
  auto &diversity = getSx1280Diversity();

  if (antenna == RFSettings::kBestSnrTxAntenna) {
    diversity.setTxAntennaMode(Sx1280Diversity::TxAntennaMode::BestSnr);
    return true;
  }
  if (antenna == RFSettings::kRoundRobinTxAntenna) {
    diversity.setTxAntennaMode(Sx1280Diversity::TxAntennaMode::RoundRobin);
    return true;
  }
  // setDesignatedTxLinkIndex() switches to Fixed itself, and refuses an index
  // with no radio behind it.
  return diversity.setDesignatedTxLinkIndex(static_cast<size_t>(antenna));
}

} // namespace

uint8_t getTxAntennaCount() {
  auto &diversity = getSx1280Diversity();
  uint8_t count = 0;
  while (diversity.getDiversityLink(count) != nullptr) {
    count++;
  }
  return count;
}

uint8_t getTxAntenna() {
  const auto &diversity = getSx1280Diversity();

  switch (diversity.getTxAntennaMode()) {
  case Sx1280Diversity::TxAntennaMode::RoundRobin:
    return RFSettings::kRoundRobinTxAntenna;

  case Sx1280Diversity::TxAntennaMode::Fixed: {
    const size_t designated = diversity.getDesignatedTxLinkIndex();
    if (designated == Sx1280Diversity::kAutoTxLink) {
      return RFSettings::kBestSnrTxAntenna; // Fixed but nothing designated.
    }
    return static_cast<uint8_t>(designated);
  }

  case Sx1280Diversity::TxAntennaMode::BestSnr:
  default:
    return RFSettings::kBestSnrTxAntenna;
  }
}

bool setTxAntenna(uint8_t antenna) {
  if (!applyTxAntenna(antenna)) {
    return false; // No such antenna -- leave the radios and the store alone.
  }

  saveRfSettings();
  return true;
}

void applyRfSettings() {
  auto &settings = getSettings();
  auto &dynamicPower = getDynamicPower();
  const RFSettings &rf = settings.get<RFSettings>();

#if defined(IS_TX_SIDE)
  dynamicPower.setDynMinPower(rf.dynMinPowerDbm);
  dynamicPower.setDynMaxPower(rf.dynMaxPowerDbm);
  dynamicPower.setEnableDynamicPower(rf.dynamicPowerEnabled);

  // Start at the stored ceiling rather than at the dynamic minimum: the link
  // is not up yet, so the algorithm has no link stats to raise power from, and
  // coming up at the lowest allowed power would make the first seconds of
  // acquisition needlessly marginal. Once packets flow, update() brings it
  // back down.
  dynamicPower.setPower(dynamicPower.getDynMaxPower());
#else
  // Receiver: fixed power, algorithm off, whatever is stored. See
  // kRxFixedPowerDbm.
  dynamicPower.setDynMinPower(kRxFixedPowerDbm);
  dynamicPower.setDynMaxPower(kRxFixedPowerDbm);
  dynamicPower.setEnableDynamicPower(false);
  dynamicPower.setPower(kRxFixedPowerDbm);
#endif

  // An antenna index this build has no radio for (settings written by a
  // diversity build, running on a single-radio one) is refused by
  // applyTxAntenna() -- fall back to best-SNR rather than leaving the radios
  // pinned to whatever was set before.
  if (!applyTxAntenna(rf.txAntenna)) {
    applyTxAntenna(RFSettings::kBestSnrTxAntenna);
  }

  applyDynamicPowerToRadios();
}

void saveRfSettings() {
  auto &settings = getSettings();
  const auto &dynamicPower = getDynamicPower();

  // Read-modify-write: the record holds both the power and the antenna, and
  // each caller only knows about its own half.
  RFSettings rf = settings.get<RFSettings>();
  rf.dynamicPowerEnabled = dynamicPower.isDynamicPowerEnabled();
  rf.dynMinPowerDbm = dynamicPower.getDynMinPower();
  rf.dynMaxPowerDbm = dynamicPower.getDynMaxPower();
  rf.txAntenna = getTxAntenna();
  settings.set(rf);
}

} // namespace VCTR::ExVectrLink::datalink
