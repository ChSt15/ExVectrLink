#ifndef EXVECRLINK_RFSETUP_HPP
#define EXVECRLINK_RFSETUP_HPP

#include <cstdint>

namespace VCTR::ExVectrLink::datalink {

/**
 * @brief Applies the stored RF settings (transmit power and TX antenna) to the
 * radios. Call once at startup, after the hardware power limits are set (they
 * clamp whatever is loaded) and after setupSettings().
 *
 * @note On the receiver side the power part is not taken from the settings at
 * all: the RX always runs the dynamic-power algorithm off, at
 * kRxFixedPowerDbm. See the .cpp.
 */
void applyRfSettings();

/**
 * @brief Copies the radios' current RF settings back into the settings store.
 * Only touches the store's RAM copy; the sync task writes it out about a
 * second later (see setupSettings()).
 */
void saveRfSettings();

/**
 * @brief Antenna transmissions go out on: 0 is the first radio, 1 the next,
 * and so on; RFSettings::kBestSnrTxAntenna means whichever is receiving best,
 * RFSettings::kRoundRobinTxAntenna means cycling one antenna per packet.
 */
uint8_t getTxAntenna();

/**
 * @brief Pins transmissions to one antenna, or selects one of the two
 * algorithmic choices (RFSettings::kBestSnrTxAntenna /
 * RFSettings::kRoundRobinTxAntenna). Applies immediately and stores the
 * change.
 * @return true if the antenna exists (or an algorithmic mode was selected);
 * false leaves everything as it was.
 */
bool setTxAntenna(uint8_t antenna);

/**
 * @returns how many antennas (radios) this build can transmit on. 1 on a
 * single-radio build.
 */
uint8_t getTxAntennaCount();

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_RFSETUP_HPP
