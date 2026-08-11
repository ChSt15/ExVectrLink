#ifndef EXVECRLINK_SETTINGS_HPP
#define EXVECRLINK_SETTINGS_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <tuple>

#include "ExVectrData/memory_interface.hpp"
#include "ExVectrData/memory_internal.hpp"
#include "ExVectrData/memory_manager.hpp"

namespace VCTR::ExVectrLink {

// ---------------------------------------------------------------------------
// The settings themselves.
//
// One struct per group of related settings. Give each member its default value
// right here -- that is what a device that has never stored anything comes up
// with -- and each struct a storage key that is never reused.
//
// To add a group: write the struct, give it a fresh kStorageKey, and add it to
// Settings::Records. Nothing else needs touching; loading, allocating and
// saving are generic over the list.
// ---------------------------------------------------------------------------

/// @brief All radio settings: transmit power and antenna choice.
struct RFSettings {
  /// @brief Key this struct is stored under. Must be non-zero and unique among
  /// the records (both are checked at compile time).
  /// @warning Adding, removing or resizing a member changes the stored size,
  /// and an item of the old size is then ignored but stays behind under the
  /// same key. Give the struct a NEW key whenever its layout changes -- the
  /// static_assert below exists to make that decision deliberate.
  /// @note 0x0101 was this struct without txAntenna, back when it was called
  /// PowerSettings. Devices carrying that item come up at the defaults below.
  static constexpr uint16_t kStorageKey = 0x0102;

  /// @brief txAntenna value that leaves the choice to the diversity algorithm,
  /// i.e. whichever antenna is currently receiving best.
  static constexpr uint8_t kDiversityTxAntenna = UINT8_MAX;

  /// @brief Whether the dynamic power algorithm runs at all. See
  /// datalink::DynamicPower::setEnableDynamicPower().
  bool dynamicPowerEnabled = true;
  /// @brief Lowest power (dBm) the dynamic power algorithm may use.
  uint8_t dynMinPowerDbm = 10;
  /// @brief Highest power (dBm) the dynamic power algorithm may use, and the
  /// fixed operating power while dynamic power is disabled.
  uint8_t dynMaxPowerDbm = 20;
  /// @brief Antenna to transmit on: 0 is the first radio, 1 the next, and so
  /// on; kDiversityTxAntenna (the default) lets diversity pick per slot. An
  /// index no radio exists for falls back to diversity too -- see
  /// datalink::applyRfSettings().
  uint8_t txAntenna = kDiversityTxAntenna;
};
static_assert(sizeof(RFSettings) == 4,
              "RFSettings changed size -- give it a new kStorageKey so the "
              "old, differently sized item is not left behind under that one");

// ---------------------------------------------------------------------------
// The store.
// ---------------------------------------------------------------------------

/**
 * @brief Size of the settings image, in RAM and in the backing memory alike.
 *
 * The Memory_Manager needs 20 bytes of its own header, plus 10 bytes per
 * record on top of the record itself, and it only ever appends. Growing this
 * later is harmless (stored records keep their positions); shrinking it below
 * what is in use is not.
 */
constexpr size_t kSettingsMemorySize = 128;

/**
 * @brief The firmware's settings: every settings struct, held live in RAM and
 * kept in the EEPROM across resets.
 *
 * Reading and writing a setting is a plain struct access by type:
 *
 *     auto rf = settings.get<RFSettings>();  // a copy to edit
 *     rf.dynMaxPowerDbm = 27;
 *     settings.set(rf);                      // marks it to be saved
 *
 * Persistence is one call, sync():
 *
 *  - the first sync() LOADS: it pulls the stored image in, reads every record
 *    by key and allocates the ones that are not there yet from their defaults
 *    (a blank device, or a firmware that just gained a record), then stores
 *    whatever it had to create;
 *  - every later sync() SAVES: the records are assumed loaded, so it only
 *    writes them back out, and only when something actually changed.
 *
 * setupSettings() does the first one at startup; a task takes care of the rest
 * shortly after each change. Everything below sync() -- which item lives
 * where, how records are split into bytes -- is the Data::Memory_Manager's
 * job, over a RAM image that is transferred to the backing memory as a whole.
 *
 * @note Records are stored in the machine's native layout and byte order
 * (little endian on every target this firmware builds for). The image never
 * leaves the device, so that is not a portability problem.
 */
class Settings {
public:
  /// @brief Every settings struct the firmware has. Add new ones here.
  using Records = std::tuple<RFSettings>;

  /**
   * @brief Flushes the backing memory so what was transferred into it survives
   * a reset. Returns true on success.
   *
   * A callback because flushing is specific to the memory in use (EEPROM
   * emulations need an explicit commit; memories that write straight through
   * need nothing at all), which keeps this class free of platform details.
   */
  using StorageFlushFn = bool (*)();

  /**
   * @param storage Backing memory holding the settings image between resets.
   * Must be at least kSettingsMemorySize bytes.
   * @param flush Called after the image is transferred into `storage`, to push
   * it out to the actual storage. nullptr for memories that need no flush.
   */
  Settings(VCTR::Data::Memory_Interface &storage, StorageFlushFn flush);

  /// @brief The live values of one settings struct. Cheap -- a direct member
  /// access, no lookup. Holds the defaults until the first sync() loads them.
  template <typename RECORD> const RECORD &get() const {
    return std::get<RECORD>(records_);
  }

  /**
   * @brief Replaces one settings struct. Marks the settings to be saved when
   * the new values differ from the current ones; nothing reaches the backing
   * memory until the next sync().
   */
  template <typename RECORD> void set(const RECORD &record) {
    RECORD &stored = std::get<RECORD>(records_);
    if (memcmp(&stored, &record, sizeof(RECORD)) == 0) {
      return; // Unchanged -- nothing to save later.
    }
    stored = record;
    markChanged();
  }

  /**
   * @brief Loads every settings struct on the first call, saves the changed
   * ones on every call after that. See the class documentation.
   * @note The saving half writes to flash, which blocks for some milliseconds
   * on flash backed memories -- prefer syncIfDirty() from a periodic task over
   * calling this straight from a packet handler.
   * @return true on success, false if the image could not be read, a record
   * could not be allocated, or the storage could not be written.
   */
  bool sync();

  /**
   * @brief sync(), but only when something changed and it has been unchanged
   * for long enough. Lets bursts of changes (a user scrolling through a menu)
   * collapse into a single write.
   * @param settleTimeNs How long the settings must have been unchanged before
   * they are saved. 0 saves immediately.
   * @return true if a sync happened and succeeded.
   */
  bool syncIfDirty(int64_t settleTimeNs = 0);

  /// @brief Whether a setting changed since the last sync().
  bool isDirty() const { return dirty_; }

  /// @brief Whether the settings have been loaded from the backing memory,
  /// i.e. whether sync() has run at least once.
  bool isLoaded() const { return loaded_; }

  /// @brief Puts every setting back to its built-in default. Marks them to be
  /// saved -- sync() to make it stick.
  void resetToDefaults();

  /**
   * @brief Wipes the stored image and re-creates every record at its default
   * value, i.e. a factory reset. Writes to the backing memory immediately.
   * @return true if the image was re-initialised and stored.
   */
  bool clearStorage();

private:
  /// @brief First sync(): pull the stored image in, then complete the load.
  bool load();
  /// @brief Reads every record from the image, allocating the missing ones,
  /// and stores the image again if anything had to be created. Shared by
  /// load() and clearStorage(), which starts from an empty image instead of a
  /// stored one.
  bool completeLoad();
  /// @brief Later syncs: write the records back out.
  bool save();

  /// @brief Reads one record from the image, creating its item from the
  /// record's current (default) values when it does not exist yet.
  template <typename RECORD> bool loadRecord(RECORD &record);
  /// @brief Writes one record into the image, creating its item if it went
  /// missing.
  template <typename RECORD> bool storeRecord(const RECORD &record);

  /// @brief Transfers the RAM image into the backing memory and flushes it.
  bool writeStorage();

  /// @brief Notes that a setting changed and when, for syncIfDirty()'s settle
  /// time.
  void markChanged();

  /// @brief The image of the whole store -- manager header and items. Only
  /// touched by sync(), never by a get()/set().
  VCTR::Data::Memory_Internal<kSettingsMemorySize> ram_;
  /// @brief Keyed access to the records inside `ram_`.
  VCTR::Data::Memory_Manager manager_;

  VCTR::Data::Memory_Interface &storage_;
  StorageFlushFn flush_;

  /// @brief The live settings. This is what get()/set() work on.
  Records records_;

  bool loaded_ = false;
  bool dirty_ = false;
  int64_t lastChangeNs_ = 0;
};

/// @brief Whether every record's storage key is non-zero (key 0 means "no key"
/// to the Memory_Manager) and unique. Checked at compile time below.
template <typename... RECORDS> constexpr bool settingsKeysUsable() {
  constexpr size_t count = sizeof...(RECORDS);
  const uint16_t keys[] = {RECORDS::kStorageKey...};
  for (size_t i = 0; i < count; i++) {
    if (keys[i] == 0) {
      return false;
    }
    for (size_t j = i + 1; j < count; j++) {
      if (keys[i] == keys[j]) {
        return false;
      }
    }
  }
  return true;
}
template <typename RECORDS> struct SettingsKeysUsable;
template <typename... RECORDS>
struct SettingsKeysUsable<std::tuple<RECORDS...>> {
  static constexpr bool value = settingsKeysUsable<RECORDS...>();
};
static_assert(SettingsKeysUsable<Settings::Records>::value,
              "every settings struct needs a non-zero, unique kStorageKey");

/**
 * @brief The firmware's settings, backed by the on-chip EEPROM.
 * @note Only holds what is stored on the device after setupSettings(); before
 * that every setting still has its compiled-in default.
 */
Settings &getSettings();

/**
 * @brief Brings up the EEPROM, loads the settings (the first getSettings().
 * sync()) and starts the task that saves them again shortly after they change.
 * Safe to call twice.
 */
void setupSettings();

} // namespace VCTR::ExVectrLink
#endif // EXVECRLINK_SETTINGS_HPP
