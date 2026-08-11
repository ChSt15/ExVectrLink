#include "Settings.hpp"

#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#if defined(ESP32) || defined(ESP8266)
#include <EEPROM.h>

#include "ExVectrArduinoPlatform/memory_eeprom.hpp"
#endif

namespace VCTR::ExVectrLink {

namespace {

/// @brief How long the settings have to sit unchanged before the sync task
/// saves them. Long enough that scrolling a handset menu produces one flash
/// write instead of one per step, short enough that a user who changes
/// something and pulls the plug keeps the change.
constexpr int64_t kSyncSettleTimeNs = 1 * Core::SECONDS;

#if defined(ESP32) || defined(ESP8266)

/// @brief The on-chip EEPROM (flash-emulated) the settings image is kept in.
VCTR::Data::Memory_ArduinoEEPROM &settingsStorage() {
  static VCTR::Data::Memory_ArduinoEEPROM storage(EEPROM, kSettingsMemorySize);
  return storage;
}

/// @brief Brings up the EEPROM emulation, which only allocates its RAM mirror
/// here. Must happen before the storage is read or written.
void beginSettingsStorage() { EEPROM.begin(kSettingsMemorySize); }

/// @brief Pushes the emulation's RAM mirror out to flash. Nothing written to
/// the storage survives a reset until this runs.
bool flushSettingsStorage() { return EEPROM.commit(); }

#else

/// @brief No EEPROM on this target: keep the image in RAM so everything still
/// works, it just does not survive a reset.
VCTR::Data::Memory_Internal<kSettingsMemorySize> &settingsStorage() {
  static VCTR::Data::Memory_Internal<kSettingsMemorySize> storage;
  return storage;
}

void beginSettingsStorage() {}

bool flushSettingsStorage() { return true; }

#endif

/**
 * @brief Saves the settings shortly after they change.
 * @note Deliberately not done from the packet handlers that change settings:
 * the actual flash write blocks for a few milliseconds, which is better spent
 * in a low-rate task than in the middle of handling a CRSF or OTA packet.
 */
class SettingsSyncTask : public Core::Task_Periodic {
public:
  SettingsSyncTask() : Task_Periodic("SettingsSync", 250 * Core::MILLISECONDS) {
    Core::getSystemScheduler().addTask(*this);
  }

  void taskInit() override {}

  void taskThread() override { getSettings().syncIfDirty(kSyncSettleTimeNs); }
};

} // namespace

Settings::Settings(VCTR::Data::Memory_Interface &storage, StorageFlushFn flush)
    : manager_(ram_), storage_(storage), flush_(flush) {}

void Settings::markChanged() {
  dirty_ = true;
  lastChangeNs_ = Core::NowNs();
}

template <typename RECORD> bool Settings::loadRecord(RECORD &record) {
  if (manager_.readItem(record, RECORD::kStorageKey)) {
    return true;
  }

  // No item under this key yet: a blank device, or a firmware that just gained
  // this record. Create it holding the current (default) values.
  manager_.allocateItem(record, RECORD::kStorageKey);
  // allocateItem() hands the key straight back whether or not the allocation
  // actually succeeded, so read the item to find out whether it is really
  // there.
  if (!manager_.readItem(record, RECORD::kStorageKey)) {
    return false;
  }

  markChanged(); // Newly created -- the image needs storing.
  return true;
}

template <typename RECORD> bool Settings::storeRecord(const RECORD &record) {
  if (manager_.writeItem(record, RECORD::kStorageKey)) {
    return true;
  }

  // The item is not there (never allocated, or the image was cleared behind
  // our back). Create it and write again rather than dropping the change.
  manager_.allocateItem(record, RECORD::kStorageKey);
  return manager_.writeItem(record, RECORD::kStorageKey);
}

bool Settings::writeStorage() {
  if (storage_.transferFrom(ram_, kSettingsMemorySize) !=
      kSettingsMemorySize) {
    return false;
  }
  if (flush_ != nullptr && !flush_()) {
    return false;
  }
  dirty_ = false;
  return true;
}

bool Settings::load() {
  // Pull the stored image into the working memory. Everything below runs on
  // this RAM copy; the records themselves then live in plain members.
  ram_.transferFrom(storage_, kSettingsMemorySize);

  // An image the manager cannot read (blank device) or one written by an
  // incompatible manager version has to be initialised before any record can
  // be found or allocated. clearMemory() is what does that -- it drops
  // whatever was there, which for a version mismatch is unavoidable.
  if (manager_.getMemoryVersion() != manager_.getManagerVersion()) {
    manager_.clearMemory();
    markChanged();
  }

  return completeLoad();
}

bool Settings::completeLoad() {
  bool allOk = true;
  std::apply(
      [&](auto &...record) { ((allOk = loadRecord(record) && allOk), ...); },
      records_);

  loaded_ = true;

  // Store whatever was just initialised or allocated. Free when nothing was:
  // the transfer only writes bytes that differ, and the EEPROM flush is a
  // no-op when its mirror is unchanged.
  if (dirty_ && !writeStorage()) {
    allOk = false;
  }
  return allOk;
}

bool Settings::save() {
  if (!dirty_) {
    return true; // Nothing changed -- do not touch the storage at all.
  }

  bool allOk = true;
  std::apply(
      [&](const auto &...record) {
        ((allOk = storeRecord(record) && allOk), ...);
      },
      records_);

  return writeStorage() && allOk;
}

bool Settings::sync() { return loaded_ ? save() : load(); }

bool Settings::syncIfDirty(int64_t settleTimeNs) {
  if (!dirty_) {
    return false;
  }
  if (settleTimeNs > 0 && (Core::NowNs() - lastChangeNs_) < settleTimeNs) {
    return false;
  }
  return sync();
}

void Settings::resetToDefaults() {
  records_ = Records{};
  markChanged();
}

bool Settings::clearStorage() {
  // Start from an empty image and the defaults, then let the load path
  // re-create every item. Deliberately not routed through sync()/load(): those
  // begin by pulling the stored image back in, which would undo the wipe.
  manager_.clearMemory();
  records_ = Records{};
  markChanged();
  return completeLoad();
}

Settings &getSettings() {
  static Settings settings(settingsStorage(), &flushSettingsStorage);
  return settings;
}

void setupSettings() {
  static bool setupDone = false;
  if (setupDone) {
    return;
  }
  setupDone = true;

  beginSettingsStorage();
  getSettings().sync(); // First sync: loads everything, allocating what is new.

  static SettingsSyncTask syncTask;
}

} // namespace VCTR::ExVectrLink
