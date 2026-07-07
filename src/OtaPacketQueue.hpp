#ifndef EXVECRLINK_OTAPACKETQUEUE_HPP
#define EXVECRLINK_OTAPACKETQUEUE_HPP

#include <cstddef>
#include <cstdint>

#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrNetwork/DataPacket.hpp"

namespace VCTR::ExVectrLink {

/**
 * @brief A small outgoing-packet queue that coalesces entries by a
 * caller-chosen key, so a handful of independent senders (RC channels, link
 * stats, telemetry, individually-sent pieces of one logically larger
 * packet, ...) can share a single-slot transport like FHSS::
 * transmitDataframe() without racing to overwrite each other's pending
 * frame.
 *
 * updateQueue() is the only way in: if `key` is already queued, its packet
 * is replaced in place (same position in the queue) so a fast-changing
 * source is always sent with fresh data instead of falling behind or
 * flooding the queue with stale duplicates of itself; a key not already
 * present is appended to the back. dequeue() pops in FIFO order, so every
 * distinct key gets a fair, round-robin-ish turn at the shared channel.
 *
 * @tparam CAPACITY Maximum number of distinct keys queued at once.
 */
template <size_t CAPACITY> class OtaPacketQueue {
public:
  /// Queues `packet` under `key`: if `key` is already queued (see
  /// contains()), its packet is overwritten in place; otherwise a new entry
  /// is appended to the back.
  /// @return false only if `key` is new and the queue is already full.
  bool updateQueue(uint8_t key, const network::DataPacket &packet) {
    for (size_t i = 0; i < entries_.size(); i++) {
      if (entries_[i].key == key) {
        entries_[i].packet = packet;
        return true;
      }
    }
    return entries_.placeBack(Entry{key, packet});
  }

  /// Removes and returns the oldest queued entry.
  /// @return false if the queue was empty (`key`/`packet` left untouched).
  bool dequeue(uint8_t &key, network::DataPacket &packet) {
    Entry entry;
    if (!entries_.takeFront(entry)) {
      return false;
    }
    key = entry.key;
    packet = entry.packet;
    return true;
  }

  /// @return true if a packet tagged `key` is currently queued.
  bool contains(uint8_t key) const {
    for (size_t i = 0; i < entries_.size(); i++) {
      if (entries_[i].key == key) {
        return true;
      }
    }
    return false;
  }

  size_t size() const { return entries_.size(); }
  bool isEmpty() const { return entries_.size() == 0; }
  bool isFull() const { return entries_.size() >= CAPACITY; }

private:
  struct Entry {
    uint8_t key = 0;
    network::DataPacket packet;
  };

  Core::ListBuffer<Entry, CAPACITY> entries_;
};

} // namespace VCTR::ExVectrLink

#endif // EXVECRLINK_OTAPACKETQUEUE_HPP
