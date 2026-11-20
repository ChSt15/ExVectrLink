#pragma once
// SourcePort: latest value of one subscribed port, accepted from one source node only.
#include <stdint.h>

#include "ExVectrNetwork.hpp"

namespace VCTR::ExVectrLink::netmode {

/// Latest value of one subscribed port, accepted from one source node only. Plain port handler: copies the
/// decoded packet and does no control work (the library's receive-callback rule).
template <typename T>
class SourcePort : public network::PortHandlerI {
public:
  SourcePort(network::PortRegistryI &mux, uint8_t port, uint8_t acceptedSrc)
      : src_(acceptedSrc) {
    subscribed_ = mux.subscribe(port, *this);
  }
  bool subscribed() const { return subscribed_; }
  bool hasValue() const { return has_; }
  const T &value() const { return value_; }
  int64_t rxNs() const { return rxNs_; }
  uint32_t rejectedSource() const { return rejectedSrc_; }
  uint32_t decodeErrors() const { return decodeErrors_; }

  void onPacket(const network::RxInfo &info, const uint8_t *payload, uint8_t len) override {
    if (info.src != src_) {
      rejectedSrc_++;
      return;
    }
    T tmp;
    if (!tmp.decode(payload, len)) {
      decodeErrors_++;
      return;
    }
    value_ = tmp;
    rxNs_ = info.rxTimeNs;
    has_ = true;
  }

private:
  uint8_t src_;
  bool subscribed_ = false;
  bool has_ = false;
  T value_{};
  int64_t rxNs_ = 0;
  uint32_t rejectedSrc_ = 0, decodeErrors_ = 0;
};

} // namespace VCTR::ExVectrLink::netmode
