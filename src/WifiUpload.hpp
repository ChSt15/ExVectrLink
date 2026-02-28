#ifndef EXVECRLINK_WIFIUPLOAD_HPP
#define EXVECRLINK_WIFIUPLOAD_HPP

#include "ExVectrLink/serial/SerialTelecoms.hpp"

namespace VCTR::WifiUpload {

void initWifiUpload(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms);

void loopWifiUpload();

bool wifiUploadIsActive();

/**
 * @brief Starts WiFi upload mode. All other tasks will be stopped. Reset the
 * device to exit.
 * @param forceOnly If true, then all other tasks will not be stopped.
 */
void startWiFiUploadMode(bool forceOnly = false);

} // namespace VCTR::WifiUpload

#endif // EXVECRLINK_WIFIUPLOAD_HPP