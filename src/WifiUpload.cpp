#include <Arduino.h>
#if defined(ESP32)
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#elif defined(ESP8266)
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#endif
#include <time.h>

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "Hardware.hpp"
#include "WifiUpload.hpp"

#include "ExVectrLink/serial/SerialTelecomPackets.hpp"
#include "ExVectrLink/serial/SerialTelecoms.hpp"

namespace VCTR::WifiUpload {
extern const char *ssid;
extern const char *password;
} // namespace VCTR::WifiUpload

namespace VCTR::WifiUpload {

using namespace VCTR::ExVectrLink;

SerialTelecoms *serialTelecomsPtr = nullptr;

class WiFiTimeSync : public Core::Task_Periodic {
private:
  uint32_t last_ota_time = 0;

  bool wifiStarted = false;

  bool uploadStuffInitialized = false;

  bool wifiConnected = false;

  bool paused = true;

public:
  int64_t timeUpdateTimestamp = 0;

  WiFiTimeSync()
      : Core::Task_Periodic("WiFi Connection", Core::MILLISECONDS * 1000) {
    Core::getSystemScheduler().addTask(*this);
  }

  bool isWiFiConnected() { return wifiConnected; }

  bool isUploadPaused() { return paused; }

  IPAddress getIPAddress() { return WiFi.localIP(); }

  void startUploadMode() { paused = false; }
  void taskInit() override {}

  void taskThread() override {

    if (paused) {
      return;
    }

    if (WiFi.status() == WL_CONNECTED && !uploadStuffInitialized) {
      uploadStuffInitialized = true;
      wifiConnected = true;
      // Serial.println("Connected to WiFi");
      if (!MDNS.begin("exvectr_link")) { // http://exvectr_link.local
        // Serial.println("Error setting up MDNS responder!");
      }
      ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
          type = "sketch";
        } else { // U_SPIFFS
          type = "filesystem";
        }

        // NOTE: if updating SPIFFS this would be the place to unmount
        // SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      });
      ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
      ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
        if (millis() - last_ota_time > 500) {
          Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
          last_ota_time = millis();
        }
      });
      ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
          Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
          Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
          Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
          Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
          Serial.println("End Failed");
        }
      });

      ArduinoOTA.begin();
    } else if (WiFi.status() != WL_CONNECTED && !wifiStarted) {
      wifiStarted = true;
      WiFi.begin(ssid, password);
      // Serial.println("Connecting to WiFi");
    }
  }
} wifiTimeSync;

void startButtonWifiUploadMode() {
  // Serial.println("Starting WiFi upload mode via button. All other tasks will
  // "
  //                "be stopped. Reset the device to exit.");
  wifiTimeSync.startUploadMode();
  auto &tasks = Core::getSystemScheduler().getTasks();
  for (int i = 0; i < tasks.size(); i++) {
    Core::getSystemScheduler().removeTask(*tasks[i]);
  }
  Core::getSystemScheduler().addTask(wifiTimeSync);
}

void initWifiUpload(VCTR::ExVectrLink::SerialTelecoms &serialTelecoms) {

  pinMode(HardwareConfig::button, INPUT_PULLUP);

  serialTelecomsPtr = &serialTelecoms;
  serialTelecomsPtr->addSerialPacketHandler(
      packets::UpdateMode, [](const Core::ListArray<uint8_t> &data) {
        wifiTimeSync.startUploadMode();
        serialTelecomsPtr->sendSerialPacket(packets::UpdateMode, 0);
      });

#ifdef ENABLE_FORCEWIFI
  wifiTimeSync.startUploadMode();
#endif
}

void loopWifiUpload() {

  static int64_t buttonPressStartTime = 0;
  static bool buttonPressed = false;

  if (wifiTimeSync.isWiFiConnected()) {
    ArduinoOTA.handle();
  } else if (wifiTimeSync.isUploadPaused()) {
    if (digitalRead(HardwareConfig::button) == LOW) {
      if (!buttonPressed) {
        buttonPressed = true;
        buttonPressStartTime = millis();
      } else if (millis() - buttonPressStartTime > 3000) {
        wifiTimeSync.startUploadMode();
      }
    } else {
      buttonPressed = false;
    }

#if !defined(ENABLE_SBUS) && !defined(ENABLE_CRSF)
    if (millis() > 10000 &&
        (serialTelecomsPtr != nullptr && !serialTelecomsPtr->isConnected())) {
      wifiTimeSync.startUploadMode();
    }
#endif
  }
}

bool wifiUploadIsActive() { return !wifiTimeSync.isUploadPaused(); }

void startWiFiUploadMode(bool forceOnly) {
  // Serial.println("Starting WiFi upload mode... Reset the device to exit.");
  wifiTimeSync.startUploadMode();
  if (!forceOnly) {
    auto &tasks = Core::getSystemScheduler().getTasks();
    for (int i = 0; i < tasks.size(); i++) {
      if (tasks[i] != &wifiTimeSync) {
        Core::getSystemScheduler().removeTask(*tasks[i]);
      }
    }
  }
}

} // namespace VCTR::WifiUpload