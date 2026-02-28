#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <time.h>

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "Hardware.hpp"
#include "WifiUpload.hpp"

#include "ExVectrLink/serial/SerialTelecomPackets.hpp"
#include "ExVectrLink/serial/SerialTelecoms.hpp"

namespace VCTR::WifiUpload {

using namespace VCTR::ExVectrLink;

const char *ssid = "WIFICC";
const char *password = "R8765432109R";

SerialTelecoms *serialTelecomsPtr = nullptr;

class WiFiTimeSync : public Core::Task_Periodic {
private:
  uint32_t last_ota_time = 0;

  bool wifiStarted = false;

  bool uploadStuffInitialized = false;

  bool wifiConnected = false;

public:
  int64_t timeUpdateTimestamp = 0;

  WiFiTimeSync()
      : Core::Task_Periodic("WiFi Connection", Core::MILLISECONDS * 1000) {
    Core::getSystemScheduler().addTask(*this);
    setPaused(true);
  }

  bool isWiFiConnected() { return wifiConnected; }

  IPAddress getIPAddress() { return WiFi.localIP(); }

private:
  void taskInit() override {}

  void taskThread() override {
    if (WiFi.status() == WL_CONNECTED && !uploadStuffInitialized) {
      uploadStuffInitialized = true;
      wifiConnected = true;
      Serial.println("Connected to WiFi");
      if (!MDNS.begin("exvectr_link")) { // http://exvectr_link.local
        Serial.println("Error setting up MDNS responder!");
      }
      ArduinoOTA
          .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
              type = "sketch";
            } else { // U_SPIFFS
              type = "filesystem";
            }

            // NOTE: if updating SPIFFS this would be the place to unmount
            // SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
          })
          .onEnd([]() { Serial.println("\nEnd"); })
          .onProgress([this](unsigned int progress, unsigned int total) {
            if (millis() - last_ota_time > 500) {
              Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
              last_ota_time = millis();
            }
          })
          .onError([](ota_error_t error) {
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
      Serial.println("Connecting to WiFi");
    }
  }
} wifiTimeSync;

void startButtonWifiUploadMode() {
  Serial.println("Starting WiFi upload mode via button. All other tasks will "
                 "be stopped. Reset the device to exit.");
  wifiTimeSync.setPaused(false);
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
        wifiTimeSync.setPaused(false);
        serialTelecomsPtr->sendSerialPacket(packets::UpdateMode, 0);
      });
}

void loopWifiUpload() {

  static int64_t buttonPressStartTime = 0;
  static bool buttonPressed = false;

  if (wifiTimeSync.isWiFiConnected()) {
    ArduinoOTA.handle();
  } else if (wifiTimeSync.getPaused()) {
    if (digitalRead(HardwareConfig::button) == LOW &&
        wifiTimeSync.getPaused()) {
      if (!buttonPressed) {
        buttonPressed = true;
        buttonPressStartTime = millis();
      } else if (millis() - buttonPressStartTime > 200) {
        startButtonWifiUploadMode();
      }
    }

    if (millis() > 5000 &&
        (serialTelecomsPtr != nullptr && !serialTelecomsPtr->isConnected())) {
      wifiTimeSync.setPaused(false);
    }
  } else {
    buttonPressed = false;
  }
}

bool wifiUploadIsActive() { return !wifiTimeSync.getPaused(); }

void startWiFiUploadMode(bool forceOnly) {
  Serial.println("Starting WiFi upload mode... Reset the device to exit.");
  wifiTimeSync.setPaused(false);
  if (!forceOnly) {
    auto &tasks = Core::getSystemScheduler().getTasks();
    for (int i = 0; i < tasks.size(); i++) {
      Core::getSystemScheduler().removeTask(*tasks[i]);
    }
    Core::getSystemScheduler().addTask(wifiTimeSync);
  }
}

} // namespace VCTR::WifiUpload