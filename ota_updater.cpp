// https://rntlab.com/question/how-to-use-esp32-light-sleep/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_system.h"

#include "LumaVibe.h"

#define TIMEOUT_MINUTE_TO_MS (1 * 60 * 1000)
#define SLEEPTIME_MINUTE_TO_uS (2 * 60 * 1000000)

#if (TIMEOUT_MINUTE_TO_MS >= SLEEPTIME_MINUTE_TO_uS)
#error "timeout must be less than wakeup time"
#endif

static const char* ssid = "UPC2597763";
static const char* password = "PCJNSGCF";
static const char* fakePassword = "shitshit";
static uint8_t sleepCount = 0;
static bool isLedOff = false;

void ota_updater_begin() {
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  Serial.println("\nBooting OTA");
  Serial.printf("Timeout is set to %u sec\n", TIMEOUT_MINUTE_TO_MS / 1000);
  Serial.printf("(Light) sleep time is set to %u sec\n", SLEEPTIME_MINUTE_TO_uS / 1000000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, fakePassword);
  uint64_t currentTime_ms = millis();
  while (WiFi.waitForConnectResult() != WL_CONNECTED) { // This checks every 10s
    Serial.println("checking wifi status");
    if (millis() - currentTime_ms > TIMEOUT_MINUTE_TO_MS / 2) { // Timeout
      Serial.println("Connection Failed! Going to light sleep...");
      if (btStop()) {
        Serial.println("Bluetooth stopped");
      }
      if (WiFi.mode(WIFI_OFF)) {
        Serial.println("Wifi off");
      }
      delay(1000); // (!?)
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // (!?)
      if (ESP_OK != esp_sleep_enable_timer_wakeup(30 * 1000000)) {
        Serial.println("Cannot enable sleep timer");
      }
      else {
        Serial.println("Sleeping");
        delay(1000);
        Serial.flush();
        Serial.end();
        esp_light_sleep_start();
      }      
      Serial.begin(115200);
      delay(1000);
      g_Vibe.setLED(CRGB::Red);
      Serial.println("Out of light sleep");
      sleepCount++;
      Serial.print("sleep count: "); Serial.println(sleepCount);
      if (3 == sleepCount) {
        Serial.println("restarting");
        esp_restart();
      }
      WiFi.mode(WIFI_STA);
      if (2 == sleepCount) {
        WiFi.begin(ssid, password);
      }
      else {
        WiFi.begin("UPC2597763", fakePassword);
      }
      currentTime_ms = millis();
    }
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  g_Vibe.setLED(CRGB::Green);

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Only reach here when connection has been established
  while (1) {
    ArduinoOTA.handle();
    // Nothing should be down here..
    // ..else light sleep won't work (?!)
  }
}
