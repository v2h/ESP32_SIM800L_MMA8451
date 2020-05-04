// https://rntlab.com/question/how-to-use-esp32-light-sleep/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_system.h"

#include "LumaVibe.h"

#define TIMEOUT_MINUTE_TO_MS (1 * 60 * 1000)
#define SLEEPTIME_MINUTE_TO_uS (2 * 60 * 1000000)

#define SSID "UPC2597763"
#define PASSWORD "PCJNSGCF"

bool RTC_DATA_ATTR g_isEmergency;
static uint8_t RTC_DATA_ATTR sleepCount = 0;
static bool isLedOn = false;

void ota_updater_begin() {
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      SerialUSB.println("Start updating " + type);
    })
    .onEnd([]() {
      SerialUSB.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      SerialUSB.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      SerialUSB.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) SerialUSB.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) SerialUSB.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) SerialUSB.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) SerialUSB.println("Receive Failed");
      else if (error == OTA_END_ERROR) SerialUSB.println("End Failed");
    });

  SerialUSB.println("\nBooting OTA");
  SerialUSB.printf("Timeout is set to %u sec\n", TIMEOUT_MINUTE_TO_MS / 1000);
  SerialUSB.printf("(Light) sleep time is set to %u sec\n", SLEEPTIME_MINUTE_TO_uS / 1000000);
  SerialUSB.printf("\nSleep count: %d\n", sleepCount);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  uint64_t currentTime_ms = millis();
  uint64_t ledTimer = millis();
  while (WiFi.waitForConnectResult() != WL_CONNECTED) { // This checks every 10s
    // Flash LED
    if (millis() - ledTimer > 400) {
      if (isLedOn) {
        isLedOn = false;
        g_Vibe.clearLED();
      }
      else {
        isLedOn = true;
        g_Vibe.setLED(CRGB::Red);
      }
      ledTimer = millis();
    }
    // Check for timeout -> sleep
    if (millis() - currentTime_ms > TIMEOUT_MINUTE_TO_MS) { // Timeout
      sleepCount++;
      SerialUSB.println("Connection Failed! Going to deep sleep...");
      if (btStop()) {
        SerialUSB.println("Bluetooth stopped");
      }
      if (WiFi.mode(WIFI_OFF)) {
        SerialUSB.println("Wifi off");
      }
      delay(1000); // (!?)
      g_Vibe.setLED(CRGB::Red);
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // (!?)
      esp_sleep_enable_timer_wakeup(SLEEPTIME_MINUTE_TO_uS);
      SerialUSB.println("Sleeping");
      delay(1000);
      SerialUSB.flush();
      g_isEmergency = true;
      esp_deep_sleep_start();      
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

  SerialUSB.println("Ready");
  SerialUSB.print("IP address: ");
  SerialUSB.println(WiFi.localIP());

  // Only reach here when connection has been established
  while (1) {
    ArduinoOTA.handle();
  }
}
