/*
  LumaVibe V2.00
*/

// https://github.com/Xinyuan-LilyGO/TTGO-T-Call
// https://pubsubclient.knolleary.net/api.html for pubsubclient
// RTC_DATA_ATTR and RTC_RODATA_ATTR are placed in..
// ..the RTC fast memory segment otherwise it goes to RTC slow memory (default option)..
// .. see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/deep-sleep-stub.html
// .. On printf() and \n: https://esp32.com/viewtopic.php?t=3447

#include "esp_attr.h"
#include <freertos/portmacro.h>
#include <WiFi.h>

#include "LumaVibe.h"
#include "debug_macros.h"
#include "ota_updater.h"
#include "helper_macros.h"
#include "esp_task_wdt.h"

xTaskHandle mainTaskHandle = NULL;
xTaskHandle setupTaskHandle = NULL;

void printLocalTime(void);

void setup() {
  xTaskCreatePinnedToCore(LumaVibe_setup, str(LumaVibe_setup), 20000, NULL, 1, &setupTaskHandle, 1);
  vTaskDelete(NULL);
}

//
void loop() {
  PRINTS("Loop\r\n");
  vTaskDelete(NULL);
}

void LumaVibe_setup(void *param) {
  esp_task_wdt_delete(&setupTaskHandle);
  uint32_t setupTimer = millis();
  SerialUSB.begin(115200);
  delay(2000);
  SerialUSB.println("MAC address: " + WiFi.macAddress());
  SerialUSB.println("Firmware version: " + String(FIRMWARE_VERSION));
  Serial.print("Setup: priority = ");
  Serial.println(uxTaskPriorityGet(NULL));

  // For pin defines, change them in LumaVibe_globals.h
  // https://www.reddit.com/r/FastLED/comments/e4w6xh/not_usable_in_a_constant_expression/
  LumaVibe_Settings_t settings = {
    /*accelSCL*/              ACCEL_SCL_PIN,
    /*accelSDA*/              ACCEL_SDA_PIN,
    /*accelAddress*/          MMA8451_DEFAULT_ADDRESS_A0_HIGH,
    /*accelInterruptPin*/     ACCEL_INTERRUPT_PIN,
    /*numberOfMeas*/          2048,
    /*measureFrequency*/      200,
    /*sleepTime_ms*/          60000,
    /*watchDogTime_ms*/       60000,  
    /*mqttBroker[20]*/        "mastap.net",
    /*mqttUserName[10]*/      "user",
    /*mqttPassword[10]*/      "mqtt",
    /*firmwareVersion[20]*/   FIRMWARE_VERSION,
    /*moduleID[13]*/          "SV010_000000",
    /*moduleType[16]*/        "Solar Vibe 1.1",
    /*msgType*/               2,
    /*format[5]*/             "int16"
  };

  LumaVibe_Error_t err; 
  err = LumaVibe_init(&settings);
  if (LUMAVIBE_ERROR_NONE != err)
    LumaVibe_LOG_ERROR(err);
  LumaVibe_setLED(CRGB::Yellow);
  
  err = LumaVibe_begin();
  if (LUMAVIBE_ERROR_NONE != err)
    LumaVibe_LOG_ERROR(err);

  PRINTF("Setup took %lu ms\r\n", millis() - setupTimer);
  PRINTS("End of setup()\r\n");
  xTaskCreatePinnedToCore(LumaVibe_main, str(LumaVibe_main), 20000, NULL, 1, &mainTaskHandle, 1);
  vTaskDelete(NULL);
}

void LumaVibe_main(void *param) {
  esp_task_wdt_delete(&mainTaskHandle);
  for (;;) {
    // This checks if the board wakes up from emergency-OTA
    if (g_isEmergency) {
      LumaVibe_detachAccelInterrupt();
      PRINTS("Re-entering emergency mode\r\n");
      ota_updater_begin((gpio_num_t)(ACCEL_INTERRUPT_PIN));
    }
    if (0 == g_bootCount) {
      LumaVibe_goToSleep();
    }

    uint64_t start = millis();
    LumaVibe_Error_t err;

    LumaVibe_setLED(CRGB::Purple);
    time_t timeAtMeasure;
    err = LumaVibe_measure(&timeAtMeasure);
    
    if (LUMAVIBE_ERROR_NONE != err)
      LumaVibe_LOG_ERROR(err);

    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
    if (wakeCause == ESP_SLEEP_WAKEUP_GPIO) g_accelInterruptFlag = true;
    PRINTF("Accel Flag: %d\r\n", g_accelInterruptFlag);
    
    LumaVibe_setLED(CRGB::Aqua);
    uint32_t bytesPacked;
    err = LumaVibe_packData(timeAtMeasure, &bytesPacked);
    
    if (LUMAVIBE_ERROR_NONE != err)
      LumaVibe_LOG_ERROR(err);
    PRINTLN("Packed Bytes: ", bytesPacked);

    LumaVibe_setLED(CRGB::Green);
    err = LumaVibe_publishData("ngd/demo/HSRW_Hung/data", bytesPacked, 512);
    
    if (LUMAVIBE_ERROR_NONE != err)
      LumaVibe_LOG_ERROR(err);
    
    LumaVibe_getCommandsFromServer("ngd/demo/HSRW_Hung/command");
    
    printLocalTime();

    PRINTF("Error Count: %d\n", LumaVibe_countError());
    // Handle errors here


    if (0 != LumaVibe_countError()) {
      PRINTS("There is error\r\n");
      if (LumaVibe_countNetworkError() >= MAX_NETWORK_ERROR_COUNT || LumaVibe_countError() >= MAX_ERROR_COUNT) {
        // Jump to emergency-OTA
        // LumaVibe_endWatchDog();
        ota_updater_begin((gpio_num_t)(ACCEL_INTERRUPT_PIN));
      }
      else { // Publish list of (not so critical) errors
        uint32_t bytesPacked;
        LumaVibe_Error_t err;
        //bytesPacked = 0;
        PRINTS("Packing up error\r\n");
        err = LumaVibe_packError(&bytesPacked);
        if (LUMAVIBE_ERROR_NONE != err) {
          LumaVibe_LOG_ERROR(err);
        }
        err = LumaVibe_publishError("ngd/demo/HSRW_Hung/error", bytesPacked, bytesPacked);
        if (LUMAVIBE_ERROR_NONE != err) {
          LumaVibe_LOG_ERROR(err);
        } else {
          LumaVibe_clearError();
          PRINTS("Errors cleared\r\n");
        }
      }
    }
    printf("Time elapsed: %llu sec\r\n", (millis() - start)/1000);
    LumaVibe_flashLED(CRGB::Green, 200, 3, false);
    LumaVibe_clearAccelInterrupt();
    portENTER_CRITICAL(&g_mux);
    g_accelInterruptFlag = false;
    portEXIT_CRITICAL(&g_mux);
    LumaVibe_goToSleep();
  }
}

void printLocalTime(void) {
  struct tm now;
  getLocalTime(&now, 0); // This returns POSIX time
  printf("System time: %04d-%02d-%02dT%02d:%02d:%02dZ\r\n", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, 
                                                       now.tm_hour, now.tm_min, now.tm_sec);
}
