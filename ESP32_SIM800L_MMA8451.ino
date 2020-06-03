/*
  LumaVibe V2.00
*/

// https://github.com/Xinyuan-LilyGO/TTGO-T-Call
// https://pubsubclient.knolleary.net/api.html for pubsubclient
// RTC_DATA_ATTR and RTC_RODATA_ATTR are placed in..
// ..the RTC fast memory segment otherwise it goes to RTC slow memory (default option)..
// .. see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/deep-sleep-stub.html

#include "esp_attr.h"
#include <WiFi.h>

#include "LumaVibe.h"
#include "debug_macros.h"
#include "ota_updater.h"

void printLocalTime(void);

void setup() { // takes 33ms
  uint32_t setupTimer = millis();
  LumaVibe_setPowerBoostKeepOn(true);
  LumaVibe_enableModem();
  SerialUSB.begin(115200);
  SerialUSB.print("\nMAC address: ");
  SerialUSB.println(WiFi.macAddress());
  SerialUSB.print("\nFirmware version: ");
  SerialUSB.println(FIRMWARE_VERSION);

  // This checks if the board wakes up from emergency-OTA
  if (g_isEmergency) {
    LumaVibe_detachAccelInterrupt();
    PRINTS("\nRe-entering emergency mode");
    ota_updater_begin();
  }

  // For pin defines, change them in LumaVibe_globals.h
  // https://www.reddit.com/r/FastLED/comments/e4w6xh/not_usable_in_a_constant_expression/
  LumaVibe_Settings_t settings = {
    /*accelSCL*/              ACCEL_SCL_PIN,
    /*accelSDA*/              ACCEL_SDA_PIN,
    /*accelAddress*/          MMA8451_DEFAULT_ADDRESS_A0_HIGH,
    /*accelInterruptPin*/     ACCEL_INTERRUPT_PIN,
    /*numberOfMeas*/          2048,
    /*measureFrequency*/      200,
    /*sleepTime_ms*/          60000*10,
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

  PRINTF("\nSetup took %lu ms", millis() - setupTimer);
  PRINTS("\nEnd of setup()\n");
}

//
void loop() { 
  if (1 == g_bootCount) {
    // delay(5000);
    if (g_accelInterruptFlag) {
      LumaVibe_clearAccelInterrupt();
      g_accelInterruptFlag = false;
      PRINTS("\nFirst boot");
      
      LumaVibe_goToSleep();
    }
  }
  
  if (g_accelInterruptFlag || g_timerInterruptFlag) {
    uint64_t start = millis();
    // PRINT("\naccelInterruptFlag: ", g_accelInterruptFlag);
    // PRINT("\ntimerInterruptFlag: ", g_timerInterruptFlag);

    LumaVibe_Error_t err;

    LumaVibe_setLED(CRGB::Purple);
    time_t timeAtMeasure;
    err = LumaVibe_measure(&timeAtMeasure);
    
    if (LUMAVIBE_ERROR_NONE != err)
      LumaVibe_LOG_ERROR(err);
    
    LumaVibe_setLED(CRGB::Aqua);
    uint32_t bytesPacked;
    err = LumaVibe_packData(timeAtMeasure, &bytesPacked);
    
    if (LUMAVIBE_ERROR_NONE != err)
      LumaVibe_LOG_ERROR(err);
    PRINT("\nPacked Bytes: ", bytesPacked);

    LumaVibe_setLED(CRGB::Green);
    err = LumaVibe_publishData("ngd/demo/HSRW_Hung/data", bytesPacked, 512);
    
    if (LUMAVIBE_ERROR_NONE != err)
      LumaVibe_LOG_ERROR(err);
    
    LumaVibe_getCommandsFromServer("ngd/demo/HSRW_Hung/command");
    

    /*
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_PUBLISH_BEGIN_FAIL;
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_SENSOR_INIT;
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_MODEM_GPRS_NOT_CONNECTED;
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_MODEM_GPRS_NOT_CONNECTED;
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_MODEM_GPRS_NOT_CONNECTED;
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_MODEM_GPRS_NOT_CONNECTED;
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_MODEM_GPRS_NOT_CONNECTED;
    LumaVibe__errorStream[LumaVibe__errorStreamWriter++] = (uint8_t)LumaVibe_ERROR_MODEM_GPRS_NOT_CONNECTED;
    */
    printLocalTime();

    PRINT("\nError Count: ", LumaVibe_countError());
    // Handle errors here
    if (0 != LumaVibe_countError()) {
      PRINTS("\nThere is error");
      if (LumaVibe_countNetworkError() >= MAX_NETWORK_ERROR_COUNT || LumaVibe_countError() >= MAX_ERROR_COUNT) {
        // Jump to emergency-OTA
        LumaVibe_endWatchDog();
        LumaVibe_disableModem();
        LumaVibe_setPowerBoostKeepOn(false);
        LumaVibe_setLED(CRGB::Red);
        ota_updater_begin();
      }
      else { // Publish list of (not so critical) errors
        uint32_t bytesPacked;
        LumaVibe_Error_t err;
        //bytesPacked = 0;
        PRINTS("\nPacking up error");
        err = LumaVibe_packError(&bytesPacked);
        if (LUMAVIBE_ERROR_NONE != err) {
          LumaVibe_LOG_ERROR(err);
        }
        err = LumaVibe_publishError("ngd/demo/HSRW_Hung/error", bytesPacked, bytesPacked);
        if (LUMAVIBE_ERROR_NONE != err) {
          LumaVibe_LOG_ERROR(err);
        } else {
          LumaVibe_clearError();
          PRINTS("\nErrors cleared");
        }
      }
    }
    SerialUSB.printf("\nTime elapsed: %llu sec", (millis() - start)/1000);

    LumaVibe_flashLED(CRGB::Green, 200, 3, false);
    LumaVibe_clearAccelInterrupt();
    g_accelInterruptFlag = false;
    g_timerInterruptFlag = false;
    LumaVibe_goToSleep();
  }
  // 
}

void printLocalTime(void) {
  struct tm now;
  getLocalTime(&now, 0); // This returns POSIX time
  SerialUSB.printf("\nSystem time: %04d-%02d-%02dT%02d:%02d:%02dZ", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, 
                                                       now.tm_hour, now.tm_min, now.tm_sec);
}
