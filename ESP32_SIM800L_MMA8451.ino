/*
  LumaVibe V2.00
*/

// https://github.com/Xinyuan-LilyGO/TTGO-T-Call
// https://pubsubclient.knolleary.net/api.html for pubsubclient

#include "esp_attr.h"
#include <WiFi.h>

#include "LumaVibe.h"
#include "debug_macros.h"
#include "ota_updater.h"

void printLocalTime(void);
LumaVibe g_Vibe;

//
void accelerometerISR() {
  g_Vibe.accelInterruptFlag = true;
}

//
void IRAM_ATTR watchDogISR() {
  g_Vibe.hardResetModem();
  g_Vibe.restart();
}

//
void setup() { // takes 33ms
  uint32_t setupTimer = millis();
  g_Vibe.setPowerBoostKeepOn(true);
  g_Vibe.setModemPins();
  SerialUSB.begin(115200);
  PRINT("\nMAC address: ", WiFi.macAddress());
  PRINT("\nFirmware version: ", FIRMWARE_VERSION);

  // This checks if the board wakes up from emergency-OTA
  if (g_isEmergency) {
    g_Vibe.detachAccelInterrupt();
    g_Vibe.initLed();
    PRINTS("\nRe-entering emergency mode");
    ota_updater_begin();
  }


  // For pin defines, change them in LumaVibe_globals.h
  // https://www.reddit.com/r/FastLED/comments/e4w6xh/not_usable_in_a_constant_expression/
  const LumaVibe::Parameters_t params = {
    "LV020_000000B",            // moduleID
    "LumaVibe 2.0",             // moduleType
    "mastap.net",               // mqttBroker
    "ngd/demo/HSRW_Balcony/data",    // publishDataTopic
    "ngd/demo/HSRW_Balcony/error", // publishErrorTopic
    "ngd/demo/HSRW_Balcony/command", // subscribeTopic
    "int16",                    // format
    2,                          // msgType
    200,                        // frequency
    2048,                       // samplesPerMeasurement
    5,                          // measurementInterval_ms
    MMA8451Q::RANGE_4G,         // accelerationRange
    30*60000,                   // sleepTime_ms
    60000,                      // watchDogTime_ms
    100,                        // transientThreshold in mG
    10,                         // transientDuration
    &watchDogISR,
    &accelerometerISR
  };

  LumaVibe::ERROR err; 
  err = g_Vibe.init(&params);
  if (LumaVibe::ERROR_NONE != err)
    g_Vibe.LOG_ERROR(err);
  g_Vibe.setLED(CRGB::Yellow);

  err = g_Vibe.begin();
  if (LumaVibe::ERROR_NONE != err)
    g_Vibe.LOG_ERROR(err);

  PRINTF("\nSetup took %lu ms", millis() - setupTimer);
  PRINTS("\nEnd of setup()\n");
}

//
void loop() {
  if (g_Vibe.isFirstBoot()) {
    delay(5000);
    if (g_Vibe.accelInterruptFlag) {
      g_Vibe.clearAccelInterrupt();
      g_Vibe.accelInterruptFlag = false;
      PRINTS("\nFirst boot");
      yield();
      g_Vibe.goToSleep();
    }
  }
  
  if (g_Vibe.accelInterruptFlag || g_Vibe.timerInterruptFlag) {
    g_Vibe.detachAccelInterrupt();
    PRINT("\naccelInterruptFlag: ", g_Vibe.accelInterruptFlag);
    PRINT("\ntimerInterruptFlag: ", g_Vibe.timerInterruptFlag);

    LumaVibe::ERROR err;

    g_Vibe.setLED(CRGB::Purple);
    time_t timeAtMeasure;
    err = g_Vibe.measure(&timeAtMeasure);
    if (LumaVibe::ERROR_NONE != err)
      g_Vibe.LOG_ERROR(err);
    
    g_Vibe.setLED(CRGB::Aqua);
    uint32_t bytesPacked;
    err = g_Vibe.packData(timeAtMeasure, &bytesPacked);
    if (LumaVibe::ERROR_NONE != err)
      g_Vibe.LOG_ERROR(err);
    PRINT("\nPacked Bytes: ", bytesPacked);

    g_Vibe.setLED(CRGB::Green);
    err = g_Vibe.publishData(bytesPacked, 512);
    if (LumaVibe::ERROR_NONE != err)
      g_Vibe.LOG_ERROR(err);
    
    g_Vibe.getCommandsFromServer(mqttCallback);

    PRINT("\nError Count: ", g_Vibe.countError());
    // Handle errors here
    if (0 != g_Vibe.countError()) {
      PRINTS("\nThere is error");
      if (g_Vibe.countNetworkError() >= MAX_NETWORK_ERROR_COUNT || g_Vibe.countError() >= MAX_ERROR_COUNT) {
        // Jump to emergency-OTA
        g_Vibe.endWatchDog();
        g_Vibe.disableModem();
        g_Vibe.setPowerBoostKeepOn(false);
        g_Vibe.setLED(CRGB::Red);
        ota_updater_begin();
      }
      else { // Publish list of (not so critical) errors
        uint32_t bytesPacked;
        LumaVibe::ERROR err;
        //bytesPacked = 0;
        PRINTS("\nPacking up error");
        err = g_Vibe.packError(&bytesPacked);
        if (LumaVibe::ERROR_NONE != err) {
          g_Vibe.LOG_ERROR(err);
        }
        err = g_Vibe.publishError(bytesPacked, bytesPacked);
        if (LumaVibe::ERROR_NONE != err) {
          g_Vibe.LOG_ERROR(err);
        } else {
          g_Vibe.clearError();
          PRINTS("\nErrors cleared");
        }
      }
    }

    g_Vibe.flashLED(CRGB::Green, 400, 5, false);
  
    g_Vibe.clearAccelInterrupt();
    g_Vibe.accelInterruptFlag = false;
    g_Vibe.timerInterruptFlag = false;
    g_Vibe.enableAccelInterrupt();
    g_Vibe.goToSleep();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  PRINTS("\nMessage arrived\n");
  PRINTVAL(topic);
  PRINTVAL("\n");
  for (int i = 0; i < length; i++) {
    PRINTVAL((char)payload[i]);
  }
  PRINTVAL("\n");
  mpack_reader_t reader;
  mpack_reader_init_data(&reader, (char *)payload, length);
  uint8_t count = mpack_expect_map_range(&reader, 3, 8);
  PRINTS("\nNumber of fields: "); PRINTVAL(count);

  mpack_expect_cstr_match(&reader, "sender");
  char sender[15];
  mpack_expect_cstr(&reader, sender, 15);

  mpack_expect_cstr_match(&reader, "trans_threshold");
  uint16_t transientThreshold;
  transientThreshold = mpack_expect_u16(&reader);

  mpack_expect_cstr_match(&reader, "trans_debcntr");
  uint16_t transientDuration;
  transientDuration = mpack_expect_u16(&reader);

  /*
    mpack_expect_cstr_match(&reader, "deadtime");
    uint16_t deadtime;
    deadtime = mpack_expect_u16(&reader);
  */
  mpack_expect_cstr_match(&reader, "period");
  uint64_t period;
  period = mpack_expect_u16(&reader);


  mpack_expect_cstr_match(&reader, "_msgid");
  char _msgid[20];
  mpack_expect_cstr(&reader, _msgid, 20);

  mpack_done_map(&reader);

  g_Vibe.setTransientThreshold(transientThreshold);
  g_Vibe.setTransientDuration(transientDuration);
  g_Vibe.setPeriod(period);

  PRINTS("\nsender: "); PRINTVAL(sender);
  PRINT("\n_msgid: ", _msgid);
  PRINT("\ntrans_threshold (mG): ", transientThreshold);
  PRINT("\ntrans_debcntr: ", transientDuration);
  PRINT("\nperiod: ", (long)period);
}

void printLocalTime(void) {
  struct tm now;
  getLocalTime(&now, 0); // This returns POSIX time
  SerialUSB.printf("\nSystem time: %04d-%02d-%02dT%02d:%02d:%02dZ", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, 
                                                       now.tm_hour, now.tm_min, now.tm_sec);
}
