/*
  LumaVibe V2.00
*/

// https://github.com/Xinyuan-LilyGO/TTGO-T-Call
// https://pubsubclient.knolleary.net/api.html for pubsubclient

#include "esp_attr.h"

#define LUMAVIBE_ENABLE_ERROR_LOGGING 1
#define LUMAVIBE_PUBLIC_ALL 0 // Turn all private members into public
#include "LumaVibe.h"

#define SerialUSB Serial
#define DEBUG_MACROS_ENABLE 1
#include "debug_macros.h"

#include "ota_updater.h"

LumaVibe g_Vibe;

//
void IRAM_ATTR accelerometerISR() {
  PRINTS("\nAccel interrupt");
  g_Vibe.accelInterruptFlag = true;
}

//
void IRAM_ATTR watchDogISR() {
  PRINTS("\nRestarting\n");
  g_Vibe.hardResetModem();
  g_Vibe.restart();
}

//
void IRAM_ATTR sleepTimerISR() {
  ;
}

//
void setup() { // takes 231 ms
  uint32_t setupTimer = millis();
  SerialUSB.begin(115200);
  if (g_isEmergency) {
    g_Vibe.detachAccelInterrupt();
    g_Vibe.initLed();
    PRINTS("\nRe-entering emergency mode");
    ota_updater_begin();
  }
  g_Vibe.setPowerBoostKeepOn(true);
  g_Vibe.setModemPins();
  g_Vibe.hardResetModem();
  PRINTS("\nHello there\n");

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
    300,                        // transientThreshold in mG
    30,                         // transientDuration
    &watchDogISR,
    &sleepTimerISR,
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

  SerialUSB.printf("\nSetup took %lu ms", millis() - setupTimer);
  PRINTS("\nEnd of setup()\n");
}

//
void loop() {
  PRINTS("loop");
  if (g_Vibe.isFirstBoot()) {
    g_Vibe.detachAccelInterrupt();
    delay(1000);
    g_Vibe.clearAccelInterrupt();
    g_Vibe.accelInterruptFlag = false;
    g_Vibe.timerInterruptFlag = false;
    g_Vibe.enableAccelInterrupt();
    PRINTS("First boot");
    g_Vibe.setLED(CRGB::DarkBlue);
    g_Vibe.goToSleep();
  }
  
  if (g_Vibe.accelInterruptFlag || g_Vibe.timerInterruptFlag) {
    g_Vibe.detachAccelInterrupt();
    PRINT("\naccelInterruptFlag: ", g_Vibe.accelInterruptFlag);
    PRINT("\ntimerInterruptFlag: ", g_Vibe.timerInterruptFlag);

    LumaVibe::ERROR err;

    g_Vibe.setLED(CRGB::Purple);
    err = g_Vibe.measure();
    if (LumaVibe::ERROR_NONE != err)
      g_Vibe.LOG_ERROR(err);
    //delay(10000);
    
    g_Vibe.setLED(CRGB::Aqua);
    uint32_t packedData;
    err = g_Vibe.packData(&packedData);
    if (LumaVibe::ERROR_NONE != err)
      g_Vibe.LOG_ERROR(err);
    PRINT("\nPacked Bytes: ", packedData);

    g_Vibe.setLED(CRGB::Green);
    err = g_Vibe.publishData(packedData, 512);
    if (LumaVibe::ERROR_NONE != err)
      g_Vibe.LOG_ERROR(err);
    
    g_Vibe.getCommandsFromServer(mqttCallback);
    PRINT("\nError Count: ", g_Vibe.countError());
    // Handle errors here
    if (0 != g_Vibe.countError()) {
      PRINTS("\nThere is error");
      if (g_Vibe.countNetworkError() >= 6 || g_Vibe.countError() >= MAX_ERROR_COUNT) {
        // Jump to emergency-OTA
        g_Vibe.endWatchDog();
        g_Vibe.disableModem();
        g_Vibe.setPowerBoostKeepOn(false);
        g_Vibe.setLED(CRGB::Red);
        ota_updater_begin();
      }
      else { // Publish list of (not so critical) errors
        uint32_t packedData;
        LumaVibe::ERROR err;
        //packedData = 0;
        PRINTS("\nPacking up error");
        err = g_Vibe.packError(&packedData);
        if (LumaVibe::ERROR_NONE != err) {
          g_Vibe.LOG_ERROR(err);
        }
        err = g_Vibe.publishError(packedData, packedData);
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
    g_Vibe.setLED(CRGB::DarkBlue);
    FastLED.delay(1000);
    g_Vibe.clearLED();
    g_Vibe.goToSleep();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  SerialUSB.println(F("\nMessage arrived"));
  SerialUSB.println(topic);
  SerialUSB.println();
  for (int i = 0; i < length; i++) {
    SerialUSB.print((char)payload[i]);
  }
  SerialUSB.println();
  mpack_reader_t reader;
  mpack_reader_init_data(&reader, (char *)payload, length);
  uint8_t count = mpack_expect_map_range(&reader, 3, 8);
  SerialUSB.print("\nNumber of fields: "); SerialUSB.println(count);

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

  PRINTS("\nsender: "); PRINTS(sender);
  PRINT("\n_msgid: ", _msgid);
  PRINT("\ntrans_threshold (mG): ", transientThreshold);
  PRINT("\ntrans_debcntr: ", transientDuration);
  PRINT("\nperiod: ", (long)period);
}
