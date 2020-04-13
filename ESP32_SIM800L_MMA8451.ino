/*
  LumaVibe V2.00
*/

// https://github.com/Xinyuan-LilyGO/TTGO-T-Call
// https://pubsubclient.knolleary.net/api.html for pubsubclient

#include "esp_attr.h"

#define LUMAVIBE_ENABLE_HANDLING_ENABLE 1
#define TTGO
#include "LumaVibe.h"

#define SerialUSB Serial
#define DEBUG_MACROS_ENABLE 1
#include "debug_macros.h"


LumaVibe g_Vibe;

//
void IRAM_ATTR accelerometerISR() {
  PRINTS("\nAccel interrupt");
  g_Vibe.accelInterruptFlag = true;
}

//
void IRAM_ATTR watchDogISR() {
  PRINTS("\nRestarting\n");
  hardResetModem();
  g_Vibe.restart();
}

//
void IRAM_ATTR sleepTimerISR() {
  ;
}

//
void setup() {
  SerialUSB.begin(115200);
  setPowerBoostKeepOn(true);
  setModemPins();
  PRINTS("\nHello there\n");

  LumaVibe::Parameters_t params = {
    "LV020_000000B",            // moduleID
    "LumaVibe 2.0",             // moduleType
    "mastap.net",               // mqttBroker
    "ngd/demo/HSRW_Balcony/data",    // publishTopic
    "ngd/demo/HSRW_Balcony/command", // subscribeTopic
    "int16",                    // format
    2,                          // msgType
    200,                        // frequency
    2048,                       // samplesPerMeasurement
    5,                          // measurementInterval_ms
    MMA8451Q::RANGE_4G,         // accelerationRange
    240000,                     // sleepTime_ms
    60000,                      // watchDogTime_ms
    300,                        // transientThreshold in mG
    30,                         // transientDuration
    15,                         // accelInterruptPin
    &watchDogISR,
    &sleepTimerISR,
    &accelerometerISR
  };

  LumaVibe::ERROR err; 
  err = g_Vibe.init(&params);
  if (LumaVibe::ERROR_NONE != err)
    g_Vibe.HANDLE_ERROR(err);

/*   err = g_Vibe.setupModem();
  if(LumaVibe::ERROR_NONE != err)
    g_Vibe.HANDLE_ERROR(err); */

  err = g_Vibe.begin();
  if (LumaVibe::ERROR_NONE != err)
    g_Vibe.HANDLE_ERROR(err);
  
  PRINTS("\nEnd of setup()\n");
}

//
void loop() {
  if (g_Vibe.isFirstBoot()) {
    PRINTS("\nFirst blood");
    g_Vibe.detachAccelInterrupt();
    delay(1000);
    g_Vibe.clearAccelInterrupt();
    g_Vibe.accelInterruptFlag = false;
    g_Vibe.timerInterruptFlag = false;
    g_Vibe.enableAccelInterrupt();
    g_Vibe.goToSleep(240000);
  }
  if (g_Vibe.accelInterruptFlag || g_Vibe.timerInterruptFlag) {
    g_Vibe.detachAccelInterrupt();
    PRINT("\nINT PINT: ", digitalRead(15));
    PRINTS("\nlooping");
    PRINT("\naccelInterruptFlag: ", g_Vibe.accelInterruptFlag);
    PRINT("\ntimerInterruptFlag: ", g_Vibe.timerInterruptFlag);

    LumaVibe::ERROR err;

    err = g_Vibe.measure();
    if (LumaVibe::ERROR_NONE != err)
      g_Vibe.HANDLE_ERROR(err);
    delay(10000);
    
    uint32_t packedData;
    err = g_Vibe.packData(&packedData);
    if (LumaVibe::ERROR_NONE != err) 
      g_Vibe.HANDLE_ERROR(err);
    PRINT("\nPacked Bytes: ", packedData);

    err = g_Vibe.publishData(packedData, 512);
    if (LumaVibe::ERROR_NONE != err) 
      g_Vibe.HANDLE_ERROR(err);
    
    g_Vibe.getCommandsFromServer(mqttCallback);
  
    g_Vibe.clearAccelInterrupt();
    g_Vibe.accelInterruptFlag = false;
    g_Vibe.timerInterruptFlag = false;
    g_Vibe.enableAccelInterrupt();
    g_Vibe.goToSleep(240000);
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
    mpack_expect_cstr_match(&reader, "period");
    uint16_t period;
    period = mpack_expect_u16(&reader);
  */

  mpack_expect_cstr_match(&reader, "_msgid");
  char _msgid[20];
  mpack_expect_cstr(&reader, _msgid, 20);

  mpack_done_map(&reader);

  g_Vibe.setTransientThreshold(transientThreshold);
  g_Vibe.setTransientDuration(transientDuration);

  PRINTS("\nsender: "); PRINTS(sender);
  PRINT("\n_msgid: ", _msgid);
  PRINT("\ntrans_threshold (mG): ", transientThreshold);
  PRINT("\ntrans_debcntr: ", transientDuration);
}