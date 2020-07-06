#ifndef LUMAVIBE_H
#define LUMAVIBE_H

#include <Arduino.h>
#include "LumaVibe_globals.h"
#include "esp32-hal-timer.h"
#include "custom_src\bsp\mma8451.h"

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_USE_GPRS     true
#define TINY_GSM_USE_WIFI     false
// #define TINY_GSM_DEBUG        Serial // uncomment to see debugs, BUT WILL TRIGGER WDT
#include <TinyGsmClient.h> // https://github.com/vshymanskyy/TinyGSM

#include "PubSubClient.h"  // https://github.com/knolleary/pubsubclient
#include "mpack.h"         // https://ludocode.github.io/mpack/
#include "FastLED.h"

// Marcro for handdling errors
#define LumaVibe_LOG_ERROR(e) LumaVibe_logError(e, __LINE__)

typedef enum {
    LUMAVIBE_ERROR_NONE = 0,
    LUMAVIBE_ERROR_TIME_ZERO,
    LUMAVIBE_ERROR_FREQUENCY,
    LUMAVIBE_ERROR_TIMER_NULL,
    LUMAVIBE_ERROR_NOT_ENOUGH_MEMORY,
    LUMAVIBE_ERROR_MODEM_RESTART_FAIL,
    LUMAVIBE_ERROR_MODEM_NETWORK_NOT_CONNECTED,
    LUMAVIBE_ERROR_MODEM_GPRS_NOT_CONNECTED,
    LUMAVIBE_ERROR_MQTT_NOT_CONNECTED,
    LUMAVIBE_ERROR_PACKING_NOT_FINISHED,
    LUMAVIBE_ERROR_PUBLISH_BEGIN_FAIL,
    LUMAVIBE_ERROR_PUBLISH_END_FAIL,
    LUMAVIBE_ERROR_SENSOR_INIT,
    LUMAVIBE_ERROR_SENSOR_SETTING,
    LUMAVIBE_ERROR_MAX
} LumaVibe_Error_t;

typedef struct {
  uint8_t  accelSCL;
  uint8_t  accelSDA;
  uint8_t  accelAddress;
  uint8_t  accelInterruptPin;
  uint16_t numberOfMeas;
  uint16_t measureFrequency;
  uint64_t sleepTime_ms;
  uint64_t watchDogTime_ms;
  char     mqttBroker[20];
  char     mqttUserName[10];
  char     mqttPassword[10];
  char     firmwareVersion[20];
  char     moduleID[16];
  char     moduleType[16];
  uint8_t  msgType;
  char     format[8];
  char     publishDataTopic[50];
  char     publishErrorTopic[50];
  char     subscribeTopic[50];
} LumaVibe_Settings_t;

bool LumaVibe_setPowerBoostKeepOn(bool en);
void LumaVibe_hardResetModem();
void LumaVibe_disableModem();
void LumaVibe_enableModem();

LumaVibe_Error_t LumaVibe_init(LumaVibe_Settings_t * const s);
LumaVibe_Error_t LumaVibe_begin();
LumaVibe_Error_t LumaVibe_measure(time_t *timeAtMeasure_s);
LumaVibe_Error_t LumaVibe_packData(time_t timeAtMeasure_s, uint32_t *bytesPacked);
LumaVibe_Error_t LumaVibe_publishData(uint32_t bytesToPublish, uint16_t bytesPerWrite);
void LumaVibe_getCommandsFromServer();
void LumaVibe_clearAccelInterrupt();
void LumaVibe_restart();
void LumaVibe_logError(LumaVibe_Error_t error, uint16_t line);
uint8_t LumaVibe_countNetworkError(void);
LumaVibe_Error_t LumaVibe_packError(uint32_t *bytesPacked);
LumaVibe_Error_t LumaVibe_publishError(int32_t bytesToPublish, uint16_t bytesPerWrite);
uint8_t LumaVibe_countError(void);
void LumaVibe_clearError(void);
void LumaVibe_detachAccelInterrupt();
void LumaVibe_enableAccelInterrupt();
void LumaVibe_setTransientThreshold_mG(uint16_t threshold_mG);
void LumaVibe_settransientDebounceCounter(uint16_t duration);
void LumaVibe_setPeriod(uint32_t period_s);
void LumaVibe_goToSleep(void);
void LumaVibe_setLED(CRGB::HTMLColorCode color);
void LumaVibe_clearLED();
void LumaVibe_flashLED(CRGB::HTMLColorCode color, uint16_t duration_ms, uint8_t numberOfTimes, bool retainColor);
void LumaVibe_endWatchdog();

// Must be first defined in the main file
// Declared with 'extern' so other files can use..
// ..whenever this header is included
extern bool g_isEmergency;
extern volatile bool g_accelInterruptFlag;
extern RTC_DATA_ATTR uint64_t g_bootCount;
extern portMUX_TYPE  g_mux;

#endif //LUMAVIBE_H

