#ifndef LUMAVIBE_H
#define LUMAVIBE_H

#include <Arduino.h>
#include "MMA845XQ_Vibe.h"
#include "esp32-hal-timer.h"

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_USE_GPRS     true
#define TINY_GSM_USE_WIFI     false
#include <TinyGsmClient.h> // https://github.com/vshymanskyy/TinyGSM

#include "PubSubClient.h"  // https://github.com/knolleary/pubsubclient
#include "mpack.h"         // https://ludocode.github.io/mpack/

class LumaVibe {
  public: 
    enum ERROR {
      ERROR_NONE               = 0,
      ERROR_TIME_ZERO          = 10,
      ERROR_FREQUENCY_ZERO     = 11,
      ERROR_TIMER_NULL         = 20,
      ERROR_NOT_ENOUGH_MEMORY  = 30,
      ERROR_MODEM_RESTART_FAIL = 40,
      ERROR_MODEM_NETWORK      = 41,
      ERROR_MODEM_GPRS         = 42,
      ERROR_MQTT               = 43,
      ERROR_PUBLISH_BEGIN_FAIL = 50,
      ERROR_PUBLISH_END_FAIL   = 51,
      ERROR_SENSOR_INIT        = 100
    };

    typedef struct {
      char            moduleID[20];
      char            moduleType[20];
      char            mqttBroker[20];
      char            publishTopic[20];
      char            subscribeTopic[20];
      char            format[10]; // always set to "Int16" for now, remove completely later? any use?
      uint8_t         msgType;
      uint16_t        frequency; // What should frequency actually mean?
      uint16_t        samplesPerMeasurement;
      uint16_t        measurementInterval_ms;
      MMA8451Q::RANGE accelerationRange;

      uint64_t        sleepTime_ms;
      uint64_t        watchDogTime_ms;
      uint16_t        transientThreshold;
      uint16_t        transientDuration;
      uint8_t         accelInterruptPin;
      void            (*watchDogISR)(void);
      void            (*sleepTimerISR)(void); 
      void            (*accelISR)(void);
    } Parameters_t;

    LumaVibe();
    ERROR init(Parameters_t params);
    ERROR begin();
    ERROR measure();
    ERROR packData(uint32_t *bytesPacked);
    ERROR publishData(uint32_t bytesToPublish, uint16_t bytesPerWrite);
    void  getCommandsFromServer(MQTT_CALLBACK_SIGNATURE);
    void  clearAccelInterrupt();
    void dumpSimInfo();

  private: 
    Parameters_t   _params;
    MMA8451Q       _accel;
    TinyGsm        _modem;
    TinyGsmClient  _client;
    PubSubClient   _mqtt;
    mpack_writer_t _writer;
    char           *_packBuffer;

    struct {
      union {
        int16_t v[3];
        struct {
          int16_t xi;
          int16_t yi;
          int16_t zi;
        };
      } *data;
      bool isBufferAllocated;
    } _accelBuffer;

    struct {
    hw_timer_t *watchDogTimer;
    hw_timer_t *sleepTimer;
    void (*watchDogISR)(void);
    void (*sleepTimerISR)(void);
    } _timers;

    ERROR enableTimer(hw_timer_t *timer, uint8_t timerNumber, uint64_t timer_ms, void (*timerISR)(void));
    void  keepAlive();
    void  clearMeasurementData();
    ERROR setupModem();
    ERROR getTimestampFromNetwork(String &timeStamp);
    void  packArray(mpack_writer_t *writer, const char *entryNames[3], const uint16_t length);
};

#endif //LUMAVIBE_H