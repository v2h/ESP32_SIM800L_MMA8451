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

// TODO: move this thing somewhere else
#ifdef TTGO
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
bool setPowerBoostKeepOn(bool en)
{
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35); // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}

// https://electronics.stackexchange.com/questions/287418/sim800-pwrkey-automatic-start
#define MODEM_RST      5  // Active low (also low after modem.sleepEnable() is called)
#define MODEM_PWKEY    4 // [Pulled up] drive down to turn on the modem
#define MODEM_POWER_ON 23 // [Pulled-up] active high
// Set-up modem reset, enable, power pins
void setModemPins() {
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
}

void hardResetModem() {
  digitalWrite(MODEM_RST, LOW);
  delay(200); // must be more than 105ms
  digitalWrite(MODEM_RST, HIGH);
}
#endif // TTGO

// Marcro for handdling errors
#if LUMAVIBE_ENABLE_HANDLING_ENABLE
#define HANDLE_ERROR(e) handleError(e, __LINE__)
#endif // LUMAVIBE_ENABLE_HANDLING_ENABLE

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
      char            publishTopic[40];
      char            subscribeTopic[40];
      char            format[10]; // always set to "Int16" for now, remove completely later? any use?
      uint8_t         msgType;
      uint16_t        frequency; // What should frequency actually mean?
      uint16_t        samplesPerMeasurement;
      uint16_t        measurementInterval_ms;
      MMA8451Q::RANGE accelerationRange;

      uint64_t        sleepTime_ms;
      uint64_t        watchDogTime_ms;
      uint16_t        transientThreshold_mG;
      uint16_t        transientDuration;
      uint8_t         accelInterruptPin;
      void            (*watchDogISR)();
      void            (*sleepTimerISR)(); 
      void            (*accelISR)();
    } Parameters_t;

    bool accelInterruptFlag;
    bool timerInterruptFlag;

    LumaVibe();
    ERROR init(Parameters_t *p);
    ERROR begin();
    ERROR measure();
    ERROR packData(uint32_t *bytesPacked);
    ERROR publishData(uint32_t bytesToPublish, uint16_t bytesPerWrite);
    void  getCommandsFromServer(MQTT_CALLBACK_SIGNATURE);
    void  restart();
    void  handleError(ERROR error, uint16_t line);
    void  readSingle(int16_t *xi, int16_t *yi, int16_t *zi);
    void  detachAccelInterrupt();
    void  clearAccelInterrupt();
    void  enableAccelInterrupt();
    void  dumpSimInfo();
    ERROR setupModem();

    void endWatchDog();

    void setTransientThreshold(uint16_t threshold_mG);
    void setTransientDuration(uint16_t duration);
    void goToSleep(void);
    bool isFirstBoot();

  private: 
    Parameters_t   _params;
    static RTC_DATA_ATTR Parameters_t _params; // stored in RTC memory
    MMA8451Q       _accel;
    TinyGsm        _modem;
    TinyGsmClient  _client;
    PubSubClient   _mqtt;
    mpack_writer_t _writer;
    char           *_packBuffer;

    static RTC_DATA_ATTR uint64_t _bootCount;

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
    } _timers;

    ERROR enableTimer(hw_timer_t **timer, uint8_t timerNumber, uint64_t timer_ms, void (*timerISR)());
    void  keepAlive();
    void  clearMeasurementData();
    //ERROR setupModem();
    ERROR getTimestampFromNetwork(char timeStamp[21]);
    void  packArray(mpack_writer_t *writer, const char *entryNames[3], const uint16_t length);
};

#endif //LUMAVIBE_H