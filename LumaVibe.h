#ifndef LUMAVIBE_H
#define LUMAVIBE_H

#include "MMA845XQ_Vibe.h"
#include "esp32-hal-timer.h"

class LumaVibe {
  public: 
    enum ERROR {
      ERROR_NONE              = 0,
      ERROR_TIME_ZERO         = 10,
      ERROR_FREQUENCY_ZERO    = 11,
      ERROR_TIMER_NULL        = 20,
      ERROR_NOT_ENOUGH_MEMORY = 30,
      ERROR_SENSOR_INIT       = 100
    };

    typedef struct {
      char            moduleID[20];
      char            moduleType[20];
      char            publishTopic[20];
      char            subscribeTopic[20];
      char            format[10]; // always set to "Int16" for now, remove completely later? any use?
      uint16_t        frequency; // What should frequency actually mean?
      uint16_t        samplesPerMeasurement;
      uint16_t        measurementInterval_Ms;
      MMA8451Q::RANGE accelerationRange;

      uint64_t        sleepTimeMs;
      uint64_t        watchDogTimeMs;
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

  private: 
    Parameters_t _params;

    MMA8451Q _accel;

    struct {
      int16_t *xi;
      int16_t *yi;
      int16_t *zi;
      bool isBufferAllocated;
    } _accelBuffer;

    struct {
    hw_timer_t *watchDogTimer;
    hw_timer_t *sleepTimer;
    void (*watchDogISR)(void);
    void (*sleepTimerISR)(void);
    } _timers;

    ERROR enableTimer(hw_timer_t *timer, uint8_t timerNumber, uint64_t timerMs, void (*timerISR)(void));
    void keepAlive();
    void clearMeasurementData();
};

#endif //LUMAVIBE_H