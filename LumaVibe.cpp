#include "LumaVibe.h"

#define FCLK_DIVIDER          80
#define WATCHDOG_TIMER_NUMBER 1
#define SLEEP_TIMER_NUMBER    0

LumaVibe::LumaVibe() : _accel(0x1D), _modem(SerialAT), _client(_modem), _mqtt(_client) {}

// Copy parameters only, doesn't really do anything hardware-wise
LumaVibe::ERROR LumaVibe::init(LumaVibe::Parameters_t params) {
  if (0 == params.sleepTime_ms || 0 == params.watchDogTime_ms) {
    return ERROR_TIME_ZERO;
  }
  if (0 == params.frequency) {
    return ERROR_FREQUENCY_ZERO;
  }

  memcpy(&_params, &params, sizeof(Parameters_t));

  _timers.watchDogTimer = NULL;
  _timers.sleepTimer    = NULL;

  _accelBuffer.isBufferAllocated = false;

  return ERROR_NONE;
}

//
LumaVibe::ERROR LumaVibe::begin() {
  // Initialize watchdog timer and sleep timer

  // Begin Serial (?!)

  // Initialize accelerometer

  // Enable interrupt
}

//
LumaVibe::ERROR LumaVibe::measure() {
  _accelBuffer.data = (decltype(_accelBuffer.data))malloc(_params.samplesPerMeasurement * sizeof(*(_accelBuffer.data)));
  if (NULL == _accelBuffer.data) {
    return ERROR_NOT_ENOUGH_MEMORY;
  }
  _accelBuffer.isBufferAllocated = true;
  for (uint16_t index = 0; index  < _params.samplesPerMeasurement; index++) {
    uint32_t startTime = millis();
    while (millis() - startTime < _params.measurementInterval_ms);
    _accel.update();
    _accelBuffer.data[index].xi = (_accel._xi);
    _accelBuffer.data[index].yi = (_accel._yi);
    _accelBuffer.data[index].zi = (_accel._zi);

    if (index % 100 == 0) {
    Serial.print(F("Index: ")); Serial.print(index);
    Serial.print(F(" at: ")); Serial.println(startTime);
    }
  }
  Serial.print(F("Stop time: ")); Serial.println(millis());
  return ERROR_NONE;
}

//////////////////////
/* Private methods */
/////////////////////

//
LumaVibe::ERROR LumaVibe::enableTimer(hw_timer_t *timer, uint8_t timerNumber, uint64_t timer_ms, void (*timerISR)(void)) {
  if (NULL == timerBegin(timerNumber, FCLK_DIVIDER, true)) {
    return ERROR_TIMER_NULL;
  }
  timerAttachInterrupt(timer, timerISR, true);
  timerAlarmWrite(timer, timer_ms * 1000, false);
  timerAlarmEnable(timer);
  return ERROR_TIMER_NULL;
}

//
void LumaVibe::keepAlive() {
  timerWrite(_timers.watchDogTimer, 0);
}

//
void LumaVibe::clearMeasurementData() {
  if (_accelBuffer.isBufferAllocated) {
    free(_accelBuffer.data);
  }
}

