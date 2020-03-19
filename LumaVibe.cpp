#include "LumaVibe.h"

#define FCLK_DIVIDER          80
#define WATCHDOG_TIMER_NUMBER 1
#define SLEEP_TIMER_NUMBER    0

LumaVibe::LumaVibe() : _accel(0x1D) {}

// Copy parameters only, doesn't really do anything hardware-wise
LumaVibe::ERROR LumaVibe::init(LumaVibe::Parameters_t params) {
  if (0 == params.sleepTime_ms || 0 == params.watchDogTime_ms) {
    return ERROR_TIME_ZERO;
  }
  if (0 == params.frequency) {
    return ERROR_FREQUENCY_ZERO;
  }

  memcpy(&this->_params, &params, sizeof(Parameters_t));

  this->_timers.watchDogTimer = NULL;
  this->_timers.sleepTimer    = NULL;

  this->_accelBuffer.isBufferAllocated = false;

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
  this->_accelBuffer.xi = (int16_t *)malloc(this->_params.samplesPerMeasurement * sizeof(int16_t));
  this->_accelBuffer.yi = (int16_t *)malloc(this->_params.samplesPerMeasurement * sizeof(int16_t));
  this->_accelBuffer.zi = (int16_t *)malloc(this->_params.samplesPerMeasurement * sizeof(int16_t));
  if ((NULL == this->_accelBuffer.xi) || (NULL != this->_accelBuffer.yi) || (NULL != this->_accelBuffer.zi)) {
    return ERROR_NOT_ENOUGH_MEMORY;
  }
  this->_accelBuffer.isBufferAllocated = true;
  for (uint16_t index = 0; index  < this->_params.samplesPerMeasurement; index++) {
    uint32_t startTime = millis();
    this->_accel.update();
    this->_accelBuffer.xi[index] = (this->_accel._xi);
    this->_accelBuffer.yi[index] = (this->_accel._yi);
    this->_accelBuffer.zi[index] = (this->_accel._zi);
    while (millis() - startTime < _params.measurementInterval_ms);

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
  timerWrite(this->_timers.watchDogTimer, 0);
}

//
void LumaVibe::clearMeasurementData() {
  if (this->_accelBuffer.isBufferAllocated) {
    free(this->_accelBuffer.xi);
    free(this->_accelBuffer.yi);
    free(this->_accelBuffer.zi);
  }
}

