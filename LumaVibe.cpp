#include "LumaVibe.h"

#define FCLK_DIVIDER          80
#define WATCHDOG_TIMER_NUMBER 1
#define SLEEP_TIMER_NUMBER    0

#define SerialAT  Serial1
#define SerialUSB Serial

#define MODEM_TX 27
#define MODEM_RX 26

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
LumaVibe::ERROR LumaVibe::begin () {
  // Initialize watchdog timer
  if (ERROR_NONE != enableTimer(_timers.watchDogTimer, WATCHDOG_TIMER_NUMBER, _params.watchDogTime_ms, _timers.watchDogISR)) {
    return ERROR_TIMER_NULL;
  }
  // Begin Serial (?!)

  // set mqtt broker
  _mqtt.setServer(_params.mqttBroker, 1883);

  // Initialize accelerometer
  _accel.SWreset();
  _accel.setCommonParameters(_params.accelerationRange, 
                            MMA8451Q::RES_MAX, MMA8451Q::LN_OFF, MMA8451Q::DR_100, MMA8451Q::OS_NORMAL, MMA8451Q::HPF_OFF);
  _accel.setTransientThresholdN(_params.transientThreshold, false);
  _accel.setTransientDebounceCounter(_params.transientDuration);
  _accel.setHPFilterCutOff(3);
  _accel.setInterrupt(MMA8451Q::INT_EN_TRANS, MMA8451Q::INT2, true);
  uint8_t mo_src = _accel.getMotionSource(); // Read to clear EA flag
  uint8_t tr_src = _accel.getTransientSource(); // Read to clear EA flag
  
  // Enable interrupt pin(s)
  pinMode(_params.accelInterruptPin, INPUT_PULLUP);
  attachInterrupt(_params.accelInterruptPin, _params.accelISR, FALLING);

  //
  keepAlive();
  return ERROR_NONE;
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
    SerialUSB.print(F("Index: ")); SerialUSB.print(index);
    SerialUSB.print(F(" at: ")); SerialUSB.println(startTime);
    }
  }
  SerialUSB.print(F("Stop time: ")); SerialUSB.println(millis());
  keepAlive();
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

  // if the following yield() is removed, the timer will not be enabled the second time
  // REF: https://github.com/espressif/arduino-esp32/issues/1313
  yield();

  timerAlarmEnable(timer);
  return ERROR_NONE;
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

//
LumaVibe::ERROR LumaVibe::setupModem() {
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  if (!_modem.restart())
    return ERROR_MODEM_RESTART_FAIL;
  if (!_modem.waitForNetwork(240000L))
    if (!_modem.isNetworkConnected())
      return ERROR_MODEM_NETWORK;
  if (_modem.gprsConnect("","","")) // .gprsConnect(apn, gprsUser, gprsPass)
    return ERROR_MODEM_GPRS;
  
  // TODO: function to dump network and sim card information

  keepAlive();
  return ERROR_NONE;
}