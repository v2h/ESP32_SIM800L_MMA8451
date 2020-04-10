/*
Only RTC IO can be used as a source for external wake
source. They are pins: 0,2,4,12-15,25-27,32-39.
https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/DeepSleep/ExternalWakeUp/ExternalWakeUp.ino
*/

#include "LumaVibe.h"
#include "esp_attr.h"
#include <stdlib.h> // For malloc()

#define DEBUG_MACROS_ENABLE 1
#include "debug_macros.h"

#define FCLK_DIVIDER          80
#define WATCHDOG_TIMER_NUMBER 1
#define SLEEP_TIMER_NUMBER    0
#define PACKER_CAPACITY       40000
#define BYTES_PER_WRITE       512

#define SerialAT  Serial1
#define SerialUSB Serial

#define MODEM_TX 27
#define MODEM_RX 26

#define mG_PER_COUNT 63

static const struct {
  const char * const timestamp    = "timestamp";
  const char * const moduleID     = "moduleID";
  const char * const moduletype   = "moduletype";
  const char * const msgtype      = "msgtype";
  const char * const format       = "format";
  const char * const freq         = "freq";
  const char * const numberOfMeas = "numberOfMeas";
  const char * const x_accel      = "x-accel";
  const char * const y_accel      = "y-accel";
  const char * const z_accel      = "z-accel";
} StringToPack;

// Initialize static variables in class
uint64_t LumaVibe::_bootCount = 0;

// Default Initializer
LumaVibe::LumaVibe() :
  _accel(0x1D), // TODO: Move 0x1D somewhere else
  _modem(SerialAT), 
  _client(_modem), 
  _mqtt(_client) 
  {}

// Copy parameters + initialize watchdog timer
LumaVibe::ERROR LumaVibe::init(LumaVibe::Parameters_t *p) {
  if (0 == p->sleepTime_ms || 0 == p->watchDogTime_ms) {
    return ERROR_TIME_ZERO;
  }
  if (0 == p->frequency) {
    return ERROR_FREQUENCY_ZERO;
  }
  // Initialize watchdog timer
  if (ERROR_NONE != enableTimer(&_timers.watchDogTimer, WATCHDOG_TIMER_NUMBER, _params.watchDogTime_ms, _params.watchDogISR)) {
    return ERROR_TIMER_NULL;
  }
  memcpy(&_params, p, sizeof(Parameters_t));
  _mqtt.setServer(_params.mqttBroker, 1883);
  _accelBuffer.isBufferAllocated = false;
  _timers.sleepTimer = NULL;

  return ERROR_NONE;
}

//
LumaVibe::ERROR LumaVibe::begin() {
  //if (0 == _bootCount) {
    PRINTS("\nFirst blood...I mean first boot");
    // Initialize accelerometer
    PRINTS("\nInitializing accelerometer");
    Wire.begin();
    _accel.SWreset();
    _accel.setCommonParameters(_params.accelerationRange,
                               MMA8451Q::RES_MAX, MMA8451Q::LN_OFF, MMA8451Q::DR_100, MMA8451Q::OS_NORMAL, MMA8451Q::HPF_OFF);
    _accel.setTransientDetection();
    _accel.setTransientThresholdN(_params.transientThreshold_mG / mG_PER_COUNT, false);  // 0 - 127 is 0 - 8g in 0.063g increments
    _accel.setTransientDebounceCounter(_params.transientDuration);
    _accel.setHPFilterCutOff(3);
    _accel.setInterrupt(MMA8451Q::INT_EN_TRANS, MMA8451Q::INT2, true);
    uint8_t whoami = _accel._read_register(MMA8451Q::WHO_AM_I);
    PRINTHEX("\nwhoami: ", whoami);
    clearAccelInterrupt();
    enableAccelInterrupt();
    accelInterruptFlag = false;
    timerInterruptFlag = false;
    keepAlive();
  //} 
  //else {
    //Wire.begin(); // This prevents the AccelISR to be called 
    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    switch (wakeupReason) {
      case ESP_SLEEP_WAKEUP_EXT0:
        accelInterruptFlag = true;
        PRINTS("\nWoken up by accelerometer");
        break;
      case ESP_SLEEP_WAKEUP_TIMER:
        timerInterruptFlag = true;
        PRINTS("\nWoken up by timer");
        break;
      default:
        PRINTS("\nWakeup was not caused by deep sleep: %d\n");
        break;
    }
  //}
  _bootCount++; // Caution: when to increment bootCount??
  return ERROR_NONE;
}

//
LumaVibe::ERROR LumaVibe::measure() {
  clearMeasurementData();
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
      PRINT("\nIndex: ", index);
      PRINT(" at: ", startTime);
    }
  }
  PRINT("\nStop time: ", millis());
  keepAlive();
  return ERROR_NONE;
}

// 
LumaVibe::ERROR LumaVibe::packData(uint32_t *bytesPacked) {
/*   PRINTS("\nPacking data...");
  if (!_modem.isGprsConnected()) {
    PRINTS("\nGPRS connecting.."); // For getting network time
    if (!_modem.gprsConnect("", "", ""))
      return ERROR_MODEM_GPRS;
  }
  PRINTS("\nGPRS connected"); */

  ERROR err = setupModem();
  if (ERROR_NONE != err) {
    return err;
  }

  _packBuffer = (char *)malloc(PACKER_CAPACITY);
  if (NULL == _packBuffer) {
    return ERROR_NOT_ENOUGH_MEMORY;
  }
  PRINTS("\nPacking header");
  mpack_writer_init(&_writer, _packBuffer, PACKER_CAPACITY);
  mpack_start_map(&_writer, 10);
  char timeStamp[21] = {0};
  getTimestampFromNetwork(timeStamp);
  // TODO: refactor this shit, from here..
  mpack_write_cstr(&_writer, StringToPack.timestamp); 
  mpack_write_cstr(&_writer, timeStamp);
  mpack_write_cstr(&_writer, StringToPack.moduleID); 
  mpack_write_cstr(&_writer, _params.moduleID);
  mpack_write_cstr(&_writer, StringToPack.moduletype);
  mpack_write_cstr(&_writer, _params.moduleType);
  mpack_write_cstr(&_writer, StringToPack.msgtype);
  mpack_write_u8(&_writer, _params.msgType);
  mpack_write_cstr(&_writer, StringToPack.format);
  mpack_write_cstr(&_writer, _params.format);
  mpack_write_cstr(&_writer, StringToPack.freq);
  mpack_write_u16(&_writer, _params.frequency);
  mpack_write_cstr(&_writer, StringToPack.numberOfMeas);
  mpack_write_u16(&_writer, _params.samplesPerMeasurement);
  // ..to here
  PRINTS("\nPacking array");
  const char *entries[] = {StringToPack.x_accel, StringToPack.y_accel, StringToPack.z_accel};
  packArray(&_writer, entries, _params.samplesPerMeasurement);
  *bytesPacked = mpack_writer_buffer_used(&_writer);
  mpack_finish_map(&_writer);

  clearMeasurementData();
  keepAlive();
  return ERROR_NONE;
}

//
LumaVibe::ERROR LumaVibe::publishData(uint32_t bytesToPublish, uint16_t bytesPerWrite) {
  PRINT("\nBytes to be published: ", bytesToPublish);
  do {
    PRINTS("\nConnecting MQTT");
    _mqtt.connect("sim800l", "user", "mqtt");
    PRINT("\nMQTT state: ", _mqtt.state()); // Should be 0
    delay(1000);
  } while (!_mqtt.connected());
  if(!_mqtt.beginPublish(_params.publishTopic, bytesToPublish, false)) {
    return ERROR_PUBLISH_BEGIN_FAIL;
  }
  keepAlive();
  PRINTS("\nPublishing...");
  PRINT("\nBytes Total: ", bytesToPublish);
  uint8_t *pointerToBuffer = (uint8_t *)_packBuffer;
  while (bytesToPublish) {
    uint16_t bytesToWrite = (bytesToPublish > bytesToPublish % bytesPerWrite ? bytesPerWrite : bytesToPublish % bytesPerWrite);
    uint16_t bytesWritten = _mqtt.write(pointerToBuffer, bytesToWrite);
    bytesToPublish -= bytesWritten;
    pointerToBuffer += bytesWritten;
    _mqtt.loop();
    yield();
    PRINT("\nBytes left: ", bytesToPublish);
    while (MQTT_CONNECTED != _mqtt.state()) {
      PRINTS("\nReconnecting MQTT");
      _mqtt.connect("sim800l", "user", "mqtt");
    }
  }
  if (!_mqtt.endPublish()) {
    return ERROR_PUBLISH_END_FAIL;
  }

  free(_packBuffer);
  _packBuffer = NULL;
  clearMeasurementData();
  PRINTS("\nPublishing done");
  return ERROR_NONE;
}

//
void LumaVibe::getCommandsFromServer(MQTT_CALLBACK_SIGNATURE) {
  _mqtt.setCallback(callback);
  _mqtt.subscribe(_params.subscribeTopic);
  delay(2000);
  _mqtt.loop();
  _mqtt.loop();
  _mqtt.disconnect();
  keepAlive();
}

//
void LumaVibe::clearAccelInterrupt() {
  PRINTS("\nClearing interrupt");
  // Reading TRANS_SRC by calling getTransientSource() already clears the SRC_TRANS bit in INT_SOURCE
  // uint8_t motionSrc = _accel.getMotionSource(); // No need
  // uint8_t intSrc    = _accel._read_register(MMA8451Q::INT_SOURCE); // No need
  uint8_t tranSrc   = _accel.getTransientSource();
  PRINT("\ntranSrc: ", tranSrc);
}

//
void LumaVibe::restart() {
  esp_restart();
}

// 
void LumaVibe::handleError(ERROR error, uint16_t line) {
  char errorString[40];
  sprintf(errorString, "\nError: %u at line: %u\n", error, line);
  PRINTS(errorString);
  while (1);
}

//
void LumaVibe::readSingle(int16_t *xi, int16_t *yi, int16_t *zi) {
  _accel.update();
  *xi = _accel._xi;
  *yi = _accel._yi;
  *zi = _accel._zi;
}

//
void LumaVibe::detachAccelInterrupt() {
  detachInterrupt(_params.accelInterruptPin);
}

//
void LumaVibe::enableAccelInterrupt() {
  pinMode(_params.accelInterruptPin, INPUT_PULLUP);
  attachInterrupt(_params.accelInterruptPin, _params.accelISR, FALLING);
  PRINTS("\nAcceleration interrupt enabled");
}

// 
void LumaVibe::dumpSimInfo() {
  Serial.print(F("\nSIM status: "));
  uint8_t simStatus = _modem.getSimStatus();
  Serial.println(simStatus);
  String ccid = _modem.getSimCCID();
  Serial.print(F("SIM CCID: "));
  Serial.println(ccid);
  String emei = _modem.getIMEI();
  Serial.print(F("SIM IMEI: "));
  Serial.println(emei);
  String cop = _modem.getOperator();
  Serial.print(F("SIM Operator: "));
  Serial.println(cop);
}

//
void LumaVibe::setTransientThreshold(uint16_t threshold_mG) {
  _params.transientThreshold_mG = threshold_mG;
  _accel.setTransientThresholdN((uint8_t)(_params.transientThreshold_mG / mG_PER_COUNT));
}

//
void LumaVibe::setTransientDuration(uint16_t duration) {
  _params.transientDuration = duration;
  _accel.setTransientDebounceCounter(_params.transientDuration);
}

//
void LumaVibe::goToSleep(uint64_t duration_ms) {
  PRINTS("\nGoing to sleep..");
  if (_modem.isGprsConnected())
    if (_modem.gprsDisconnect()) {
      PRINTS("\nGPRS disconnected");
    }
  
  if (_modem.sleepEnable()) {
    PRINTS("\nModem put to sleep");
  }
  
  esp_sleep_enable_timer_wakeup(duration_ms * 1000);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, LOW); //(gpio_num_t)_params.accelInterruptPin
  PRINTS("\nGoodnight!\n");
  SerialUSB.flush();
  delay(3000);
  endWatchDog();
  esp_deep_sleep_start();
}

//
void LumaVibe::endWatchDog(){
  timerEnd(_timers.watchDogTimer);
}

//
bool LumaVibe::isFirstBoot() {
  return (1 == _bootCount); // CAUTION: 1 or 0?
}

//////////////////////////////////////////////////////////////////
///////////////////////* Private methods *///////////////////////
/////////////////////////////////////////////////////////////////

// Passing a pointer to a pointer to hw_timer_t, so that the original timer is updated.
LumaVibe::ERROR LumaVibe::enableTimer(hw_timer_t **timer, uint8_t timerNumber, uint64_t timer_ms, void (*timerISR)()) {
  *timer = timerBegin(timerNumber, FCLK_DIVIDER, true);
  if (NULL == *timer)
    return ERROR_TIMER_NULL;
  timerAttachInterrupt(*timer, timerISR, true);
  timerAlarmWrite(*timer, timer_ms * 1000, false);

  // if the following yield() is removed, the timer will not be enabled the second time
  // REF: https://github.com/espressif/arduino-esp32/issues/1313
  yield();

  timerAlarmEnable(*timer);
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
    _accelBuffer.data = NULL;
    _accelBuffer.isBufferAllocated = false;
    PRINTS("\nMeasurement Data Cleared");
  }
}

//
LumaVibe::ERROR LumaVibe::setupModem() {
  PRINTS("\nSetting up modem");
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  //_modem.restart();
  if (!_modem.restart())
    return ERROR_MODEM_RESTART_FAIL;
  String modemInfo = _modem.getModemInfo();
  PRINT("\nmodem info: ", modemInfo);
  dumpSimInfo();

  PRINTS("\nwaiting for network...");
  if (!_modem.waitForNetwork(240000L))
    if (!_modem.isNetworkConnected())
      return ERROR_MODEM_NETWORK;
  PRINTS("\nnetwork connected...");
  PRINTS("\nconnecting to GPRS");
  while (!_modem.isGprsConnected()) { // ATTENTION: WHILE LOOP!!
    _modem.gprsConnect("", "", "");
    delay(1000);
  }
  PRINTS("\nGPRS connected");
  
  keepAlive();
  return ERROR_NONE;
}

//
LumaVibe::ERROR LumaVibe::getTimestampFromNetwork(char timeStamp[21]) {
  // TODO: handle possible error
  int year, month, day, hour, minute, second;
  year = month = day = hour = minute = second = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (!_modem.getGsmLocationTime(&year, &month, &day, &hour, &minute, &second)) {
      PRINTS("\nCannot retrieve network time, defaulting to 0000-00-00T00:00:00Z");
      yield();
    }
    else {
      PRINTS("\nNetwork time retrieved");
      break;
    }
  }
  // 2020-03-26T18:37:00Z
  sprintf(timeStamp,"%04u-%02u-%02uT%02u:%02u:%02uZ", year, month, day, hour, minute, second);
  PRINTS("\ntimestamp:");
  Serial.println(String(timeStamp));
  keepAlive();
  return ERROR_NONE;
}

//
void LumaVibe::packArray(mpack_writer_t *writer, const char *entryNames[3], const uint16_t length) {
  PRINT("\nLength: ", length);
  for (uint8_t i = 0; i < 3; i++) {
    PRINTS("\nentry: "); PRINTS(entryNames[i]);
    mpack_write_cstr(writer, entryNames[i]);
    mpack_write_tag(writer, mpack_tag_make_array(length));
    for (uint16_t j = 0; j < length; j++) {
      mpack_write_i16(writer, _accelBuffer.data[j].v[i]);
    }
  }
}