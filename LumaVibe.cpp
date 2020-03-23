#include "LumaVibe.h"

#define FCLK_DIVIDER          80
#define WATCHDOG_TIMER_NUMBER 1
#define SLEEP_TIMER_NUMBER    0
#define PACKER_CAPACITY       40000
#define BYTES_PER_WRITE       512

#define SerialAT  Serial1
#define SerialUSB Serial

#define MODEM_TX 27
#define MODEM_RX 26

static const struct {
  char *timestamp    = "timestamp";
  char *moduleID     = "moduleID";
  char *moduletype   = "moduletype";
  char *msgtype      = "msgtype";
  char *format       = "format";
  char *freq         = "freq";
  char *numberOfMeas = "numberOfMeas";
  char *x_accel      = "x-accel";
  char *y_accel      = "y-accel";
  char *z_accel      = "z-accel";
} StringToPack;

// Default Initializer
LumaVibe::LumaVibe() : 
  _accel(0x1D), 
  _modem(SerialAT), 
  _client(_modem), 
  _mqtt(_client) 
  {}

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
  if (_accelBuffer.isBufferAllocated) {
    clearMeasurementData();
  }
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

//
LumaVibe::ERROR LumaVibe::packData(uint32_t *bytesPacked) {
  _packBuffer = (char *)malloc(PACKER_CAPACITY);
  if (NULL == _packBuffer) {
    return ERROR_NOT_ENOUGH_MEMORY;
  }
  mpack_writer_init(&_writer, _packBuffer, PACKER_CAPACITY);
  mpack_start_map(&_writer, 10);
  String timeStamp;
  getTimestampFromNetwork(timeStamp);
  // TODO: refactor this shit, from here..
  mpack_write_cstr(&_writer, StringToPack.timestamp); 
  mpack_write_cstr(&_writer, timeStamp.c_str());
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
  const char * entries[] = {StringToPack.x_accel, StringToPack.y_accel, StringToPack.z_accel};
  packArray(&_writer, entries, _params.samplesPerMeasurement);

  *bytesPacked = mpack_writer_buffer_used(&_writer); // for debugging, add later

  keepAlive();
  return ERROR_NONE;
}

//
LumaVibe::ERROR LumaVibe::publishData(uint32_t bytesToPublish, uint16_t bytesPerWrite) {
  if (!_modem.isGprsConnected()) {
    if (!_modem.gprsConnect("", "", ""));
      return ERROR_MODEM_GPRS;
  }
  if (!_mqtt.connected()) {
    if (!_mqtt.connect("sim800l", "user", "mqtt"))
      return ERROR_MQTT;
  }
  keepAlive();
  if (_mqtt.beginPublish(_params.publishTopic, bytesToPublish, false)) {
    return ERROR_PUBLISH_BEGIN_FAIL;
  }
  uint8_t *pointerToBuffer = (uint8_t *)_packBuffer;
  while (bytesToPublish) {
    uint16_t bytesToWrite = (bytesToPublish > bytesToPublish % bytesPerWrite ? bytesPerWrite : bytesToPublish % bytesPerWrite);
    uint16_t bytesWritten = _mqtt.write(pointerToBuffer, bytesToWrite);
    bytesToPublish -= bytesWritten;
    pointerToBuffer += bytesWritten;
    _mqtt.loop();
    yield();
  }
  if (!_mqtt.endPublish()) {
    return ERROR_PUBLISH_END_FAIL;
  }

  free(_packBuffer);
  _packBuffer = NULL;
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
  _accel._read_register(MMA8451Q::INT_SOURCE);
  _accel.getTransientSource();
}

// 
void LumaVibe::dumpSimInfo() {
  Serial.print(F("SIM status: "));
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

//////////////////////////////////////////////////////////////////
///////////////////////* Private methods *///////////////////////
/////////////////////////////////////////////////////////////////

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
    _accelBuffer.data = NULL;
    _accelBuffer.isBufferAllocated = false;
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
  
  keepAlive();
  return ERROR_NONE;
}

//
LumaVibe::ERROR LumaVibe::getTimestampFromNetwork(String &timeStamp) {
  //TODO: Get rid of Arduino's String, handle possible errors
  String location = _modem.getGsmLocation();
  uint8_t index = location.lastIndexOf(',', 19);
  timeStamp = location.substring(index + 1);
  timeStamp.replace('/', '-');
  timeStamp.replace(',', 'T');

  return ERROR_NONE;
}

//
void LumaVibe::packArray(mpack_writer_t *writer, const char *entryNames[3], const uint16_t length) {
  for (uint8_t i = 0; i < 3; i++) {
    mpack_write_cstr(writer, entryNames[i]);
    mpack_write_tag(writer, mpack_tag_make_array(length));
    for (uint16_t j = 0; j < length; i++) {
      mpack_write_i16(writer, _accelBuffer.data[j].v[i]);
    }
  }
}