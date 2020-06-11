/*
Only RTC IO can be used as a source for external wake
source. They are pins: 0,2,4,12-15,25-27,32-39.
https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/DeepSleep/ExternalWakeUp/ExternalWakeUp.ino
*/

#include "LumaVibe.h"
#include "esp_attr.h"
#include <WiFi.h>
#include <Wire.h>
#include "debug_macros.h"
#include "soc/rtc_wdt.h"

#include "custom_src\phy\mma8451_i2c.h"
#include "custom_src\bsp\mma8451_registers.h"

#define MQTT_DATA_PACKBUFFER_SIZE  40000
#define MQTT_ERROR_PACKBUFFER_SIZE 1000

/*
// Global variables, extern-ed to main -------
*/
volatile bool g_accelInterruptFlag = false;
RTC_DATA_ATTR uint64_t g_bootCount = 0;
portMUX_TYPE  g_mux = portMUX_INITIALIZER_UNLOCKED;
//--------------------------------------------

/*
// static/private variables -------------------------------------
*/
static RTC_DATA_ATTR LumaVibe_Settings_t Settings;
static RTC_DATA_ATTR uint8_t  ErrorStream[ERROR_STREAM_SIZE];
static RTC_DATA_ATTR uint8_t  ErrorStreamWriter;
static               MMA8451_Data_t *AccelDataPtr = NULL;
static               char *PackBuffer = NULL;
static               bool _timeNeverSynced = true;

static MMA8451_t     accel;
static TinyGsm       modem(SerialAT);
static TinyGsmClient client(modem);
static PubSubClient  mqtt(client);
static CRGB          led[NUM_LEDS];
static hw_timer_t *  watchDogTimer = NULL;

static const struct {
  const char * const timestamp    = "timestamp";
  const char * const moduleID     = "moduleID";
  const char * const moduletype   = "moduletype";
  const char * const firmware     = "firmware";
  const char * const msgtype      = "msgtype";
  const char * const format       = "format";
  const char * const freq         = "freq";
  const char * const numberOfMeas = "numberOfMeas";
  const char * const range        = "range";
  const char * const divider      = "divider";
  const char * const x_accel      = "x-accel";
  const char * const y_accel      = "y-accel";
  const char * const z_accel      = "z-accel";
  const char * const interrupted  = "interrupted";
  const char * const bootcount    = "bootcount";
} StringToPack;

static const char * RangeStr[(uint8_t)MMA8451_RANGE_MAX] = { "2G", "4G", "8G"};
static const uint16_t DividerStr[(uint8_t)MMA8451_RANGE_MAX] = {
  (uint16_t)SCALE_FACTOR_2G_MODE, 
  (uint16_t)SCALE_FACTOR_4G_MODE, 
  (uint16_t)SCALE_FACTOR_8G_MODE
};
// -----------------------------------------------------------------------------

/*
// static/private functions -------------------------------------
*/
static void LumaVibe_dumpSimInfo();
static void LumaVibe_keepAlive();
static LumaVibe_Error_t LumaVibe_setupModem();
static LumaVibe_Error_t LumaVibe_syncTimeWithNetwork(time_t timeAtMeasure_s, char timeStamp[21]);
static LumaVibe_Error_t LumaVibe_connectMQTT();
static void LumaVibe_packArray(mpack_writer_t *writer, const char *entryNames[3], const uint16_t length);
static LumaVibe_Error_t LumaVibe_publish(const char *publishTopic, uint32_t bytesToPublish, uint16_t bytesPerWrite);
static void mqttCallback(char* topic, byte* payload, unsigned int length);
static void LumaVibe_delay(uint64_t milliSeconds);

/*
// Interrupt(s) -------------------------------------
*/
static void watchDogISR(void) {
  esp_restart();
}


/*
// Public/Extern-ed functions 
*/

// TTGO-specific functions for power and modem
#ifdef TTGO
// https://electronics.stackexchange.com/questions/287418/sim800-pwrkey-automatic-start
// https://github.com/Xinyuan-LilyGO/LilyGo-T-Call-SIM800L/blob/master/datasheet/SIM800_Hardware%20Design_V1.08.pdf
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool LumaVibe_setPowerBoostKeepOn(bool en)
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

void LumaVibe_hardResetModem() {
  digitalWrite(MODEM_RST, LOW);
  LumaVibe_delay(200); // must be more than 105ms
  digitalWrite(MODEM_RST, HIGH);
}

void LumaVibe_disableModem() {
  digitalWrite(MODEM_RST, LOW);
  digitalWrite(MODEM_PWKEY, HIGH);
  digitalWrite(MODEM_POWER_ON, LOW);
}

void LumaVibe_enableModem() {
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
}
#endif // TTGO

#ifdef ZHAGA
  bool LumaVibe_setPowerBoostKeepOn(bool en) {
    return true;
  }
  void LumaVibe_hardResetModem() {
    digitalWrite(MODEM_RST, HIGH);
    LumaVibe_delay(200);
    digitalWrite(MODEM_RST, LOW);
  }
  void LumaVibe_disableModem() {
    return;
  }
  void LumaVibe_enableModem() {
    return;
  }
#endif

//
LumaVibe_Error_t LumaVibe_init(LumaVibe_Settings_t * const s) {
  rtc_wdt_disable();
  if (0 == s->sleepTime_ms || 0 == s->watchDogTime_ms) {
    return LUMAVIBE_ERROR_TIME_ZERO;
  }
  if (0 == s->measureFrequency || s->measureFrequency > 1000) {
    return LUMAVIBE_ERROR_FREQUENCY;
  }

  watchDogTimer = timerBegin(WATCHDOG_TIMER_NUMBER, FCLK_DIVIDER, true);
  if (NULL == watchDogTimer) return LUMAVIBE_ERROR_TIMER_NULL;
  timerAttachInterrupt(watchDogTimer, watchDogISR, true);
  timerAlarmWrite(watchDogTimer, s->watchDogTime_ms * 1000, false);
  timerAlarmEnable(watchDogTimer);

  PRINTF("g_bootCount: %lu\r\n", (long)g_bootCount);
  if (0 == g_bootCount) {
    timeval rtcTime = {0, 0};
    settimeofday(&rtcTime, NULL); // set rtc time to POSIX-zero
  }

  memcpy(&Settings, s, sizeof(LumaVibe_Settings_t));

  esp_err_t err = MMA8451_init(&accel, s->accelSCL, s->accelSDA, s->accelAddress);
  if (ESP_OK != err) return LUMAVIBE_ERROR_SENSOR_INIT;

  mqtt.setServer(Settings.mqttBroker, 1883);
  PRINTF("broker: %s\r\n", s->mqttBroker);

  // Turn of Bluetooth and Wifi
  btStop();
  WiFi.mode(WIFI_OFF);
  
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS); // CAUTION
  FastLED.setBrightness(30);
    
  return LUMAVIBE_ERROR_NONE;
}

// Acceleration settings should only be changed on the node-red side
// Don't set ODR less than 100Hz
LumaVibe_Error_t LumaVibe_begin() {
  uint8_t whoami = 0xFF;
  MMA8451_I2C_readReg8(accel.i2cAddress, MMA8451_REG_WHOAMI, &whoami, 1);
  PRINTF("who am i: %d\r\n", whoami);
  if (MMA8451_standby(&accel) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_setOutputDataRate(&accel, MMA8451_DATA_RATE_800Hz) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_setHpfCutOff(&accel, MMA8451_HPF_CUTOFF_2Hz_ODR_800Hz) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_setLowNoiseMode(&accel, MMA8451_LOWNOISE_ON) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_setRange(&accel, MMA8451_RANGE_2G) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_setTransientThreshold_mG(&accel, 5 * TRANS_THS_mG_per_COUNT) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_setTransientDebounceCounter(&accel, 3) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_setOverSamplingMode(&accel, MMA8451_OVERSAMPLING_NORMAL) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_enableTransientDetection(&accel, false) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_enableTransientInterrupt(&accel, false, false)) return LUMAVIBE_ERROR_SENSOR_SETTING;
  if (MMA8451_active(&accel) != ESP_OK) return LUMAVIBE_ERROR_SENSOR_SETTING;

  PRINTF("accel range: %d\r\n", accel.params.range);

  gpio_config_t ioconfig;
  ioconfig.pin_bit_mask = (BIT(Settings.accelInterruptPin));
  ioconfig.mode = GPIO_MODE_INPUT;
  ioconfig.pull_up_en = GPIO_PULLUP_ENABLE;
  ioconfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ioconfig.intr_type = GPIO_INTR_NEGEDGE;
  gpio_config(&ioconfig);
  gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

  LumaVibe_keepAlive();
  return LUMAVIBE_ERROR_NONE;
}

//
LumaVibe_Error_t LumaVibe_measure(time_t *timeAtMeasure_s) {
  if (NULL != AccelDataPtr) 
    free(AccelDataPtr);
  AccelDataPtr = (MMA8451_Data_t *)malloc(Settings.numberOfMeas * sizeof(MMA8451_Data_t));
  if (NULL == AccelDataPtr) {
    return LUMAVIBE_ERROR_NOT_ENOUGH_MEMORY;
  }
  struct tm now;
  getLocalTime(&now, 0);
  *timeAtMeasure_s = mktime(&now);

  uint8_t measureInterval_ms = 1000 / Settings.measureFrequency;
  MMA8451_Data_t data;
  uint32_t startTime = millis();
  for (uint16_t index = 0; index  < Settings.numberOfMeas; index++) {
    while (millis() - startTime < measureInterval_ms) {
      LumaVibe_keepAlive();
    }
    startTime = millis();
    MMA8451_readData(&accel, &data);
    AccelDataPtr[index].xi = data.xi;
    AccelDataPtr[index].yi = data.yi;
    AccelDataPtr[index].zi = data.zi;

    if (index % 100 == 0) {
      PRINT("Index: ", index);
      PRINTLN(" at: ", startTime);
    }
  }
  PRINTF("Stop time: %lu\r\n", millis());
  LumaVibe_keepAlive();
  return LUMAVIBE_ERROR_NONE;
}

// 
LumaVibe_Error_t LumaVibe_packData(time_t timeAtMeasure_s, uint32_t *bytesPacked) {
  LumaVibe_Error_t err = LumaVibe_setupModem();
  if (LUMAVIBE_ERROR_NONE != err) {
    return err;
  }

  PackBuffer = (char *)malloc(MQTT_DATA_PACKBUFFER_SIZE);
  if (NULL == PackBuffer) {
    return LUMAVIBE_ERROR_NOT_ENOUGH_MEMORY;
  }
  PRINTS("Packing header\r\n");
  mpack_writer_t writer;
  mpack_writer_init(&writer, PackBuffer, MQTT_DATA_PACKBUFFER_SIZE);
  mpack_start_map(&writer, 15);
  char timeStamp[21] = {0};
  LumaVibe_syncTimeWithNetwork(timeAtMeasure_s, timeStamp);
  // TODO: refactor this, from here..
  mpack_write_cstr(&writer, StringToPack.timestamp);    mpack_write_cstr(&writer, timeStamp);
  mpack_write_cstr(&writer, StringToPack.moduleID);     mpack_write_cstr(&writer, Settings.moduleID);
  mpack_write_cstr(&writer, StringToPack.moduletype);   mpack_write_cstr(&writer, Settings.moduleType);
  mpack_write_cstr(&writer, StringToPack.firmware);     mpack_write_cstr(&writer, Settings.firmwareVersion);
  mpack_write_cstr(&writer, StringToPack.msgtype);      mpack_write_u8(&writer,   Settings.msgType);
  mpack_write_cstr(&writer, StringToPack.format);       mpack_write_cstr(&writer, Settings.format);
  mpack_write_cstr(&writer, StringToPack.freq);         mpack_write_u16(&writer,  Settings.measureFrequency);
  mpack_write_cstr(&writer, StringToPack.numberOfMeas); mpack_write_u16(&writer,  Settings.numberOfMeas);
  mpack_write_cstr(&writer, StringToPack.interrupted);  mpack_write_bool(&writer, g_accelInterruptFlag);
  mpack_write_cstr(&writer, StringToPack.range);        mpack_write_cstr(&writer, RangeStr[(uint8_t)accel.params.range]);
  mpack_write_cstr(&writer, StringToPack.divider);      mpack_write_u16(&writer, DividerStr[(uint8_t)accel.params.range]);
  mpack_write_cstr(&writer, StringToPack.bootcount);    mpack_write_u32(&writer, (uint32_t)g_bootCount);

  // ..to here
  PRINTF("Packing array of %d elements\r\n", Settings.numberOfMeas);
  const char *entries[] = {StringToPack.x_accel, StringToPack.y_accel, StringToPack.z_accel};
  LumaVibe_packArray(&writer, entries, Settings.numberOfMeas);
  *bytesPacked = mpack_writer_buffer_used(&writer);
  mpack_finish_map(&writer);
  if (mpack_ok != mpack_writer_destroy(&writer)) {
    PRINTS("Error destroying writer\r\n");
    return LUMAVIBE_ERROR_PACKING_NOT_FINISHED;
  }

  free(AccelDataPtr); AccelDataPtr = NULL;
  LumaVibe_keepAlive();
  return LUMAVIBE_ERROR_NONE;
}

//
LumaVibe_Error_t LumaVibe_publishData(const char *publishDataTopic, uint32_t bytesToPublish, uint16_t bytesPerWrite) {
  return LumaVibe_publish(publishDataTopic, bytesToPublish, bytesPerWrite);
}

//
void LumaVibe_getCommandsFromServer(const char *subscribeTopic) {
  mqtt.setCallback(mqttCallback);
  mqtt.subscribe(subscribeTopic);
  LumaVibe_keepAlive();
  LumaVibe_delay(2000);
  mqtt.loop();
  LumaVibe_delay(2000);
  LumaVibe_keepAlive();
  mqtt.loop();
  mqtt.disconnect();
}

//
void LumaVibe_clearAccelInterrupt() {
  uint8_t tranSrc;
  MMA8451_readTransientSource(&accel, &tranSrc);
}

//
void LumaVibe_restart() {
  esp_restart();
}

// 
void LumaVibe_logError(LumaVibe_Error_t error, uint16_t line) {
  ErrorStream[ErrorStreamWriter++] = (uint8_t)error;

  char errorString[40];
  sprintf(errorString, "Error: %u at line: %u\r\n", error, line);
  PRINTS(errorString);
}


//
uint8_t LumaVibe_countNetworkError(void) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < ErrorStreamWriter; i++) {
    if ((uint8_t)LUMAVIBE_ERROR_MODEM_NETWORK_NOT_CONNECTED == ErrorStream[i] ||
        (uint8_t)LUMAVIBE_ERROR_MQTT_NOT_CONNECTED == ErrorStream[i] ||
        (uint8_t)LUMAVIBE_ERROR_MODEM_GPRS_NOT_CONNECTED == ErrorStream[i]) {
      count++;
    }
  }
  return count;
}

//
LumaVibe_Error_t LumaVibe_packError(uint32_t *bytesPacked) {
  // CAUTION: setupModem() not called (should've been called in packData())
  // Think of a better flow
  PackBuffer = (char *)malloc(MQTT_ERROR_PACKBUFFER_SIZE);
  if (NULL == PackBuffer) {
    return LUMAVIBE_ERROR_NOT_ENOUGH_MEMORY;
  }
  PRINTS("Packing error header\r\n");
  mpack_writer_t writer;
  mpack_writer_init(&writer, PackBuffer, MQTT_ERROR_PACKBUFFER_SIZE);
  mpack_start_map(&writer, 3);
  mpack_write_cstr(&writer, StringToPack.moduleID); mpack_write_cstr(&writer, Settings.moduleID);
  mpack_write_cstr(&writer, StringToPack.firmware); mpack_write_cstr(&writer, Settings.firmwareVersion);
  mpack_write_cstr(&writer, "Error");
  mpack_write_tag(&writer, mpack_tag_make_array(ErrorStreamWriter));
  for (uint8_t i = 0; i < ErrorStreamWriter; i++) {
    mpack_write_u8(&writer, ErrorStream[i]);
  }
  *bytesPacked = mpack_writer_buffer_used(&writer);
  if (mpack_ok != mpack_writer_destroy(&writer)) {
    PRINTS("Error destroying writer\r\n");
    return LUMAVIBE_ERROR_PACKING_NOT_FINISHED;
  }
  LumaVibe_keepAlive();
  return LUMAVIBE_ERROR_NONE;
}

//
LumaVibe_Error_t LumaVibe_publishError(const char *publishErrorTopic, int32_t bytesToPublish, uint16_t bytesPerWrite) {
  return LumaVibe_publish(publishErrorTopic, bytesToPublish, bytesPerWrite);
}

//
uint8_t LumaVibe_countError(void) {
  return ErrorStreamWriter;
}

//
void LumaVibe_clearError(void) {
  memset(ErrorStream, 0, ERROR_STREAM_SIZE);
  ErrorStreamWriter = 0;
}

//
void LumaVibe_detachAccelInterrupt() {
  gpio_intr_disable((gpio_num_t)Settings.accelInterruptPin);
}

//
void LumaVibe_enableAccelInterrupt() {
  gpio_intr_enable((gpio_num_t)Settings.accelInterruptPin);
}

//
void LumaVibe_setPeriod(uint32_t period_s) {
  Settings.sleepTime_ms = period_s * 1000;
}

//
// TODO: Handle errors in here
void LumaVibe_goToSleep(void) {
  LumaVibe_keepAlive();
  LumaVibe_clearLED();
  PRINTS("Going to sleep..\r\n");
  PRINTF("Sleep time: %lu\r\n", (long)Settings.sleepTime_ms);
  if (modem.isGprsConnected()) {
    if (modem.gprsDisconnect()) {
      PRINTS("GPRS disconnected\r\n");
    }
  }
  if (btStop()) {
    PRINTS("Bluetooth stopped\r\n");
  }
  if (WiFi.mode(WIFI_OFF)) {
    PRINTS("Wifi off\r\n");
  }
#ifdef TTGO
  if (modem.sleepEnable()) {
    PRINTS("Modem put to sleep\r\n");
  }
  if (LumaVibe_setPowerBoostKeepOn(false)) {
    PRINTS("Power boost turned off\r\n");
  }
  LumaVibe_disableModem();
#endif
  LumaVibe_keepAlive();
  PRINTS("Goodnight!\r\n");
  // SerialUSB.flush();
  SerialAT.end();
  delay(3000);
  LumaVibe_keepAlive();
  //SerialUSB.end();

  LumaVibe_enableAccelInterrupt();
  esp_sleep_enable_timer_wakeup(Settings.sleepTime_ms * 1000);
  gpio_wakeup_enable((gpio_num_t)Settings.accelInterruptPin, GPIO_INTR_LOW_LEVEL); // edge interrupt not supported
  esp_sleep_enable_gpio_wakeup();
  timerAlarmDisable(watchDogTimer);
  
  esp_light_sleep_start();
  LumaVibe_detachAccelInterrupt();
  timerAlarmEnable(watchDogTimer);
  LumaVibe_keepAlive();
  g_bootCount++;
}

//
void LumaVibe_setLED(CRGB::HTMLColorCode color) {
  led[0] = color;
  FastLED.show();
}

//
void LumaVibe_clearLED() {
  FastLED.clear();
  FastLED.show(); // This puts clear() into effect
}

void LumaVibe_flashLED(CRGB::HTMLColorCode color, uint16_t duration_ms, uint8_t numberOfTimes, bool retainColor) {
  if (0 != duration_ms && 0 != numberOfTimes) {
    while(numberOfTimes) {
      LumaVibe_setLED(color);
      FastLED.delay(duration_ms);
      LumaVibe_clearLED();
      FastLED.delay(duration_ms);
      numberOfTimes--;
    }
  }
  if (retainColor) {
    led[0] = color;
    FastLED.show();
  }
}

//
void LumaVibe_endWatchdog(){
  timerEnd(watchDogTimer);
}


//////////////////////////////////////////////////////////////////
///////////////////////* Private methods *///////////////////////
/////////////////////////////////////////////////////////////////

// 
static void LumaVibe_dumpSimInfo() {
  Serial.print(F("SIM status: "));
  uint8_t simStatus = modem.getSimStatus();
  Serial.println(simStatus);
  String ccid = modem.getSimCCID();
  Serial.print(F("SIM CCID: "));
  Serial.println(ccid);
  String emei = modem.getIMEI();
  Serial.print(F("SIM IMEI: "));
  Serial.println(emei);
  String cop = modem.getOperator();
  Serial.print(F("SIM Operator: "));
  Serial.println(cop);
}

//
static void LumaVibe_keepAlive() {
  timerWrite(watchDogTimer, 0);
  delay(1);
}

//
static LumaVibe_Error_t LumaVibe_setupModem() {
  LumaVibe_setPowerBoostKeepOn(true);
  LumaVibe_enableModem();
  PRINTS("Setting up modem\r\n");
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  LumaVibe_delay(3000);
  PRINTS("waiting for network...\r\n");
  uint32_t start = millis();
  while (!modem.isNetworkConnected() && millis() - start < 30000) {
    LumaVibe_keepAlive();
  }
  if (!modem.isNetworkConnected())
    return LUMAVIBE_ERROR_MODEM_NETWORK_NOT_CONNECTED;
  PRINTS("network connected...\r\n");
  LumaVibe_keepAlive();
  PRINTS("connecting to GPRS..\r\n");
  uint8_t gprsTry = 0;
  do {
    PRINT("retry..", gprsTry);
    modem.gprsConnect("", "", "");
    LumaVibe_delay(1000);
    gprsTry++;
  } while (!modem.isGprsConnected() && gprsTry < 5); // ATTENTION: WHILE LOOP!!
  if (!modem.isGprsConnected()) {
    return LUMAVIBE_ERROR_MODEM_GPRS_NOT_CONNECTED;
  }
  PRINTS("\nGPRS connected\r\n");

  int16_t signalQuality = modem.getSignalQuality();
  PRINTLN("Signal quality: ", signalQuality);
  
  LumaVibe_keepAlive();
  return LUMAVIBE_ERROR_NONE;
}

//
static LumaVibe_Error_t LumaVibe_syncTimeWithNetwork(time_t timeAtMeasure_s, char timeStamp[21]) {
  // TODO: handle possible error
  // Month of time coming from network is the actual month, month of struct tm is "months since January"
  tm timeBuffer;
  bool isNetWorkTimeReceived = false;
  uint8_t syncTry = 3;
  PRINTS("Syncing time with network\r\n");
  do {
    isNetWorkTimeReceived = modem.getGsmLocationTime(&timeBuffer.tm_year, &timeBuffer.tm_mon, &timeBuffer.tm_mday,
                                                      &timeBuffer.tm_hour, &timeBuffer.tm_min, &timeBuffer.tm_sec);
    LumaVibe_keepAlive();
  } while (false == isNetWorkTimeReceived && syncTry-->0);
  
  if (false == isNetWorkTimeReceived) {
    // Set exported time to be zero
    if (_timeNeverSynced) {
      memset(&timeBuffer, 0, sizeof(timeBuffer));
    } 
    else {
      timeBuffer = *localtime(&timeAtMeasure_s);
    }
    sprintf(timeStamp,"%04d-%02d-%02dT%02d:%02d:%02dZ", timeBuffer.tm_year + 1900, timeBuffer.tm_mon + 1, timeBuffer.tm_mday, 
                                                        timeBuffer.tm_hour, timeBuffer.tm_min, timeBuffer.tm_sec);
  }
  else {
    PRINTS("Network time retrieved: \r\n");
    PRINTF("%04d-%02d-%02dT%02d:%02d:%02dZ\r\n", timeBuffer.tm_year, timeBuffer.tm_mon, timeBuffer.tm_mday, 
                                             timeBuffer.tm_hour, timeBuffer.tm_min, timeBuffer.tm_sec);
    struct tm now;
    getLocalTime(&now, 0);
    time_t now_s = mktime(&now); 

    timeBuffer.tm_year -= 1900; // Convert from human year to POSIX year
    timeBuffer.tm_mon -= 1; // month of struct tm is supposed to be "months since January"
    time_t networkTime_s = mktime(&timeBuffer); // POSIX time in sec
    time_t time_s = networkTime_s - (now_s - timeAtMeasure_s); // POSIX time in sec at 1st measurement
    PRINTLN("Network time in sec: ", networkTime_s);
    tm shiftedTime = *localtime(&time_s); // convert from time_t back to struct tm

    // Export time for further use (i.e. packing). Format: 2020-03-26T18:37:00Z
    sprintf(timeStamp,"%04d-%02d-%02dT%02d:%02d:%02dZ", shiftedTime.tm_year + 1900, shiftedTime.tm_mon + 1, shiftedTime.tm_mday, 
                                                        shiftedTime.tm_hour, shiftedTime.tm_min, shiftedTime.tm_sec);
    PRINTF("Time at measurement: %04d-%02d-%02dT%02d:%02d:%02dZ\r\n", shiftedTime.tm_year, shiftedTime.tm_mon + 1, shiftedTime.tm_mday, 
                                                                    shiftedTime.tm_hour, shiftedTime.tm_min, shiftedTime.tm_sec);
    timeval rtcTime = {networkTime_s, 0}; 
    settimeofday(&rtcTime, NULL); // Update RTC time value
    if (_timeNeverSynced) {
      _timeNeverSynced = false;
    }
  }
  
  PRINTF("timestamp: %s\r\n", timeStamp);
  LumaVibe_keepAlive();
  return LUMAVIBE_ERROR_NONE;
}

//
static LumaVibe_Error_t LumaVibe_connectMQTT() {
  uint8_t mqttTry = 0;
  do {
    PRINTS("Connecting MQTT\r\n");
    PRINT("..", mqttTry);
    mqtt.connect("sim800l", "user", "mqtt");
    //mqtt.connect(Settings.moduleID, Settings.mqttUserName, Settings.mqttPassword);
    PRINTLN("\nMQTT state: ", mqtt.state()); // Should be 0
    mqttTry++;
    yield();
    LumaVibe_keepAlive();
  } while (!mqtt.connected() && mqttTry < 5);
  if (!mqtt.connected()) {
    return LUMAVIBE_ERROR_MQTT_NOT_CONNECTED; 
  }
  return LUMAVIBE_ERROR_NONE;
}

//
static void LumaVibe_packArray(mpack_writer_t *writer, const char *entryNames[3], const uint16_t length) {
  PRINTLN("Length: ", length);
  for (uint8_t i = 0; i < 3; i++) {
    PRINTLN("entry: ", entryNames[i]);
    mpack_write_cstr(writer, entryNames[i]);
    mpack_write_tag(writer, mpack_tag_make_array(length));
    for (uint16_t j = 0; j < length; j++) {
      mpack_write_i16(writer, AccelDataPtr[j].v[i]);
    }
  }
}

static void LumaVibe_delay(uint64_t milliSeconds) {
  uint32_t start = millis();
  while (millis() - start < milliSeconds) {
    LumaVibe_keepAlive();
  }
}

// 
static LumaVibe_Error_t LumaVibe_publish(const char *publishTopic, uint32_t bytesToPublish, uint16_t bytesPerWrite) {
  PRINTLN("Bytes to be published: ", bytesToPublish);
  LumaVibe_Error_t err = LumaVibe_connectMQTT();
  if (LUMAVIBE_ERROR_NONE != err) {
    return err;
  }
  if(!mqtt.beginPublish(publishTopic, bytesToPublish, false)) {
    return LUMAVIBE_ERROR_PUBLISH_BEGIN_FAIL;
  }
  LumaVibe_keepAlive();
  PRINTS("Publishing...\r\n");
  PRINTLN("Bytes Total: ", bytesToPublish);
  uint8_t *pointerToBuffer = (uint8_t *)PackBuffer;
  while (bytesToPublish) {
    // PRINTLN("Bytes left: ", bytesToPublish);
    uint16_t bytesToWrite = (bytesToPublish > bytesToPublish % bytesPerWrite ? bytesPerWrite : bytesToPublish % bytesPerWrite);
    uint16_t bytesWritten = mqtt.write(pointerToBuffer, bytesToWrite);
    bytesToPublish -= bytesWritten;
    pointerToBuffer += bytesWritten;
    mqtt.loop();
    LumaVibe_keepAlive();
    if (MQTT_CONNECTED != mqtt.state()) {
      return LUMAVIBE_ERROR_MQTT_NOT_CONNECTED;
    }
  }
  if (!mqtt.endPublish()) {
    return LUMAVIBE_ERROR_PUBLISH_END_FAIL;
  }

  free(PackBuffer);   PackBuffer = NULL;
  free(AccelDataPtr); AccelDataPtr = NULL;
  PRINTS("Publishing done\r\n");
  return LUMAVIBE_ERROR_NONE;
}

//
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  PRINTS("Message arrived\r\n");
  PRINTVAL(topic);
  PRINTVAL("");
  for (int i = 0; i < length; i++) {
    PRINTVAL((char)payload[i]);
  }
  PRINTVAL("");
  mpack_reader_t reader;
  mpack_reader_init_data(&reader, (char *)payload, length);
  uint8_t count = mpack_expect_map_range(&reader, 3, 8);
  PRINTLN("Number of fields: ", count);

  mpack_expect_cstr_match(&reader, "sender");
  char sender[15];
  mpack_expect_cstr(&reader, sender, 15);

  mpack_expect_cstr_match(&reader, "trans_threshold");
  uint16_t transientThreshold;
  transientThreshold = mpack_expect_u16(&reader);

  mpack_expect_cstr_match(&reader, "trans_debcntr");
  uint16_t transientDbCtr;
  transientDbCtr = mpack_expect_u16(&reader);

  /*
    mpack_expect_cstr_match(&reader, "deadtime");
    uint16_t deadtime;
    deadtime = mpack_expect_u16(&reader);
  */
  mpack_expect_cstr_match(&reader, "period");
  uint64_t period;
  period = mpack_expect_u16(&reader);

  mpack_expect_cstr_match(&reader, "range");
  uint16_t range;
  range = mpack_expect_u16(&reader);

  mpack_expect_cstr_match(&reader, "lownoise");
  bool lownoise;
  lownoise = mpack_expect_bool(&reader);

  mpack_expect_cstr_match(&reader, "_msgid");
  char _msgid[20];
  mpack_expect_cstr(&reader, _msgid, 20);

  mpack_done_map(&reader);

  MMA8451_standby(&accel);
  MMA8451_setTransientThreshold_mG(&accel, transientThreshold);
  MMA8451_setTransientDebounceCounter(&accel, transientDbCtr);
  MMA8451_setLowNoiseMode(&accel, lownoise ? MMA8451_LOWNOISE_ON : MMA8451_LOWNOISE_OFF);
  MMA8451_setRange(&accel, (MMA8451_Range_t)range);
  MMA8451_active(&accel);

  LumaVibe_setPeriod(period);

  PRINTLN("sender: ", sender);
  PRINTLN("_msgid: ", _msgid);
  PRINTLN("received trans_threshold (mG): ", transientThreshold);
  PRINTLN("received trans_debcntr: ", transientDbCtr);
  PRINTLN("received period: ", (long)period);
  PRINTLN("received range: ", range);
  PRINTLN("accel threshold (mg): ", accel.params.transientThresholdCount * TRANS_THS_mG_per_COUNT);
  PRINTLN("accel debcntr:", accel.params.transientDebouncCount);
  PRINTLN("accel range: ", accel.params.range);
}
