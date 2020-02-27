/*
  LumaVibe V2.00
*/

// https://github.com/Xinyuan-LilyGO/TTGO-T-Call
// https://pubsubclient.knolleary.net/api.html for pubsubclient

#include <Arduino.h>
//#include <Wire.h>
//#include "custom_src/MMA8451.h"
#include "mpack.h"
#include <FastLED.h>
#include "MMA845XQ_Vibe.h"

//#define TGO_BOARD

// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "";//"iot.1nce.net";//""; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = "";//"7526"; // SIM card PIN code, if any

// Defines for TinyGSM library
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_USE_GPRS     true
#define TINY_GSM_USE_WIFI     false
#include <TinyGsmClient.h> // https://github.com/vshymanskyy/TinyGSM

// MQTT library
#include <PubSubClient.h>

#define MODEM_TX 27
#define MODEM_RX 26

#define INT_PIN   15 // Interrupt of the MMA8451
#define LED_DATA_PIN 32
#define NUM_LEDS 1

#define SerialAT  Serial1
#define SerialUSB Serial

#define TIMER_TIMEOUT_S (120*1000000)
#define BUFFER_SIZE     (2048)
#define PACKER_CAPACITY (40000)
#define WDT_TIMEOUT_S   (40*1000000)
#define MEASUREMENT_PER_MS (5)

TinyGsm       g_modem(SerialAT);
TinyGsmClient g_client(g_modem);
PubSubClient  g_mqtt(g_client);//mma8451_t     g_sensor;
MMA8451Q accel(0x1D);
CRGB leds[NUM_LEDS];

hw_timer_t *g_timer    = NULL;
hw_timer_t *g_wdt      = NULL;

static const char *broker   = "mastap.net";
static const char *topic    = "ngd/demo/lv2_001/data";
static const char *subscribeTopic = "ngd/demo/lv2_001/command";

static const char *moduleID = "LV020_0000001";
static const char *moduletype = "LumaVibe 2.0";
static const char *format = "Int16";
static const uint8_t msgtype = 2;
static const uint8_t freq = 200;
static const uint16_t numberOfMeas = BUFFER_SIZE;

volatile bool g_accelerometerInterruptFlag = false;
static bool g_timerFlag = false;

volatile uint16_t trans_threshold = 1;
volatile uint16_t trans_debcntr  = 2;


typedef struct {
  int16_t *x;
  int16_t *y;
  int16_t *z;
  const char *moduleID;
  const char *moduletype;
  const char *format;
  const uint8_t freg; /// freg? should be freq with a "q". "freg" never used?
  const uint16_t numberOfMeas;
} data_t;

void IRAM_ATTR accelerometerISR();

void setup() {
  startWatchDog();
  Wire.begin();
  Serial.begin(115200);
  Serial.println(F("LumaVibe V2 Starting ..."));

  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Red;
  FastLED.show();

  setupSensor();
  feedWatchDog();
  setupModem();
  feedWatchDog();

  g_mqtt.setServer(broker, 1883);

  Serial.println(F("Ininitalizing timer"));
  startTimer();
  Serial.println(F("Set up done!"));

  leds[0] = CRGB::Green;
  FastLED.show();

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(INT_PIN, accelerometerISR, FALLING);
  // Read to clear EA flag in case we recieved an interrupt in between setupSensor() and attachInterrupt
  uint8_t tr_src = accel.getTransientSource();
  endWatchDog();
}


void loop() {

  static uint8_t tr_src;

  if (g_accelerometerInterruptFlag || g_timerFlag) {

    startWatchDog();

    uint16_t trans_threshold_temp = trans_threshold;
    uint16_t trans_debcntr_temp = trans_debcntr;

    leds[0] = CRGB::Blue;
    FastLED.show();

    if (g_accelerometerInterruptFlag) {
      //uint8_t mo_src = accel.getMotionSource(); // Read to clear EA flag
      uint8_t res = accel._read_register(accel.INT_SOURCE);
      tr_src = accel.getTransientSource(); // Read to clear EA flag
      accel._PrintHex8(&res, 1);  Serial.print("\t");
      Serial.println(F("\nAcceleration interrupt"));
      //g_accelerometerInterruptFlag = false;
    }
    if (g_timerFlag) {
      Serial.println(F("\nTimer interrupt"));
      g_timerFlag = false;
    }

    endTimer();

    Serial.println();
    delay(100);

    Serial.print(F("capacity: ")); Serial.println((uint32_t)PACKER_CAPACITY);

    data_t *data = (data_t *)malloc(sizeof(*data));
    data->x      = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    data->y      = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    data->z      = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));

    for (uint16_t index = 0; index  < BUFFER_SIZE; index++) {
      uint32_t startTime = millis();
      while (millis() - startTime < MEASUREMENT_PER_MS);
      accel.update();
      //MMA8451_readData(&g_sensor);
      //data->x[index] = (g_sensor.data.x);
      //data->y[index] = (g_sensor.data.y);
      //data->z[index] = (g_sensor.data.z);
      data->x[index] = (accel._xi);
      data->y[index] = (accel._yi);
      data->z[index] = (accel._zi);

      if (index % 100 == 0) {
        Serial.print(F("Index: ")); Serial.print(index);
        Serial.print(F(" at: ")); Serial.println(startTime);
      }
    }
    Serial.print(F("Stop time: ")); Serial.println(millis());

    feedWatchDog();
    setupMQTT();
    feedWatchDog();

    String location = g_modem.getGsmLocation();
    Serial.print(F("location: ")); Serial.println(location);
    uint8_t index = location.lastIndexOf(',', 19);
    String timeStamp = location.substring(index + 1);
    timeStamp.replace('/', '-');
    timeStamp.replace(',', 'T');
    Serial.print(F("timestamp: ")); Serial.println(timeStamp);

    char *buffer = (char *)malloc(PACKER_CAPACITY);

    Serial.println(F("check 1"));
    mpack_writer_t writer;
    mpack_writer_init(&writer, buffer, PACKER_CAPACITY);
    mpack_start_map(&writer, 10);
    mpack_write_cstr(&writer, "timestamp"); mpack_write_cstr(&writer, timeStamp.c_str());
    mpack_write_cstr(&writer, "moduleID");  mpack_write_cstr(&writer, moduleID);
    mpack_write_cstr(&writer, "moduletype");  mpack_write_cstr(&writer, moduletype);
    mpack_write_cstr(&writer, "msgtype");   mpack_write_u8(&writer, 2);
    mpack_write_cstr(&writer, "format");    mpack_write_cstr(&writer, format);
    mpack_write_cstr(&writer, "freq");      mpack_write_u8(&writer, 200); // Measurement frequency not sampling! 200hz. Every 5ms????
    mpack_write_cstr(&writer, "numberOfMeas"); mpack_write_u16(&writer, BUFFER_SIZE);

    mpack_write_cstr(&writer, "x-accel");
    mpack_write_tag(&writer, mpack_tag_make_array(BUFFER_SIZE));
    Serial.println(F("check 2"));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
      mpack_write_i16(&writer, data->x[i]);
    }
    Serial.println(F("check 3"));
    mpack_write_cstr(&writer, "y-accel");
    mpack_write_tag(&writer, mpack_tag_make_array(BUFFER_SIZE));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
      mpack_write_i16(&writer, data->y[i]);
    }
    Serial.println(F("check 4"));
    mpack_write_cstr(&writer, "z-accel");
    mpack_write_tag(&writer, mpack_tag_make_array(BUFFER_SIZE));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
      mpack_write_i16(&writer, data->z[i]);
    }
    Serial.println(F("check 5"));

    size_t bytesSerialized = mpack_writer_buffer_used(&writer);
    mpack_finish_map(&writer);
    Serial.print(F("Bytes serialized: ")); Serial.println(bytesSerialized);
    Serial.println(F("check 6"));

    feedWatchDog();

    leds[0] = CRGB::Yellow;
    FastLED.show();

    publishMQTT((uint8_t *)buffer, bytesSerialized, 512);

    free(buffer);
    //buffer = NULL;
    free(data);
    feedWatchDog();

    //g_mqtt.setCallback(mqttCallback);
    g_mqtt.subscribe(subscribeTopic);
    Serial.println(F("topic subscribed"));
    Serial.println(F("checking for command.."));
    delay(2000);
    g_mqtt.loop();
    g_mqtt.loop();
    Serial.println(F("disconnecting mqtt"));
    g_mqtt.disconnect();

    Serial.print(F("MQTT connected: ")); Serial.println(g_mqtt.connected() ? "YES" : "NO");

    if ((trans_threshold != trans_threshold_temp) || (trans_debcntr != trans_debcntr_temp)) {
      Serial.println(F("Reconfiguring Threshold and Debounce Counter"));
      tr_src = accel.getTransientSource(); // Read to clear EA fla
      accel.setTransientThresholdN(trans_threshold, false); // 0 - 127 is 0 - 8g in 0.063g increments
      accel.setTransientDebounceCounter(trans_debcntr);
    }

    endWatchDog();

    leds[0] = CRGB::Green;
    FastLED.show();
    FastLED.delay(400); FastLED.clear();
    leds[0] = CRGB::Green;
    FastLED.show();
    FastLED.delay(400); FastLED.clear();
    leds[0] = CRGB::Green;
    FastLED.show();
    FastLED.delay(400); FastLED.clear();

    if (g_accelerometerInterruptFlag) {
      Serial.println(F("Reattaching interrupt"));
      g_accelerometerInterruptFlag = false;
      attachInterrupt(INT_PIN, accelerometerISR, FALLING);
      tr_src = accel.getTransientSource(); // Read to clear EA fla
    }
    startTimer();
  }
}

void updateSerial()
{
  //  delay(500);
  while (Serial.available())
  {
    SerialAT.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (SerialAT.available())
  {
    Serial.write(SerialAT.read());//Forward what Software Serial received to Serial Port
  }
}


void setupModem() {
  // Set-up modem reset, enable, power pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  g_modem.sendAT("AT"); // not necessary

  delay(3000);
  Serial.println(F("initializing modem"));
  bool ret = g_modem.restart(); // or modem.init();
  if (!ret) Serial.println(F("modem restart failed"));
  delay(3000);

  String modemInfo = g_modem.getModemInfo();
  Serial.print(F("Modem: "));
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && g_modem.getSimStatus() != 3 ) {
    Serial.println(F("Unlocking sim card"));
    bool ret = g_modem.simUnlock(simPIN);
    delay(1000);
    Serial.print(F("SIM status: "));
    uint8_t simStatus = g_modem.getSimStatus();
    Serial.println(ret);
  }
  Serial.print(F("SIM status: "));
  uint8_t simStatus = g_modem.getSimStatus();
  Serial.println(simStatus);
  String ccid = g_modem.getSimCCID();
  Serial.print(F("SIM CCID: "));
  Serial.println(ccid);
  String emei = g_modem.getIMEI();
  Serial.print(F("SIM IMEI: "));
  Serial.println(emei);
  String cop = g_modem.getOperator();
  Serial.print(F("SIM Operator: "));
  Serial.println(cop);

  Serial.print(F("Waiting for network..."));
  if (!g_modem.waitForNetwork(240000L)) {
    Serial.println(F(" fail"));
    delay(10000);
    return;
  }
  Serial.println(F(" OK"));

  if (g_modem.isNetworkConnected()) {
    Serial.println(F("Network connected"));
  }

  Serial.print(F("Connecting to APN: "));
  Serial.print(apn);
  if (!g_modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(F(" fail"));
    delay(10000);
    return;
  }

  if (g_modem.isGprsConnected()) {
    Serial.println(F("GPRS connected"));
  }
}

bool mqttConnect() {
  Serial.print(F("Connecting to mqtt broker: "));
  Serial.println(broker);

  // Connect to MQTT Broker
  bool status = g_mqtt.connect("sim800l", "user", "mqtt");

  if (status == false) {
    Serial.println(F(" fail"));
    return false;
  }
  Serial.println(F(" success"));
  return g_mqtt.connected();
}

void setupMQTT() {
  while (!g_modem.isGprsConnected()) {
    Serial.println(F("connecting to gprs"));
    g_modem.gprsConnect(apn, gprsUser, gprsPass);
    delay(1000);
  }
  while (!g_mqtt.connected()) {
    static long lastReconnectAttempt = 0;
    Serial.println(F("connecting to mqtt"));
    // Reconnect every 10 seconds
    unsigned long t = millis();
    //Serial.println(t, DEC);
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (!mqttConnect()) {
        lastReconnectAttempt = 0;
        Serial.println(F("Retrying.."));
      }
    }
    g_mqtt.setCallback(mqttCallback);
    Serial.println();
    Serial.println(F("mqtt connected"));
  }
}

void setupSensor() {
  Serial.println(F("Initializing MMA8451"));

  accel.SWreset(); // RESET THE REGS TO FACTORY DEFAULT FIRST!
  accel.setCommonParameters(accel.RANGE_4G, accel.RES_MAX, accel.LN_OFF, accel.DR_100, accel.OS_NORMAL, accel.HPF_OFF);
  // USE TRANSIENT DETECTION!
  accel.setTransientDetection();
  //accel.setTransientThresholdG(0.07, false);
  accel.setTransientThresholdN(trans_threshold, false);
  accel.setTransientDebounceCounter(trans_debcntr);
  accel.setHPFilterCutOff(3);

  //// Using INT_PIN 2 = INT1 - DO NOT USE WITH Vibe V2.00 !!!
  //accel.setInterrupt(accel.INT_EN_TRANS, accel.INT1, true); // Which event should raise an interrupt, and to which MMA8451Q pin is it routed (pins INT1 and INT2)
  // Using INT_PIN 15 = INT2
  accel.setInterrupt(accel.INT_EN_TRANS, accel.INT2, true); // Which event should raise an interrupt, and to which MMA8451Q pin is it routed (pins INT1 and INT2)

  uint8_t mo_src = accel.getMotionSource(); // Read to clear EA flag
  uint8_t tr_src = accel.getTransientSource(); // Read to clear EA flag
}

void IRAM_ATTR accelerometerISR() {
  detachInterrupt(INT_PIN);
  //MMA8451_getMotionSource(&g_sensor);
  g_accelerometerInterruptFlag = true;
  Serial.println(F("\nInterruptted"));
}

void IRAM_ATTR timerISR() {
  g_timerFlag = true;
}

void IRAM_ATTR resetModule() {
  Serial.println("\nrebooting\n");
  esp_restart();
}

void startTimer() {
  g_timer = timerBegin(0, 80, true); // 80 is correlated to fclk
  timerAttachInterrupt(g_timer, timerISR, true);
  timerAlarmWrite(g_timer, TIMER_TIMEOUT_S, true);

  // if the following yield() is removed, the timer will not be enabled the second time
  // REF: https://github.com/espressif/arduino-esp32/issues/1313
  yield();

  timerAlarmEnable(g_timer);
}

void startWatchDog() {
  g_wdt = timerBegin(1, 80, true);
  timerAttachInterrupt(g_wdt, &resetModule, true);
  timerAlarmWrite(g_wdt, WDT_TIMEOUT_S, false);

  // if the following yield() is removed, the timer will not be enabled the second time
  // REF: https://github.com/espressif/arduino-esp32/issues/1313
  yield();

  timerAlarmEnable(g_wdt);
}

void feedWatchDog() {
  timerWrite(g_wdt, 0);
}

void endWatchDog() {
  timerEnd(g_wdt);
  g_wdt = NULL;
}

void endTimer() {
  timerEnd(g_timer);
  g_timer = NULL;
}

void publishMQTT(const uint8_t *buffer, uint32_t bytesTotal, uint16_t bytesPerWrite) {
  Serial.println(F("Publishing.."));
  bool ret;
  ret = g_mqtt.beginPublish(topic, bytesTotal, false);
  Serial.print(F("ret")); Serial.println(ret ? " OK" : " Failed");
  uint8_t *pointerToBuffer = (uint8_t *)buffer;
  Serial.print(F("0. bytes remaining: ")); Serial.println(bytesTotal);
  while (bytesTotal) {
    uint16_t bytesToWrite = (bytesTotal > bytesTotal % bytesPerWrite ? bytesPerWrite : bytesTotal % bytesPerWrite);
    uint16_t wrote = g_mqtt.write(pointerToBuffer, bytesToWrite);
    bytesTotal -= wrote;
    pointerToBuffer += wrote;
    Serial.print(F("bytes remaining: ")); Serial.println(bytesTotal);
    g_mqtt.loop();
    yield();
  }
  ret = g_mqtt.endPublish();
  Serial.print(F("ret")); Serial.println(ret ? " OK" : " Failed");
  Serial.println(F("Done publishing"));
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println(F("message arrived"));
  Serial.println(topic);
  Serial.println();
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  mpack_reader_t reader;
  mpack_reader_init_data(&reader, (char *)payload, length);
  uint8_t count = mpack_expect_map_range(&reader, 3, 8);
  Serial.print("count"); Serial.println(count);

  mpack_expect_cstr_match(&reader, "sender");
  char sender[10];
  mpack_expect_cstr(&reader, sender, 10);

  mpack_expect_cstr_match(&reader, "trans_threshold");
  uint16_t mqtt_trans_threshold;
  mqtt_trans_threshold = mpack_expect_u16(&reader);

  mpack_expect_cstr_match(&reader, "trans_debcntr");
  uint16_t mqtt_trans_debcntr;
  mqtt_trans_debcntr = mpack_expect_u16(&reader);

  /*
    mpack_expect_cstr_match(&reader, "deadtime");
    uint16_t deadtime;
    deadtime = mpack_expect_u16(&reader);
    mpack_expect_cstr_match(&reader, "period");
    uint16_t period;
    period = mpack_expect_u16(&reader);
  */

  mpack_expect_cstr_match(&reader, "_msgid");
  char _msgid[20];
  mpack_expect_cstr(&reader, _msgid, 20);

  mpack_done_map(&reader);

  trans_threshold = mqtt_trans_threshold;
  trans_debcntr = mqtt_trans_debcntr;

  Serial.print("sender: "); Serial.println(sender);
  Serial.print("_msgid: "); Serial.println(_msgid);
  Serial.print("trans_threshold: "); Serial.println(trans_threshold);
  Serial.print("trans_debcntr: "); Serial.println(trans_debcntr);
}
