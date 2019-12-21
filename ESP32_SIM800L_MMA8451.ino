// https://github.com/Xinyuan-LilyGO/TTGO-T-Call

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_MMA8451.h>
//#include <ArduinoJson.h>
#include "custom_src/MMA8451.h"
#include "custom_src/mpack.h"

// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "";//"iot.1nce.net";//""; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = "";//"7526"; // SIM card PIN code, if any

// Defines for TinyGSM library
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_USE_GPRS     true
#define TINY_GSM_USE_WIFI     false
#include <TinyGsmClient.h>

// MQTT library
#include <PubSubClient.h>

// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

#define SerialAT  Serial1
#define SerialUSB Serial
#define LED_PIN   13
#define STACK_SIZE 200

// Defines for testing
#define TEST_MQTT  false
#define TEST_TIMER true
#define TEST_CORE  false

TinyGsm       modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
mma8451_t sensor;
bool intFlag = false;
bool timerSet = false;

static int32_t timerCounter = 0;

//StaticTask_t xTaskBuffer;
//StackType_t xStack[ STACK_SIZE ];
xTaskHandle core0TaskHandle;

static const char *broker = "mastap.net";
static const uint8_t INT_PIN = 14;
static long lastReconnectAttempt = 0;
static bool apnConnected = false;

//Adafruit_MMA8451 mma = Adafruit_MMA8451();
hw_timer_t *timer    = NULL;
hw_timer_t *wdt      = NULL;
const uint32_t wdtTimeout_s = 40*1000000;

const uint16_t BUFFER_SIZE = 2048;
const size_t capacity = 40000; //4*JSON_ARRAY_SIZE(BUFFER_SIZE) + JSON_OBJECT_SIZE(5) + 128;
//static char buffer[capacity];

#define I2C_SDA                     21
#define I2C_SCL                     22
#define IP5306_ADDR                 0x75
#define IP5306_REG_SYS_CTL0         0x00

bool setPowerBoostKeepOn(int en)
{
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_SYS_CTL0);
    if (en)
        Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
    else
        Wire.write(0x35); // 0x37 is default reg value
    return Wire.endTransmission() == 0;
}

void IRAM_ATTR onTimer() {
  timerCounter++;
  timerSet = true;
}

void IRAM_ATTR resetModule() {
  Serial.println("\nrebooting\n");
  esp_restart();
}

void startTimer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, onTimer, true);
  timerAlarmWrite(timer, 120*1000000, true);
  timerAlarmEnable(timer);
}

void startWatchDog() {
  wdt = timerBegin(1, 80, true);
  timerAttachInterrupt(wdt, &resetModule, true);
  timerAlarmWrite(wdt, wdtTimeout_s, false);
  timerAlarmEnable(wdt);
}

void feedWatchDog() {
  timerWrite(wdt, 0);
}

void endWatchDog() {
  timerEnd(wdt);
  wdt = NULL;
}

void endTimer() {
  timerEnd(timer);
  timer = NULL; 
}

void IRAM_ATTR accInterrupt() {
  MMA8451_getMotionSource(&sensor);
  detachInterrupt(INT_PIN);
  intFlag = true;
  Serial.println(F("\nInterruptted"));
}

void core0Test(void *parameter) {
  for (;;) {
    Serial.print(F("This task runs on core: "));
    Serial.println(xPortGetCoreID());
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}

bool mqttConnect() {
  Serial.print(F("Connecting to mqtt broker: "));
  Serial.println(broker);

  // Connect to MQTT Broker
  bool status = mqtt.connect("sim800l", "user", "mqtt");
  //bool status = mqtt.connect("testtesttest"));

  if (status == false) {
    Serial.println(F(" fail"));
    return false;
  }
  Serial.println(F(" success"));
  return mqtt.connected();
}

void setupModem() {
    // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  Serial.println(F("initializing modem"));
  bool ret = modem.restart(); // or modem.init();
  if (!ret) Serial.println(F("modem restart failed"));
  delay(3000);

  String modemInfo = modem.getModemInfo();
  Serial.print(F("Modem: "));
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    Serial.println(F("Unlocking sim card"));
    bool ret = modem.simUnlock(simPIN);
    delay(1000);
    Serial.print(F("SIM status: "));
    uint8_t simStatus = modem.getSimStatus();
    Serial.println(ret);
  }
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

  Serial.print(F("Waiting for network..."));
  if (!modem.waitForNetwork(240000L)) {
    Serial.println(F(" fail"));
    delay(10000);
    return;
  }
  Serial.println(F(" OK"));

  if (modem.isNetworkConnected()) {
    Serial.println(F("Network connected"));
  }

  Serial.print(F("Connecting to APN: "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(F(" fail"));
    delay(10000);
    return;
  }

  if (modem.isGprsConnected()) {
    Serial.println(F("GPRS connected"));
  }
}

void setupSensor() {
  sensor.i2cAddress = MMA8451_DEFAULT_ADDRESS;
  bool ret = MMA8451_begin(&sensor);
  if (!ret) {
    Serial.println(F("Init failed"));
    while(1);
  }
  MMA8451_enableInterrupt(&sensor, 700, 300, true);
}

void setupMQTT() {
  while(!modem.isGprsConnected()) {
    Serial.println(F("connecting to gprs"));
    modem.gprsConnect(apn, gprsUser, gprsPass);
    delay(1000);
  }
  while (!mqtt.connected()) {
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
  }
  Serial.println();
  Serial.println(F("mqtt connected"));
}

void publishMQTT(const uint8_t *buffer, uint32_t bytesTotal, uint16_t bytesPerWrite) {
  Serial.println(F("Publishing.."));
  bool ret;
  ret = mqtt.beginPublish("ngd/demo/gprs001/data", bytesTotal, false); 
  Serial.print(F("ret")); Serial.println(ret ? " OK" : " Failed");
  uint8_t *pointerToBuffer = (uint8_t *)buffer;
  Serial.print(F("0. bytes remaining: ")); Serial.println(bytesTotal);
  while (bytesTotal) {
    uint16_t bytesToWrite = (bytesTotal > bytesTotal % bytesPerWrite ? bytesPerWrite : bytesTotal % bytesPerWrite);
    uint16_t wrote = mqtt.write(pointerToBuffer, bytesToWrite);
    bytesTotal -= wrote;
    pointerToBuffer += wrote;
    Serial.print(F("bytes remaining: ")); Serial.println(bytesTotal);
    mqtt.loop();
    yield();
  }
  ret = mqtt.endPublish(); 
  Serial.print(F("ret")); Serial.println(ret ? " OK" : " Failed");
  mqtt.disconnect();
  Serial.println(F("Done publishing"));
}

void memoryInfo(void) {
  uint32_t heapSize = esp_get_free_heap_size();
  Serial.print(F("esp_get_free_heap_size: ")); Serial.println(heapSize);
  heapSize = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  Serial.print(F("heap_caps_get_largest_free_block MALLOC_CAP_SPIRAM: ")); Serial.println(heapSize);
}

void setup () {
  Serial.begin(115200);
  Serial.println(F("Here we go"));

#if TEST_CORE
  pinMode(LED_PIN, OUTPUT);
  Serial.print(F("Stack: "));
  xTaskCreatePinnedToCore(core0Test, "Core_0_Test", 1000, NULL, 1, &core0TaskHandle, 0);
  Serial.println(F("Task created"));
#endif


  setupSensor();
  setupModem();
  mqtt.setServer(broker, 1883);
#if TEST_TIMER
  Serial.println(F("Ininitalizing timer"));
  startTimer();
#endif

  pinMode(INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(INT_PIN, accInterrupt, RISING);
}


void loop() {
  if (intFlag || timerSet) {
    if (intFlag) {
      Serial.println(F("\nacceleration interrupt"));
      intFlag = false;
    } 
    if (timerSet) {
      Serial.println(F("\ntimer interrupt"));
      timerSet = false;
    }
  
    detachInterrupt(INT_PIN);
    endTimer();
    
    Serial.println();
    delay(100);
   
    Serial.print(F("capacity: ")); Serial.println((uint32_t)capacity);

    int16_t *x = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    int16_t *y = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    int16_t *z = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    uint16_t *t = (uint16_t *)malloc(BUFFER_SIZE * sizeof(uint16_t));

    for (uint16_t index = 0; index  < BUFFER_SIZE; index++) {
      uint32_t startTime = millis();
      while (millis() - startTime < 10);  
      MMA8451_readData(&sensor);
      x[index] = (sensor.data.x);
      y[index] = (sensor.data.y);
      z[index] = (sensor.data.z);
      t[index] = (index);

      if (index % 100 == 0) {
        Serial.print(F("Index: ")); Serial.print(index);
        Serial.print(F(" at: ")); Serial.println(startTime);
      }
    }
    Serial.print(F("Stop time: ")); Serial.println(millis());

    startWatchDog();
    setupMQTT(); 
    feedWatchDog();

    String location = modem.getGsmLocation();
    Serial.print(F("location: ")); Serial.println(location);
    uint8_t index = location.lastIndexOf(',');
    String timeStamp = location.substring(index+1);
    Serial.print(F("timestamp: ")); Serial.println(timeStamp);

    char *buffer = (char *)malloc(capacity);

    Serial.println(F("check 1"));
    mpack_writer_t writer;
    mpack_writer_init(&writer, buffer, capacity);
    mpack_start_map(&writer, 9);
    mpack_write_cstr(&writer, "timestamp"); mpack_write_cstr(&writer, timeStamp.c_str());
    mpack_write_cstr(&writer, "moduleID"); mpack_write_cstr(&writer, "GPRS010_0000001");
    mpack_write_cstr(&writer, "msgtype"); mpack_write_u8(&writer, 2);
    mpack_write_cstr(&writer, "format"); mpack_write_cstr(&writer, "Int16");
    mpack_write_cstr(&writer, "freq"); mpack_write_u8(&writer, 200);

    mpack_write_cstr(&writer, "x-accel");
    mpack_write_tag(&writer, mpack_tag_make_array(BUFFER_SIZE));
    Serial.println(F("check 2"));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        mpack_write_i16(&writer, x[i]);
    }
    Serial.println(F("check 3"));
    mpack_write_cstr(&writer, "y-accel");
    mpack_write_tag(&writer, mpack_tag_make_array(BUFFER_SIZE));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        mpack_write_i16(&writer, y[i]);
    }
    Serial.println(F("check 4"));
    mpack_write_cstr(&writer, "z-accel");
    mpack_write_tag(&writer, mpack_tag_make_array(BUFFER_SIZE));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        mpack_write_i16(&writer, z[i]);
    }
    Serial.println(F("check 5"));
    mpack_write_cstr(&writer, "tba");
    mpack_write_tag(&writer, mpack_tag_make_array(BUFFER_SIZE));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        mpack_write_i16(&writer, t[i]);
    }
    Serial.println(F("check 6"));

    size_t bytesSerialized = mpack_writer_buffer_used(&writer);
    mpack_finish_map(&writer);
    Serial.print(F("Bytes serialized: ")); Serial.println(bytesSerialized);
    Serial.println(F("check 7"));

    feedWatchDog();

    publishMQTT((uint8_t *)buffer, bytesSerialized, 512);

    free(buffer);
    buffer = NULL;
    free(x); x = NULL; 
    free(y); y = NULL; 
    free(z); z = NULL; 
    free(t); t = NULL;
    feedWatchDog();
    endWatchDog();

    Serial.print(F("MQTT connected: ")); Serial.println(mqtt.connected() ? "YES" : "NO");
    Serial.print(F("FF_MT_SRC: ")); Serial.println(MMA8451_getMotionSource(&sensor), BIN);
    delay(100);
    Serial.print(F("INT_SRC: "));Serial.println(MMA8451_getInterruptSource(&sensor), BIN);
    
    //digitalRead(INT_PIN);
    digitalWrite(INT_PIN, 0);
    attachInterrupt(INT_PIN, accInterrupt, RISING);
    startTimer();
  }
}
