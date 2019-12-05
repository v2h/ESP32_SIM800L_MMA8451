// https://github.com/Xinyuan-LilyGO/TTGO-T-Call

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_MMA8451.h>
#include <ArduinoJson.h>
#include "custom_src/MMA8451.h"

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
#define TEST_ACC   false
#define TEST_SIM   false
#define TEST_MQTT  false
#define TEST_TIMER false
#define TEST_CORE  false
#define TEST_JSON  false

TinyGsm       modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
mma8451_t sensor;
bool intFlag = false;

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

void IRAM_ATTR onTimer() {
  Serial.print(F("time: "));
  Serial.print(F(millis() / 1000));
  Serial.print(F("; counter: "));
  Serial.println(timerCounter);
  timerCounter++;
}

void IRAM_ATTR accInterrupt() {
  Serial.println(F("Interruptted"));
  intFlag = true;
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
  modem.restart(); // or modem.init();

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
  while (!mqtt.connected() && modem.isGprsConnected()) {
    // Reconnect every 10 seconds
    unsigned long t = millis();
    //Serial.println(t, DEC);
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
      else {
        lastReconnectAttempt = 0;
        Serial.println(F("Retrying.."));
        Serial.println(t, DEC);
        Serial.println(lastReconnectAttempt, DEC);
      }
    }
    delay(100);
  }

  Serial.println();

  Serial.println(F("mqtt connected"));
}

void setup () {
  Serial.begin(115200);
  Serial.println(F("Here we go"));

#if TEST_TIMER
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, onTimer, true);
  timerAlarmWrite(timer, 2*1000000, true);
  timerAlarmEnable(timer);
#endif

#if TEST_CORE
  pinMode(LED_PIN, OUTPUT);
  Serial.print(F("Stack: "));
  xTaskCreatePinnedToCore(core0Test, "Core_0_Test", 1000, NULL, 1, &core0TaskHandle, 0);
  Serial.println(F("Task created"));
#endif
  setupSensor();
  pinMode(INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(INT_PIN, accInterrupt, RISING);
  setupModem();
  mqtt.setServer(broker, 1883);
}

void loop() {
  //attachInterrupt(INT_PIN, accInterrupt, RISING);

  // MMA8451_readData(&sensor);
  // Serial.print(F("x: ")); Serial.print(F((float)sensor.data.x/2048); Serial.print(F("; "));
  // Serial.print(F("y: ")); Serial.print(F((float)sensor.data.y/2048); Serial.print(F("; "));
  // Serial.print(F("z: ")); Serial.print(F((float)sensor.data.z/2048); Serial.print(F("; "));
  // Serial.println();

  // uint8_t intSource = MMA8451_getInterruptSource(&sensor);
  // Serial.print(F("int src: ")); Serial.println(intSource, BIN);
  Serial.print(F("."));
  if (intFlag) {
    intFlag = false;
    detachInterrupt(INT_PIN);
    
    Serial.println();
    delay(100);

    sensors_event_t event;

    const uint16_t BUFFER_SIZE = 1024;

    uint32_t heapSize = esp_get_free_heap_size();
    Serial.print(F("Free heap size: ")); Serial.println(heapSize);

    const size_t capacity = 4*JSON_ARRAY_SIZE(BUFFER_SIZE) + JSON_OBJECT_SIZE(5) + 50;
    DynamicJsonDocument doc(capacity);
    JsonArray x = doc.createNestedArray("xba");
    JsonArray y = doc.createNestedArray("yba");
    JsonArray z = doc.createNestedArray("zba");
    JsonArray t = doc.createNestedArray("tba");
    doc["timestamp"] = modem.getGSMDateTime(DATE_TIME);
    Serial.print(F("timestamp: ")); Serial.println(modem.getGSMDateTime(DATE_TIME));

    uint32_t startTime = millis();
    uint32_t newTime, oldTime;
    newTime = oldTime = startTime;

    Serial.print(F("Start time: ")); 
    Serial.println(startTime);
    for (uint16_t index = 0; index  < BUFFER_SIZE; index++) {
      while (newTime - oldTime < 10) {
        newTime = millis();
      }
      oldTime = newTime;
    
      MMA8451_readData(&sensor);
      x.add((float)sensor.data.x/2048);
      y.add((float)sensor.data.y/2048);
      z.add((float)sensor.data.z/2048);
      t.add(index);

      if (index % 100 == 0) {
        Serial.print(F("Index: ")); Serial.print(index);
        Serial.print(F(" at: ")); Serial.println(newTime);
      }
    }
    Serial.print(F("Stop time: ")); 
    Serial.println(newTime);

    heapSize = esp_get_free_heap_size();
    Serial.print(F("Free heap size: ")); Serial.println(heapSize);
    Serial.print(F("JSON cap: ")); Serial.println(capacity);
    Serial.print(F("JSON size :")); Serial.println(doc.size());

    char *buffer = (char *)malloc(capacity);
    uint16_t n = measureMsgPack(doc);
    uint16_t bytesSerialized = serializeMsgPack(doc, buffer, n + 1);
    Serial.print(F("Size of payload: ")); Serial.println(n);
    Serial.print(F("Bytes serialized: ")); Serial.println(bytesSerialized);

    heapSize = esp_get_free_heap_size();
    Serial.print(F("Free heap size: ")); Serial.println(heapSize);

    setupMQTT();

    Serial.println(F("Publishing.."));
    bool ret;
    ret = mqtt.beginPublish("ngd/demo/gprs001/data", n, false); 
    Serial.print(F("ret")); Serial.println(ret ? " OK" : " Failed");
    uint16_t bytesRemaining = n;
    uint8_t *pointerToBuffer = (uint8_t *)buffer;
    Serial.print(F("0. bytes remaining: ")); Serial.println(bytesRemaining);
    while (bytesRemaining) {
      uint16_t bytesToWrite = (bytesRemaining > n % 512 ? 512 : n % 512);
      Serial.print(F("bytes to write:")); Serial.println(bytesToWrite);
      uint16_t wrote = mqtt.write(pointerToBuffer, bytesToWrite);
      Serial.print(F("wrote ")); Serial.println(wrote);
      bytesRemaining -= wrote;
      pointerToBuffer += wrote;
      Serial.print(F("bytes remaining: ")); Serial.println(bytesRemaining);
      //mqtt.loop();
    }
    ret = mqtt.endPublish(); 
    Serial.print(F("ret")); Serial.println(ret ? " OK" : " Failed");

    Serial.print(F("ret")); Serial.println(ret ? " OK" : " Failed");
    Serial.println(F("Done publishing"));

    pointerToBuffer = NULL;
    free(buffer);
    buffer = NULL;
    
    Serial.print(F("MQTT connected: ")); Serial.println(mqtt.connected() ? "YES" : "NO");
    attachInterrupt(INT_PIN, accInterrupt, RISING);
    Serial.print(F("FF_MT_SRC: ")); Serial.println(MMA8451_getMotionSource(&sensor), BIN);
    Serial.print(F("INT_SRC: "));Serial.println(MMA8451_getInterruptSource(&sensor), BIN);
    delay(100);
    Serial.print(F("FF_MT_SRC: ")); Serial.println(MMA8451_getMotionSource(&sensor), BIN);
    Serial.print(F("INT_SRC: "));Serial.println(MMA8451_getInterruptSource(&sensor), BIN);
    //delay(20000);
  }
  delay(100); 
}
