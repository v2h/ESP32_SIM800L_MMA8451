#include <Arduino.h>
// https://github.com/Xinyuan-LilyGO/TTGO-T-Call
// https://pubsubclient.knolleary.net/api.html for pubsubclient

#include <Arduino.h>
#include <Wire.h>
#include "custom_src/MMA8451.h"
#include "custom_src/mpack.h"

#define TGO_BOARD

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


// TTGO T-Call pin definitions
#define MODEM_TX 27
#define MODEM_RX 26

// For the voltage regulator chip thingy
#define I2C_SDA 21
#define I2C_SCL 22

#define INT_PIN   15
#define LED_GREEN 18 //14
#define LED_RED   19 //12

#define SerialAT  Serial1
#define SerialUSB Serial

#define TIMER_TIMEOUT_S (120*1000000)
#define BUFFER_SIZE     (2048)
#define PACKER_CAPACITY (40000)
#define WDT_TIMEOUT_S   (40*1000000)

TinyGsm       g_modem(SerialAT);
TinyGsmClient g_client(g_modem);
PubSubClient  g_mqtt(g_client);
mma8451_t     g_sensor;

hw_timer_t *g_timer    = NULL;
hw_timer_t *g_wdt      = NULL;

static const char *broker = "mastap.net";

static bool g_accelerometerInterruptFlag = false;
static bool g_timerFlag = false;

void IRAM_ATTR accelerometerISR();

void setup() {
#ifdef TGO_BOARD
  #define MODEM_RST            5
  #define MODEM_PWKEY          4
  #define MODEM_POWER_ON       23
  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
#endif

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  
  Serial.begin(115200);
  Serial.println(F("Hello World.."));

  Wire.begin(I2C_SDA, I2C_SCL);
  bool   isOk = setPowerBoostKeepOn(1);
  Serial.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
  
  setupSensor();
  setupModem();
  g_mqtt.setServer(broker, 1883);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Serial.println(F("Ininitalizing timer"));
  startTimer();
  Serial.println(F("Set up done!"));

  pinMode(INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(INT_PIN, accelerometerISR, RISING);
}


void loop() {
  //updateSerial();
  if (g_accelerometerInterruptFlag || g_timerFlag) {
    digitalWrite(LED_RED, HIGH);
    if (g_accelerometerInterruptFlag) {
      Serial.println(F("\nacceleration interrupt"));
      g_accelerometerInterruptFlag = false;
    } 
    if (g_timerFlag) {
      Serial.println(F("\ntimer interrupt"));
      g_timerFlag = false;
    }
  
    detachInterrupt(INT_PIN);
    endTimer();
    
    Serial.println();
    delay(100);
   
    Serial.print(F("capacity: ")); Serial.println((uint32_t)PACKER_CAPACITY);

    int16_t *x = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    int16_t *y = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    int16_t *z = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    uint16_t *t = (uint16_t *)malloc(BUFFER_SIZE * sizeof(uint16_t));

    for (uint16_t index = 0; index  < BUFFER_SIZE; index++) {
      uint32_t startTime = millis();
      while (millis() - startTime < 10);  
      MMA8451_readData(&g_sensor);
      x[index] = (g_sensor.data.x);
      y[index] = (g_sensor.data.y);
      z[index] = (g_sensor.data.z);
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

    String location = g_modem.getGsmLocation();
    Serial.print(F("location: ")); Serial.println(location);
    uint8_t index = location.lastIndexOf(',');
    String timeStamp = location.substring(index+1);
    Serial.print(F("timestamp: ")); Serial.println(timeStamp);

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);

    char *buffer = (char *)malloc(PACKER_CAPACITY);

    Serial.println(F("check 1"));
    mpack_writer_t writer;
    mpack_writer_init(&writer, buffer, PACKER_CAPACITY);
    mpack_start_map(&writer, 9);
    mpack_write_cstr(&writer, "timestamp"); mpack_write_cstr(&writer, timeStamp.c_str());
    mpack_write_cstr(&writer, "moduleID");  mpack_write_cstr(&writer, "GPRS010_0000001");
    mpack_write_cstr(&writer, "msgtype");   mpack_write_u8(&writer, 2);
    mpack_write_cstr(&writer, "format");    mpack_write_cstr(&writer, "Int16");
    mpack_write_cstr(&writer, "freq");      mpack_write_u8(&writer, 200);

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

    Serial.print(F("MQTT connected: ")); Serial.println(g_mqtt.connected() ? "YES" : "NO");
    Serial.print(F("FF_MT_SRC: ")); Serial.println(MMA8451_getMotionSource(&g_sensor), BIN);
    delay(100);
    Serial.print(F("INT_SRC: "));Serial.println(MMA8451_getInterruptSource(&g_sensor), BIN);
    

    digitalWrite(LED_GREEN, LOW);
    blinkLED(LED_GREEN, 300, 5);

    //digitalRead(INT_PIN);
    digitalWrite(INT_PIN, 0);
    attachInterrupt(INT_PIN, accelerometerISR, RISING);
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
  while(SerialAT.available()) 
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
  while(!g_modem.isGprsConnected()) {
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
  }
  Serial.println();
  Serial.println(F("mqtt connected"));
}

void setupSensor() {
  g_sensor.i2cAddress = MMA8451_DEFAULT_ADDRESS;
  bool ret = MMA8451_begin(&g_sensor);
  if (!ret) {
    Serial.println(F("Init failed"));
    while(1);
  }
  MMA8451_enableInterrupt(&g_sensor, 200, 10, false, true);
}

void IRAM_ATTR accelerometerISR() {
  MMA8451_getMotionSource(&g_sensor);
  detachInterrupt(INT_PIN);
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
  timerAlarmEnable(g_timer);
}

void startWatchDog() {
  g_wdt = timerBegin(1, 80, true);
  timerAttachInterrupt(g_wdt, &resetModule, true);
  timerAlarmWrite(g_wdt, WDT_TIMEOUT_S, false);
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
  ret = g_mqtt.beginPublish("ngd/demo/gprs001/data", bytesTotal, false); 
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
  g_mqtt.disconnect();
  Serial.println(F("Done publishing"));
}

void blinkLED(uint8_t ledPin, uint8_t ms, uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(ms);
    digitalWrite(ledPin, LOW);
    delay(ms);
  }
}


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en)
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