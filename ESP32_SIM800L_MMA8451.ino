// https://github.com/Xinyuan-LilyGO/TTGO-T-Call

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>

// Your GPRS credentials (leave empty, if missing)
const char apn[]      = ""; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = "7526"; // SIM card PIN code, if any

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

// Defines for testing
#define TEST_ACC false
#define TEST_SIM true
#define TEST_MQTT true

TinyGsm       modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

static const char *broker = "mastap.net"; //"broker.hivemq.com";// 
static long lastReconnectAttempt = 0;
static bool apnConnected = false;

Adafruit_MMA8451 mma = Adafruit_MMA8451();

bool mqttConnect() {
  Serial.print("Connecting to mqtt broker: ");
  Serial.println(broker);

  // Connect to MQTT Broker
  bool status = mqtt.connect("sim800l", "user", "mqtt");
  //bool status = mqtt.connect("testtesttest");

  if (status == false) {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" success");
  return mqtt.connected();
}

void setup () {
  Serial.begin(115200);

#if TEST_ACC
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
#endif

#if TEST_SIM
  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  Serial.println("initializing modem");
  modem.restart(); // or modem.init();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    Serial.println("Unlocking sim card");
    bool ret = modem.simUnlock(simPIN);
    Serial.println(ret);
  }
  Serial.print("SIM status: ");
  uint8_t simStatus = modem.getSimStatus();
  Serial.println(simStatus);
  String ccid = modem.getSimCCID();
  Serial.print("SIM CCID: ");
  Serial.println(ccid);
  String emei = modem.getIMEI();
  Serial.print("SIM IMEI: ");
  Serial.println(emei);
  String cop = modem.getOperator();
  Serial.print("SIM Operator: ");
  Serial.println(cop);

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" OK");

  if (modem.isNetworkConnected()) {
    Serial.println("Network connected");
  }

  Serial.print(F("Connecting to APN: "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
    delay(10000);
    return;
  }

  if (modem.isGprsConnected()) {
    Serial.println("GPRS connected");
  }
#endif

#if TEST_MQTT
  mqtt.setServer(broker, 1883);
#endif
}

void loop() {
#if TEST_ACC
  // Read the 'raw' data in 14-bit counts
  mma.read();
  Serial.print("X:\t"); Serial.print(mma.x); 
  Serial.print("\tY:\t"); Serial.print(mma.y); 
  Serial.print("\tZ:\t"); Serial.print(mma.z); 
  Serial.println();

  /* Get a new sensor event */ 
  sensors_event_t event; 
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");

  Serial.println();
  delay(500);
#endif

#if TEST_SIM

#endif

#if TEST_MQTT
  if (!mqtt.connected() && modem.isGprsConnected()) {
    Serial.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    unsigned long t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
        Serial.println("mqtt connected");
      }
      else {
        Serial.println("Retrying..");
      }
    }
    delay(100);
    return;
  }

  mqtt.loop();
#endif
}
