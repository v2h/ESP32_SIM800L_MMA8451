// https://rntlab.com/question/how-to-use-esp32-light-sleep/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_system.h"

#include "LumaVibe.h"

#define TIMEOUT_MINUTE_TO_MS (1 * 60 * 1000)
#define SLEEPTIME_MINUTE_TO_uS (3600000000ULL) // 1 hour

#define SSID "SVESPSERVICE"
#define PASSWORD "2JbTC9sCRkSnRKCm"

// Declared in LumaVibe_globals.h, defined in main file
bool RTC_DATA_ATTR g_isEmergency;

static uint8_t RTC_DATA_ATTR sleepCount = 0;
static bool isLedOn = false;

const char* loginIndex =
  "<form name='loginForm'>"
  "<table width='20%' bgcolor='A09F9F' align='center'>"
  "<tr>"
  "<td colspan=2>"
  "<center><font size=4><b>ESP32 Login Page</b></font></center>"
  "<br>"
  "</td>"
  "<br>"
  "<br>"
  "</tr>"
  "<td>Username:</td>"
  "<td><input type='text' size=25 name='userid'><br></td>"
  "</tr>"
  "<br>"
  "<br>"
  "<tr>"
  "<td>Password:</td>"
  "<td><input type='Password' size=25 name='pwd'><br></td>"
  "<br>"
  "<br>"
  "</tr>"
  "<tr>"
  "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
  "</tr>"
  "</table>"
  "</form>"
  "<script>"
  "function check(form)"
  "{"
  "if(form.userid.value=='admin' && form.pwd.value=='admin')"
  "{"
  "window.open('/serverIndex')"
  "}"
  "else"
  "{"
  " alert('Error Password or Username')/*displays error message*/"
  "}"
  "}"
  "</script>";
  
const char* serverIndex =
  "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
  "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
  "<input type='file' name='update'>"
  "<input type='submit' value='Update'>"
  "</form>"
  "<div id='prg'>progress: 0%</div>"
  "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')"
  "},"
  "error: function (a, b, c) {"
  "}"
  "});"
  "});"
  "</script>";

void ota_updater_begin(gpio_num_t interruptPin) {
  LumaVibe_endWatchdog();
  LumaVibe_disableModem();
  LumaVibe_setPowerBoostKeepOn(false);
  LumaVibe_setLED(CRGB::Red);
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      SerialUSB.println("Start updating " + type);
    })
    .onEnd([]() {
      SerialUSB.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      SerialUSB.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      SerialUSB.printf("Error[%u]: \n", error);
      if (error == OTA_AUTH_ERROR) SerialUSB.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) SerialUSB.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) SerialUSB.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) SerialUSB.println("Receive Failed");
      else if (error == OTA_END_ERROR) SerialUSB.println("End Failed");
      // Device will restart only when AP is turned off, that means
      // firmware can be re-uploaded as long as device is connected
    });

  SerialUSB.println("\nBooting OTA");
  SerialUSB.printf("Timeout is set to %u sec\n", TIMEOUT_MINUTE_TO_MS / 1000);
  SerialUSB.printf("deepsleep time is set to %llu sec\n", SLEEPTIME_MINUTE_TO_uS / 1000000);
  SerialUSB.printf("\nSleep count: %d\n", sleepCount);
  SerialUSB.println("Tryinng to connect to WiFi..");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  uint64_t currentTime_ms = millis();
  uint64_t ledTimer = millis();
  while (WiFi.status() != WL_CONNECTED || WiFi.status() >= WL_DISCONNECTED) { 
    // Flash LED
    if (millis() - ledTimer > 400) {
      if (isLedOn) {
        isLedOn = false;
        LumaVibe_clearLED();
      }
      else {
        isLedOn = true;
        LumaVibe_setLED(CRGB::Red);
      }
      ledTimer = millis();
    }
    // Check for timeout -> sleep
    if (millis() - currentTime_ms > TIMEOUT_MINUTE_TO_MS) { // Timeout
      sleepCount++;
      if (2 == sleepCount) {
        g_isEmergency = false;
        esp_restart();
      }
      SerialUSB.println("Connection Failed! Going to deep sleep...");
      if (btStop()) {
        SerialUSB.println("Bluetooth stopped");
      }
      if (WiFi.mode(WIFI_OFF)) {
        SerialUSB.println("Wifi off");
      }
      SerialUSB.println("Sleeping\r\n");
      LumaVibe_clearLED();
      delay(1000); // (!?)
      gpio_hold_en((gpio_num_t)LED_PIN);
      gpio_deep_sleep_hold_en();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // (!?)
      esp_sleep_enable_timer_wakeup(SLEEPTIME_MINUTE_TO_uS);
      g_isEmergency = true;
      esp_sleep_enable_ext0_wakeup(interruptPin, 0);
      esp_deep_sleep_start();      
    }
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  LumaVibe_setLED(CRGB::Green);

  ArduinoOTA.begin();

  SerialUSB.println("Ready");
  SerialUSB.print("IP address: ");
  SerialUSB.println(WiFi.localIP());

  // Only reach here when connection has been established
  while (1) {
    ArduinoOTA.handle();
    if (!WiFi.isConnected()) {
      SerialUSB.println("Wifi not connected");
      esp_restart();
    }
  }
}
