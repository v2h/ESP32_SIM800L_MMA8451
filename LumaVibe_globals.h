#ifndef LUMAVIBE_GLOBAL_H
#define LUMAVIBE_GLOBAL_H

#define NUM_LEDS 1
#define LED_PIN 2
#define ACCEL_PIN 25

#define SerialAT  Serial1
#define SerialUSB Serial

#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_RST 5        // Active low (also low after modem.sleepEnable() is called)
#define MODEM_PWKEY 4      // [Pulled up] drive down to turn on the modem
#define MODEM_POWER_ON 23  // [Pulled-up] active high

#define FCLK_DIVIDER          80
#define WATCHDOG_TIMER_NUMBER 1
#define SLEEP_TIMER_NUMBER    0

#define LUMAVIBE_ENABLE_ERROR_LOGGING 1
#define LUMAVIBE_PUBLIC_ALL 0 // Turn all private members into public
#define DEBUG_MACROS_ENABLE 1

#define ERROR_STREAM_SIZE 20
#define MAX_ERROR_COUNT (ERROR_STREAM_SIZE - 4)
#define MAX_NETWORK_ERROR_COUNT 6

#define FIRMWARE_VERSION "LumaVibe_FW_1.00.0"

#endif // LUMAVIBE_GLOBAL_H
