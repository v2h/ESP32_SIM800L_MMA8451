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

#endif // LUMAVIBE_GLOBAL_H
