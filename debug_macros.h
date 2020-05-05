#ifndef  DEBUG_MACROS_H
#define DEBUG_MACROS_H

#if DEBUG_MACROS_ENABLE
#define PRINTS(s) do {  \
    Serial.print(F(s)); \
  } while (0)           \

#define PRINT(s,v) do { \
    Serial.print(F(s)); \
    Serial.print(v);    \
  } while(0)            \

#define PRINTHEX(s,v) do { \
    Serial.print(F(s));    \
    Serial.print(F("0x")); \
    Serial.print(v, HEX);  \
  } while (0)

#define PRINTVAL(v) Serial.print(v)
#define PRINTF(f_, ...) SerialUSB.printf((f_), __VA_ARGS__)

#else
#define PRINTS(s)
#define PRINT(s,v)
#define PRINTHEX(s,v)
#define PRINTVAL(v)
#define PRINTF(f_, ...)

#endif // DEBUG_MACROS_ENABLE

#endif // DEBUG_MACROS_H
