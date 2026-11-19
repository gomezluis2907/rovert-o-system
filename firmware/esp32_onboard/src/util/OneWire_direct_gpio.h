#ifndef OneWire_Direct_GPIO_h
#define OneWire_Direct_GPIO_h

#include <Arduino.h>
#include "OneWire_direct_regtype.h"

#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             (pin)
#define IO_REG_TYPE                     uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR                __attribute__((unused))
#define DIRECT_READ(base, pin)          digitalRead(pin)
#define DIRECT_MODE_INPUT(base, pin)    pinMode(pin, INPUT)
#define DIRECT_MODE_OUTPUT(base, pin)   pinMode(pin, OUTPUT)
#define DIRECT_WRITE_LOW(base, pin)     digitalWrite(pin, LOW)
#define DIRECT_WRITE_HIGH(base, pin)    digitalWrite(pin, HIGH)

#endif
