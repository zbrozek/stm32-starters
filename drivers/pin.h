#ifndef PIN_H
#define PIN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32.h"

typedef enum PinModeT {
  ePinModeInput = 0x00,
  ePinModeOutput = 0x01,
  ePinModeAlt = 0x02,
  ePinModeAnalog = 0x03
} PinMode;

typedef enum PinOutputTypeT {
  ePinOutputPushPull = 0x00,
  ePinOutputOpenDrain = 0x01
} PinOutputType;

typedef enum PinSpeedT {
  ePinSpeedLow = 0x00,
  ePinSpeedMedium = 0x01,
  ePinSpeedHigh = 0x02,
  ePinSpeedMax = 0x03
} PinSpeed;

typedef enum PinPullT {
  ePinPullNone = 0x00,
  ePinPullUp = 0x01,
  ePinPullDown = 0x02,
  ePinPullReserved = 0x03
} PinPull;

// Holding struct for ongoing access.
// port needs to be something like GPIOA.
// pin needs to be a number from 0 to 15.
typedef __packed struct PinT {
  GPIO_TypeDef* port;
  uint32_t pin_num;
} Pin;

// Atomic set pin high.
inline void Pin_SetHigh(Pin* pin) { pin->port->BSRR = (0x1U << pin->pin_num); }

// Atomic set pin low.
inline void Pin_SetLow(Pin* pin) {
  pin->port->BSRR = (0x1U << pin->pin_num) << 16;
}

// Non-atomic toggle pin.
void Pin_Toggle(Pin* pin);

// Non-atomic set pin to value.
void Pin_Set(Pin* pin, bool value);

// Read the digital value of a pin.
bool Pin_Read(Pin* pin);

// Configure a GPIO pin with the given parameters.
void Pin_ConfigGpioPin(Pin* pin, PinMode mode, PinOutputType output_type,
                       PinSpeed speed, PinPull pull, uint32_t af);

#endif
