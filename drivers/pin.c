#include "pin.h"
#include "stm32.h"

// Atomic set pin high.
extern inline void Pin_SetHigh(Pin *pin);

// Atomic set pin low.
extern inline void Pin_SetLow(Pin *pin);

// Non-atomic toggle pin.
void Pin_Toggle(Pin *pin) {
  if(pin->port->ODR & (0x1U << pin->pin_num)) {
    Pin_SetLow(pin);
  } else {
    Pin_SetHigh(pin);
  }
}

// Non-atomic set pin to value.
void Pin_Set(Pin *pin, bool value) {
  if(value) {
    Pin_SetHigh(pin);
  } else {
    Pin_SetLow(pin);
  }
}

// Read the digital value of a pin.
bool Pin_Read(Pin *pin) {
  return (pin->port->IDR & pin->pin_num) ? true : false;
}

// Configure a GPIO pin with the given parameters.
void Pin_ConfigGpioPin(Pin *pin, PinMode mode, PinOutputType output_type,
    PinSpeed speed, PinPull pull, uint32_t af) {
  // Enable the clock for this GPIO.
  uint32_t port_num = ((uint32_t)(pin->port) - AHB1PERIPH_BASE) >> 10;
  RCC->AHB1ENR |= (1U << port_num);

  // Assign the simple GPIO parameters
  pin->port->MODER = (pin->port->MODER & ~(3U << (2 * pin->pin_num))) |
      (mode << (2 * pin->pin_num));
  pin->port->OTYPER =(pin->port->OTYPER & ~(1U << pin->pin_num)) |
      (output_type << pin->pin_num);
  pin->port->OSPEEDR = (pin->port->OSPEEDR & ~(3U << (2 * pin->pin_num))) |
      (speed << (2 * pin->pin_num));
  pin->port->PUPDR = (pin->port->PUPDR & ~(3U << (2 * pin->pin_num))) |
      (speed << (2 * pin->pin_num));

  // The alternate function configuration spans two registers, so get fancy.
  if(pin->pin_num <= 7) {
    pin->port->AFR[0] = (pin->port->AFR[0] & ~(0xF << (4 * pin->pin_num))) |
      (af << (4 * pin->pin_num));
  } else {
    pin->port->AFR[1] =
      (pin->port->AFR[1] & ~(0xF << (4 * (pin->pin_num - 8)))) |
      (af << (4 * (pin->pin_num - 8)));
  }
}
