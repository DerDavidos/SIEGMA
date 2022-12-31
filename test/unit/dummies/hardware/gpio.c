#include "gpio.h"

bool gpio_get(uint8_t pin) {
    return GPIO_RETURN;
}

void gpio_init(uint8_t pin) { GPIO_RETURN = false; }

void gpio_set_dir(uint8_t pin, int i) {}