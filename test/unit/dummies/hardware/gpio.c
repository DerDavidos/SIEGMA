#include "gpio.h"

bool gpio_get(uint8_t pin) {
    return GPIO_RETURN;
}

void gpio_init(uint8_t pin) { GPIO_RETURN = false; }

void gpio_set_dir(uint8_t pin, int i) {}

void gpio_pull_down(uint8_t pin) {}

void gpio_pull_up(uint8_t pin) {}
