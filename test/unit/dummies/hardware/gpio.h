#ifndef SIEGMA_GPIO_H
#define SIEGMA_GPIO_H

#include <stdint.h>
#include <stdbool.h>

enum gpio_function {
    GPIO_FUNC_UART = 0,
    GPIO_FUNC_SIO = 0,
    GPIO_FUNC_GPCK = 0,
    GPIO_FUNC_NULL = 0,
};

#define GPIO_OUT 1
#define GPIO_IN 0

bool GPIO_RETURN;

bool gpio_get(uint8_t pin);

void gpio_init(uint8_t pin);

void gpio_set_dir(uint8_t pin, int i);

void gpio_pull_down(uint8_t pin);

void gpio_pull_up(uint8_t pin);

#endif //SIEGMA_GPIO_H
