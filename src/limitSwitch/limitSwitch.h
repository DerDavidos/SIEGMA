#ifndef SIEGMA_LIMITSWITCH_H
#define SIEGMA_LIMITSWITCH_H

#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"

#define STEPPER0_LS_PIN 7
#define STEPPER1_LS_PIN 3
#define STEPPER2_LS_PIN 6
#define STEPPER3_LS_PIN 2

#define LIMIT_SWITCH_0 (limitSwitch_t) {.pin = STEPPER0_LS_PIN}
#define LIMIT_SWITCH_1 (limitSwitch_t) {.pin = STEPPER1_LS_PIN}
#define LIMIT_SWITCH_2 (limitSwitch_t) {.pin = STEPPER2_LS_PIN}
#define LIMIT_SWITCH_3 (limitSwitch_t) {.pin = STEPPER3_LS_PIN}

typedef struct limitSwitch {
    uint8_t pin;
} limitSwitch_t;

bool limitSwitchIsClosed(limitSwitch_t limitSwitch);

limitSwitch_t createLimitSwitch(uint8_t id);

#endif //SIEGMA_LIMITSWITCH_H
