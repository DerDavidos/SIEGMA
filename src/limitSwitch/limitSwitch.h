#ifndef SIEGMA_LIMITSWITCH_H
#define SIEGMA_LIMITSWITCH_H

#include <hardware/gpio.h>

#include <stdint.h>
#include <stdbool.h>

#define STEPPER0_LS_PIN 26
#define STEPPER1_LS_PIN 27
#define STEPPER2_LS_PIN 28
#define STEPPER3_LS_PIN 29

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
