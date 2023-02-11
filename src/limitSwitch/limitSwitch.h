#ifndef SIEGMA_LIMITSWITCH_H
#define SIEGMA_LIMITSWITCH_H

#include <hardware/gpio.h>

#include <stdint.h>
#include <stdbool.h>

// Pins of the connected Limit switches
#define STEPPER0_LS_PIN 29
#define STEPPER1_LS_PIN 28
#define STEPPER2_LS_PIN 27
#define STEPPER3_LS_PIN 26

// assign the limit switches to their stepper motors
#define LIMIT_SWITCH_0 (limitSwitch_t) {.pin = STEPPER0_LS_PIN}
#define LIMIT_SWITCH_1 (limitSwitch_t) {.pin = STEPPER1_LS_PIN}
#define LIMIT_SWITCH_2 (limitSwitch_t) {.pin = STEPPER2_LS_PIN}
#define LIMIT_SWITCH_3 (limitSwitch_t) {.pin = STEPPER3_LS_PIN}

// a limit switch consists of its pin on the pico
typedef struct limitSwitch {
    uint8_t pin;
} limitSwitch_t;

// retrieve if the limit switch is pressed or not
// @param limit switch to check
// @return true is the switch is closed and false if its open
bool limitSwitchIsClosed(limitSwitch_t limitSwitch);

// create a new limit switch
// @param the id of the new switch, if it is not accepted the pin is set to -1 and not initialized
// @return a new limit switch with initialized pins
limitSwitch_t createLimitSwitch(uint8_t id);

#endif //SIEGMA_LIMITSWITCH_H
