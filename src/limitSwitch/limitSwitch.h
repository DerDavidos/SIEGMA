#ifndef SIEGMA_LIMITSWITCH_H
#define SIEGMA_LIMITSWITCH_H

#include <stdint.h>
#include <stdbool.h>

#define NUMBER_OF_LIMIT_SWITCHES 4

#define STEPPER0_LS_PIN 7
#define STEPPER1_LS_PIN 3
#define STEPPER2_LS_PIN 6
#define STEPPER3_LS_PIN 2

bool limitSwitchIsClosed(uint8_t id);

void setUpLimitSwitch(uint8_t id);

void setUpAllLimitSwitches(void);

bool allLimitSwitchesAreClosed(void);

#endif //SIEGMA_LIMITSWITCH_H
