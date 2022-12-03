#ifndef SIEGMA_TOUCHSENSOR_H
#define SIEGMA_TOUCHSENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define STEPPER0_LS_PIN 7
#define STEPPER1_LS_PIN 3
#define STEPPER2_LS_PIN 6
#define STEPPER3_LS_PIN 2

bool touchSensorIsClosed(uint8_t id);

void setUpTouchSensor(uint8_t id);

void setUpAllTouchSensors(void);

#endif //SIEGMA_TOUCHSENSOR_H
