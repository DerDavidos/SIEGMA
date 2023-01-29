#ifndef SIEGMA_MOTOR_H
#define SIEGMA_MOTOR_H

#include "serialUART.h"
#include "tmc2209.h"

#include <stdint.h>

#define MOTOR_ENABLE_PINT_0 2
#define MOTOR_ENABLE_PINT_1 3
#define MOTOR_ENABLE_PINT_2 6
#define MOTOR_ENABLE_PINT_3 7

#define MOTOR_SPEED 50000

#define DIRECTION_UP 1
#define DIRECTION_DOWN (-1)

typedef struct Motor {
    uint8_t enablePin;
    TMC2209_t tmc2209;
    SerialAddress_t address;
    int direction;
} Motor_t;

void setUpMotor(Motor_t *motor, SerialAddress_t address, SerialUART_t uart);

void enableMotorByPin(Motor_t *motor);

void disableMotorByPin(Motor_t *motor);

void moveMotorUp(Motor_t *motor);

void moveMotorDown(Motor_t *motor);

void stopMotor(Motor_t *motor);

Motor_t createMotor(SerialAddress_t address, SerialUART_t uart);

bool motorIsCommunicating(Motor_t *motor);

#endif //SIEGMA_MOTOR_H
