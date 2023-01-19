#ifndef SIEGMA_MOTOR_H
#define SIEGMA_MOTOR_H

#include "serialUART.h"
#include "tmc2209.h"

#include <stdint.h>

#define MOTOR_ENABLE_PINT_0 26
#define MOTOR_ENABLE_PINT_1 27
#define MOTOR_ENABLE_PINT_2 28
#define MOTOR_ENABLE_PINT_3 29

#define MOTOR_SPEED 50000

typedef struct Motor {
    uint8_t enablePin;
    TMC2209_t tmc2209;
    SerialAddress_t address;
} Motor_t;

void setUpMotor(Motor_t *motor, SerialAddress_t address, SerialUART_t uart);

void enableMotorByPin(Motor_t *motor);

void disableMotorByPin(Motor_t *motor);

void moveMotorUp(Motor_t *motor);

void moveMotorDown(Motor_t *motor);

void stopMotor(Motor_t *motor);

Motor_t createMotor(SerialAddress_t address, SerialUART_t uart);

#endif //SIEGMA_MOTOR_H
