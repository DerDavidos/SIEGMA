#ifndef SIEGMA_MOTOR_H
#define SIEGMA_MOTOR_H

#include <stdint.h>
#include "SerialUARt.h"
#include "tmc2209.h"

typedef struct Motor {
    TMC2209_t tmc2209;
    SerialAddress_t address;
} Motor_t;

void setUpMotor(Motor_t *motor, SerialAddress_t address, SerialUART_t uart);

void moveMotorUp(Motor_t *motor);

void moveMotorDown(Motor_t *motor);

void stopMotor(Motor_t *motor);

Motor_t createMotor(SerialAddress_t address, SerialUART_t uart);

#endif //SIEGMA_MOTOR_H
