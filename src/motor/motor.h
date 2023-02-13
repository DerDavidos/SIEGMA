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

// struct containing information (pins, stepper driver, direction, Serial address) for a motor
typedef struct Motor {
    uint8_t enablePin;
    TMC2209_t tmc2209;
    SerialAddress_t address;
    int direction;
} Motor_t;

// initializes a new motor
// @param1 The new initialized motor
// @param2 Serial address of the motor (for more information: https://github.com/janelia-arduino/TMC2209)
// @param3 Uart pins to be used
// @return void
void setUpMotor(Motor_t *motor, SerialAddress_t address, SerialUART_t uart);

// Enable the motor
// @param1 motor to be enabled
// @return void
void enableMotorByPin(Motor_t *motor);

// Disable the motor
// @param1 motor to be disabled
// @return void
void disableMotorByPin(Motor_t *motor);

// Move the motor up
// @param1 motor to be moved
// @return void
void moveMotorUp(Motor_t *motor);

// Move the motor down
// @param1 motor to be moved
// @return void
void moveMotorDown(Motor_t *motor);

// Stop the motor
// @param1 motor to be stopped
// @return void
void stopMotor(Motor_t *motor);

// Create a new motor and initialize its pins
// @param1 Serial address of the stepper driver (for more information: https://github.com/janelia-arduino/TMC2209)
// @param2 Uart pins to be used
// @return Motor struct of the new motor
Motor_t createMotor(SerialAddress_t address, SerialUART_t uart);

// Check if the motor is capable of communication over uart
// @param1 Motor to be checked
// @return true if the motor communicates, otherwise false
bool motorIsCommunicating(Motor_t *motor);

#endif //SIEGMA_MOTOR_H
