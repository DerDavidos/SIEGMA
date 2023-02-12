#include "motor.h"

#include <pico/time.h>
#include <hardware/gpio.h>

#include <stdio.h>

void setUpEnablePin(Motor_t *motor, SerialAddress_t id) {
    switch (id) {
        case 0:
            motor->enablePin = MOTOR_ENABLE_PINT_0;
            break;
        case 1:
            motor->enablePin = MOTOR_ENABLE_PINT_1;
            break;
        case 2:
            motor->enablePin = MOTOR_ENABLE_PINT_2;
            break;
        case 3:
            motor->enablePin = MOTOR_ENABLE_PINT_3;
            break;
        default:
            motor->enablePin = 0;
    }
    gpio_init( motor->enablePin);
    gpio_set_dir( motor->enablePin, GPIO_OUT);
}

void setUpMotor(Motor_t *motor, SerialAddress_t address, SerialUART_t uart) {
    setUpEnablePin(motor, address);
    disableMotorByPin(motor);

    TMC2209_setup(&motor->tmc2209, uart, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(&motor->tmc2209)) {
#ifdef DEBUG
        if (TMC2209_disabledByInputPin(&motor->tmc2209)) {
            printf("Setup: Stepper driver with address %i DISABLED by input pin!\n", address);
        }
        printf("Setup: Stepper driver with address %i NOT communicating and setup!\n", address);
#endif

        TMC2209_setup(&motor->tmc2209, uart, SERIAL_BAUD_RATE, address);
        sleep_ms(500);
    }

#ifdef DEBUG
    printf("Setup: Stepper driver with address %i communicating and setup!\n", address);
#endif

    TMC2209_setRunCurrent(&motor->tmc2209, 100);
    TMC2209_setHoldCurrent(&motor->tmc2209, 50);
    TMC2209_enable(&motor->tmc2209);

    motor->direction = DIRECTION_UP;
}

bool motorIsCommunicating(Motor_t *motor) {
    return TMC2209_isSetupAndCommunicating(&motor->tmc2209);
}

void enableMotorByPin(Motor_t *motor) {
    gpio_pull_down(motor->enablePin);
}

void disableMotorByPin(Motor_t *motor) {
    gpio_pull_up(motor->enablePin);
}

Motor_t createMotor(SerialAddress_t address, SerialUART_t uart) {
    Motor_t motor = {.address = address};
    setUpMotor(&motor, address, uart);
    return motor;
}

void moveMotorUp(Motor_t *motor) {
    TMC2209_moveAtVelocity(&motor->tmc2209, motor->direction * MOTOR_SPEED);
}

void moveMotorDown(Motor_t *motor) {
    TMC2209_moveAtVelocity(&motor->tmc2209, motor->direction * -MOTOR_SPEED);
}

void stopMotor(Motor_t *motor) {
    TMC2209_moveAtVelocity(&motor->tmc2209, 0);
}
