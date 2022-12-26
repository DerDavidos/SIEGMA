#include "tmc2209.h"
#include "pico/time.h"
#include "serialUART.h"
#include "motor.h"

#include <stdio.h>

void setUpMotor(Motor_t *motor, SerialAddress_t address, SerialUART_t uart) {
    TMC2209_setup(&motor->tmc2209, uart, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(&motor->tmc2209)) {
        if (TMC2209_disabledByInputPin(&motor->tmc2209)) {
            printf("Setup: Stepper driver with address %i DISABLED by input pin!\n", address);
        }
        printf("Setup: Stepper driver with address %i NOT communicating and setup!\n", address);
        TMC2209_setup(&motor->tmc2209, uart, SERIAL_BAUD_RATE, address);
        sleep_ms(500);
    }

    printf("Setup: Stepper driver with address %i communicating and setup!\n", address);

    TMC2209_setRunCurrent(&motor->tmc2209, 100);
    TMC2209_enable(&motor->tmc2209);
}

Motor_t createMotor(SerialAddress_t address, SerialUART_t uart) {
    Motor_t motor = {.address = address};
    setUpMotor(&motor, address, uart);
    return motor;
}

void moveMotorUp(Motor_t *motor) {
    printf("Move motor %i\n", motor->address);
    TMC2209_moveAtVelocity(&motor->tmc2209, -50000);
}

void moveMotorDown(Motor_t *motor) {
    TMC2209_moveAtVelocity(&motor->tmc2209, 50000);
}

void stopMotor(Motor_t *motor) {
    TMC2209_moveAtVelocity(&motor->tmc2209, 0);
}
