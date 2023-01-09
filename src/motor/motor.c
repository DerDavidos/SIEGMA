#include "motor.h"
#include <stdio.h>
#include "pico/time.h"

bool enablePinEnabled = false;

void setUpMotor(Motor_t *motor, SerialAddress_t address, SerialUART_t uart) {
    disableMotorsByPin();

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

    enableMotorsByPin();
}

void inline initEnablePin(void) {
    if (enablePinEnabled)
        return;
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
}

void enableMotorsByPin(void) {
    initEnablePin();
    gpio_pull_up(ENABLE_PIN);
}

void disableMotorsByPin(void) {
    initEnablePin();
    gpio_pull_down(ENABLE_PIN);
}

Motor_t createMotor(SerialAddress_t address, SerialUART_t uart) {
    Motor_t motor = {.address = address};
    setUpMotor(&motor, address, uart);
    return motor;
}

void moveMotorUp(Motor_t *motor) {
    printf("Move motor %i\n", motor->address);
    TMC2209_moveAtVelocity(&motor->tmc2209, -MOTOR_SPEED);
}

void moveMotorDown(Motor_t *motor) {
    TMC2209_moveAtVelocity(&motor->tmc2209, MOTOR_SPEED);
}

void stopMotor(Motor_t *motor) {
    TMC2209_moveAtVelocity(&motor->tmc2209, 0);
}
