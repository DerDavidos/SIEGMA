#include "dispenser.h"
#include "tmc2209.h"
#include "pico/time.h"
#include "serialUART.h"

#include <stdio.h>

TMC2209_t TMCS[4] = {};

void setUpDispenser_intern(TMC2209_t *tmc, SerialAddress_t address, SerialUART_t uart) {
    TMC2209_setup(tmc, uart, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(tmc)) {
        printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", address);
        sleep_ms(1000);
        TMC2209_setup(tmc, uart, SERIAL_BAUD_RATE, address);
    }
    printf("Setup: Stepper driver with address %i setup and communicating!\n", address);
    TMC2209_setRunCurrent(tmc, 100);
    TMC2209_enable(tmc);
}

void setUpDispenser(uint8_t id, SerialUART_t uart) {
    setUpDispenser_intern(&TMCS[id], id, uart);
}

void setUpAllDispensers(SerialUART_t uart) {
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        setUpDispenser(i, uart);
    }
}

void moveDispenserUp(uint8_t id) {
    TMC2209_moveAtVelocity(&TMCS[id], -50000);
}

void moveDispenserDown(uint8_t id) {
    TMC2209_moveAtVelocity(&TMCS[id], 50000);
}

void stopDispenser(uint8_t id) {
    TMC2209_moveAtVelocity(&TMCS[id], 0);
}
