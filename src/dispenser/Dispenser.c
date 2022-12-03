#include "Dispenser.h"
#include "TMC2209.h"
#include "pico/time.h"
#include "SerialUART.h"

#include <stdio.h>

TMC2209_t TMCS[4] = {};

void setUpDispenser_intern(TMC2209_t *tmc, SerialAddress_t address) {
    TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(tmc)) {
        printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", address);
        sleep_ms(1000);
        TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);
    }
    printf("Setup: Stepper driver with address %i setup and communicating!\n", address);
    TMC2209_setRunCurrent(tmc, 100);
    TMC2209_enable(tmc);
}

void setUpDispenser(uint8_t id) {
    setUpDispenser_intern(&TMCS[id], id);
}

void setUpAllDispensers(void) {
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        setUpDispenser(i);
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
