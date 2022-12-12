#include "dispenser.h"
#include "tmc2209.h"
#include "pico/time.h"
#include "serialUART.h"

#include <stdio.h>

Dispenser dispensers[4] = {};

void setUpDispenser_intern(Dispenser *dispenser, SerialAddress_t address, SerialUART_t uart) {
    TMC2209_setup(dispenser->tmc2209, uart, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(dispenser->tmc2209)) {
        printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", address);
        sleep_ms(1000);
        TMC2209_setup(dispenser->tmc2209, uart, SERIAL_BAUD_RATE, address);
    }
    printf("Setup: Stepper driver with address %i setup and communicating!\n", address);
    TMC2209_setRunCurrent(dispenser->tmc2209, 100);
    TMC2209_enable(dispenser->tmc2209);

    dispenser->direction = STOP;
}

void setUpDispenser(uint8_t id, SerialUART_t uart) {
    setUpDispenser_intern(&dispensers[id], id, uart);
}

void setUpAllDispensers(SerialUART_t uart) {
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        setUpDispenser(i, uart);
    }
}

void moveDispenserUp(uint8_t id) {
    TMC2209_moveAtVelocity(dispensers[id].tmc2209, -50000);
    dispensers[id].direction = UP;
}

void moveDispenserDown(uint8_t id) {
    TMC2209_moveAtVelocity(dispensers[id].tmc2209, 50000);
    dispensers[id].direction = DOWN;
}

void stopDispenser(uint8_t id) {
    TMC2209_moveAtVelocity(dispensers[id].tmc2209, 0);
    dispensers[id].direction = STOP;
}

dispenserDirection getDispenserDirection(uint8_t id) {
    return dispensers[id].direction;
}
