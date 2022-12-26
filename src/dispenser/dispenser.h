#ifndef SIEGMA_DISPENSER_H
#define SIEGMA_DISPENSER_H

#include <stdint.h>
#include "serialUART.h"
#include "motor.h"

#define NUMBER_OF_DISPENSERS 4
#define TIME_DISPENSERS_ARE_MOVING_UP 5000

#if NUMBER_OF_DISPENSERS > 4
#error ONLY 4 DISPENERS AVAILABLE
#endif

typedef enum DispenserState {
    SLEEP,
    UP,
    TOP,
    DOWN,
} DispenserState_t;

typedef struct Dispenser {
    Motor_t motor;
    DispenserState_t state;
    SerialAddress_t address;
    uint32_t haltTime;
} Dispenser_t;

Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart);

void startDispenser(Dispenser_t *dispenser);

void dispenserDoStep(Dispenser_t *dispenser, int timeElapsed);

bool allDispenserSleep(Dispenser_t *dispenser, uint8_t number_of_dispenser);

void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime);

#endif //SIEGMA_DISPENSER_H
