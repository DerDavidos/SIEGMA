#ifndef SIEGMA_DISPENSER_H
#define SIEGMA_DISPENSER_H

#include <stdint.h>
#include "serialUART.h"
#include "motor.h"
#include "limitSwitch.h"

#define NUMBER_OF_DISPENSERS 4
#define TIME_DISPENSERS_ARE_MOVING_UP 5000

#if NUMBER_OF_DISPENSERS > 4
#error ONLY 4 DISPENERS AVAILABLE
#endif

typedef enum DispenserState {
    DISPENSER_SLEEP,
    DISPENSER_UP,
    DISPENSER_TOP,
    DISPENSER_DOWN,
} DispenserState_t;

typedef struct Dispenser {
    SerialAddress_t address;
    DispenserState_t state;
    uint32_t haltTime;
    Motor_t motor;
    limitSwitch_t limitSwitch;
} Dispenser_t;

Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart);

void startDispenser(Dispenser_t *dispenser);

void dispenserDoStep(Dispenser_t *dispenser, uint32_t timeElapsed);

bool allDispenserSleep(Dispenser_t *dispenser, uint8_t number_of_dispenser);

void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime);

#endif //SIEGMA_DISPENSER_H
