#ifndef SIEGMA_DISPENSER_H
#define SIEGMA_DISPENSER_H

#include <stdint.h>
#include "serialUART.h"
#include "motor/motor.h"
#include "limitSwitch.h"

#define NUMBER_OF_DISPENSERS 4
#define DISPENSER_STEP_TIME_MS 10
#define MS_DISPENSERS_ARE_MOVING_UP 6000
#define STEPS_DISPENSERS_ARE_MOVING_UP (MS_DISPENSERS_ARE_MOVING_UP / DISPENSER_STEP_TIME_MS)

#if NUMBER_OF_DISPENSERS > 4
#error ONLY 4 DISPENERS AVAILABLE
#endif

typedef struct Dispenser Dispenser_t;

typedef struct dispenserState {
    struct dispenserState (*function)(struct Dispenser *);
} DispenserState_t;

typedef struct Dispenser {
    SerialAddress_t address;
    uint16_t stepsDone;
    uint16_t haltSteps;
    DispenserState_t state;
    Motor_t motor;
    limitSwitch_t limitSwitch;
    SerialUART_t uart;
} Dispenser_t;

Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart);

void startDispenser(Dispenser_t *dispenser);

void dispenserDoStep(Dispenser_t *dispenser);

bool allDispenserInSleepState(Dispenser_t *dispenser, uint8_t number_of_dispenser);

void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime);

static DispenserState_t sleepState(Dispenser_t *dispenser);

static DispenserState_t upState(Dispenser_t *dispenser);

static DispenserState_t topState(Dispenser_t *dispenser);

static DispenserState_t downState(Dispenser_t *dispenser);

#endif //SIEGMA_DISPENSER_H
