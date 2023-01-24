#ifndef SIEGMA_DISPENSER_H
#define SIEGMA_DISPENSER_H

#include <stdint.h>
#include "serialUART.h"
#include "motor.h"
#include "limitSwitch.h"

#define NUMBER_OF_DISPENSERS 1
#define DISPENSER_STEP_TIME_MS 10
#define MS_DISPENSERS_ARE_MOVING_UP 3700
#define STEPS_DISPENSERS_ARE_MOVING_UP (MS_DISPENSERS_ARE_MOVING_UP / DISPENSER_STEP_TIME_MS)

// error check if the number of dispenser exceeds its limits
#if NUMBER_OF_DISPENSERS > 4
#error ONLY 4 DISPENERS AVAILABLE
#endif

typedef struct Dispenser Dispenser_t;

typedef struct dispenserState {
    struct dispenserState (*function)(struct Dispenser *);
} DispenserState_t;

// Dispenser struct holding all important values to control the stepper driver
typedef struct Dispenser {
    SerialAddress_t address;
    uint16_t stepsDone;
    uint16_t haltSteps;
    DispenserState_t state;
    Motor_t motor;
    limitSwitch_t limitSwitch;
    SerialUART_t uart;
    int direction;
} Dispenser_t;

// initialize a new Dispenser with all of its components (Uart, Limit Switch)
// @param1 The Address for the uart connection (1, 2, 3, 4)
// @param2 which Uart Pins will be used
// @return an initialized Dispenser
Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart);

// no usage
void startDispenser(Dispenser_t *dispenser);

// Dispenser cycles to the next state
// @param the Dispenser to take action on
// @return void
void dispenserDoStep(Dispenser_t *dispenser);

// Check if all Dispenser are sleeping
// @param1 an array holding all Dispenser
// @param2 The amount of initialized Dispenser
// @return true if all Dispenser are in a sleeping state
bool allDispenserInSleepState(Dispenser_t *dispenser, uint8_t number_of_dispenser);

void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime);

static DispenserState_t sleepState(Dispenser_t *dispenser);

static DispenserState_t upState(Dispenser_t *dispenser);

static DispenserState_t topState(Dispenser_t *dispenser);

static DispenserState_t downState(Dispenser_t *dispenser);

static DispenserState_t errorState(Dispenser_t *dispenser);

#endif //SIEGMA_DISPENSER_H
