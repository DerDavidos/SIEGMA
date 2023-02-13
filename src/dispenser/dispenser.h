#ifndef SIEGMA_DISPENSER_H
#define SIEGMA_DISPENSER_H

#include <stdint.h>
#include "serialUART.h"
#include "motor.h"
#include "limitSwitch.h"

#ifdef RONDELL
#define NUMBER_OF_DISPENSERS 1
#else
#define NUMBER_OF_DISPENSERS 4
#endif
#define DISPENSER_STEP_TIME_MS 100
#ifdef RONDELL
#define MS_DISPENSERS_ARE_MOVING_UP 7500
#else
#define MS_DISPENSERS_ARE_MOVING_UP 8000
#endif
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
} Dispenser_t;

// initialize a new Dispenser with all of its components (Uart, Limit Switch)
// @param1 The Address for the uart connection (1, 2, 3, 4)
// @param2 which Uart Pins will be used
// @return an initialized Dispenser
Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart);

// Dispenser cycles to the next state
// @param the Dispenser to take action on
// @return void
void dispenserDoStep(Dispenser_t *dispenser);

// Check if all Dispenser are sleeping
// @param1 an array holding all Dispenser
// @param2 The amount of initialized Dispenser
// @return true if all Dispenser are in a sleeping state
bool allDispenserInSleepState(Dispenser_t *dispenser, uint8_t number_of_dispenser);

// Set the halt time for the dispenser to wait at the "top"
// @param1 Dispenser to be set
// @param2 The time (in ms) to halt
// @return void
void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime);

// Change dispenser state to the sleep state (wait for new command)
// @param1 Dispenser to be set
// @return new state (sleep state) of the dispenser
static DispenserState_t sleepState(Dispenser_t *dispenser);

// Change dispenser state to the up state (drive upwards)
// @param1 Dispenser to be set
// @return new state (up state) of the dispenser
static DispenserState_t upState(Dispenser_t *dispenser);

// Change dispenser state to the top state (stay in the up position)
// @param1 Dispenser to be set
// @return new state (top state) of the dispenser
static DispenserState_t topState(Dispenser_t *dispenser);

// Change dispenser state to the down state (drive downwards)
// @param1 Dispenser to be set
// @return new state (down state) of the dispenser
static DispenserState_t downState(Dispenser_t *dispenser);

// Change dispenser state to the error state (no connection to the tmc -> try again)
// @param1 Dispenser to be set
// @return new state (error state) of the dispenser
static DispenserState_t errorState(Dispenser_t *dispenser);

#endif //SIEGMA_DISPENSER_H
