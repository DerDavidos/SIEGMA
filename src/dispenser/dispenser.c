#include "dispenser.h"
#include "pico/time.h"

#include <stdio.h>

static DispenserState_t sleepState_t = (DispenserState_t) {.function=&sleepState};
static DispenserState_t upState_t = (DispenserState_t) {.function=&upState};
static DispenserState_t topState_t = (DispenserState_t) {.function=&topState};
static DispenserState_t downState_t = (DispenserState_t) {.function=&downState};
static DispenserState_t errorState_t = (DispenserState_t) {.function=&errorState};

void resetDispenserPosition(Dispenser_t *dispenser) {
    moveMotorUp(&dispenser->motor);
    while (limitSwitchIsClosed(dispenser->limitSwitch));
    moveMotorDown(&dispenser->motor);
    while (!limitSwitchIsClosed(dispenser->limitSwitch));
    stopMotor(&dispenser->motor);
}

#ifdef RONDELL
#define FIND_TIME 750
#else
#define FIND_TIME 250
#endif

void findDirection(Dispenser_t *dispenser, uint32_t time) {
    time = time + FIND_TIME;
    if (limitSwitchIsClosed(dispenser->limitSwitch)) {
        moveMotorUp(&dispenser->motor);
        sleep_ms(time);
        if (!limitSwitchIsClosed(dispenser->limitSwitch)) {
            stopMotor(&dispenser->motor);
            return;
        } else {
            moveMotorDown(&dispenser->motor);
            sleep_ms(time + FIND_TIME);
            if (!limitSwitchIsClosed(dispenser->limitSwitch)) {
                stopMotor(&dispenser->motor);
                dispenser->motor.direction = DIRECTION_DOWN;
                return;
            } else
                findDirection(dispenser, time + FIND_TIME);
        }
    } else {
        moveMotorDown(&dispenser->motor);
        sleep_ms(time);
        if (limitSwitchIsClosed(dispenser->limitSwitch)) {
            stopMotor(&dispenser->motor);
            dispenser->motor.direction = DIRECTION_UP;
            return;
        } else {
            moveMotorUp(&dispenser->motor);
            sleep_ms(time + FIND_TIME);
            if (limitSwitchIsClosed(dispenser->limitSwitch)) {
                stopMotor(&dispenser->motor);
                return;
            } else
                findDirection(dispenser, time + FIND_TIME);
        }
    }
}

Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart) {
    Dispenser_t dispenser;
    dispenser.address = address;
    dispenser.uart = uart;
    dispenser.haltSteps = 0;
    dispenser.stepsDone = 0;
    dispenser.state = (DispenserState_t) {.function=&sleepState};
    dispenser.motor = createMotor(address, uart);
    dispenser.limitSwitch = createLimitSwitch(address);
    dispenser.othersTriggered = 0;

    switch (address) {
        case 0:
            dispenser.stepsUp = MS_DISPENSERS_ARE_MOVING_UP_0 / DISPENSER_STEP_TIME_MS;
            break;
#ifndef RONDELL
        case 1:
            dispenser.stepsUp = MS_DISPENSERS_ARE_MOVING_UP_1 / DISPENSER_STEP_TIME_MS;
            break;
        case 2:
            dispenser.stepsUp = MS_DISPENSERS_ARE_MOVING_UP_2 / DISPENSER_STEP_TIME_MS;
            break;
        case 3:
            dispenser.stepsUp = MS_DISPENSERS_ARE_MOVING_UP_3 / DISPENSER_STEP_TIME_MS;
            break;
#endif
    }

    findDirection(&dispenser, 250);

    // Reset Dispenser position
    resetDispenserPosition(&dispenser);

    disableMotorByPin(&dispenser.motor);

    return dispenser;
}

static DispenserState_t errorState(Dispenser_t *dispenser) {
    setUpMotor(&dispenser->motor, dispenser->address, dispenser->uart);
    if (motorIsCommunicating(&dispenser->motor)) {
        disableMotorByPin(&dispenser->motor);
        dispenser->haltSteps = 0;
        return sleepState_t;
    }
    return errorState_t;
}

static DispenserState_t sleepState(Dispenser_t *dispenser) {
    if (dispenser->haltSteps > 0) {
        enableMotorByPin(&dispenser->motor);
        moveMotorUp(&dispenser->motor);
        return upState_t;
    }
    return sleepState_t;
}

static DispenserState_t upState(Dispenser_t *dispenser) {
    printf("upState\n");
    printf("%i\n" ,dispenser->stepsDone);
    printf("%i\n" ,dispenser->stepsUp + 2 * dispenser->othersTriggered);
    if (dispenser->stepsDone > dispenser->stepsUp + 2 * dispenser->othersTriggered) {
        stopMotor(&dispenser->motor);
        return topState_t;
    }
    if (!limitSwitchIsClosed(dispenser->limitSwitch))
        dispenser->stepsDone++;
    return upState_t;
}

static DispenserState_t topState(Dispenser_t *dispenser) {
    printf("topState\n");
    if (dispenser->stepsDone > dispenser->stepsUp + 2 * dispenser->othersTriggered + dispenser->haltSteps) {
        moveMotorDown(&dispenser->motor);
        return downState_t;
    }
    dispenser->stepsDone++;
    return topState_t;
}

static DispenserState_t downState(Dispenser_t *dispenser) {
    printf("downState\n");
    if (limitSwitchIsClosed(dispenser->limitSwitch)) {
        stopMotor(&dispenser->motor);
        disableMotorByPin(&dispenser->motor);
        dispenser->haltSteps = 0;
        return sleepState_t;
    }
    if (dispenser->stepsDone > 2 * dispenser->stepsUp + 2 * dispenser->othersTriggered + dispenser->haltSteps + 10) {
        stopMotor(&dispenser->motor);
        disableMotorByPin(&dispenser->motor);
        dispenser->haltSteps = 0;
        return sleepState_t;
    }
    dispenser->stepsDone++;
    return downState_t;
}

void dispenserDoStep(Dispenser_t *dispenser) {
    if (!motorIsCommunicating(&dispenser->motor)) {
        dispenser->state = errorState_t;
    }
    dispenser->state = dispenser->state.function(dispenser);
}

void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime) {
    dispenser->haltSteps = haltTime / DISPENSER_STEP_TIME_MS;
    dispenser->stepsDone = 0;
#ifdef DEBUG
    printf("Dispenser %i will stop %hu steps\n", dispenser->address, dispenser->haltSteps);
#endif
}

bool allDispenserInSleepState(Dispenser_t *dispenser, uint8_t number_of_dispenser) {
    for (uint8_t i = 0; i < number_of_dispenser; ++i) {
        if (dispenser[i].state.function != sleepState_t.function)
            return false;
    }
    return true;
}
