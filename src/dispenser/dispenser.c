#include "dispenser.h"

Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart) {
    Dispenser_t dispenser;
    dispenser.address = address;
    dispenser.state = DISPENSER_SLEEP;
    dispenser.haltTime = 0;
    dispenser.motor = createMotor(address, uart);
    dispenser.limitSwitch = createLimitSwitch(address);
    return dispenser;
}

void startDispenser(Dispenser_t *dispenser) {
    moveMotorUp(&dispenser->motor);
    dispenser->state = DISPENSER_UP;
}

void dispenserDoStep(Dispenser_t *dispenser, uint32_t timeElapsed) {
    switch (dispenser->state) {
        case DISPENSER_SLEEP:
            return;
        case DISPENSER_UP:
            if (timeElapsed > TIME_DISPENSERS_ARE_MOVING_UP) {
                stopMotor(&dispenser->motor);
                dispenser->state = DISPENSER_TOP;
            }
            return;
        case DISPENSER_TOP:
            if (timeElapsed > TIME_DISPENSERS_ARE_MOVING_UP + dispenser->haltTime) {
                moveMotorDown(&dispenser->motor);
                dispenser->state = DISPENSER_DOWN;
            }
        case DISPENSER_DOWN:
            if (limitSwitchIsClosed(dispenser->limitSwitch)) {
                stopMotor(&dispenser->motor);
                dispenser->state = DISPENSER_SLEEP;
            }
    }
}

void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime) {
    dispenser->haltTime = haltTime;
}

bool allDispenserSleep(Dispenser_t *dispenser, uint8_t number_of_dispenser) {
    for (uint8_t i = 0; i < number_of_dispenser; ++i) {
        if (dispenser[i].state != DISPENSER_SLEEP)
            return false;
    }
    return true;
}
