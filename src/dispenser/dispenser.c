#include "dispenser.h"
#include "motor.h"

#include "limitSwitch/limitSwitch.h"

Dispenser_t createDispenser(SerialAddress_t address, SerialUART_t uart) {
    Dispenser_t dispenser;
    dispenser.motor = createMotor(address, uart);
    dispenser.state = SLEEP;
    dispenser.address = address;
    return dispenser;
}

void startDispenser(Dispenser_t *dispenser) {
    moveMotorUp(&dispenser->motor);
}

void dispenserDoStep(Dispenser_t *dispenser, int timeElapsed) {
    switch (dispenser->state) {
        case SLEEP:
            return;
        case UP:
            if (timeElapsed > TIME_DISPENSERS_ARE_MOVING_UP) {
                stopMotor(&dispenser->motor);
                dispenser->state = TOP;
            }
            return;
        case TOP:
            if (timeElapsed > TIME_DISPENSERS_ARE_MOVING_UP + dispenser->haltTime) {
                moveMotorDown(&dispenser->motor);
                dispenser->state = DOWN;
            }
        case DOWN:
            if (limitSwitchIsClosed(dispenser->address)) {
                stopMotor(&dispenser->motor);
                dispenser->state = SLEEP;
            }
    }
}

void setDispenserHaltTime(Dispenser_t *dispenser, uint32_t haltTime) {
    dispenser->haltTime = haltTime;
}

bool allDispenserSleep(Dispenser_t *dispenser, uint8_t number_of_dispenser) {
    for (int i = 0; i < number_of_dispenser; ++i) {
        if (dispenser[i].state != SLEEP)
            return false;
    }
    return true;
}
