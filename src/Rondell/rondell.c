//
// Created by Baran Demir on 09.01.23.
//

#include <stdio.h>

#include "pico/time.h"
#include "hardware/adc.h"
#include "TMC2209.h"
#include "SerialUART.h"

#include "rondell.h"
#include "touchSensor.h"

TMC2209_t TMCS_RONDELL[2] = {};

static enum RondellPos rondell_pos = UNDEFINED;
static enum RondellState rondell_state = RONDELL_SLEEP;

void setUpRondell_intern(TMC2209_t *tmc, SerialAddress_t address) {
    TMC2209_setup(tmc, SERIAL2, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(tmc)) {
        printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", address);
        sleep_ms(1000);
        TMC2209_setup(tmc, SERIAL2, SERIAL_BAUD_RATE, address);
    }
    printf("Setup: Stepper driver with address %i setup and communicating!\n", address);
    TMC2209_setRunCurrent(tmc, 100);
    TMC2209_enable(tmc);
}

void setUpRondell(uint8_t id) {
    setUpRondell_intern(&TMCS_RONDELL[id], id);
}

void setUpRondellAll(void) {
    for (int i = 0; i < 2; ++i) {
        setUpRondell(i);
    }
}

void moveRondellCounterClockwise(uint8_t id) {
    TMC2209_moveAtVelocity(&TMCS_RONDELL[id], -30000);
}


//Needs to be deleted perhaps since there might be no use for this function.
void moveRondellClockwise(uint8_t id) {
    TMC2209_moveAtVelocity(&TMCS_RONDELL[id], 30000);
}

void stopRondell(void) {
    TMC2209_moveAtVelocity(&TMCS_RONDELL[0], 0);
}

void startRondell(void) {
    moveRondellCounterClockwise(0);
    rondell_state = RONDELL_MOVING_COUNTER_CLOCKWISE;
    sleep_ms(500);
}

void passBrightPeriod(uint16_t threshhold, uint32_t *counter, uint16_t duration);
void passDarkPeriod(uint16_t threshhold, uint32_t *counter, uint16_t duration);

void findLongHole(bool *longHoleFound) {
    int high_counter = 0;
    passDarkPeriod(600, 0, 10);
    while(adc_read() > 600) {
        if(high_counter >= 1000) {
            break;
        }
        high_counter += 10;
        sleep_ms(10);
    }
    if (high_counter >= 1000) {
        *longHoleFound = true;
        return;
    }
}

void passLongHole(void) {
    passBrightPeriod(600, 0, 10);
}

void findLongHoleAndPassIt(void) {
    if (rondell_state != RONDELL_MOVING_COUNTER_CLOCKWISE || rondell_state != RONDELL_MOVING_CLOCKWISE) {
        startRondell();
    }
    bool longHoleFound = false;

    while(!longHoleFound) {
        findLongHole(&longHoleFound);
    }

    passLongHole();
}


// The following two functions check whether the ADC reads above/below a certain value. So long as that
// value is read, the rondell keeps moving.

void passDarkPeriod(uint16_t threshhold, uint32_t *counter, uint16_t duration) {
    while (adc_read() < threshhold) {
        if (counter) {
            *counter += duration;
        }
        sleep_ms(duration);
    }
}

void passBrightPeriod(uint16_t threshhold, uint32_t *counter, uint16_t duration) {
    while (adc_read() > threshhold) {
        if(counter) {
            *counter += duration;
        }
        sleep_ms(duration);
    }
}

void identifyPosition(void) {
    // The next two lines ensure that the long hole is really passed and counts the time for that period.
    uint32_t counterLongHoleToFirstHole = 100;
    sleep_ms(100);

    passDarkPeriod(600, &counterLongHoleToFirstHole, 10);

    // If one of the first two if statements evaluates to true the position can be determined immediately due
    // to the rondell's shape.
    if (counterLongHoleToFirstHole >= 1300 && counterLongHoleToFirstHole <= 1500) {
        rondell_pos = Pos0;
        return;
    }
    if (counterLongHoleToFirstHole >= 700 && counterLongHoleToFirstHole <= 900) {
        rondell_pos = Pos3;
        return;
    }

    // If the first two if statements were not evaluated to true, another two if statements are necessary because
    // there are two areas on the rondell with the same value for "counterLongHoleToFirstHole"
    if (counterLongHoleToFirstHole >= 200 && counterLongHoleToFirstHole <= 400) {
        uint32_t counterFirstHoleToSecondHole = 0;
        passBrightPeriod(600, 0, 10);
        passDarkPeriod(600, &counterFirstHoleToSecondHole, 10);
        if (counterFirstHoleToSecondHole >= 200 && counterFirstHoleToSecondHole<= 400) {
            rondell_pos = Pos1;
            return;
        }
        rondell_pos = Pos2;
        return;
    }
}

int8_t moveRondellToKeyPosition(void) {
    findLongHoleAndPassIt();
    identifyPosition();
    switch (rondell_pos) {
        case Pos0:
            passBrightPeriod(600, 0, 1);
            return 0;

        case Pos1:
            passBrightPeriod(600, 0, 10);
            passDarkPeriod(600, 0, 10);
            passBrightPeriod(600, 0, 1);
            return 0;

        case Pos2:
            passBrightPeriod(600, 0, 1);
            return 0;

        case Pos3:
            passBrightPeriod(600, 0, 10);
            passDarkPeriod(600, 0, 10);
            passBrightPeriod(600, 0, 1);
            return 0;
        default:
            return -1;
    }
}

void ResetRondell(void) {
    if (rondell_state == RONDELL_SLEEP || rondell_state == RONDELL_IN_KEY_POS) {
        startRondell();
        rondell_state = RONDELL_MOVING_COUNTER_CLOCKWISE;
    }
    moveRondellToKeyPosition();
    stopRondell();
}

void moveToDispenserWithId(enum RondellPos positionToDriveTo) {

    if (!(rondell_state == RONDELL_IN_KEY_POS)) {
        moveRondellToKeyPosition();
        rondell_state = RONDELL_IN_KEY_POS;
    }

    if (positionToDriveTo == rondell_pos) {
        return;
    }

    bool reachedDesiredPosition = false;
    while(!reachedDesiredPosition) {
        moveRondellToKeyPosition();
        if (rondell_pos == positionToDriveTo) reachedDesiredPosition = true;
    }
    rondell_state = RONDELL_IN_KEY_POS;
    stopRondell();
}

uint8_t getRondellPosition(void) {
    return rondell_pos;
}