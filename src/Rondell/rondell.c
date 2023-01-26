//
// Created by Baran Demir on 09.01.23.
//

#include <stdio.h>

#include "pico/time.h"
#include "hardware/adc.h"
#include "TMC2209.h"
#include "serialUART.h"

#include "rondell.h"
#include "limitSwitch.h"

static Rondell_t rondell;

Rondell_t createRondell(SerialAddress_t address, SerialUART_t uart) {
    rondell.address = address;
    rondell.uart = uart;
    rondell.position = UNDEFINED;
    rondell.state = RONDELL_SLEEP;
    rondell.motor = createMotor(address, uart);

    return rondell;
}

void setUpRondell(SerialAddress_t address, SerialUART_t uart) {
    createRondell(address, uart);
}

void moveRondellCounterClockwise(void) {
    moveMotorUp(&rondell.motor);
}


//Needs to be deleted perhaps since there might be no use for this function.
void moveRondellClockwise(void) {
    moveMotorDown(&rondell.motor);
}

void stopRondell(void) {
    stopMotor(&rondell.motor);
    disableMotorByPin(&rondell.motor);
}

void startRondell(void) {
    enableMotorByPin(&rondell.motor);
    moveRondellCounterClockwise();
    rondell.state = RONDELL_MOVING_COUNTER_CLOCKWISE;
    sleep_ms(500);
}

void passBrightPeriod(uint16_t threshhold, uint32_t *counter, uint16_t duration);
void passDarkPeriod(uint16_t threshhold, uint32_t *counter, uint16_t duration);

void findLongHole(bool *longHoleFound) {
    int high_counter = 0;
    passDarkPeriod(2500, 0, 10);
    while(adc_read() > 2500) {
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
    passBrightPeriod(2500, 0, 10);
}

void findLongHoleAndPassIt(void) {
    if (rondell.state != RONDELL_MOVING_COUNTER_CLOCKWISE || rondell.state != RONDELL_MOVING_CLOCKWISE) {
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

    passDarkPeriod(2500, &counterLongHoleToFirstHole, 10);

    // If one of the first two if statements evaluates to true the position can be determined immediately due
    // to the rondell's shape.
    if (counterLongHoleToFirstHole >= 1300 && counterLongHoleToFirstHole <= 1500) {
        rondell.position = Pos0;
        return;
    }
    if (counterLongHoleToFirstHole >= 700 && counterLongHoleToFirstHole <= 900) {
        rondell.position = Pos3;
        return;
    }

    // If the first two if statements were not evaluated to true, another two if statements are necessary because
    // there are two areas on the rondell with the same value for "counterLongHoleToFirstHole"
    if (counterLongHoleToFirstHole >= 200 && counterLongHoleToFirstHole <= 400) {
        uint32_t counterFirstHoleToSecondHole = 0;
        passBrightPeriod(2500, 0, 10);
        passDarkPeriod(2500, &counterFirstHoleToSecondHole, 10);
        if (counterFirstHoleToSecondHole >= 200 && counterFirstHoleToSecondHole<= 400) {
            rondell.position = Pos1;
            return;
        }
        // TO DO : IF STATEMENT FOR POS 2
        rondell.position = Pos2;
        return;
    }
}

int8_t moveRondellToKeyPosition(void) {
    findLongHoleAndPassIt();
    identifyPosition();
    switch (rondell.position) {
        case Pos0:
            passBrightPeriod(2500, 0, 1);
            return 0;

        case Pos1:
            passBrightPeriod(2500, 0, 10);
            passDarkPeriod(2500, 0, 10);
            passBrightPeriod(2500, 0, 1);
            return 0;

        case Pos2:
            passBrightPeriod(2500, 0, 1);
            return 0;

        case Pos3:
            passBrightPeriod(2500, 0, 10);
            passDarkPeriod(2500, 0, 10);
            passBrightPeriod(2500, 0, 1);
            return 0;
        default:
            return -1;
    }
}

void ResetRondell(void) {
    if (rondell.state == RONDELL_SLEEP || rondell.state == RONDELL_IN_KEY_POS) {
        startRondell();
        rondell.state = RONDELL_MOVING_COUNTER_CLOCKWISE;
    }
    moveRondellToKeyPosition();
    stopRondell();
}

void moveToDispenserWithId(enum RondellPos positionToDriveTo) {

    if (!(rondell.state == RONDELL_IN_KEY_POS)) {
        moveRondellToKeyPosition();
        rondell.state = RONDELL_IN_KEY_POS;
    }

    if (positionToDriveTo == rondell.position) {
        return;
    }

    bool reachedDesiredPosition = false;
    while(!reachedDesiredPosition) {
        moveRondellToKeyPosition();
        if (rondell.position == positionToDriveTo) reachedDesiredPosition = true;
    }
    rondell.state = RONDELL_IN_KEY_POS;
    stopRondell();
}

uint8_t getRondellPosition(void) {
    return rondell.position;
}