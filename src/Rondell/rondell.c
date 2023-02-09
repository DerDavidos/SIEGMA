#include "pico/time.h"
#include "hardware/adc.h"
#include "TMC2209.h"
#include "serialUART.h"

#include "rondell.h"

#define DEBUGRONDELL 1

#ifdef DEBUGRONDELL
#include <stdio.h>
#endif

#define MEAN_OF_LDR_VALUES ((rondell.max_ldr_value + rondell.min_ldr_value) / 2)

static Rondell_t rondell;

static void createRondell(SerialAddress_t address, SerialUART_t uart) {
    rondell.address = address;
    rondell.uart = uart;
    rondell.position = UNDEFINED;
    rondell.state = RONDELL_SLEEP;
    rondell.positionToDriveTo = UNDEFINED;
    rondell.max_ldr_value = 0;
    rondell.min_ldr_value = 4095;
    rondell.motor = createMotor(address, uart);
}

static void setExtrema(void);
static void moveRondellClockwise(void);

void setUpRondell(SerialAddress_t address, SerialUART_t uart) {
    createRondell(address, uart);
    setExtrema();

}

static void moveRondellCounterClockwise(void) {
    moveMotorUp(&rondell.motor);
    rondell.state = RONDELL_MOVING_COUNTER_CLOCKWISE;
    sleep_ms(200);
}

static void moveRondellClockwise(void) {
    moveMotorDown(&rondell.motor);
    rondell.state = RONDELL_MOVING_CLOCKWISE;
    sleep_ms(200);
}

static void stopRondell(void) {
    stopMotor(&rondell.motor);
    disableMotorByPin(&rondell.motor);
}

/*
Being at position 0 or 3 and wanting to get to 0 or 3 is special, because applied to the set {0,1,2,3} some rules of arithmetics, specifically
the absolute value function, are
different and therefore need special consideration. Returns a bool.
 */
static uint8_t specialPositionGiven(void) {
    uint8_t specialPosition;
    (rondell.position == 0 && rondell.positionToDriveTo == 3) || (rondell.position == 3 && rondell.positionToDriveTo == 0) ? (specialPosition = 1) : (specialPosition = 0);
    return specialPosition;
}

static int8_t subtractPositions(void) {
    int8_t current_pos = (int8_t) rondell.position;
    int8_t positionToDriveTo = (int8_t) rondell.positionToDriveTo;
    return (current_pos - positionToDriveTo);
}

static uint8_t calculatePositionDifference(void) {
    if (specialPositionGiven()) return 1;
    uint8_t positionDifference;
    ((subtractPositions()) >= 0) ? (positionDifference = subtractPositions()) : (positionDifference = -(subtractPositions()));
    return positionDifference;
}

static void setExtrema(void) {
#ifdef DEBUGRONDELL
    printf("ENTERED setExtrema\n");
#endif
    enableMotorByPin(&rondell.motor);
    moveRondellClockwise();
    uint16_t dataCollectionTime_ms = 15000;
    uint16_t counter = 0;
    while(counter <= dataCollectionTime_ms) {
        uint16_t current_val = adc_read();
        if (current_val > rondell.max_ldr_value) {
            rondell.max_ldr_value = current_val;
        }
        if (current_val < rondell.min_ldr_value) {
            rondell.min_ldr_value = current_val;
        }
        counter += 10;
        sleep_ms(10);
    }
    stopRondell();
    rondell.state = RONDELL_IN_KEY_POS;
    sleep_ms(1000);
    moveToDispenserWithId(0);
    rondell.state = RONDELL_SLEEP;
#ifdef DEBUGRONDELL
    printf("LEAVING SET EXTREMA, MAX LDR: %d, MIN LDR: %d\n", rondell.max_ldr_value, rondell.min_ldr_value);
#endif
}

// In case a direction change is needed the rondell shall stop and wait some time before actually changing the direction to ensure smoothness.
static void smoothDirectionChange(enum RondellState desiredDirection) {
    if (desiredDirection != rondell.state) {
        if (rondell.state != RONDELL_SLEEP) {
            stopRondell();
            sleep_ms(1000);
        }
        desiredDirection == RONDELL_MOVING_CLOCKWISE ? moveRondellClockwise() : moveRondellCounterClockwise();
    }
}

void handleSpecialPosition(void) {
#ifdef DEBUGRONDELL
    printf("ENTERED handleSpecialPosition\n");
#endif
    if (rondell.positionToDriveTo == 3 && rondell.position == 0) {
        smoothDirectionChange(RONDELL_MOVING_COUNTER_CLOCKWISE);
    } else {
        smoothDirectionChange(RONDELL_MOVING_CLOCKWISE);
    }
}

/*
"OrdinaryPosition" means that (position, positionToDriveTo)
have a difference of 1 but are neither (3,0) nor (0,3).
*/
void handleOrdinaryPosition(void) {
#ifdef DEBUGRONDELL
    printf("ENTERED handleOrdinaryPosition\n");
#endif
    // The if-condition may seem arbitrary, but it is not; it results from the corresponding dispenser IDs.
    if (rondell.positionToDriveTo > rondell.position) {
        smoothDirectionChange(RONDELL_MOVING_CLOCKWISE);
    }
    else {
        smoothDirectionChange(RONDELL_MOVING_COUNTER_CLOCKWISE);
    }
}

// Depending on the difference between the rondell's current position and its desired position a decision is being
// made whether to move clockwise or counter-clockwise.
static void startRondellAndDecideDirection(void) {
#ifdef DEBUGRONDELL
    printf("started rondell and deciding direction\n");
#endif
    enableMotorByPin(&rondell.motor);
    if (rondell.position != UNDEFINED) {
        uint8_t positionDifference = calculatePositionDifference();
#ifdef DEBUGRONDELL
        printf("POSITION DIFFERENCE: %u\n", positionDifference);
#endif
        if (positionDifference == 1) {
            if(specialPositionGiven()) {
                handleSpecialPosition();
                return;
            }
            else {
                handleOrdinaryPosition();
                return;
            }
        }
    }
    moveRondellCounterClockwise();
}

static void passBrightPeriod(void);
static void passDarkPeriod(uint32_t *counter);

static void findLongHole(bool *longHoleFound) {
#ifdef DEBUGRONDELL
    printf("entered FINDLONGHOLE\n");
#endif
    int high_counter = 0;
    passDarkPeriod(0);
    while(adc_read() < MEAN_OF_LDR_VALUES) {
        if(high_counter >= 500) {
            break;
        }
        high_counter += 10;
        sleep_ms(10);
    }
    if (high_counter >= 500) {
        *longHoleFound = true;
        return;
    }
}

static void passLongHole(void) {
    passBrightPeriod();
}

static void findLongHoleAndPassIt(void) {
    if (rondell.state != RONDELL_MOVING_COUNTER_CLOCKWISE || rondell.state != RONDELL_MOVING_CLOCKWISE) {
        startRondellAndDecideDirection();
    }
    bool longHoleFound = false;

    while(!longHoleFound) {
        findLongHole(&longHoleFound);
    }
#ifdef DEBUGRONDELL
    printf("LONG HOLE FOUND\n");
#endif

    passLongHole();
}


/* The following two functions check whether the ADC reads above/below a certain value. So long as that
/ value is read, the rondell keeps moving.
 The values may seem confusing; the reader might think that "adc_read() > 2500" in "passDARKPeriod" does not make sense
 but this is necessary because of a hardware restriction that could not be changed anymore.

 The decision to not generalize the sleep-duration is based on the need for consistent behaviour. The sleep period should always
 be the same; therefore the usage of a constant.

 */
static void passDarkPeriod(uint32_t *counter) {
    while (adc_read() > MEAN_OF_LDR_VALUES) {
        if (counter) {
            *counter += 5;
        }
        sleep_ms(5);
    }
}

static void passBrightPeriod(void) {
    while (adc_read() < MEAN_OF_LDR_VALUES) {
        sleep_ms(5);
    }
}


/*
The idea for the algorithm of "identify position" is to determine time differences between certain areas on the rondell.
Tests have shown these values to be quite stable. There is a tolerance of about ±100 for each critical value, though tests
have shown that a lesser tolerance probably would have worked too.
*/
static void identifyPosition(void) {
    // The next two lines ensure a proper transition from the long hole and counts the time for that period.
    uint32_t counterLongHoleToFirstHole = 50;
    sleep_ms(50);

    passDarkPeriod(&counterLongHoleToFirstHole);
    sleep_ms(25);

#ifdef DEBUGRONDELL
    printf("LH TO FH: %u\n", counterLongHoleToFirstHole);
#endif
    // If one of the first two if statements evaluates to true the position can be determined immediately due
    // to the rondell's shape.
    // Tests have shown that the time difference for Pos2 needs wider range of tolerance.
    if (counterLongHoleToFirstHole >= 700 && counterLongHoleToFirstHole <= 1000) {
#ifdef DEBUGRONDELL
        printf("RONDELL POS2\n");
#endif
        rondell.position = Pos2;
        return;
    }
    if (counterLongHoleToFirstHole >= 400 && counterLongHoleToFirstHole <= 600) {
#ifdef DEBUGRONDELL
        printf("RONDELL POS3\n");
#endif
        rondell.position = Pos3;
        return;
    }

    // If the first two if statements were not evaluated to true, another two if statements are necessary because
    // there are two areas on the rondell with the same value for "counterLongHoleToFirstHole"
    if (counterLongHoleToFirstHole >= 100 && counterLongHoleToFirstHole <= 300) {
        passBrightPeriod();

        //ensure hole has really been left and count time for that period
        uint32_t counterFirstHoleToSecondHole = 50;
        sleep_ms(50);

        passDarkPeriod(&counterFirstHoleToSecondHole);
#ifdef DEBUGRONDELL
        printf("FH TO 2ndH: %u\n", counterFirstHoleToSecondHole);
#endif
        if (counterFirstHoleToSecondHole >= 100 && counterFirstHoleToSecondHole<= 300) {
#ifdef DEBUGRONDELL
            printf("RONDELL POS1\n");
#endif
            rondell.position = Pos1;
            return;
        }
        if (counterFirstHoleToSecondHole >= 400 && counterFirstHoleToSecondHole <= 600) {
#ifdef DEBUGRONDELL
            printf("RONDELL POS0\n");
#endif
            rondell.position = Pos0;
            return;
        }
    }
}


/*
This function moves the dispenser in alignment with the hopper.
After each passBrightPeriod/passDarkPeriod there is some extra sleep time to ensure a smooth transition.
Some values/instruction may seem arbitrary; this is because of some slight inaccuracies of the rondell-pattern.
*/
static void moveRondellToKeyPosition(void) {
    findLongHoleAndPassIt();
    identifyPosition();
    switch (rondell.position) {
        case Pos2:
            if (rondell.state == RONDELL_MOVING_COUNTER_CLOCKWISE) {
                sleep_ms(50);
                passBrightPeriod();
                sleep_ms(200);
            }
            return;

        case Pos1:
            passBrightPeriod();
            sleep_ms(100);
            passDarkPeriod(0);
            if (rondell.state == RONDELL_MOVING_COUNTER_CLOCKWISE) {
                sleep_ms(50);
                passBrightPeriod();
                sleep_ms(200);
            }
            return;

        case Pos0:
            if (rondell.state == RONDELL_MOVING_COUNTER_CLOCKWISE) {
                sleep_ms(50);
                passBrightPeriod();
                sleep_ms(200);
            }
            return;

        case Pos3:
            passBrightPeriod();
            sleep_ms(100);
            passDarkPeriod(0);
            if (rondell.state == RONDELL_MOVING_COUNTER_CLOCKWISE) {
                sleep_ms(50);
                passBrightPeriod();
                sleep_ms(200);
            }
            return;
        default:
            return;
    }
}

void moveToDispenserWithId(enum RondellPos positionToDriveTo) {

    rondell.positionToDriveTo = positionToDriveTo;

    if (rondell.positionToDriveTo == rondell.position) {
        return;
    }

    bool reachedDesiredPosition = false;
    while(!reachedDesiredPosition) {
        moveRondellToKeyPosition();
        if (rondell.position == rondell.positionToDriveTo) reachedDesiredPosition = true;
    }
    rondell.state = RONDELL_IN_KEY_POS;
    stopRondell();
#ifdef DEBUGRONDELL
    printf("reached desired position: %d, while position variable is: %d\n", positionToDriveTo, rondell.position);
#endif
}