#include <unity.h>

#include "dispenser.h"
#include "serialUART.h"

#define ID 0

void setUp() {
    // If Fails, struct was changed and tests need to be adjusted
    TEST_ASSERT_EQUAL(24, sizeof(Dispenser_t));
}

void tearDown() {}

void dispenserIsCreatedCorrectly(void) {
    Dispenser_t actual = createDispenser(ID, SERIAL);

    Dispenser_t expected = (Dispenser_t) {.address=ID, .state=DISPENSER_SLEEP, .haltTime=0,
            .motor=createMotor(ID, SERIAL),
            .limitSwitch=createLimitSwitch(ID)};

    TEST_ASSERT_EQUAL(expected.limitSwitch.pin, actual.limitSwitch.pin);
    TEST_ASSERT_EQUAL(expected.state, actual.state);
    TEST_ASSERT_EQUAL(expected.haltTime, actual.haltTime);
    TEST_ASSERT_EQUAL(expected.motor.address, actual.motor.address);
    TEST_ASSERT_EQUAL(expected.motor.tmc2209.serial_address, actual.motor.tmc2209.serial_address);
    TEST_ASSERT_EQUAL(expected.address, actual.address);
}

void allDispenserCanBeCreated() {
    Dispenser_t dispenser[NUMBER_OF_DISPENSERS];

    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        dispenser[i] = createDispenser(i, SERIAL);

        Dispenser_t expected = (Dispenser_t) {.address=i, .state=DISPENSER_SLEEP, .haltTime=0,
                .motor=createMotor(i, SERIAL),
                .limitSwitch=createLimitSwitch(i)};

        TEST_ASSERT_EQUAL(expected.limitSwitch.pin, dispenser[i].limitSwitch.pin);
        TEST_ASSERT_EQUAL(expected.state, dispenser[i].state);
        TEST_ASSERT_EQUAL(expected.haltTime, dispenser[i].haltTime);
        TEST_ASSERT_EQUAL(expected.motor.address, dispenser[i].motor.address);
        TEST_ASSERT_EQUAL(expected.motor.tmc2209.serial_address, dispenser[i].motor.tmc2209.serial_address);
        TEST_ASSERT_EQUAL(expected.address, dispenser[i].address);
    }
}

void dispenserMovesWhenStarted(void) {
    Dispenser_t dispenser = createDispenser(ID, SERIAL);
    startDispenser(&dispenser);
    TEST_ASSERT_EQUAL(-MOTOR_SPEED, CURRENT_VELOCITY);
    TEST_ASSERT_EQUAL(DISPENSER_UP, dispenser.state);
}

void dispenserHaltTimeCanBeSet(void) {
    Dispenser_t dispenser = createDispenser(ID, SERIAL);
    setDispenserHaltTime(&dispenser, 1);
    TEST_ASSERT_EQUAL(1, dispenser.haltTime);
}

void correctStepsWhenSleep(void) {
    Dispenser_t dispenser = createDispenser(ID, SERIAL);
    dispenserDoStep(&dispenser, 0);
    TEST_ASSERT_EQUAL(DISPENSER_SLEEP, dispenser.state);
    TEST_ASSERT_EQUAL(0, CURRENT_VELOCITY);

    dispenserDoStep(&dispenser, UINT32_MAX);
    TEST_ASSERT_EQUAL(DISPENSER_SLEEP, dispenser.state);
    TEST_ASSERT_EQUAL(0, CURRENT_VELOCITY);
}

void correctStepsWhenUp(void) {
    Dispenser_t dispenser = createDispenser(ID, SERIAL);
    dispenser.state = DISPENSER_UP;

    dispenserDoStep(&dispenser, TIME_DISPENSERS_ARE_MOVING_UP);
    TEST_ASSERT_EQUAL(DISPENSER_UP, dispenser.state);

    dispenserDoStep(&dispenser, TIME_DISPENSERS_ARE_MOVING_UP + 1);
    TEST_ASSERT_EQUAL(DISPENSER_TOP, dispenser.state);
}

void correctStepsWhenTop(void) {
    Dispenser_t dispenser = createDispenser(ID, SERIAL);
    dispenser.state = DISPENSER_TOP;
    static uint8_t HALT_TIME = 1;
    dispenser.haltTime = HALT_TIME;

    dispenserDoStep(&dispenser, TIME_DISPENSERS_ARE_MOVING_UP + HALT_TIME);
    TEST_ASSERT_EQUAL(DISPENSER_TOP, dispenser.state);

    dispenserDoStep(&dispenser, TIME_DISPENSERS_ARE_MOVING_UP + HALT_TIME + 1);
    TEST_ASSERT_EQUAL(DISPENSER_DOWN, dispenser.state);
}

void correctStepsWhenDown(void) {
    Dispenser_t dispenser = createDispenser(ID, SERIAL);
    dispenser.state = DISPENSER_DOWN;
    static uint8_t HALT_TIME = 1;
    dispenser.haltTime = HALT_TIME;

    GPIO_RETURN = false;
    dispenserDoStep(&dispenser, 0);
    TEST_ASSERT_EQUAL(DISPENSER_DOWN, dispenser.state);
    dispenserDoStep(&dispenser, UINT32_MAX);
    TEST_ASSERT_EQUAL(DISPENSER_DOWN, dispenser.state);

    GPIO_RETURN = true;
    dispenserDoStep(&dispenser, 0);
    TEST_ASSERT_EQUAL(DISPENSER_SLEEP, dispenser.state);
}

void allDispensersDoSleep(void) {
    Dispenser_t dispenser[NUMBER_OF_DISPENSERS];
    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        dispenser[i] = createDispenser(i, SERIAL);
    }

    TEST_ASSERT_EQUAL(true, allDispenserSleep(dispenser, NUMBER_OF_DISPENSERS));

    dispenser[0].state = DISPENSER_UP;
    TEST_ASSERT_EQUAL(false, allDispenserSleep(dispenser, NUMBER_OF_DISPENSERS));
    dispenser[0].state = DISPENSER_SLEEP;
    TEST_ASSERT_EQUAL(true, allDispenserSleep(dispenser, NUMBER_OF_DISPENSERS));

    dispenser[NUMBER_OF_DISPENSERS-1].state = DISPENSER_UP;
    TEST_ASSERT_EQUAL(false, allDispenserSleep(dispenser, NUMBER_OF_DISPENSERS));
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(dispenserIsCreatedCorrectly);
    RUN_TEST(allDispenserCanBeCreated);

    RUN_TEST(dispenserMovesWhenStarted);
    RUN_TEST(dispenserHaltTimeCanBeSet);

    RUN_TEST(correctStepsWhenSleep);
    RUN_TEST(correctStepsWhenUp);
    RUN_TEST(correctStepsWhenTop);
    RUN_TEST(correctStepsWhenDown);

    RUN_TEST(allDispensersDoSleep);

    return UNITY_END();
}
