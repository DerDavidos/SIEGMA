#include "dispenser.h"

#include "serialUART.h"

#include <unity.h>

#define ID 0

void setUp() {
    // If Fails, struct was changed and tests need to be adjusted
    TEST_ASSERT_EQUAL(32, sizeof(Dispenser_t));
}

void tearDown() {}

void testDispenserCycle(void) {
//    Dispenser_t dispenser = createDispenser(ID, SERIAL);
//
//    setDispenserHaltTime(&dispenser, 10);
//
//    TEST_ASSERT_EQUAL(sleepState, dispenser.state.function);
//
//    for (int i = 0; i < STEPS_DISPENSERS_ARE_MOVING_UP; ++i) {
//        dispenserDoStep(&dispenser);
//    }
//
//    TEST_ASSERT_EQUAL(sleepState, dispenser.state.function);
//    Dispenser_t expected = (Dispenser_t) {.address=ID, .state=DISPENSER_SLEEP, .haltTime=0,
//            .motor=createMotor(ID, SERIAL),
//            .limitSwitch=createLimitSwitch(ID)};

}


int main(void) {
    UNITY_BEGIN();

    RUN_TEST(testDispenserCycle);

    return UNITY_END();
}
