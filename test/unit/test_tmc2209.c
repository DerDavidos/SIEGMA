#include <unity.h>

#include "tmc2209.h"

void setUp() {}

void tearDown() {}

void setUpTMC2209() {
    TMC2209_t *tmc;
    TMC2209_setup(tmc, , 0, 0);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(setUpTMC2209);

    return UNITY_END();
}
