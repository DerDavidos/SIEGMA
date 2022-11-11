#include <stdio.h>

#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "pico/bootrom.h"

#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "TMC2209.h"
#include "SerialUART.h"

#define SERIAL_BAUD_RATE 115200

TMC2209_t setupTMC(TMC2209_t *tmc, SerialAddress_t address) {
    TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(tmc)) {
//        printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", address);
        sleep_ms(1000);
        TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);
    }
//    printf("Setup: Stepper driver with address %i setup and communicating!\n", address);
    TMC2209_setRunCurrent(tmc, 100);
    TMC2209_enable(tmc);
    return (*tmc);
}

int main() {

    if (watchdog_enable_caused_reboot()) {
        reset_usb_boot(0, 0);
    }

    // init usb
    stdio_init_all();
    // Time to make sure everything is ready
    sleep_ms(1000);

    // waits for usb connection, REMOVE to continue without waiting for connection
//    while ((!stdio_usb_connected()));

    TMC2209_t tmc0;
    tmc0 = setupTMC(&tmc0, SERIAL_ADDRESS_0);

    TMC2209_t tmc1;
    tmc1 = setupTMC(&tmc1, SERIAL_ADDRESS_1);

    while (1) {
//        if (!TMC2209_isSetupAndCommunicating(&tmc0))
//            printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", 0);
//        if (!TMC2209_isSetupAndCommunicating(&tmc1))
//            printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", 1);
//        if (TMC2209_disabledByInputPin(&tmc0))
//            printf("Setup: Stepper driver with address %i DISABLED by input bin!\n", 0);
//        if (TMC2209_disabledByInputPin(&tmc1))
//            printf("Setup: Stepper driver with address %i DISABLED by input bin!\n", 1);

        char input = getchar_timeout_us(10000000); /* 10 seconds wait */

        switch (input) {
            case '1':
                TMC2209_moveAtVelocity(&tmc0, 0);
                break;
            case '2':
                TMC2209_moveAtVelocity(&tmc0, 50000);
                break;
            case '3':
                TMC2209_moveAtVelocity(&tmc1, 0);
                break;
            case '4':
                TMC2209_moveAtVelocity(&tmc1, 50000);
                break;
            default:
                break;
        }
    }

    return 0;
}
