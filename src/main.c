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

int main() {

    if (watchdog_enable_caused_reboot()) {
        reset_usb_boot(0, 0);
    }

    // init usb
    stdio_init_all();

    // waits for usb connection, REMOVE to continue without waiting for connection
//    while ((!stdio_usb_connected()));

    TMC2209_t tmc0 = TMC2209_getNew();
    TMC2209_setup(tmc0, SERIAL1, SERIAL_BAUD_RATE, SERIAL_ADDRESS_0);
    TMC2209_setRunCurrent(tmc0, 100);
    TMC2209_enable(tmc0);

    while (!TMC2209_isSetupAndCommunicating(tmc0)) {
        printf("Setup: Stepper driver 1 NOT setup and communicating!\n");
        sleep_ms(1000);
        TMC2209_setup(tmc0, SERIAL1, SERIAL_BAUD_RATE, SERIAL_ADDRESS_0);
    }
    printf("Setup: Stepper driver 1 setup and communicating!\n");


    TMC2209_t tmc1 = TMC2209_getNew();
    TMC2209_setup(tmc1, SERIAL1, SERIAL_BAUD_RATE, SERIAL_ADDRESS_1);
    TMC2209_setRunCurrent(tmc1, 100);
    TMC2209_enable(tmc1);

    while (!TMC2209_isSetupAndCommunicating(tmc1)) {
        printf("Setup: Stepper driver 2 NOT setup and communicating!\n");
        sleep_ms(1000);
        TMC2209_setup(tmc1, SERIAL1, SERIAL_BAUD_RATE, SERIAL_ADDRESS_0);
    }
    printf("Setup: Stepper driver 2 setup and communicating!\n");

    while (1) {
        while (!TMC2209_isSetupAndCommunicating(tmc0)) {
            printf("Stepper driver 1 NOT setup and communicating!\n");
            sleep_ms(1000);
        }
        while (!TMC2209_isSetupAndCommunicating(tmc1)) {
            printf("Stepper driver 2 NOT setup and communicating!\n");
            sleep_ms(1000);
        }
        TMC2209_moveAtVelocity(tmc0, 0);
        TMC2209_moveAtVelocity(tmc1, 50000);
        sleep_ms(1000);
        TMC2209_moveAtVelocity(tmc0, 50000);
        TMC2209_moveAtVelocity(tmc1, 0);
        sleep_ms(1000);
    }

    return 0;
}
