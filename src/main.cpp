#include <cstdio>
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "hardware/uart.h"

#include "TMC2209.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
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

    sleep_ms(100);
    printf("Connected\n");


    TMC2209 tmc;
    tmc.setup(Serial1, SERIAL_BAUD_RATE);
    printf("Setup: TMC2209 setup done.\n");

    while (!tmc.isSetupAndCommunicating()) {
        printf("Setup: Stepper driver NOT setup and communicating!\n");
        sleep_ms(1000);
        tmc.setup(Serial1, SERIAL_BAUD_RATE);
    }
    printf("Setup: Stepper driver setup and communicating!\n");

    tmc.setRunCurrent(100);
    tmc.enable();

    while (1) {
        while (!tmc.isSetupAndCommunicating()) {
            printf("Stepper driver NOT setup and communicating!\n");
            sleep_ms(1000);
        }
        tmc.moveAtVelocity(0);
        sleep_ms(1000);
        tmc.moveAtVelocity(50000);
        sleep_ms(1000);
    }

    return 0;
}
