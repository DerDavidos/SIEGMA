#include "dispenser.h"
#include "serialUART.h"

#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include "pico/printf.h"

#define SERIAL_UART SERIAL2

void initPico(bool waitForUSBConnection) {
    if (watchdog_enable_caused_reboot())
        reset_usb_boot(0, 0);

    stdio_init_all(); // init usb
    sleep_ms(2500); // Time to make sure everything is ready

    if (waitForUSBConnection)
        while ((!stdio_usb_connected())); // waits for usb connection
}

int main() {
    initPico(true);

    printf("First write Stepper ID (0-%i) and then command: Setup (s), Up (u), Down (d), Stop(s)",
           NUMBER_OF_DISPENSERS);

    while (true) {
        int id = getchar_timeout_us(10000000); // 10 seconds wait
        char command = getchar_timeout_us(10000000); // 10 seconds wait
        if (id == '\0' || command == '\0') {
            printf("First write Stepper ID (0-%i) and then command: Setup (s), Up (u), Down (d), Halt(h), Get Direction (g)",
                   NUMBER_OF_DISPENSERS);
            continue;
        }
        switch (command) {
            case 's':
                printf("Setup dispenser %i", id);
                setUpDispenser(id, SERIAL_UART);
                break;
            case 'u':
                printf("Move dispenser %i up", id);
                moveDispenserUp(id);
                break;
            case 'd':
                printf("Move dispenser %i down", id);
                moveDispenserDown(id);
            case 'h':
                printf("Stop dispenser %i", id);
                stopDispenser(id);
                break;
            case 'g':
                printf("Direction of dispenser %i: %i (UP: %i, DOWN: %i, STOP: %i)", id, getDispenserDirection(id), UP,
                       DOWN, STOP);
            default:
                printf("Wrong Command");
        }

    }

}
