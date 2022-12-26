#include "limitSwitch.h"
#include "dispenser/dispenser.h"

#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include "pico/printf.h"

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

    limitSwitch_t limitSwitch[NUMBER_OF_DISPENSERS];

    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        limitSwitch[i] = createLimitSwitch(i);
    }

    while (true) {
        for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
            if (limitSwitchIsClosed(limitSwitch[i]))
                printf("Switch %i is closed\n", i);
            else
                printf("Switch %i is open\n", i);
        }
        printf("#####################\n");
        sleep_ms(1000);
    }
}
