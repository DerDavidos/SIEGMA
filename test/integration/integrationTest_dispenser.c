#include "dispenser.h"
#include "serialUART.h"

#include <hardware/watchdog.h>
#include <pico/bootrom.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <pico/stdio_usb.h>
#include <pico/printf.h>

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

    printf("First write Stepper ID (0-%i) and then command: Setup (s), Set Time (t), Run (r), Halt (while in run) (h)\n",
           NUMBER_OF_DISPENSERS - 1);

    Dispenser_t dispenser[NUMBER_OF_DISPENSERS];

    while (true) {
        uint32_t id = getchar_timeout_us(10000000); // 10 seconds wait
        if (id == PICO_ERROR_TIMEOUT)
            continue;
        id = id - 48;
        if (id > NUMBER_OF_DISPENSERS - 1) {
            printf("Wrong ID\n");
            continue;
        }
        uint32_t command = getchar_timeout_us(10000000); // 10 seconds wait
        if (command == PICO_ERROR_TIMEOUT) {
            printf("No command received\n");
            continue;
        }
        switch (command) {
            case 's':
                printf("Setup dispenser: %lu\n", id);
                dispenser[id] = createDispenser(id, SERIAL_UART);
                break;
            case 't':
                printf("Set halt time for dispenser %lu\n: ", id);
                uint32_t haltTime = getchar_timeout_us(10000000);
                printf("%lu", haltTime);
                setDispenserHaltTime(&dispenser[id], haltTime);
                break;
            case 'r':
                printf("Running");
                absolute_time_t time = make_timeout_time_ms(DISPENSER_STEP_TIME_MS);
                do {
                    uint32_t commandInRun = getchar_timeout_us(0); // 10 seconds wait
                    if (commandInRun == 'h') {
                        break;
                    }
                    sleep_until(time);
                    time = make_timeout_time_ms(DISPENSER_STEP_TIME_MS);
                    // Checks for each dispenser if their next state is reached and perform the according action
                    dispenserDoStep(&dispenser[id]);
                    // When all dispensers are finished, they are in the state sleep
                } while (dispenser->state.function != sleepState);
                printf("Ready\n");
                break;
            default:
                printf("Wrong Command!\n");
                printf("First write Stepper ID (0-%i) and then command: Setup (s), Set Time (t), Run (r) [ID not relevant], Halt (while in run) (h)\n",
                       NUMBER_OF_DISPENSERS - 1);
        }
        printf("\n");
    }
}
