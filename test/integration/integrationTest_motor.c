#include "dispenser.h"
#include "serialUART.h"

#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include "pico/printf.h"
#include "motor/motor.h"

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

    printf("First write Stepper ID (0-%i) and then command: Setup (s), Up (u), Down (d), Stop(s)\n",
           NUMBER_OF_DISPENSERS-1);

    Motor_t motor[4];

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
                motor[id] = createMotor(id, SERIAL_UART);
                break;
            case 'u':
                printf("Move dispenser up: %lu\n", id);
                moveMotorUp(&motor[id]);
                break;
            case 'd':
                printf("Move dispenser down: %lu\n", id);
                moveMotorDown(&motor[id]);
                break;
            case 'h':
                printf("Stop dispenser: %lu\n", id);
                stopMotor(&motor[id]);
                break;
            default:
                printf("Wrong Command!\n");
                printf("First write Stepper ID (0-%i) and then command: Setup (s), Up (u), Down (d), Halt(h), Get Direction (g)\n",
                       NUMBER_OF_DISPENSERS - 1);
        }
        printf("\n");
    }
}
