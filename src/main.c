#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include "pico/stdio_usb.h"
#include "hardware/watchdog.h"

#include "dispenser.h"

#define INPUT_BUFFER_LEN 26
#define SERIAL_UART SERIAL2

const char *allowedCharacters = "0123456789;\nn";
Dispenser_t dispenser[NUMBER_OF_DISPENSERS];

void initPico(bool waitForUSBConnection) {
    if (watchdog_enable_caused_reboot())
        reset_usb_boot(0, 0);

    stdio_init_all(); // init usb
    sleep_ms(2500); // Time to make sure everything is ready

    if (waitForUSBConnection)
        while ((!stdio_usb_connected())); // waits for usb connection
}

bool isAllowedCharacter(uint32_t input) {
    for (uint32_t i = 0; i < strlen(allowedCharacters); ++i) {
        if (input == allowedCharacters[i]) {
            return true;
        }
    }
    return false;
}

uint32_t parseInputString(char **message){
    char *semicolonPosition = strchr(*message, ';');
    if (semicolonPosition == NULL)
        return 0; // No Semicolon found
    uint32_t delay = strtol(*message, &semicolonPosition, 10);
    *message = semicolonPosition + 1;
    return delay;
}

void processMessage(char *message) {
    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        uint32_t dispenserHaltTimes = parseInputString(&message);
        printf("Dispenser %i will stop %lu ms\n", i, dispenserHaltTimes);
        setDispenserHaltTime(&dispenser[i], dispenserHaltTimes);
        if (dispenserHaltTimes > 0)
            startDispenser(&dispenser[i]);
    }

    uint32_t timeElapsed = 0;
    while (!allDispenserSleep(dispenser, NUMBER_OF_DISPENSERS)) {
        for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
            dispenserDoStep(&dispenser[i], timeElapsed);
        }

        sleep_ms(10);
        timeElapsed += 10;
    }
}

int main() {
    initPico(false);

    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        dispenser[i] = createDispenser(i, SERIAL_UART);
    }

    char *input_buf = malloc(INPUT_BUFFER_LEN);
    memset(input_buf, '\0', INPUT_BUFFER_LEN);
    uint16_t characterCounter = 0;

    while (true) {
        uint32_t input = getchar_timeout_us(10000000); // 10 seconds wait

        if (input == PICO_ERROR_TIMEOUT)
            continue;

        if (!isAllowedCharacter(input)) {
            printf("Received '%c' which is not allowed\n", input);
            continue;
        }

        if (input == 'n' || input == '\n') {
            printf("Process Msg len: %d\n", characterCounter);
            processMessage(input_buf);
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
        } else if (characterCounter >= INPUT_BUFFER_LEN - 1) {
            printf("Input too long, flushing...\n");
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            characterCounter = 0;
        } else {
            printf("Received: %c (counter: %d)\n", input, characterCounter);
            input_buf[characterCounter] = input;
            ++characterCounter;
        }
    }
}
