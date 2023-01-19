// Project
#include "dispenser.h"
// Pico SDK
#include <pico/stdio.h>
#include <pico/time.h>
#include <pico/bootrom.h>
#include <pico/stdio_usb.h>
#include <hardware/watchdog.h>
// Standard library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// maximum count of allowed input length
#define INPUT_BUFFER_LEN 255
// The uart Pins to be used
#define SERIAL_UART SERIAL2

// sequence of allowed character
const char *allowedCharacters = "0123456789;\nn";
// Array containing the dispenser
Dispenser_t dispenser[NUMBER_OF_DISPENSERS];

// initialize the usb connection to the pico
// @param bool if the pico should wait for an usb connection
// @return void
void initPico(bool waitForUSBConnection) {
    if (watchdog_enable_caused_reboot())
        reset_usb_boot(0, 0);

    stdio_init_all(); // init usb
    sleep_ms(2500); // Time to make sure everything is ready

    if (waitForUSBConnection)
        while ((!stdio_usb_connected())); // waits for usb connection
}

// check if the input character is allowed
// @param the character to check
// @return true if the input character is in an allowed sequence
bool isAllowedCharacter(uint32_t input) {
    for (uint32_t i = 0; i < strlen(allowedCharacters); ++i) {
        if (input == allowedCharacters[i]) {
            return true;
        }
    }
    return false;
}

// parse a String to fetch the hopper halt timings
// @param the received message containing the halt timing
// @return The parsed halt timing as an integer or on failure, a Zero
uint32_t parseInputString(char **message) {
    // every halt timing command has to end with a ';'
    // the function will search for this char and save its position
    char *semicolonPosition = strchr(*message, ';');
    if (semicolonPosition == NULL) {
        return 0; // No Semicolon found
    }
    // the string will be cast, from the beginning of the string to the ';'-Position, into an integer
    uint32_t delay = strtol(*message, &semicolonPosition, 10);
    *message = semicolonPosition + 1;
    return delay;
}

// process the received Message (received over Serial)
// @param char buffer containing the received Message
// @return void
void processMessage(char *message) {
    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        uint32_t dispenserHaltTimes = parseInputString(&message);
//        printf("Dispenser %i will stop %lu ms\n", i, dispenserHaltTimes);
        setDispenserHaltTime(&dispenser[i], dispenserHaltTimes);
    }

    do {
        // Checks for each dispenser if their next state is reached and perform the according action
        for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
            dispenserDoStep(&dispenser[i]);
        }

        sleep_ms(DISPENSER_STEP_TIME_MS);
        // When all dispensers are finished, they are in the state sleep
    } while (!allDispenserInSleepState(dispenser, NUMBER_OF_DISPENSERS));
}

// MAIN
int main() {
    initPico(false);

    // create the dispenser with their adress and save them in an array
    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        dispenser[i] = createDispenser(i, SERIAL_UART);
    }

    // Buffer for received Messages
    char *input_buf = malloc(INPUT_BUFFER_LEN);
    memset(input_buf, '\0', INPUT_BUFFER_LEN);
    uint16_t characterCounter = 0;

    // Waits for an input in the form ([0..9];)+[\n|n], each number standing for the wait time of the corresponding dispenser
    while (true) {
        uint32_t input = getchar_timeout_us(10000000); // 10 seconds wait

        if (input == PICO_ERROR_TIMEOUT)
            continue;

        // ignore the received character if it is not an allowed one
        if (!isAllowedCharacter(input)) {
            printf("Received '%c' which is not allowed\n", input);
            continue;
        }

        // received end character, message should be complete, start with processing
        if (input == 'n' || input == '\n') {
            printf("Process Msg len: %d\n", characterCounter);
            processMessage(input_buf);
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            characterCounter = 0;
            printf("Finished\n");
        }
        // received to many characters -> flushing the uart connection and start over
        else if (characterCounter >= INPUT_BUFFER_LEN - 1) {
            printf("Input too long, flushing...\n");
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            characterCounter = 0;
        }
        // character is allowed and we did not reach the end
        else {
            printf("Received: %c (counter: %d)\n", input, characterCounter);
            input_buf[characterCounter] = input;
            ++characterCounter;
        }
    }
}
