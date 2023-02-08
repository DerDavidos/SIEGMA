// Project
#include "dispenser.h"
#include "rondell.h"
// Pico SDK
#include <pico/stdio.h>
#include <pico/time.h>
#include <pico/bootrom.h>
#include <pico/stdio_usb.h>
#include <hardware/watchdog.h>
#include <hardware/adc.h>
// Standard library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// maximum count of allowed input length
#define INPUT_BUFFER_LEN 255
// The uart Pins to be used
#define SERIAL_UART SERIAL2

// sequence of allowed character
const char *allowedCharacters = "0123456789i;\nn";
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
    for (uint8_t i = 0; i < 4; ++i) {
        uint32_t dispenserHaltTimes = parseInputString(&message);
//        printf("Dispenser %i will stop %lu ms\n", i, dispenserHaltTimes);
        setDispenserHaltTime(&dispenser[0], dispenserHaltTimes);

#ifdef RONDELL
        if (dispenserHaltTimes > 0) {
            moveToDispenserWithId(i);
            absolute_time_t time = make_timeout_time_ms(DISPENSER_STEP_TIME_MS);
            do {
                sleep_until(time);
                time = make_timeout_time_ms(DISPENSER_STEP_TIME_MS);
                dispenserDoStep(&dispenser[0]);
            } while (!allDispenserInSleepState(dispenser, 1));
        }
#endif
    }

#ifndef RONDELL
    absolute_time_t time = make_timeout_time_ms(DISPENSER_STEP_TIME_MS);
    do {
        sleep_until(time);
        time = make_timeout_time_ms(DISPENSER_STEP_TIME_MS);
        // Checks for each dispenser if their next state is reached and perform the according action
        for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
            dispenserDoStep(&dispenser[i]);
        }
        // When all dispensers are finished, they are in the state sleep
    } while (!allDispenserInSleepState(dispenser, NUMBER_OF_DISPENSERS));
#endif
}

void initialize_adc(uint8_t gpio, uint8_t input) {
    adc_init();
    adc_gpio_init(gpio);
    adc_select_input(input);
}

// MAIN
int main() {
    initPico(false);

#ifdef RONDELL
    initialize_adc(28, 2);


    char *input_buf_ident = malloc(4);
    memset(input_buf_ident, '\0', 4);
    uint16_t characterCounter_ident = 0;

    volatile bool identified_rondell = false;
    while (!(identified_rondell)) {
        uint32_t input = getchar_timeout_us(10000000); // 10 seconds wait
        if (input == 'i') {
            input_buf_ident[characterCounter_ident] = input;
            input = getchar_timeout_us(10000000);
            if (input == '\n' || input == 'n') {
                free(input_buf_ident);
                input_buf_ident = 0;
                characterCounter_ident = 0;
                identified_rondell = true;
                printf("RONDELL\n");
            }
        }
        else {
            memset(input_buf_ident, '\0', 4);
            characterCounter_ident = 0;
            printf("F\n");
        }
    }

    setUpRondell(2, SERIAL2);
    dispenser[0] = createDispenser(0, SERIAL2);
#else
    // create the dispenser with their address and save them in an array
    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        dispenser[i] = createDispenser(i, SERIAL_UART);
    }
#endif

    printf("CALIR\n");
    // Buffer for received Messages
    char *input_buf = malloc(INPUT_BUFFER_LEN);
    memset(input_buf, '\0', INPUT_BUFFER_LEN);
    uint16_t characterCounter = 0;

#ifdef LEFT
    printf("LEFT\n");
#endif
#ifdef RIGHT
    printf("RIGHT\n");
#endif

    // Waits for an input in the form ([0..9];)+[\n|n], each number standing for the wait time of the corresponding dispenser
    while (true) {


        uint32_t input = getchar_timeout_us(10000000); // 10 seconds wait

        if (input == PICO_ERROR_TIMEOUT)
            continue;

        // ignore the received character if it is not an allowed one
        if (!isAllowedCharacter(input)) {
#ifdef DEBUG
            printf("Received '%c' which is not allowed\n", input);
#endif
            continue;
        }


        // received end character, message should be complete, start with processing
        if (input == 'n' || input == '\n') {
#ifdef DEBUG
            printf("Process Msg len: %d\n", characterCounter);
#endif
            processMessage(input_buf);
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            characterCounter = 0;
            printf("READY\n");
        }
            // received to many characters -> flushing the uart connection and start over
        else if (characterCounter >= INPUT_BUFFER_LEN - 1) {
#ifdef DEBUG
            printf("Input too long, flushing...\n");
#endif
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            characterCounter = 0;
        }
            // character is allowed and we did not reach the end
        else {
#ifdef DEBUG
            printf("Received: %c (counter: %d)\n", input, characterCounter);
#endif
            input_buf[characterCounter] = input;
            ++characterCounter;
        }
    }
}
