#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include "pico/stdio_usb.h"
#include "hardware/watchdog.h"

#include "dispenser.h"
#include "limitSwitch.h"

#define MAX_SLEEP_INPUT 5
#define INPUT_BUFFER_LEN 26
#define SERIAL_UART SERIAL2

const char *allowedCharacters = "0123456789;\nn";

void initPico(bool waitForUSBConnection) {
    if (watchdog_enable_caused_reboot())
        reset_usb_boot(0, 0);

    stdio_init_all(); // init usb
    sleep_ms(2500); // Time to make sure everything is ready

    if (waitForUSBConnection)
        while ((!stdio_usb_connected())); // waits for usb connection
}

int parseInputString(char *message, unsigned *message_length) {
    char *semicolonPosition = strchr(message, ';');
    if (semicolonPosition == NULL)
        return 0; // haben kein Komma gefunden -> :(

    int semi_pos = semicolonPosition - message;
    if (*message_length < semi_pos || semi_pos > MAX_SLEEP_INPUT || semi_pos <= 0)
        return 0;

    char stepper1_number_buf[MAX_SLEEP_INPUT + 1];
    memcpy(stepper1_number_buf, message, semi_pos);
    stepper1_number_buf[semi_pos] = '\0';
    stepper1_number_buf[MAX_SLEEP_INPUT] = '\0';
    *message = *message + semi_pos + 1;
    *message_length = *message_length - (semi_pos + 1);
    return strtol(stepper1_number_buf, NULL, 10);
}

void startSelectedDispensers(const int *dispenserHaltTimes) {
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        if (dispenserHaltTimes[i] > 0)
            moveDispenserUp(i);
    }
}

void stopDispensersAtTheTop(const int *dispenserHaltTimes) {
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        if (dispenserHaltTimes[i] > 0)
            stopDispenser(i);
    }
}

void moveSelectedDispensersToTopPosition(const int *dispenserHaltTimes) {
    startSelectedDispensers(dispenserHaltTimes);
    sleep_ms(TIME_DISPENSERS_ARE_MOVING_UP); // Let selected motors move
    stopDispensersAtTheTop(dispenserHaltTimes);
}

void moveDispensersDownWhenFinished(const int dispenserHaltTimes[4], int timeElapsed) {
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        if (timeElapsed > dispenserHaltTimes[i] && getDispenserDirection(i) == STOP && !limitSwitchIsClosed(i))
            moveDispenserDown(i);
    }
}

void stopDispensersIfAtTheBottom(const int *dispenserHaltTimes, int timeElapsed) {
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        if (timeElapsed > dispenserHaltTimes[i] && getDispenserDirection(i) == DOWN && limitSwitchIsClosed(i))
            stopDispenser(i);
    }
}

void processMessage(char *message, unsigned message_length) {
    int dispenserHaltTimes[4];
    for (int i = 0; i < NUMBER_OF_DISPENSERS; ++i) {
        dispenserHaltTimes[i] = parseInputString(message, &message_length);
        printf("stepper %i: %d\n", i, dispenserHaltTimes[i]);
    }

    moveSelectedDispensersToTopPosition(dispenserHaltTimes);

    int timeElapsed = 0;
    while (!allLimitSwitchesAreClosed()) {
        moveDispensersDownWhenFinished(dispenserHaltTimes, timeElapsed);
        stopDispensersIfAtTheBottom(dispenserHaltTimes, timeElapsed);

        sleep_ms(10);
        timeElapsed += 10;
    }
}

int main() {
    initPico(false);

    setUpAllLimitSwitches();

    setUpAllDispensers(SERIAL_UART);

    char *input_buf = malloc(INPUT_BUFFER_LEN);
    memset(input_buf, '\0', INPUT_BUFFER_LEN);
    unsigned characterCounter = 0;

    while (true) {
        char input = getchar_timeout_us(10000000); // 10 seconds wait

        bool isAllowedCharacter = false;
        for (int i = 0; i < strlen(allowedCharacters); ++i) {
            if (input == allowedCharacters[i]) {
                isAllowedCharacter = true;
            }
        }
        if (isAllowedCharacter == false) {
            printf("Received '%c' which is not allowed\n", input);
            continue;
        }

        if (input == 'n' || input == '\n') {
            printf("Process Msg len: %d\n", characterCounter);
            processMessage(input_buf, characterCounter);
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            characterCounter = 0;
        } else if (characterCounter >= INPUT_BUFFER_LEN - 1) {
            // Zu viele Bytes empfangen -> Buffer leeren und bereit für die nächste Anfrage sein
            // Hier kann noch eingefügt werden, dass der Pi eine Rückmeldung bekommt
            printf("Wrong input\n");
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            characterCounter = 0;
        } else {
            printf("Received: %c (counter: %d)\n", input, characterCounter);
            input_buf[characterCounter] = input;
            ++characterCounter;
        }
    }
}
