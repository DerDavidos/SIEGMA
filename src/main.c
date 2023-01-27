#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include "pico/stdio_usb.h"
#include "hardware/watchdog.h"
#include "hardware/adc.h"

#include "rondell.h"
#include "Dispenser.h"

#define MAX_SLEEP_INPUT 5


#define INPUT_BUFFER_LEN 26

Dispenser_t dispenser[1];

const char *allowedCharacters = "0123456789;\nn";

void initPico(bool waitForUSBConnection) {
    if (watchdog_enable_caused_reboot()) {
        reset_usb_boot(0, 0);
    }

    // init usb
    stdio_init_all();
    // Time to make sure everything is ready
    sleep_ms(5000);

    // waits for usb connection
    if (waitForUSBConnection) {
        while ((!stdio_usb_connected()));
    }
}

int parseInputString(char *buf, unsigned buf_len, int *p_semiPos) {
    char *semicolonPosition = strchr(buf, ';');
    if (semicolonPosition == NULL) {
        // haben kein Komma gefunden -> :(
        return 0;
    }
    int semi_pos = semicolonPosition - buf;
    *p_semiPos = semi_pos;
    if (buf_len < semi_pos || semi_pos > MAX_SLEEP_INPUT || semi_pos <= 0) {
        return 0;
    } else {
        char stepper1_number_buf[MAX_SLEEP_INPUT + 1];
        memcpy(stepper1_number_buf, buf, semi_pos);
        stepper1_number_buf[semi_pos] = '\0';
        stepper1_number_buf[MAX_SLEEP_INPUT] = '\0';
        return strtol(stepper1_number_buf, NULL, 10);
    }
}

void processMessage(char *message, unsigned message_length) {
    int dispenserHaltTimes[4];

    int semi1Pos, semi2Pos, semi3Pos, semi4Pos;

    dispenserHaltTimes[0] = parseInputString(message, message_length, &semi1Pos);
    dispenserHaltTimes[1] = parseInputString(message + semi1Pos + 1, message_length - (semi1Pos + 1), &semi2Pos);
    dispenserHaltTimes[2] = parseInputString(message + semi1Pos + semi2Pos + 2,
                                             message_length - (semi1Pos + semi2Pos + 2), &semi3Pos);
    dispenserHaltTimes[3] = parseInputString(message + semi1Pos + semi2Pos + semi3Pos + 3,
                                             message_length - (semi1Pos + semi2Pos + semi3Pos + 3), &semi4Pos);

    printf("stepper0_pos: %d\n", semi1Pos);
    printf("stepper1_pos: %d\n", semi2Pos);
    printf("stepper2_pos: %d\n", semi3Pos);
    printf("stepper3_pos: %d\n", semi4Pos);

    printf("buffer len: %d\n", message_length);

    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; i++) {
        printf("stepper %i: %d\n", i, dispenserHaltTimes[i]);
    }

    // Check whether dispenser halt time is > 0: if yes move to that dispenser and yield the drink.
    // When final dispenser halt time is processed, the rondell stops.
    for (uint8_t i = 0; i < NUMBER_OF_DISPENSERS; i++) {
        if (dispenserHaltTimes[i] > 0) {
            setDispenserHaltTime(dispenser, dispenserHaltTimes[i]);
            moveToDispenserWithId(i);
            do {
                dispenserDoStep(dispenser);
                sleep_ms(DISPENSER_STEP_TIME_MS);
            } while (allDispenserInSleepState(dispenser, 1));
        }
    }
}

void initialize_adc(uint8_t gpio, uint8_t input) {
    adc_init();
    adc_gpio_init(gpio);
    adc_select_input(input);
}

int main() {

    initPico(false);

    initialize_adc(28,2);

    // dispenser[0] = createDispenser(0, SERIAL2);

    setUpRondell(1,SERIAL2);

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
