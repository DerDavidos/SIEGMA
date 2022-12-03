#include <stdio.h>
#include <stdlib.h>

#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/bootrom.h"

#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "TMC2209.h"
#include "SerialUART.h"
#include "pico/stdio_usb.h"

#include <string.h>

#define SERIAL_BAUD_RATE 115200

#define STEPPER0_LS_PIN 7
#define STEPPER1_LS_PIN 3
#define STEPPER2_LS_PIN 6
#define STEPPER3_LS_PIN 2

#define MAX_SLEEP_INPUT 5

#define INPUT_BUFFER_LEN 26

#define TIME_MOTOR_IS_MOVING_UP 5000

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

void initTouchSensor() {
    gpio_init(STEPPER0_LS_PIN);
    gpio_set_dir(STEPPER0_LS_PIN, GPIO_IN);

    gpio_init(STEPPER1_LS_PIN);
    gpio_set_dir(STEPPER1_LS_PIN, GPIO_IN);

    gpio_init(STEPPER2_LS_PIN);
    gpio_set_dir(STEPPER2_LS_PIN, GPIO_IN);

    gpio_init(STEPPER3_LS_PIN);
    gpio_set_dir(STEPPER3_LS_PIN, GPIO_IN);
}

int parseInputString(char *buf, unsigned buf_len, int *p_semiPos) {
    char *semicolon = strchr(buf, ';');
    if (semicolon == NULL) {
        // haben kein Komma gefunden -> :(
        return 0;
    }
    int semi_pos = semicolon - buf;
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

void processMsg(char *buf, unsigned buf_len, TMC2209_t tmcs[]) {
    int stopTimes[4];

    int semi1Pos, semi2Pos, semi3Pos, semi4Pos;

    stopTimes[0] = parseInputString(buf, buf_len, &semi1Pos);
    stopTimes[1] = parseInputString(buf + semi1Pos + 1, buf_len - (semi1Pos + 1), &semi2Pos);
    stopTimes[2] = parseInputString(buf + semi1Pos + semi2Pos + 2, buf_len - (semi1Pos + semi2Pos + 2), &semi3Pos);
    stopTimes[3] = parseInputString(buf + semi1Pos + semi2Pos + semi3Pos + 3,
                                    buf_len - (semi1Pos + semi2Pos + semi3Pos + 3), &semi4Pos);

    printf("stepper0_pos: %d\n", semi1Pos);
    printf("stepper1_pos: %d\n", semi2Pos);
    printf("stepper2_pos: %d\n", semi3Pos);
    printf("stepper3_pos: %d\n", semi4Pos);

    printf("buffer len: %d\n", buf_len);

    for (int i = 0; i < 4; ++i) {
        printf("stepper %i: %d\n", i, stopTimes[i]);
    }

    bool steppersReady[4] = {true, true, true, true};

    int numberThatNeedsToBeTriggered = 0;

    for (int i = 0; i < 4; ++i) {
        if (stopTimes[i] > 0) {
            ++numberThatNeedsToBeTriggered;
            TMC2209_moveAtVelocity(&tmcs[i], -50000);
            steppersReady[i] = false;
        }
    }

    sleep_ms(TIME_MOTOR_IS_MOVING_UP); // Let selected motors move

    for (int i = 0; i < 4; ++i) {
        if (!steppersReady[i])
            TMC2209_moveAtVelocity(&tmcs[i], 0);
    }

    bool isGoingDown[4] = {false, false, false, false};
    int timeElapsed = 0;

    while (numberThatNeedsToBeTriggered > 0) {

        for (int i = 0; i < 4; ++i) {
            if (!steppersReady[i]) {
                if (isGoingDown[i]) {
                    if (gpio_get(STEPPER0_LS_PIN)) {
                        --numberThatNeedsToBeTriggered;
                        TMC2209_moveAtVelocity(&tmcs[0], 0);
                        steppersReady[i] = true;
                    }
                } else if (timeElapsed >= stopTimes[i]) {
                    TMC2209_moveAtVelocity(&tmcs[0], 50000);
                    isGoingDown[i] = true;
                }
            }
        }

        sleep_ms(10);
        timeElapsed += 10;
    }
}

void setupTMC(TMC2209_t *tmc, SerialAddress_t address) {
    TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(tmc)) {
        printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", address);
        sleep_ms(1000);
        TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);
    }
    printf("Setup: Stepper driver with address %i setup and communicating!\n", address);
    TMC2209_setRunCurrent(tmc, 100);
    TMC2209_enable(tmc);
}

int main() {

    initPico(false);

    initTouchSensor();

    TMC2209_t tmcs[4] = {};
    setupTMC(&tmcs[0], SERIAL_ADDRESS_0);
    setupTMC(&tmcs[1], SERIAL_ADDRESS_1);
    setupTMC(&tmcs[2], SERIAL_ADDRESS_2);
    setupTMC(&tmcs[3], SERIAL_ADDRESS_3);

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

        if (input == 'n') {
            printf("Process Msg len: %d\n", characterCounter);
            processMsg(input_buf, characterCounter, tmcs);
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

