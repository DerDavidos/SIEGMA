#include <stdio.h>
#include <stdlib.h>

#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/bootrom.h"

#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "TMC2209.h"
#include "SerialUART.h"

#include <string.h>

#define SERIAL_BAUD_RATE 115200

#define STEPPER0_LS_PIN 7
#define STEPPER1_LS_PIN 3
#define STEPPER2_LS_PIN 6
#define STEPPER3_LS_PIN 2

#define MAX_SLEEP_INPUT 5

#define INPUT_BUFFER_LEN 26

const char *allowedCharacters = "0123456789;\n";

int parseInputString(char *buf, unsigned buf_len, int *p_semiPos) {
    char *semicolon = strchr(buf, ';');
    if (semicolon == NULL) {
        // haben kein Komma gefunden -> :(
        return -1;
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

void processMsg(char *buf, unsigned buf_len, TMC2209_t *tmc0, TMC2209_t *tmc1, TMC2209_t *tmc2, TMC2209_t *tmc3) {
    int tmc0StopTime, tmc1StopTime, tmc2StopTime, tmc3StopTime;

    int semi1Pos, semi2Pos, semi3Pos, semi4Pos;

    tmc0StopTime = parseInputString(buf, buf_len, &semi1Pos);
    if (tmc0StopTime < 0) {
        return;
    }
    tmc1StopTime = parseInputString(buf + semi1Pos + 1, buf_len - (semi1Pos + 1), &semi2Pos);
    if (tmc1StopTime < 0) {
        return;
    }
    tmc2StopTime = parseInputString(buf + semi1Pos + semi2Pos + 2, buf_len - (semi1Pos + semi2Pos + 2), &semi3Pos);
    if (tmc2StopTime < 0) {
        return;
    }
    tmc3StopTime = parseInputString(buf + semi1Pos + semi2Pos + semi3Pos + 3,
                                    buf_len - (semi1Pos + semi2Pos + semi3Pos + 3), &semi4Pos);
    if (tmc3StopTime < 0) {
        return;
    }

    printf("stepper0_pos: %d\n", semi1Pos);
    printf("stepper1_pos: %d\n", semi2Pos);
    printf("stepper2_pos: %d\n", semi3Pos);
    printf("stepper3_pos: %d\n", semi4Pos);

    printf("buffer len: %d\n", buf_len);

    printf("stepper0: %d\n", tmc0StopTime);
    printf("stepper1: %d\n", tmc1StopTime);
    printf("stepper2: %d\n", tmc2StopTime);
    printf("stepper3: %d\n", tmc3StopTime);

    bool stepper0Ready = false;
    bool stepper1Ready = false;
    bool stepper2Ready = false;
    bool stepper3Ready = false;

    int numberThatNeedsToBeTriggered = 0;

    if (tmc0StopTime > 0) {
        ++numberThatNeedsToBeTriggered;
        TMC2209_moveAtVelocity(tmc0, -50000);
    } else {
        stepper0Ready = true;
    }
    if (tmc1StopTime > 0) {
        ++numberThatNeedsToBeTriggered;
        TMC2209_moveAtVelocity(tmc1, -50000);
    } else {
        stepper1Ready = true;
    }
    if (tmc2StopTime > 0) {
        ++numberThatNeedsToBeTriggered;
        TMC2209_moveAtVelocity(tmc2, -50000);
    } else {
        stepper2Ready = true;
    }
    if (tmc3StopTime > 0) {
        ++numberThatNeedsToBeTriggered;
        TMC2209_moveAtVelocity(tmc3, -50000);
    } else {
        stepper3Ready = true;
    }

    sleep_ms(5000);

    if (!stepper0Ready) {
        TMC2209_moveAtVelocity(tmc0, 0);
    }
    if (!stepper1Ready) {
        TMC2209_moveAtVelocity(tmc1, 0);
    }
    if (!stepper2Ready) {
        TMC2209_moveAtVelocity(tmc2, 0);
    }
    if (!stepper3Ready) {
        TMC2209_moveAtVelocity(tmc3, 0);
    }

    bool tmc0IsGoingDown = 0;
    bool tmc1IsGoingDown = 0;
    bool tmc2IsGoingDown = 0;
    bool tmc3IsGoingDown = 0;

    while (numberThatNeedsToBeTriggered) {   // kann nicht 1 bleiben
        if (!stepper0Ready) {
            if (tmc0IsGoingDown) {
                if (gpio_get(STEPPER0_LS_PIN)) {
                    --numberThatNeedsToBeTriggered;
                    TMC2209_moveAtVelocity(tmc0, 0);
                    stepper0Ready = true;
                }
            } else {
                tmc0StopTime = tmc0StopTime - 10;

                if (tmc0StopTime <= 0) {
                    TMC2209_moveAtVelocity(tmc0, 50000);
                    tmc0IsGoingDown = 1;
                }
            }
        }
        if (!stepper1Ready) {
            if (tmc1IsGoingDown) {
                if (gpio_get(STEPPER1_LS_PIN)) {
                    --numberThatNeedsToBeTriggered;
                    TMC2209_moveAtVelocity(tmc1, 0);
                    stepper1Ready = true;
                }
            } else {
                tmc1StopTime = tmc1StopTime - 10;

                if (tmc1StopTime <= 0) {
                    TMC2209_moveAtVelocity(tmc1, 50000);
                    tmc1IsGoingDown = 1;
                }
            }
        }
        if (!stepper2Ready) {
            if (tmc2IsGoingDown) {
                if (gpio_get(STEPPER2_LS_PIN)) {
                    --numberThatNeedsToBeTriggered;
                    TMC2209_moveAtVelocity(tmc2, 0);
                    stepper2Ready = true;
                }
            } else {
                tmc2StopTime = tmc2StopTime - 10;

                if (tmc2StopTime <= 0) {
                    TMC2209_moveAtVelocity(tmc2, 50000);
                    tmc2IsGoingDown = 1;
                }
            }
        }
        if (!stepper3Ready) {
            if (tmc3IsGoingDown) {
                if (gpio_get(STEPPER3_LS_PIN)) {
                    --numberThatNeedsToBeTriggered;
                    TMC2209_moveAtVelocity(tmc3, 0);
                    stepper3Ready = true;
                }
            } else {
                tmc3StopTime = tmc3StopTime - 10;

                if (tmc3StopTime <= 0) {
                    TMC2209_moveAtVelocity(tmc3, 50000);
                    tmc3IsGoingDown = 1;
                }
            }
        }
        sleep_ms(10);
    }
}

TMC2209_t setupTMC(TMC2209_t *tmc, SerialAddress_t address) {
    TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);

    while (!TMC2209_isSetupAndCommunicating(tmc)) {
        printf("Setup: Stepper driver with address %i NOT setup and communicating!\n", address);
        sleep_ms(1000);
        TMC2209_setup(tmc, SERIAL1, SERIAL_BAUD_RATE, address);
    }
    printf("Setup: Stepper driver with address %i setup and communicating!\n", address);
    TMC2209_setRunCurrent(tmc, 100);
    TMC2209_enable(tmc);
    return (*tmc);
}

int main() {

    if (watchdog_enable_caused_reboot()) {
        reset_usb_boot(0, 0);
    }

    // init usb
    stdio_init_all();
    // Time to make sure everything is ready
    sleep_ms(5000);

    gpio_init(STEPPER0_LS_PIN);
    gpio_set_dir(STEPPER0_LS_PIN, GPIO_IN);

    gpio_init(STEPPER1_LS_PIN);
    gpio_set_dir(STEPPER1_LS_PIN, GPIO_IN);

    gpio_init(STEPPER2_LS_PIN);
    gpio_set_dir(STEPPER2_LS_PIN, GPIO_IN);

    gpio_init(STEPPER3_LS_PIN);
    gpio_set_dir(STEPPER3_LS_PIN, GPIO_IN);

    // waits for usb connection, REMOVE to continue without waiting for connection
//    while ((!stdio_usb_connected()));

    TMC2209_t tmc0;
    tmc0 = setupTMC(&tmc0, SERIAL_ADDRESS_0);

    TMC2209_t tmc1;
    tmc1 = setupTMC(&tmc1, SERIAL_ADDRESS_1);

    TMC2209_t tmc2;
    tmc2 = setupTMC(&tmc2, SERIAL_ADDRESS_2);

    TMC2209_t tmc3;
    tmc3 = setupTMC(&tmc3, SERIAL_ADDRESS_3);

    char *input_buf = malloc(INPUT_BUFFER_LEN);
    memset(input_buf, '\0', INPUT_BUFFER_LEN);
    unsigned charachterConter = 0;

    while (1) {
        char input = getchar_timeout_us(10000000); // 10 seconds wait

        bool isAllowedCharacter = false;
        for (int i = 0; i < strlen(allowedCharacters); ++i) {
            if (input == allowedCharacters[i]) {
                isAllowedCharacter = true;
            }
        }
        if (isAllowedCharacter == false)
            continue;

        if (input == '\n') {
            //printf("Process Msg len: %d\n", counter);
            processMsg(input_buf, charachterConter, &tmc0, &tmc1, &tmc2, &tmc3);
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            charachterConter = 0;
        } else if (charachterConter >= INPUT_BUFFER_LEN - 1) {
            // Zu viele Bytes empfangen -> Buffer leeren und bereit für die nächste Anfrage sein
            // Hier kann noch eingefügt werden, dass der Pi eine Rückmeldung bekommt
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            charachterConter = 0;
        } else {
            printf("Received: %c (counter: %d)\n", input, charachterConter);
            input_buf[charachterConter] = input;
            ++charachterConter;
        }
    }
}
