#include <stdio.h>
#include <stdlib.h>

#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "pico/stdio_usb.h"
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

const char* okRecv = "0123456789;\n";

int parseInputString(char* buf, unsigned buf_len, int* p_semiPos) {
    char* semicolon = strchr(buf, ';');
    if (semicolon == NULL) {
        // haben kein Komma gefunden -> :(
        return -1;
    }
    int semi_pos = semicolon - buf;
    *p_semiPos = semi_pos;
    if (buf_len < semi_pos || semi_pos > MAX_SLEEP_INPUT || semi_pos <= 0) {
        return 0;
    }
    else {
        char stepper1_number_buf[MAX_SLEEP_INPUT+1];
        memcpy(stepper1_number_buf, buf, semi_pos);
        stepper1_number_buf[semi_pos] = '\0';
        stepper1_number_buf[MAX_SLEEP_INPUT] = '\0';
        return atoi(stepper1_number_buf);
    }
}

void processMsg(char* buf, unsigned buf_len, TMC2209_t *tmc0, TMC2209_t *tmc1, TMC2209_t *tmc2, TMC2209_t *tmc3){
    int stepper0 = 0;
    int stepper1 = 0;
    int stepper2 = 0;
    int stepper3 = 0;

    int semi1Pos = 0;
    int semi2Pos = 0;
    int semi3Pos = 0;
    int semi4Pos = 0;

    stepper0 = parseInputString(buf, buf_len, &semi1Pos);
    if (stepper0 <= -1) {
        return;
    }
    stepper1 = parseInputString(buf + semi1Pos + 1, buf_len-(semi1Pos+1), &semi2Pos);
    if (stepper1 <= -1) {
        return;
    }
    stepper2 = parseInputString(buf + semi1Pos + semi2Pos + 2, buf_len-(semi1Pos + semi2Pos + 2), &semi3Pos);
    if (stepper2 <= -1) {
        return;
    }
    stepper3 = parseInputString(buf + semi1Pos + semi2Pos + semi3Pos + 3, buf_len-(semi1Pos + semi2Pos + semi3Pos + 3), &semi4Pos);
    if (stepper3 <= -1) {
        return;
    }

/*
    char* erstesSemi = strchr(buf, ';');
    if(erstesSemi == NULL) {
        // haben kein Komma gefunden -> :(
        return;
    }
    int stepper0_pos = erstesSemi - buf;
    char stepper0_number_buf[6];
    if(stepper0_pos > 5 || stepper0_pos <= 0){
        stepper0 = 0;
    }
    else {
        memcpy(stepper0_number_buf, buf, stepper0_pos);
        stepper0_number_buf[stepper0_pos] = '\0';
        stepper0_number_buf[5] = '\0';
        stepper0 = atoi(stepper0_number_buf);
    }

    char* zweitesSemi = strchr(buf + stepper0_pos + 1, ';');
    if(zweitesSemi == NULL) {
        // haben kein Komma gefunden -> :(
        return;
    }
    int stepper1_pos = zweitesSemi - buf;
    char stepper1_number_buf[6];
    memcpy(stepper1_number_buf, buf + stepper0_pos + 1, (stepper1_pos - stepper0_pos) - 1);
    stepper1_number_buf[stepper1_pos - stepper0_pos] = '\0';
    stepper1_number_buf[5] = '\0';
    stepper1 = atoi(stepper1_number_buf);


    char* drittesSemi = strchr(buf + stepper1_pos + 1, ';');
    if(drittesSemi == NULL) {
        // haben kein Komma gefunden -> :(
        return;
    }
    int stepper2_pos = drittesSemi - buf;
    char stepper2_number_buf[6];
    memcpy(stepper2_number_buf, (buf + stepper1_pos) + 1, (stepper2_pos - stepper1_pos) - 1);
    stepper2_number_buf[stepper2_pos - stepper1_pos] = '\0';
    stepper2_number_buf[5] = '\0';
    stepper2 = atoi(stepper2_number_buf);

    int stepper3_pos = (buf_len - 1) - (stepper2_pos);
    char stepper3_number_buf[6];
    memcpy(stepper3_number_buf, buf + stepper2_pos + 1, stepper3_pos - 1);
    stepper3_number_buf[stepper3_pos] = '\0';
    stepper3_number_buf[5] = '\0';
    stepper3 = atoi(stepper3_number_buf);
    */

    printf("stepper0_pos: %d\n", semi1Pos);
    printf("stepper1_pos: %d\n", semi2Pos);
    printf("stepper2_pos: %d\n", semi3Pos);
    printf("stepper3_pos: %d\n", semi4Pos);

    printf("buffer len: %d\n", buf_len);

    printf("stepper0: %d\n", stepper0);
    printf("stepper1: %d\n", stepper1);
    printf("stepper2: %d\n", stepper2);
    printf("stepper3: %d\n", stepper3);


    bool stepper0_fertig = false;
    bool stepper1_fertig = false;
    bool stepper2_fertig = false;
    bool stepper3_fertig = false;

    int anzahl_die_ausgeloest_werden_muss = 0;

    if(stepper0 > 0){
        ++anzahl_die_ausgeloest_werden_muss;
        TMC2209_moveAtVelocity(tmc0, -50000);
    }
    else{
        stepper0_fertig = true;
    }
    if(stepper1 > 0){
        ++anzahl_die_ausgeloest_werden_muss;
        TMC2209_moveAtVelocity(tmc1, -50000);
    }
    else{
        stepper1_fertig = true;
    }
    if(stepper2 > 0){
        ++anzahl_die_ausgeloest_werden_muss;
        TMC2209_moveAtVelocity(tmc2, -50000);
    }
    else{
        stepper2_fertig = true;
    }
    if(stepper3 > 0){
        ++anzahl_die_ausgeloest_werden_muss;
        TMC2209_moveAtVelocity(tmc3, -50000);
    }
    else{
        stepper3_fertig = true;
    }

    sleep_ms(5000);

    if(!stepper0_fertig){
        TMC2209_moveAtVelocity(tmc0, 0);
    }
    if(!stepper1_fertig){
        TMC2209_moveAtVelocity(tmc1, 0);
    }
    if(!stepper2_fertig){
        TMC2209_moveAtVelocity(tmc2, 0);
    }
    if(!stepper3_fertig){
        TMC2209_moveAtVelocity(tmc3, 0);
    }

    int tmc0_sleep = stepper0;
    int tmc1_sleep = stepper1;
    int tmc2_sleep = stepper2;
    int tmc3_sleep = stepper3;

    bool tmc0_faehrt_runter = 0;
    bool tmc1_faehrt_runter = 0;
    bool tmc2_faehrt_runter = 0;
    bool tmc3_faehrt_runter = 0;

    while(anzahl_die_ausgeloest_werden_muss){   // kann nicht 1 bleiben
        if(!stepper0_fertig){
            if(tmc0_faehrt_runter){
                if (gpio_get(STEPPER0_LS_PIN)) {
                    --anzahl_die_ausgeloest_werden_muss;
                    TMC2209_moveAtVelocity(tmc0, 0);
                    stepper0_fertig = true;
                }
            }
            else {
                tmc0_sleep = tmc0_sleep - 10;

                if(tmc0_sleep<=0){
                    TMC2209_moveAtVelocity(tmc0, 50000);
                    tmc0_faehrt_runter = 1;
                }
            }
        }
        if(!stepper1_fertig){
            if(tmc1_faehrt_runter){
                if (gpio_get(STEPPER1_LS_PIN)) {
                    --anzahl_die_ausgeloest_werden_muss;
                    TMC2209_moveAtVelocity(tmc1, 0);
                    stepper1_fertig = true;
                }
            }
            else {
                tmc1_sleep = tmc1_sleep - 10;

                if(tmc1_sleep<=0){
                    TMC2209_moveAtVelocity(tmc1, 50000);
                    tmc1_faehrt_runter = 1;
                }
            }
        }
        if(!stepper2_fertig){
            if(tmc2_faehrt_runter){
                if (gpio_get(STEPPER2_LS_PIN)) {
                    --anzahl_die_ausgeloest_werden_muss;
                    TMC2209_moveAtVelocity(tmc2, 0);
                    stepper2_fertig = true;
                }
            }
            else {
                tmc2_sleep = tmc2_sleep - 10;

                if(tmc2_sleep<=0){
                    TMC2209_moveAtVelocity(tmc2, 50000);
                    tmc2_faehrt_runter = 1;
                }
            }
        }
        if(!stepper3_fertig){
            if(tmc3_faehrt_runter){
                if (gpio_get(STEPPER3_LS_PIN)) {
                    --anzahl_die_ausgeloest_werden_muss;
                    TMC2209_moveAtVelocity(tmc3, 0);
                    stepper3_fertig = true;
                }
            }
            else {
                tmc3_sleep = tmc3_sleep - 10;

                if(tmc3_sleep<=0){
                    TMC2209_moveAtVelocity(tmc3, 50000);
                    tmc3_faehrt_runter = 1;
                }
            }
        }
        sleep_ms(10);
    }

    /*
    while (anzahl_die_ausgeloest_werden_muss){
        //printf("%d, %d, %d, %d", gpio_get(STEPPER0_LS_PIN), gpio_get(STEPPER1_LS_PIN), gpio_get(STEPPER2_LS_PIN),gpio_get(STEPPER3_LS_PIN));

        if (!stepper0_fertig && gpio_get(STEPPER0_LS_PIN)) {
            --anzahl_die_ausgeloest_werden_muss;
            TMC2209_moveAtVelocity(tmc0, 0);
            stepper0_fertig = true;
        }
        if (!stepper1_fertig && gpio_get(STEPPER1_LS_PIN)) {
            --anzahl_die_ausgeloest_werden_muss;
            TMC2209_moveAtVelocity(tmc1, 0);
            stepper1_fertig = true;
        }
        if (!stepper2_fertig && gpio_get(STEPPER2_LS_PIN)) {
            --anzahl_die_ausgeloest_werden_muss;
            TMC2209_moveAtVelocity(tmc2, 0);
            stepper2_fertig = true;
        }
        if (!stepper3_fertig && (gpio_get(STEPPER3_LS_PIN))) {
            --anzahl_die_ausgeloest_werden_muss;
            TMC2209_moveAtVelocity(tmc3, 0);
            stepper3_fertig = true;
        }
        sleep_ms(50);
    }
*/
    return;
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

    char* input_buf = malloc(INPUT_BUFFER_LEN);
    memset(input_buf, '\0', INPUT_BUFFER_LEN);
    unsigned counter = 0;

    while (1) {
        char input = getchar_timeout_us(10000000); // 10 seconds wait

        bool gefunden = false;
        for(int i=0;i<strlen(okRecv);++i){
            if(input == okRecv[i]){
                gefunden = true;
            }
        }

        if(gefunden == false){
            continue;
        }

        if(input == '\n'){
            //printf("Process Msg len: %d\n", counter);
            processMsg(input_buf, counter, &tmc0, &tmc1, &tmc2, &tmc3);
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            counter = 0;
        }
        else if(counter >= INPUT_BUFFER_LEN-1){
            // Zu viele Bytes empfangen -> Buffer leeren und bereit für die nächste Anfrage sein
            // Hier kann noch eingefügt werden, dass der Pi eine Rückmeldung bekommt
            memset(input_buf, '\0', INPUT_BUFFER_LEN);
            counter = 0;
        }
        else{
            printf("Habe %c empfangen (counter: %d)\n", input, counter);
            input_buf[counter] = input;
            ++counter;
        }
    }

    return 0;
}