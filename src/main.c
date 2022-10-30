#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"

int main() {

    // init usb
    stdio_init_all();
    // waits for usb connection, REMOVE to continue without waiting for connection
    while ((!stdio_usb_connected()));

    while (1) {
        printf("Hello World!\n");
        sleep_ms(1000);
    }

    return 0;
}
