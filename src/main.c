#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "tmc2209Alt/tmc2209Alt.h"

int main() {

    // init usb
    stdio_init_all();
    // waits for usb connection, REMOVE to continue without waiting for connection
    while ((!stdio_usb_connected()));

    sleep_ms(100);
    printf("Connected\n");

    // Initialise UART 0
    uart_init(uart1, 115200);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);


//    TMC2209_t tmc = tmc2209_defaults;
//    TMC2209_Init();

    while (1) {
        printf("forward\n");
//        TMC2209_moveAtVelocity(0);
        sleep_ms(1000);
//        printf("backward\n");
//        TMC2209_moveAtVelocity(-50000);
//        sleep_ms(2000);
    }

    return 0;
}

//void _close(void) {
//
//}
//void _lseek(void) {
//
//}
