#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define UART_ID uart1
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE


int main() {

    // init usb
    stdio_init_all();
    // waits for usb connection, REMOVE to continue without waiting for connection
    while ((!stdio_usb_connected()));

    sleep_ms(100);
    printf("Connected\n");

    // Initialise UART 0
    uart_init(UART_ID, 115200);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);


//    uart_set_hw_flow(UART_ID, false, false);
//    // Set our data format
//    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
//    // Turn off FIFO's - we want to do this character by character
//    uart_set_fifo_enabled(UART_ID, false);

    char *forward = NULL;
    sprintf(forward, "%i%i", 0x22, 50000);


    while (1) {
        printf("forward\n");
        sleep_ms(1000);
    }

    return 0;
}


void _close(void) {

}

void _lseek(void) {

}
