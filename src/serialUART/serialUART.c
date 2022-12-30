/** ----------------------------------------------------------------------------
 * Adapted from:
 *  https://github.com/earlephilhower/arduino-pico
 *  https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/SerialUART.cpp
*** ---------------------------------------------------------------------------- */

/*
    Serial-over-UART for the Raspberry Pi Pico RP2040

    Copyright (c) 2021 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "serialUART.h"
#include "hardware/irq.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <stdlib.h>

static SerialUART_t sUART;

SerialUART_t SerialUART(uart_inst_t *uart, uint8_t tx, uint8_t rx) {
    sUART._uart = uart;
    sUART._tx = tx;
    sUART._rx = rx;
    sUART._rts = rx;
    sUART._cts = tx;
    sUART._running = false;
    sUART._polling = false;
    sUART._fifoSize = 32;
    return sUART;
}

static void SerialUART_uart0IRQ();

static void SerialUART_uart1IRQ();

void SerialUART_begin(unsigned long baud, uint16_t config) {
    if (sUART._running) {
        SerialUART_end();
    }
    sUART._overflow = false;
    sUART._queue = malloc(sUART._fifoSize);
    sUART._baud = baud;
    uart_init(sUART._uart, baud);
    int bits, stop;
    uart_parity_t parity;
    switch (config & SERIAL_PARITY_MASK) {
        case SERIAL_PARITY_EVEN:
            parity = UART_PARITY_EVEN;
            break;
        case SERIAL_PARITY_ODD:
            parity = UART_PARITY_ODD;
            break;
        default:
            parity = UART_PARITY_NONE;
            break;
    }
    switch (config & SERIAL_STOP_BIT_MASK) {
        case SERIAL_STOP_BIT_1:
            stop = 1;
            break;
        default:
            stop = 2;
            break;
    }
    switch (config & SERIAL_DATA_MASK) {
        case SERIAL_DATA_5:
            bits = 5;
            break;
        case SERIAL_DATA_6:
            bits = 6;
            break;
        case SERIAL_DATA_7:
            bits = 7;
            break;
        default:
            bits = 8;
            break;
    }
    uart_set_format(sUART._uart, bits, stop, parity);
    sUART._fcnTx = gpio_get_function(sUART._tx);
    sUART._fcnRx = gpio_get_function(sUART._rx);
    gpio_set_function(sUART._tx, GPIO_FUNC_UART);
    gpio_set_function(sUART._rx, GPIO_FUNC_UART);
    if (sUART._rts != UART_PIN_NOT_DEFINED) {
        sUART._fcnRts = gpio_get_function(sUART._rts);
        gpio_set_function(sUART._rts, GPIO_FUNC_UART);
    }
    if (sUART._cts != UART_PIN_NOT_DEFINED) {
        sUART._fcnCts = gpio_get_function(sUART._cts);
        gpio_set_function(sUART._cts, GPIO_FUNC_UART);
    }
    uart_set_hw_flow(sUART._uart, sUART._rts != UART_PIN_NOT_DEFINED, sUART._cts != UART_PIN_NOT_DEFINED);
    sUART._writer = 0;
    sUART._reader = 0;

    if (!sUART._polling) {
        if (sUART._uart == uart0) {
            irq_set_exclusive_handler(UART0_IRQ, SerialUART_uart0IRQ);
            irq_set_enabled(UART0_IRQ, true);
        } else {
            irq_set_exclusive_handler(UART1_IRQ, SerialUART_uart1IRQ);
            irq_set_enabled(UART1_IRQ, true);
        }
        // Set the IRQ enables and FIFO level to minimum
        uart_set_irq_enables(sUART._uart, true, false);
    } else {
        // Polling mode has no IRQs used
    }
    sUART._running = true;
}

void SerialUART_end() {
    if (!sUART._running) {
        return;
    }
    sUART._running = false;
    if (!sUART._polling) {
        if (sUART._uart == uart0) {
            irq_set_enabled(UART0_IRQ, false);
        } else {
            irq_set_enabled(UART1_IRQ, false);
        }
    }

    uart_deinit(sUART._uart);
    free(sUART._queue);

    // Restore pin functions
    gpio_set_function(sUART._tx, sUART._fcnTx);
    gpio_set_function(sUART._rx, sUART._fcnRx);
    if (sUART._rts != UART_PIN_NOT_DEFINED) {
        gpio_set_function(sUART._rts, sUART._fcnRts);
    }
    if (sUART._cts != UART_PIN_NOT_DEFINED) {
        gpio_set_function(sUART._cts, sUART._fcnCts);
    }
}

void __not_in_flash_func(SerialUART_handleIRQ)(bool inIRQ) {
    // ICR is write-to-clear
    uart_get_hw(sUART._uart)->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;
    while (uart_is_readable(sUART._uart)) {
        int val = uart_getc(sUART._uart);
        uint8_t next_writer = sUART._writer + 1;
        if (next_writer == sUART._fifoSize) {
            next_writer = 0;
        }
        if (next_writer != sUART._reader) {
            sUART._queue[sUART._writer] = val;
            asm volatile("":: : "memory"); // Ensure the queue is written before the written count advances
            // Avoid using division or mod because the HW divider could be in use
            sUART._writer = next_writer;
        } else {
            sUART._overflow = true;
        }
    }
}

void SerialUART_pumpFIFO() {
    int irqno = (sUART._uart == uart0) ? UART0_IRQ : UART1_IRQ;
    bool enabled = irq_is_enabled(irqno);
    irq_set_enabled(irqno, false);
    SerialUART_handleIRQ(false);
    irq_set_enabled(irqno, enabled);
}

int SerialUART_read() {
    if (sUART._polling) {
        SerialUART_handleIRQ(false);
    } else {
        SerialUART_pumpFIFO();
    }
    if (sUART._writer != sUART._reader) {
        int ret = sUART._queue[sUART._reader];
        // asm volatile("":: : "memory"); // Ensure the value is read before advancing
        uint8_t next_reader = (sUART._reader + 1) % sUART._fifoSize;
        // asm volatile("":: : "memory"); // Ensure the reader value is only written once, correctly
        sUART._reader = next_reader;
        return ret;
    }
    return -1;
}

bool SerialUART_overflow() {
    bool hold = sUART._overflow;
    sUART._overflow = false;
    return hold;
}

uint8_t SerialUART_available() {
    if (sUART._polling) {
        SerialUART_handleIRQ(false);
    } else {
        SerialUART_pumpFIFO();
    }
    return (sUART._fifoSize + sUART._writer - sUART._reader) % sUART._fifoSize;
}

size_t SerialUART_write(uint8_t c) {
    if (sUART._polling) {
        SerialUART_handleIRQ(false);
    }
    uart_putc_raw(sUART._uart, c);
    return 1;
}

static void __not_in_flash_func(SerialUART_uart0IRQ)() {
    SerialUART_handleIRQ(true);
}

static void __not_in_flash_func(SerialUART_uart1IRQ)() {
    SerialUART_handleIRQ(true);
}
