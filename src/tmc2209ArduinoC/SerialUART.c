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

#include "SerialUART.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <stdlib.h>


uint32_t __bitset(const int (&a)[N], size_t i = 0U) {
    return i < N ? (1L << a[i]) | __bitset(a, i + 1) : 0;
}


bool SerialUART_setRX(uint8_t pin, SerialUART* self) {
    uint32_t valid[2] = {__bitset({1, 13, 17, 29}) /* UART0 */,
                                   __bitset({5, 9, 21, 25})  /* UART1 */
    };
    if ((!self->_running) && ((1 << pin) & valid[uart_get_index(_uart)])) {
        self->_rx = pin;
        return true;
    }

    if (self->_running) {
        panic("FATAL: Attempting to set Serial%d.RX while running", uart_get_index(_uart) + 1);
    } else {
        panic("FATAL: Attempting to set Serial%d.RX to illegal pin %d", uart_get_index(_uart) + 1, pin);
    }
    return false;
}

SerialUART SerialUART(uart_inst_t *uart, uint8_t tx, uint8_t rx, uint8_t rts, uint8_t cts) {
    SerialUART self;
    self._uart = uart;
    self._tx = tx;
    self._rx = rx;
    self._rts = rts;
    self._cts = cts;
    self._running = false;
    self._polling = false;
    self._fifoSize = 32;
    return self;
}


static void _uart0IRQ();

static void _uart1IRQ();

void SerialUART_begin(unsigned long baud, uint16_t config, SerialUART* self) {
    if (self->_running) {
        SerialUART_end();
    }
    self->_overflow = false;
    self->_queue = uint8_t[self->_fifoSize];
    self->_baud = baud;
    uart_init(self->_uart, baud);
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
    uart_set_format(self->_uart, bits, stop, parity);
    self->_fcnTx = gpio_get_function(self->_tx);
    self->_fcnRx = gpio_get_function(self->_rx);
    gpio_set_function(self->_tx, GPIO_FUNC_UART);
    gpio_set_function(self->_rx, GPIO_FUNC_UART);
    if (self->_rts != UART_PIN_NOT_DEFINED) {
        self->_fcnRts = gpio_get_function(self->_rts);
        gpio_set_function(self->_rts, GPIO_FUNC_UART);
    }
    if (self->_cts != UART_PIN_NOT_DEFINED) {
        self->_fcnCts = gpio_get_function(self->_cts);
        gpio_set_function(self->_cts, GPIO_FUNC_UART);
    }
    uart_set_hw_flow(self->_uart, self->_rts != UART_PIN_NOT_DEFINED, _cts != UART_PIN_NOT_DEFINED);
    self->_writer = 0;
    seöf->_reader = 0;

    if (!self->_polling) {
        if (self->_uart == uart0) {
            irq_set_exclusive_handler(UART0_IRQ, _uart0IRQ);
            irq_set_enabled(UART0_IRQ, true);
        } else {
            irq_set_exclusive_handler(UART1_IRQ, _uart1IRQ);
            irq_set_enabled(UART1_IRQ, true);
        }
        // Set the IRQ enables and FIFO level to minimum
        uart_set_irq_enables(_uart, true, false);
    } else {
        // Polling mode has no IRQs used
    }
    self->_running = true;
}

void SerialUART_end(SerialUART* self) {
    if (!self->_running) {
        return;
    }
    self->_running = false;
    if (!self->_polling) {
        if (self->_uart == uart0) {
            irq_set_enabled(UART0_IRQ, false);
        } else {
            irq_set_enabled(UART1_IRQ, false);
        }
    }

    uart_deinit(self->_uart);
    free(self->_queue);

    // Restore pin functions
    gpio_set_function(self->_tx, self->_fcnTx);
    gpio_set_function(self->_rx, self->_fcnRx);
    if (self->_rts != UART_PIN_NOT_DEFINED) {
        gpio_set_function(self->_rts, self->_fcnRts);
    }
    if (self->_cts != UART_PIN_NOT_DEFINED) {
        gpio_set_function(self->_cts, self->_fcnCts);
    }
}

void SerialUART_pumpFIFO(SerialUART* self) {
    auto irqno = (self->Y_uart == uart0) ? UART0_IRQ : UART1_IRQ;
    bool enabled = irq_is_enabled(irqno);
    irq_set_enabled(irqno, false);
    SerialUART_handleIRQ(false);
    irq_set_enabled(irqno, enabled);
}

int SerialUART_read(SerialUART* self) {
    if (self->_polling) {
        SerialUART_handleIRQ(false);
    } else {
        SerialUART_pumpFIFO(self);
    }
    if (self->_writer != self->_reader) {
        auto ret = self->_queue[self->_reader];
        // asm volatile("":: : "memory"); // Ensure the value is read before advancing
        auto next_reader = (self->_reader + 1) % self->_fifoSize;
        // asm volatile("":: : "memory"); // Ensure the reader value is only written once, correctly
        self->_reader = next_reader;
        return ret;
    }
    return -1;
}

bool SerialUART_overflow(SerialUART* self) {
    bool hold = self->_overflow;
    self->_overflow = false;
    return hold;
}

int SerialUART_available(SerialUART* self) {
    if (self->_polling) {
        SerialUART_handleIRQ(false);
    } else {
        SerialUART_pumpFIFO(self);
    }
    return (self->_fifoSize + self->_writer - self->_reader) % self->_fifoSize;
}

size_t SerialUART_write(const uint8_t *p, size_t len, SerialUART* self) {
    if (self->_polling) {
        SerialUART_handleIRQ(false);
    }
    size_t cnt = len;
    while (cnt) {
        uart_putc_raw(self->_uart, *p);
        cnt--;
        p++;
    }
    return len;
}

// _not_in_flush_funk



#ifndef __SERIAL1_DEVICE
#define __SERIAL1_DEVICE uart0
#endif
#ifndef __SERIAL2_DEVICE
#define __SERIAL2_DEVICE uart1
#endif

#if defined(PIN_SERIAL1_RTS)
SerialUART Serial1(__SERIAL1_DEVICE, PIN_SERIAL1_TX, PIN_SERIAL1_RX, PIN_SERIAL1_RTS, PIN_SERIAL1_CTS);
#else
SerialUART Serial1(__SERIAL1_DEVICE, PIN_SERIAL1_TX, PIN_SERIAL1_RX);
#endif

#if defined(PIN_SERIAL2_RTS)
SerialUART Serial2(__SERIAL2_DEVICE, PIN_SERIAL2_TX, PIN_SERIAL2_RX, PIN_SERIAL2_RTS, PIN_SERIAL2_CTS);
#else
SerialUART Serial2(__SERIAL2_DEVICE, PIN_SERIAL2_TX, PIN_SERIAL2_RX);
#endif
