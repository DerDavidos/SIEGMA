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
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdlib.h>
#ifndef __SERIAL1_DEVICE
#define __SERIAL1_DEVICE uart0
#endif
#ifndef __SERIAL2_DEVICE
#define __SERIAL2_DEVICE uart1
#endif

//SerialUART Serial1(__SERIAL1_DEVICE, PIN_SERIAL1_TX, PIN_SERIAL1_RX);
//SerialUART Serial2(__SERIAL2_DEVICE, PIN_SERIAL2_TX, PIN_SERIAL2_RX);

bool _running = false;
uart_inst_t *_uart;
uint8_t _tx, _rx;
//    uint8_t _rts, _cts;
enum gpio_function _fcnTx, _fcnRx, _fcnRts, _fcnCts;
int _baud;
bool _polling = false;
bool _overflow;

// Lockless, IRQ-handled circular queue
uint32_t _writer;
uint32_t _reader;
uint8_t _fifoSize = 32;
uint8_t *_queue;

void SerialUART(uart_inst_t *uart, uint8_t tx, uint8_t rx) {
    _uart = uart;
    _tx = tx;
    _rx = rx;
}

static void __not_in_flash_func(_uart0IRQ)() {
    _handleIRQ(true);
}

//static void __not_in_flash_func(_uart1IRQ)() {
//    Serial2._handleIRQ();
//}

void begin(unsigned long baud) {
    uint16_t config = SERIAL_8N1;
    if (_running) {
        end();
    }
    _overflow = false;
    uint8_t tmp[_fifoSize];
    _queue = malloc(_fifoSize);
    _baud = baud;
    uart_init(_uart, baud);
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
    uart_set_format(_uart, bits, stop, parity);
    _fcnTx = gpio_get_function(_tx);
    _fcnRx = gpio_get_function(_rx);
    gpio_set_function(_tx, GPIO_FUNC_UART);
    gpio_set_function(_rx, GPIO_FUNC_UART);
    uart_set_hw_flow(_uart, false, false);
    _writer = 0;
    _reader = 0;

    if (!_polling) {
        if (_uart == uart0) {
            irq_set_exclusive_handler(UART0_IRQ, _uart0IRQ);
            irq_set_enabled(UART0_IRQ, true);
        }
//        else {
//            irq_set_exclusive_handler(UART1_IRQ, _uart1IRQ);
//            irq_set_enabled(UART1_IRQ, true);
//        }
        // Set the IRQ enables and FIFO level to minimum
        uart_set_irq_enables(_uart, true, false);
    } else {
        // Polling mode has no IRQs used
    }
    _running = true;
}

void end() {
    if (!_running) {
        return;
    }
    _running = false;
    if (!_polling) {
        if (_uart == uart0) {
            irq_set_enabled(UART0_IRQ, false);
        } else {
            irq_set_enabled(UART1_IRQ, false);
        }
    }

    uart_deinit(_uart);
//    delete[] _queue; // TODO

    // Restore pin functions
    gpio_set_function(_tx, _fcnTx);
    gpio_set_function(_rx, _fcnRx);
}

void _pumpFIFO() {
    uint8_t irqno = (_uart == uart0) ? UART0_IRQ : UART1_IRQ;
    bool enabled = irq_is_enabled(irqno);
    irq_set_enabled(irqno, false);
    _handleIRQ(false);
    irq_set_enabled(irqno, enabled);
}

int SerialUART_read() {
    if (_polling) {
        _handleIRQ(false);
    } else {
        _pumpFIFO();
    }
    if (_writer != _reader) {
        uint8_t ret = _queue[_reader];
        asm volatile("":: : "memory"); // Ensure the value is read before advancing
        uint8_t next_reader = (_reader + 1) % _fifoSize;
        asm volatile("":: : "memory"); // Ensure the reader value is only written once, correctly
        _reader = next_reader;
        return ret;
    }
    return -1;
}

bool overflow() {
    bool hold = _overflow;
    _overflow = false;
    return hold;
}

int available() {
    if (_polling) {
        _handleIRQ(false);
    } else {
        _pumpFIFO();
    }
    return (_fifoSize + _writer - _reader) % _fifoSize;
}


size_t SerialUART_write(uint8_t c) {
    if (_polling) {
        _handleIRQ(false);
    }
    uart_putc_raw(_uart, c);
    return 1;
}

// IRQ handler, called when FIFO > 1/8 full or when it had held unread data for >32 bit times
void __not_in_flash_func(_handleIRQ)(bool inIRQ) {
    // ICR is write-to-clear
    uart_get_hw(_uart)->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;
    while (uart_is_readable(_uart)) {
        uint8_t val = uart_getc(_uart);
        uint8_t next_writer = _writer + 1;
        if (next_writer == _fifoSize) {
            next_writer = 0;
        }
        if (next_writer != _reader) {
            _queue[_writer] = val;
            asm volatile("":: : "memory"); // Ensure the queue is written before the written count advances
            // Avoid using division or mod because the HW divider could be in use
            _writer = next_writer;
        } else {
            _overflow = true;
        }
    }
}
