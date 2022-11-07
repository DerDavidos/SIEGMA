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

//template<size_t N>
//constexpr uint32_t __bitset(const int (&a)[N], size_t i = 0U) {
//    return i < N ? (1L << a[i]) | __bitset(a, i + 1) : 0;
//}

//bool SerialUART::setRX(uint8_t pin) {
//    constexpr uint32_t valid[2] = {__bitset({1, 13, 17, 29}) /* UART0 */,
//                                   __bitset({5, 9, 21, 25})  /* UART1 */
//    };
//    if ((!_running) && ((1 << pin) & valid[uart_get_index(_uart)])) {
//        _rx = pin;
//        return true;
//    }
//
//    if (_running) {
//        panic("FATAL: Attempting to set Serial%d.RX while running", uart_get_index(_uart) + 1);
//    } else {
//        panic("FATAL: Attempting to set Serial%d.RX to illegal pin %d", uart_get_index(_uart) + 1, pin);
//    }
//    return false;
//}

//bool SerialUART::setTX(uint8_t pin) {
//    constexpr uint32_t valid[2] = {__bitset({0, 12, 16, 28}) /* UART0 */,
//                                   __bitset({4, 8, 20, 24})  /* UART1 */
//    };
//    if ((!_running) && ((1 << pin) & valid[uart_get_index(_uart)])) {
//        _tx = pin;
//        return true;
//    }
//
//    if (_running) {
//        panic("FATAL: Attempting to set Serial%d.TX while running", uart_get_index(_uart) + 1);
//    } else {
//        panic("FATAL: Attempting to set Serial%d.TX to illegal pin %d", uart_get_index(_uart) + 1, pin);
//    }
//    return false;
//}

//bool SerialUART::setRTS(uint8_t pin) {
//    constexpr uint32_t valid[2] = {__bitset({3, 15, 19}) /* UART0 */,
//                                   __bitset({7, 11, 23, 27})  /* UART1 */
//    };
//    if ((!_running) && ((pin == UART_PIN_NOT_DEFINED) || ((1 << pin) & valid[uart_get_index(_uart)]))) {
//        _rts = pin;
//        return true;
//    }
//
//    if (_running) {
//        panic("FATAL: Attempting to set Serial%d.RTS while running", uart_get_index(_uart) + 1);
//    } else {
//        panic("FATAL: Attempting to set Serial%d.RTS to illegal pin %d", uart_get_index(_uart) + 1, pin);
//    }
//    return false;
//}

//bool SerialUART::setCTS(uint8_t pin) {
//    constexpr uint32_t valid[2] = {__bitset({2, 14, 18}) /* UART0 */,
//                                   __bitset({6, 10, 22, 26})  /* UART1 */
//    };
//    if ((!_running) && ((pin == UART_PIN_NOT_DEFINED) || ((1 << pin) & valid[uart_get_index(_uart)]))) {
//        _cts = pin;
//        return true;
//    }
//
//    if (_running) {
//        panic("FATAL: Attempting to set Serial%d.CTS while running", uart_get_index(_uart) + 1);
//    } else {
//        panic("FATAL: Attempting to set Serial%d.CTS to illegal pin %d", uart_get_index(_uart) + 1, pin);
//    }
//    return false;
//}

//bool SerialUART::setPollingMode(bool mode) {
//    if (_running) {
//        return false;
//    }
//    _polling = mode;
//    return true;
//}

//bool SerialUART::setFIFOSize(size_t size) {
//    if (!size || _running) {
//        return false;
//    }
//    _fifoSize = size + 1; // Always 1 unused entry
//    return true;
//}

SerialUART::SerialUART(uart_inst_t *uart, uint8_t tx, uint8_t rx, uint8_t rts, uint8_t cts) {
    _uart = uart;
    _tx = tx;
    _rx = rx;
    _rts = rts;
    _cts = cts;
}

static void _uart0IRQ();

static void _uart1IRQ();

void SerialUART::begin(unsigned long baud, uint16_t config) {
    if (_running) {
        end();
    }
    _overflow = false;
    _queue = new uint8_t[_fifoSize];
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
    if (_rts != UART_PIN_NOT_DEFINED) {
        _fcnRts = gpio_get_function(_rts);
        gpio_set_function(_rts, GPIO_FUNC_UART);
    }
    if (_cts != UART_PIN_NOT_DEFINED) {
        _fcnCts = gpio_get_function(_cts);
        gpio_set_function(_cts, GPIO_FUNC_UART);
    }
    uart_set_hw_flow(_uart, _rts != UART_PIN_NOT_DEFINED, _cts != UART_PIN_NOT_DEFINED);
    _writer = 0;
    _reader = 0;

    if (!_polling) {
        if (_uart == uart0) {
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
    _running = true;
}

void SerialUART::end() {
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
    delete[] _queue;

    // Restore pin functions
    gpio_set_function(_tx, _fcnTx);
    gpio_set_function(_rx, _fcnRx);
    if (_rts != UART_PIN_NOT_DEFINED) {
        gpio_set_function(_rts, _fcnRts);
    }
    if (_cts != UART_PIN_NOT_DEFINED) {
        gpio_set_function(_cts, _fcnCts);
    }
}

void SerialUART::_pumpFIFO() {
    auto irqno = (_uart == uart0) ? UART0_IRQ : UART1_IRQ;
    bool enabled = irq_is_enabled(irqno);
    irq_set_enabled(irqno, false);
    _handleIRQ(false);
    irq_set_enabled(irqno, enabled);
}

//int SerialUART::peek() {
//    if (_polling) {
//        _handleIRQ(false);
//    } else {
//        _pumpFIFO();
//    }
//    if (_writer != _reader) {
//        return _queue[_reader];
//    }
//    return -1;
//}

int SerialUART::read() {
    if (_polling) {
        _handleIRQ(false);
    } else {
        _pumpFIFO();
    }
    if (_writer != _reader) {
        auto ret = _queue[_reader];
        asm volatile("":: : "memory"); // Ensure the value is read before advancing
        auto next_reader = (_reader + 1) % _fifoSize;
        asm volatile("":: : "memory"); // Ensure the reader value is only written once, correctly
        _reader = next_reader;
        return ret;
    }
    return -1;
}

bool SerialUART::overflow() {
    bool hold = _overflow;
    _overflow = false;
    return hold;
}

int SerialUART::available() {
    if (_polling) {
        _handleIRQ(false);
    } else {
        _pumpFIFO();
    }
    return (_fifoSize + _writer - _reader) % _fifoSize;
}

//int SerialUART::availableForWrite() {
//    if (_polling) {
//        _handleIRQ(false);
//    }
//    return (uart_is_writable(_uart)) ? 1 : 0;
//}

//void SerialUART::flush() {
//    if (_polling) {
//        _handleIRQ(false);
//    }
//    uart_tx_wait_blocking(_uart);
//}

size_t SerialUART::write(uint8_t c) {
    if (_polling) {
        _handleIRQ(false);
    }
    uart_putc_raw(_uart, c);
    return 1;
}

size_t SerialUART::write(const uint8_t *p, size_t len) {
    if (_polling) {
        _handleIRQ(false);
    }
    size_t cnt = len;
    while (cnt) {
        uart_putc_raw(_uart, *p);
        cnt--;
        p++;
    }
    return len;
}

SerialUART::operator bool() {
    return _running;
}

//void arduino::serialEvent1Run(void) {
//    if (serialEvent1 && Serial1.available()) {
//        serialEvent1();
//    }
//}

//void arduino::serialEvent2Run(void) {
//    if (serialEvent2 && Serial2.available()) {
//        serialEvent2();
//    }
//}

// IRQ handler, called when FIFO > 1/8 full or when it had held unread data for >32 bit times
void __not_in_flash_func(SerialUART::_handleIRQ)(bool inIRQ) {
    // ICR is write-to-clear
    uart_get_hw(_uart)->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;
    while (uart_is_readable(_uart)) {
        auto val = uart_getc(_uart);
        auto next_writer = _writer + 1;
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


static void __not_in_flash_func(_uart0IRQ)() {
    if (__SERIAL1_DEVICE == uart0) {
        Serial1._handleIRQ();
    } else {
        Serial2._handleIRQ();
    }
}

static void __not_in_flash_func(_uart1IRQ)() {
    if (__SERIAL2_DEVICE == uart1) {
        Serial2._handleIRQ();
    } else {
        Serial1._handleIRQ();
    }
}
