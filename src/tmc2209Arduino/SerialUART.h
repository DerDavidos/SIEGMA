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

#pragma once

#include <cstdarg>
#include <queue>
#include <cstdint>
#include <cstddef>
#include "hardware/gpio.h"

#define SERIAL_PARITY_EVEN   (0x1ul)
#define SERIAL_PARITY_ODD    (0x2ul)
#define SERIAL_PARITY_NONE   (0x3ul)
#define SERIAL_PARITY_MARK   (0x4ul)
#define SERIAL_PARITY_SPACE  (0x5ul)
#define SERIAL_PARITY_MASK   (0xFul)

#define SERIAL_STOP_BIT_1    (0x10ul)
#define SERIAL_STOP_BIT_1_5  (0x20ul)
#define SERIAL_STOP_BIT_2    (0x30ul)
#define SERIAL_STOP_BIT_MASK (0xF0ul)

#define SERIAL_DATA_5        (0x100ul)
#define SERIAL_DATA_6        (0x200ul)
#define SERIAL_DATA_7        (0x300ul)
#define SERIAL_DATA_8        (0x400ul)
#define SERIAL_DATA_MASK     (0xF00ul)

#define SERIAL_8N1           (SERIAL_STOP_BIT_1 | SERIAL_PARITY_NONE  | SERIAL_DATA_8)

// Serial
#define PIN_SERIAL1_TX      (0u)
#define PIN_SERIAL1_RX      (1u)

#define PIN_SERIAL2_TX      (4u)
#define PIN_SERIAL2_RX      (5u)

extern "C" typedef struct uart_inst uart_inst_t;

#define UART_PIN_NOT_DEFINED      (255u)

class SerialUART {
public:
    SerialUART(uart_inst_t *uart, uint8_t tx, uint8_t rx, uint8_t rts = UART_PIN_NOT_DEFINED,
               uint8_t cts = UART_PIN_NOT_DEFINED);

    // Select the pinout.  Call before .begin()
    bool setRX(uint8_t pin);

    bool setTX(uint8_t pin);

//    bool setRTS(uint8_t pin);

//    bool setCTS(uint8_t pin);

    bool setPinout(uint8_t tx, uint8_t rx) {
        bool ret = setRX(rx);
        ret &= setTX(tx);
        return ret;
    }

//    bool setFIFOSize(size_t size);

//    bool setPollingMode(bool mode = true);

    void begin(unsigned long baud = 115200) {
        begin(baud, SERIAL_8N1);
    };

    void begin(unsigned long baud, uint16_t config);

    void end();

//    virtual int peek();

    virtual int read();

    virtual int available();

//    virtual int availableForWrite();

//    virtual void flush();

    virtual size_t write(uint8_t c);

    virtual size_t write(const uint8_t *p, size_t len);

    bool overflow();

    operator bool();

    // Not to be called by users, only from the IRQ handler.  In public so that the C-language IQR callback can access it
    void _handleIRQ(bool inIRQ = true);

private:
    bool _running = false;
    uart_inst_t *_uart;
    uint8_t _tx, _rx;
    uint8_t _rts, _cts;
    enum gpio_function _fcnTx, _fcnRx, _fcnRts, _fcnCts;
    int _baud;
    bool _polling = false;
    bool _overflow;

    // Lockless, IRQ-handled circular queue
    uint32_t _writer;
    uint32_t _reader;
    uint8_t _fifoSize = 32;
    uint8_t *_queue;

    void _pumpFIFO(); // User space FIFO transfer
};

extern SerialUART Serial1; // HW UART 0
extern SerialUART Serial2; // HW UART 1
