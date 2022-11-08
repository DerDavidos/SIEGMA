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

#ifndef C_ARDUINO_TO_C_SERIALUART_H
#define C_ARDUINO_TO_C_SERIALUART_H

//#pragma once

//#include <cstdarg>
//#include <queue>
//#include <cstdint>
//#include <cstddef>
#include "hardware/gpio.h"
#include "hardware/uart.h"

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


#define UART_PIN_NOT_DEFINED      (255u)

void SerialUART(uart_inst_t *uart, uint8_t tx, uint8_t rx);

void begin(unsigned long baud);

void end();

int SerialUART_read();

int available();

size_t SerialUART_write(uint8_t c);

bool overflow();

// Not to be called by users, only from the IRQ handler.  In public so that the C-language IQR callback can access it
void _handleIRQ(bool inIRQ);

void _pumpFIFO(); // User space FIFO transfer


//extern SerialUART Serial1; // HW UART 0
//extern SerialUART Serial2; // HW UART 1

#endif //C_ARDUINO_TO_C_SERIALUART_H
