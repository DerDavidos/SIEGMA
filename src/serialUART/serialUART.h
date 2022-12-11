#ifndef C_ARDUINO_TO_C_SERIALUART_H
#define C_ARDUINO_TO_C_SERIALUART_H

#pragma once

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

#define SERIAL_BAUD_RATE 115200

typedef struct SerialUART_t {
    bool _running; // set to false by init
    uart_inst_t *_uart;
    uint8_t _tx, _rx;
    uint8_t _rts, _cts;
    enum gpio_function _fcnTx, _fcnRx, _fcnRts, _fcnCts;
    unsigned long _baud;
    bool _polling;  // set to false by init
    bool _overflow;

    // Lockless, IRQ-handled circular queue
    uint32_t _writer;
    uint32_t _reader;
    uint8_t _fifoSize; // set to 32 by init
    uint8_t *_queue;

} SerialUART_t;

void SerialUART_begin(unsigned long baud, uint16_t config);

void SerialUART_end();

int SerialUART_read();

uint8_t SerialUART_available();

size_t SerialUART_write(uint8_t c);

// Not to be called by users, only from the IRQ handler.  In public so that the C-language IQR callback can access it
void SerialUART_handleIRQ(bool inIRQ);

// User space FIFO transfer
void SerialUART_pumpFIFO();

SerialUART_t SerialUART(uart_inst_t *uart, uint8_t tx, uint8_t rx);

#define SERIAL1 SerialUART(uart0, PIN_SERIAL1_TX, PIN_SERIAL1_RX)

#define SERIAL2 SerialUART(uart1, PIN_SERIAL2_TX, PIN_SERIAL2_RX)

#endif //C_ARDUINO_TO_C_SERIALUART_H
