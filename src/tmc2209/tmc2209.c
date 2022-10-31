#include "tmc2209.h"
#include "pico/time.h"

uint8_t BYTE_MAX_VALUE = 0xFF;
uint8_t toff_ = TOFF_DEFAULT;
//private variables, structs, unions
bool blocking_;
uart_inst_t *serial_ptr_;
uint32_t serial_baud_rate_;
uint8_t serial_address_;

uint32_t TMC2209_reverseData(uint32_t data) {
    uint32_t reversed_data = 0;
    uint8_t right_shift;
    uint8_t left_shift;
    for (uint8_t i = 0; i < DATA_SIZE; ++i) {
        right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
        left_shift = i * BITS_PER_BYTE;
        reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
    }
    return reversed_data;
}

uint8_t TMC2209_calculateCrc(struct WriteReadReplyDatagram datagram, uint8_t datagram_size) {
    uint8_t crc = 0;
    uint8_t byte;
    for (uint8_t i = 0; i < (datagram_size - 1); ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        for (uint8_t j = 0; j < BITS_PER_BYTE; ++j) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

void TMC2209_sendDatagram(struct WriteReadReplyDatagram datagram, uint8_t datagram_size) {
    if (!serial_ptr_) {
        return;
    }

    uint8_t byte;

// clear the serial receive buffer if necessary
    while (uart_is_readable(serial_ptr_)) {
        byte = uart_getc(serial_ptr_);
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        uart_putc(serial_ptr_, byte);
    }

    // wait for bytes sent out on TX line to be echoed on RX line
    uint32_t echo_delay = 0;
    while ((uart_is_readable(serial_ptr_) < datagram_size) && (echo_delay < ECHO_DELAY_MAX_MICROSECONDS)) {
        sleep_ms(ECHO_DELAY_INC_MICROSECONDS);
        echo_delay += ECHO_DELAY_INC_MICROSECONDS;
    }

    if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS) {
        blocking_ = true;
        return;
    }

    // clear RX buffer of echo bytes
    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = uart_getc(serial_ptr_);
    }
}

void TMC2209_write(uint8_t register_address, uint32_t data) {
    struct WriteReadReplyDatagram write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = serial_address_;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = TMC2209_reverseData(data);
    write_datagram.crc = TMC2209_calculateCrc(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    TMC2209_sendDatagram(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

void TMC2209_moveAtVelocity(int32_t microsteps_per_period) {
    if (blocking_) {
        return;
    }
    TMC2209_write(ADDRESS_VACTUAL, microsteps_per_period);
}


