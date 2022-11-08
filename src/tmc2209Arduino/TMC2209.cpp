// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#include <cstdint>
#include "TMC2209.h"

extern "C" {
#include "pico/time.h"
#include "SerialUART.h"
}

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC2209.h"

TMC2209::TMC2209() {
    blocking_ = true;
//    serial_ptr_ = nullptr;
    serial_baud_rate_ = 500000;
    serial_address_ = SERIAL_ADDRESS_0;
    cool_step_enabled_ = false;
}

void TMC2209::setup(long serial_baud_rate,
                    SerialAddress serial_address) {
    blocking_ = false;
    serial_baud_rate_ = serial_baud_rate;
    setOperationModeToSerial(serial_baud_rate, serial_address);
    setRegistersToDefaults();
    readAndStoreRegisters();
    minimizeMotorCurrent();
    disable();
    disableAutomaticCurrentScaling();
    disableAutomaticGradientAdaptation();
    if (not isSetupAndCommunicating()) {
        blocking_ = true;
    }
}

bool TMC2209::isSetupAndCommunicating() {
    return serialOperationMode();
}

void TMC2209::enable() {
    if (blocking_) {
        return;
    }
    chopper_config_.toff = toff_;
    writeStoredChopperConfig();
}

void TMC2209::disable() {
    if (blocking_) {
        return;
    }
    chopper_config_.toff = TOFF_DISABLE;
    writeStoredChopperConfig();
}

void TMC2209::setRunCurrent(uint8_t percent) {
    if (blocking_) {
        return;
    }
    uint8_t run_current = percentToCurrentSetting(percent);
    driver_current_.irun = run_current;
    writeStoredDriverCurrent();
}

void TMC2209::disableAutomaticCurrentScaling() {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
    writeStoredPwmConfig();
}

void TMC2209::disableAutomaticGradientAdaptation() {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
    writeStoredPwmConfig();
}

void TMC2209::moveAtVelocity(int32_t microsteps_per_period) {
    if (blocking_) {
        return;
    }
    write(ADDRESS_VACTUAL, microsteps_per_period);
}

// private
void TMC2209::setOperationModeToSerial(long serial_baud_rate, SerialAddress serial_address) {
//    serial_ptr_ = &serial;
    begin(115200);
    serial_address_ = serial_address;

//    serial_ptr_->begin(serial_baud_rate);

    global_config_.bytes = 0;
    global_config_.i_scale_analog = 0;
    global_config_.pdn_disable = 1;
    global_config_.mstep_reg_select = 1;
    global_config_.multistep_filt = 1;

    writeStoredGlobalConfig();
}

void TMC2209::setRegistersToDefaults() {
    driver_current_.bytes = 0;
    driver_current_.ihold = IHOLD_DEFAULT;
    driver_current_.irun = IRUN_DEFAULT;
    driver_current_.iholddelay = IHOLDDELAY_DEFAULT;
    write(ADDRESS_IHOLD_IRUN, driver_current_.bytes);

    chopper_config_.bytes = CHOPPER_CONFIG_DEFAULT;
    chopper_config_.tbl = TBL_DEFAULT;
    chopper_config_.hend = HEND_DEFAULT;
    chopper_config_.hstart = HSTART_DEFAULT;
    chopper_config_.toff = TOFF_DEFAULT;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);

    pwm_config_.bytes = PWM_CONFIG_DEFAULT;
    write(ADDRESS_PWMCONF, pwm_config_.bytes);

    cool_config_.bytes = COOLCONF_DEFAULT;
    write(ADDRESS_COOLCONF, cool_config_.bytes);

    write(ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
    write(ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
    write(ADDRESS_VACTUAL, VACTUAL_DEFAULT);
    write(ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
    write(ADDRESS_SGTHRS, SGTHRS_DEFAULT);
    write(ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

void TMC2209::readAndStoreRegisters() {
    global_config_.bytes = readGlobalConfigBytes();
    chopper_config_.bytes = readChopperConfigBytes();
    pwm_config_.bytes = readPwmConfigBytes();
}

bool TMC2209::serialOperationMode() {
    GlobalConfig global_config;
    global_config.bytes = readGlobalConfigBytes();

    return global_config.pdn_disable;
}

void TMC2209::minimizeMotorCurrent() {
    driver_current_.irun = CURRENT_SETTING_MIN;
    driver_current_.ihold = CURRENT_SETTING_MIN;
    writeStoredDriverCurrent();
}

uint32_t TMC2209::reverseData(uint32_t data) {
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

uint8_t TMC2209::calculateCrcRead(ReadRequestDatagram &datagram,
                                  uint8_t datagram_size) {
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

uint8_t TMC2209::calculateCrcWrite(WriteReadReplyDatagram &datagram,
                                   uint8_t datagram_size) {
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

void TMC2209::sendDatagramRead(ReadRequestDatagram &datagram,
                               uint8_t datagram_size) {
//    if (not serial_ptr_) {
//        return;
//    }

    uint8_t byte;

    // clear the serial receive buffer if necessary
    while (available() > 0) {
        byte = SerialUART_read();
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        SerialUART_write(byte);
    }

    // wait for bytes sent out on TX line to be echoed on RX line
    uint32_t echo_delay = 0;
    while ((available() < datagram_size) and
           (echo_delay < ECHO_DELAY_MAX_MICROSECONDS)) {
        sleep_us(ECHO_DELAY_INC_MICROSECONDS);
        echo_delay += ECHO_DELAY_INC_MICROSECONDS;
    }

    if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS) {
        blocking_ = true;
        return;
    }

    // clear RX buffer of echo bytes
    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = SerialUART_read();
    }
}

void TMC2209::sendDatagramWrite(WriteReadReplyDatagram &datagram,
                                uint8_t datagram_size) {
//    if (not serial_ptr_) {
//        return;
//    }

    uint8_t byte;

    // clear the serial receive buffer if necessary
    while (available() > 0) {
        byte = SerialUART_read();
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        // Serial.println(byte);
        SerialUART_write(byte);
    }

    // wait for bytes sent out on TX line to be echoed on RX line
    uint32_t echo_delay = 0;
    while ((available() < datagram_size) and
           (echo_delay < ECHO_DELAY_MAX_MICROSECONDS)) {
        sleep_us(ECHO_DELAY_INC_MICROSECONDS);
        echo_delay += ECHO_DELAY_INC_MICROSECONDS;
    }

    if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS) {
        blocking_ = true;
        return;
    }

    // clear RX buffer of echo bytes
    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = SerialUART_read();
    }
}

void TMC2209::write(uint8_t register_address,
                    uint32_t data) {
    WriteReadReplyDatagram write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = serial_address_;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = reverseData(data);
    write_datagram.crc = calculateCrcWrite(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    sendDatagramWrite(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

uint32_t TMC2209::read(uint8_t register_address) {
//    if (not serial_ptr_) {
//        return 0;
//    }
    ReadRequestDatagram read_request_datagram;
    read_request_datagram.bytes = 0;
    read_request_datagram.sync = SYNC;
    read_request_datagram.serial_address = serial_address_;
    read_request_datagram.register_address = register_address;
    read_request_datagram.rw = RW_READ;
    read_request_datagram.crc = calculateCrcRead(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    sendDatagramRead(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    uint32_t reply_delay = 0;
    while ((available() < WRITE_READ_REPLY_DATAGRAM_SIZE) and
           (reply_delay < REPLY_DELAY_MAX_MICROSECONDS)) {
        sleep_us(REPLY_DELAY_INC_MICROSECONDS);
        reply_delay += REPLY_DELAY_INC_MICROSECONDS;
    }

    if (reply_delay >= REPLY_DELAY_MAX_MICROSECONDS) {
        blocking_ = true;
        return 0;
    }

    uint64_t byte;
    uint8_t byte_count = 0;
    WriteReadReplyDatagram read_reply_datagram;
    read_reply_datagram.bytes = 0;
    for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i) {
        byte = SerialUART_read();
        read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
    }

    // this unfortunate code was found to be necessary after testing on hardware
    uint32_t post_read_delay_repeat = POST_READ_DELAY_NUMERATOR / serial_baud_rate_;
    for (uint8_t i = 0; i < post_read_delay_repeat; ++i) {
        sleep_us(POST_READ_DELAY_INC_MICROSECONDS);
    }
    return reverseData(read_reply_datagram.data);
}

uint8_t TMC2209::percentToCurrentSetting(uint8_t percent) {
    uint8_t constrained_percent = constrain(percent,
                                            PERCENT_MIN,
                                            PERCENT_MAX);
    uint8_t current_setting = map(constrained_percent,
                                  PERCENT_MIN,
                                  PERCENT_MAX,
                                  CURRENT_SETTING_MIN,
                                  CURRENT_SETTING_MAX);
    return current_setting;
}

void TMC2209::writeStoredGlobalConfig() {
    write(ADDRESS_GCONF, global_config_.bytes);
}

uint32_t TMC2209::readGlobalConfigBytes() {
    return read(ADDRESS_GCONF);
}

void TMC2209::writeStoredDriverCurrent() {
    write(ADDRESS_IHOLD_IRUN, driver_current_.bytes);

    if (driver_current_.irun >= SEIMIN_UPPER_CURRENT_LIMIT) {
        cool_config_.seimin = SEIMIN_UPPER_SETTING;
    } else {
        cool_config_.seimin = SEIMIN_LOWER_SETTING;
    }
    if (cool_step_enabled_) {
        write(ADDRESS_COOLCONF, cool_config_.bytes);
    }
}

void TMC2209::writeStoredChopperConfig() {
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

uint32_t TMC2209::readChopperConfigBytes() {
    return read(ADDRESS_CHOPCONF);
}

void TMC2209::writeStoredPwmConfig() {
    write(ADDRESS_PWMCONF, pwm_config_.bytes);
}

uint32_t TMC2209::readPwmConfigBytes() {
    return read(ADDRESS_PWMCONF);
}
