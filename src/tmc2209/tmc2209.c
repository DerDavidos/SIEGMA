/** ----------------------------------------------------------------------------
 * Adapted from:
 *  https://github.com/peterpolidoro/TMC2209
 *  Peter Polidoro peter@polidoro.io
*** ---------------------------------------------------------------------------- */

/*
    Janelia Open-Source Software
    (3-clause BSD License)

    Copyright (c) 2022 Howard Hughes Medical Institute

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or
    other materials provided with the distribution.

    * Neither the name of HHMI nor the names of its contributors may be used to
    endorse or promote products derived from this software without specific prior
    written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "tmc2209.h"
#include "tmc2209_intern.h"

#include "serialUART.h"

#include <pico/time.h>

#include <stdio.h>

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void TMC2209_setup(TMC2209_t *tmc2209, SerialUART_t serial, uint32_t serial_baud_rate, SerialAddress_t serial_address) {
    tmc2209->toff_ = TMC2209_TOFF_DEFAULT;

    tmc2209->serial_baud_rate = 500000;
    tmc2209->serial_address = serial_address;
    tmc2209->cool_step_enabled = false;
    tmc2209->blocking = false;
    tmc2209->serial_baud_rate = serial_baud_rate;

    TMC2209_setOperationModeToSerial(tmc2209, serial, serial_baud_rate, serial_address);
    TMC2209_setRegistersToDefaults(tmc2209);
    TMC2209_readAndStoreRegisters(tmc2209);
    TMC2209_minimizeMotorCurrent(tmc2209);
    TMC2209_disable(tmc2209);
    TMC2209_disableAutomaticCurrentScaling(tmc2209);
    TMC2209_disableAutomaticGradientAdaptation(tmc2209);

    TMC2209_enableCoolStep(tmc2209, SEMIN_MIN, SEMAX_MAX);

    if (!TMC2209_isSetupAndCommunicating(tmc2209))
        tmc2209->blocking = true;
}

bool TMC2209_isCommunicating(TMC2209_t *tmc2209) {
    return (TMC2209_getVersion(tmc2209) == VERSION);
}

bool TMC2209_isSetupAndCommunicating(TMC2209_t *tmc2209) {
    return TMC2209_serialOperationMode(tmc2209);
}

bool TMC2209_isCommunicatingButNotSetup(TMC2209_t *tmc2209) {
    return (TMC2209_isCommunicating(tmc2209) && (!
            TMC2209_isSetupAndCommunicating(tmc2209)));
}

void TMC2209_enable(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->chopper_config.toff = tmc2209->toff_;
    TMC2209_writeStoredChopperConfig(tmc2209);
}

void TMC2209_disable(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->chopper_config.toff = TOFF_DISABLE;
    TMC2209_writeStoredChopperConfig(tmc2209);
}

bool TMC2209_disabledByInputPin(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return false;
    TMC2209_Input_t input;
    input.bytes = TMC2209_read(tmc2209, ADDRESS_IOIN);

    return input.enn;
}

void TMC2209_setRunCurrent(TMC2209_t *tmc2209, uint8_t percent) {
    if (tmc2209->blocking)
        return;
    uint8_t run_current = TMC2209_percentToCurrentSetting(percent);
    tmc2209->driver_current.irun = run_current;
    TMC2209_writeStoredDriverCurrent(tmc2209);
}

void TMC2209_disableAutomaticCurrentScaling(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->pwm_config.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
    TMC2209_writeStoredPwmConfig(tmc2209);
}

void TMC2209_disableAutomaticGradientAdaptation(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->pwm_config.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
    TMC2209_writeStoredPwmConfig(tmc2209);
}

void TMC2209_moveAtVelocity(TMC2209_t *tmc2209, int32_t microsteps_per_period) {
    if (tmc2209->blocking) {
        TMC2209_setup(tmc2209, *tmc2209->serial_ptr, tmc2209->serial_baud_rate, tmc2209->serial_address);
    }
    if (!TMC2209_isSetupAndCommunicating(tmc2209)) {
        TMC2209_setup(tmc2209, *tmc2209->serial_ptr, tmc2209->serial_baud_rate, tmc2209->serial_address);
    }
    TMC2209_write(tmc2209, ADDRESS_VACTUAL, microsteps_per_period);
}

void TMC2209_enableCoolStep(TMC2209_t *tmc2209, uint8_t lower_threshold,
                            uint8_t upper_threshold) {
    if (tmc2209->blocking)
        return;
    lower_threshold = constrain(lower_threshold, SEMIN_MIN, SEMIN_MAX);
    tmc2209->cool_config.semin = lower_threshold;
    upper_threshold = constrain(upper_threshold, SEMAX_MIN, SEMAX_MAX);
    tmc2209->cool_config.semax = upper_threshold;
    TMC2209_write(tmc2209, ADDRESS_COOLCONF, tmc2209->cool_config.bytes);
    tmc2209->cool_step_enabled = true;
}

void TMC2209_setOperationModeToSerial(TMC2209_t *tmc2209, SerialUART_t serial, long serial_baud_rate,
                                      SerialAddress_t serial_address) {
    tmc2209->serial_ptr = &serial;
    tmc2209->serial_address = serial_address;

    SerialUART_begin(serial_baud_rate, SERIAL_8N1);

    tmc2209->global_config.
            bytes = 0;
    tmc2209->global_config.
            i_scale_analog = 0;
    tmc2209->global_config.
            pdn_disable = 1;
    tmc2209->global_config.
            mstep_reg_select = 1;
    tmc2209->global_config.
            multistep_filt = 1;

    TMC2209_writeStoredGlobalConfig(tmc2209);
}

void TMC2209_setRegistersToDefaults(TMC2209_t *tmc2209) {
    tmc2209->driver_current.bytes = 0;
    tmc2209->driver_current.ihold = IHOLD_DEFAULT;
    tmc2209->driver_current.irun = IRUN_DEFAULT;
    tmc2209->driver_current.iholddelay = IHOLDDELAY_DEFAULT;
    TMC2209_write(tmc2209, ADDRESS_IHOLD_IRUN, tmc2209->driver_current.bytes);

    tmc2209->chopper_config.bytes = CHOPPER_CONFIG_DEFAULT;
    tmc2209->chopper_config.tbl = TBL_DEFAULT;
    tmc2209->chopper_config.hend = HEND_DEFAULT;
    tmc2209->chopper_config.hstart = HSTART_DEFAULT;
    tmc2209->chopper_config.toff = TMC2209_TOFF_DEFAULT;
    TMC2209_write(tmc2209, ADDRESS_CHOPCONF, tmc2209->chopper_config.bytes);

    tmc2209->pwm_config.bytes = PWM_CONFIG_DEFAULT;
    TMC2209_write(tmc2209, ADDRESS_PWMCONF, tmc2209->pwm_config.bytes);

    tmc2209->cool_config.bytes = COOLCONF_DEFAULT;
    TMC2209_write(tmc2209, ADDRESS_COOLCONF, tmc2209->cool_config.bytes);

    TMC2209_write(tmc2209, ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
    TMC2209_write(tmc2209, ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
    TMC2209_write(tmc2209, ADDRESS_VACTUAL, VACTUAL_DEFAULT);
    TMC2209_write(tmc2209, ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
    TMC2209_write(tmc2209, ADDRESS_SGTHRS, SGTHRS_DEFAULT);
    TMC2209_write(tmc2209, ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

void TMC2209_readAndStoreRegisters(TMC2209_t *tmc2209) {
    tmc2209->global_config.bytes = TMC2209_readGlobalConfigBytes(tmc2209);
    tmc2209->chopper_config.bytes = TMC2209_readChopperConfigBytes(tmc2209);
    tmc2209->pwm_config.bytes = TMC2209_readPwmConfigBytes(tmc2209);
}

uint8_t TMC2209_getVersion(TMC2209_t *tmc2209) {
    TMC2209_Input_t input;
    input.bytes = TMC2209_read(tmc2209, ADDRESS_IOIN);

    return input.version;
}

bool TMC2209_serialOperationMode(TMC2209_t *tmc2209) {
    TMC2209_GlobalConfig_t global_config;
    global_config.bytes = TMC2209_readGlobalConfigBytes(tmc2209);

    return global_config.pdn_disable;
}

void TMC2209_minimizeMotorCurrent(TMC2209_t *tmc2209) {
    tmc2209->driver_current.irun = CURRENT_SETTING_MIN;
    tmc2209->driver_current.ihold = CURRENT_SETTING_MIN;
    TMC2209_writeStoredDriverCurrent(tmc2209);
}

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

uint8_t TMC2209_calculateCrcRead(TMC2209_ReadRequestDatagram_t datagram, uint8_t datagram_size) {
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

void TMC2209_sendDatagramRead(TMC2209_t *tmc2209, TMC2209_ReadRequestDatagram_t datagram, uint8_t datagram_size) {
    uint8_t byte;
    // clear the serial receive buffer if necessary
    while (SerialUART_available() > 0) {
        SerialUART_read();
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        SerialUART_write(byte);
    }

    // wait for bytes sent out on TX line to be echoed on RX line
    uint32_t echo_delay = 0;
    while ((SerialUART_available() < datagram_size) && (echo_delay < ECHO_DELAY_MAX_MICROSECONDS)) {
        sleep_us(ECHO_DELAY_INC_MICROSECONDS);
        echo_delay += ECHO_DELAY_INC_MICROSECONDS;
    }

    if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS) {
        tmc2209->blocking = true;
        return;
    }

    // clear RX buffer of echo bytes
    for (uint8_t i = 0; i < datagram_size; ++i) {
        SerialUART_read();
    }
}

uint8_t TMC2209_calculateCrcWrite(TMC2209_WriteReadReplyDatagram_t datagram, uint8_t datagram_size) {
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

void TMC2209_sendDatagramWrite(TMC2209_t *tmc2209, TMC2209_WriteReadReplyDatagram_t datagram, uint8_t datagram_size) {
    uint8_t byte;
    // clear the serial receive buffer if necessary
    while (SerialUART_available() > 0) {
        SerialUART_read();
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        SerialUART_write(byte);
    }

    // wait for bytes sent out on TX line to be echoed on RX line
    uint32_t echo_delay = 0;
    while ((SerialUART_available() < datagram_size) && (echo_delay < ECHO_DELAY_MAX_MICROSECONDS)) {
        sleep_us(ECHO_DELAY_INC_MICROSECONDS);
        echo_delay += ECHO_DELAY_INC_MICROSECONDS;
    }

    if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS) {
        tmc2209->blocking = true;
        return;
    }

    // clear RX buffer of echo bytes
    for (uint8_t i = 0; i < datagram_size; ++i) {
        SerialUART_read();
    }

    // Make sure UART is free again
    sleep_ms(10);
}

void TMC2209_write(TMC2209_t *tmc2209, uint8_t register_address, uint32_t data) {
    TMC2209_WriteReadReplyDatagram_t write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = tmc2209->serial_address;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = TMC2209_reverseData(data);
    write_datagram.crc = TMC2209_calculateCrcWrite(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    TMC2209_sendDatagramWrite(tmc2209, write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

uint32_t TMC2209_read(TMC2209_t *tmc2209, uint8_t register_address) {
    TMC2209_ReadRequestDatagram_t read_request_datagram;
    read_request_datagram.bytes = 0;
    read_request_datagram.sync = SYNC;
    read_request_datagram.serial_address = tmc2209->serial_address;
    read_request_datagram.register_address = register_address;
    read_request_datagram.rw = RW_READ;
    read_request_datagram.crc = TMC2209_calculateCrcRead(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    TMC2209_sendDatagramRead(tmc2209, read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    uint32_t reply_delay = 0;
    while ((SerialUART_available() < WRITE_READ_REPLY_DATAGRAM_SIZE) && (reply_delay < REPLY_DELAY_MAX_MICROSECONDS)) {
        sleep_us(REPLY_DELAY_INC_MICROSECONDS);
        reply_delay += REPLY_DELAY_INC_MICROSECONDS;
    }

    if (reply_delay >= REPLY_DELAY_MAX_MICROSECONDS) {
        tmc2209->blocking = true;
        return 0;
    }

    uint64_t byte;
    uint8_t byte_count = 0;
    TMC2209_WriteReadReplyDatagram_t read_reply_datagram;
    read_reply_datagram.bytes = 0;
    for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i) {
        byte = SerialUART_read();
        read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
    }

    // this unfortunate code was found to be necessary after testing on hardware
    uint32_t post_read_delay_repeat = POST_READ_DELAY_NUMERATOR / tmc2209->serial_baud_rate;
    for (uint32_t i = 0; i < post_read_delay_repeat; ++i) {
        sleep_us(POST_READ_DELAY_INC_MICROSECONDS);
    }

    // Make sure UART is free again
    sleep_ms(10);

    return TMC2209_reverseData(read_reply_datagram.data);
}

uint8_t TMC2209_percentToCurrentSetting(uint8_t percent) {
    uint8_t constrained_percent = constrain(percent, PERCENT_MIN, PERCENT_MAX);
    uint8_t current_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, CURRENT_SETTING_MIN,
                                  CURRENT_SETTING_MAX);
    return current_setting;
}

void TMC2209_writeStoredGlobalConfig(TMC2209_t *tmc2209) {
    TMC2209_write(tmc2209, ADDRESS_GCONF, tmc2209->global_config.bytes);
}

uint32_t TMC2209_readGlobalConfigBytes(TMC2209_t *tmc2209) {
    return TMC2209_read(tmc2209, ADDRESS_GCONF);
}

void TMC2209_writeStoredDriverCurrent(TMC2209_t *tmc2209) {
    TMC2209_write(tmc2209, ADDRESS_IHOLD_IRUN, tmc2209->driver_current.bytes);

    if (tmc2209->driver_current.irun >= SEIMIN_UPPER_CURRENT_LIMIT) {
        tmc2209->cool_config.seimin = SEIMIN_UPPER_SETTING;
    } else {
        tmc2209->cool_config.seimin = SEIMIN_LOWER_SETTING;
    }
    if (tmc2209->cool_step_enabled) {
        TMC2209_write(tmc2209, ADDRESS_COOLCONF, tmc2209->cool_config.bytes);
    }
}

void TMC2209_writeStoredChopperConfig(TMC2209_t *tmc2209) {
    TMC2209_write(tmc2209, ADDRESS_CHOPCONF, tmc2209->chopper_config.bytes);
}

uint32_t TMC2209_readChopperConfigBytes(TMC2209_t *tmc2209) {
    return TMC2209_read(tmc2209, ADDRESS_CHOPCONF);
}

void TMC2209_writeStoredPwmConfig(TMC2209_t *tmc2209) {
    TMC2209_write(tmc2209, ADDRESS_PWMCONF, tmc2209->pwm_config.bytes);
}

uint32_t TMC2209_readPwmConfigBytes(TMC2209_t *tmc2209) {
    return TMC2209_read(tmc2209, ADDRESS_PWMCONF);
}
