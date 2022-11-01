// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include <cstdint>
#include "TMC2209.h"
#include "pico/time.h"

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

TMC2209::TMC2209() {
    blocking_ = true;
    serial_ptr_ = nullptr;
    serial_baud_rate_ = 500000;
    serial_address_ = SERIAL_ADDRESS_0;
    cool_step_enabled_ = false;
}

void TMC2209::setup(HardwareSerial &serial,
                    long serial_baud_rate,
                    SerialAddress serial_address) {
    blocking_ = false;
    serial_baud_rate_ = serial_baud_rate;
    setOperationModeToSerial(serial, serial_baud_rate, serial_address);
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

bool TMC2209::isCommunicating() {
    return (getVersion() == VERSION);
}

bool TMC2209::isSetupAndCommunicating() {
    return serialOperationMode();
}

bool TMC2209::isCommunicatingButNotSetup() {
    return (isCommunicating() && (not isSetupAndCommunicating()));
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

bool TMC2209::disabledByInputPin() {
    if (blocking_) {
        return false;
    }
    Input input;
    input.bytes = read(ADDRESS_IOIN);

    return input.enn;
}

void TMC2209::setMicrostepsPerStep(uint16_t microsteps_per_step) {
    if (blocking_) {
        return;
    }
    uint16_t microsteps_per_step_shifted = constrain(microsteps_per_step,
                                                     MICROSTEPS_PER_STEP_MIN,
                                                     MICROSTEPS_PER_STEP_MAX);
    microsteps_per_step_shifted = microsteps_per_step >> 1;
    uint16_t exponent = 0;
    while (microsteps_per_step_shifted > 0) {
        microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
        ++exponent;
    }
    setMicrostepsPerStepPowerOfTwo(exponent);
}

void TMC2209::setMicrostepsPerStepPowerOfTwo(uint8_t exponent) {
    if (blocking_) {
        return;
    }
    switch (exponent) {
        case 0: {
            chopper_config_.mres = MRES_001;
            break;
        }
        case 1: {
            chopper_config_.mres = MRES_002;
            break;
        }
        case 2: {
            chopper_config_.mres = MRES_004;
            break;
        }
        case 3: {
            chopper_config_.mres = MRES_008;
            break;
        }
        case 4: {
            chopper_config_.mres = MRES_016;
            break;
        }
        case 5: {
            chopper_config_.mres = MRES_032;
            break;
        }
        case 6: {
            chopper_config_.mres = MRES_064;
            break;
        }
        case 7: {
            chopper_config_.mres = MRES_128;
            break;
        }
        case 8:
        default: {
            chopper_config_.mres = MRES_256;
            break;
        }
    }
    writeStoredChopperConfig();
}

uint16_t TMC2209::getMicrostepsPerStep() {
    if (blocking_) {
        return 0;
    }
    uint16_t microsteps_per_step_exponent;
    switch (chopper_config_.mres) {
        case MRES_001: {
            microsteps_per_step_exponent = 0;
            break;
        }
        case MRES_002: {
            microsteps_per_step_exponent = 1;
            break;
        }
        case MRES_004: {
            microsteps_per_step_exponent = 2;
            break;
        }
        case MRES_008: {
            microsteps_per_step_exponent = 3;
            break;
        }
        case MRES_016: {
            microsteps_per_step_exponent = 4;
            break;
        }
        case MRES_032: {
            microsteps_per_step_exponent = 5;
            break;
        }
        case MRES_064: {
            microsteps_per_step_exponent = 6;
            break;
        }
        case MRES_128: {
            microsteps_per_step_exponent = 7;
            break;
        }
        case MRES_256:
        default: {
            microsteps_per_step_exponent = 8;
            break;
        }
    }
    return 1 << microsteps_per_step_exponent;
}

void TMC2209::setRunCurrent(uint8_t percent) {
    if (blocking_) {
        return;
    }
    uint8_t run_current = percentToCurrentSetting(percent);
    driver_current_.irun = run_current;
    writeStoredDriverCurrent();
}

void TMC2209::setHoldCurrent(uint8_t percent) {
    if (blocking_) {
        return;
    }
    uint8_t hold_current = percentToCurrentSetting(percent);

    driver_current_.ihold = hold_current;
    writeStoredDriverCurrent();
}

void TMC2209::setHoldDelay(uint8_t percent) {
    if (blocking_) {
        return;
    }
    uint8_t hold_delay = percentToHoldDelaySetting(percent);

    driver_current_.iholddelay = hold_delay;
    writeStoredDriverCurrent();
}

void TMC2209::setAllCurrentValues(uint8_t run_current_percent,
                                  uint8_t hold_current_percent,
                                  uint8_t hold_delay_percent) {
    if (blocking_) {
        return;
    }
    uint8_t run_current = percentToCurrentSetting(run_current_percent);
    uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
    uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

    driver_current_.irun = run_current;
    driver_current_.ihold = hold_current;
    driver_current_.iholddelay = hold_delay;
    writeStoredDriverCurrent();
}

TMC2209::Settings TMC2209::getSettings() {
    Settings settings;
    settings.is_communicating = isCommunicating();

    if (settings.is_communicating) {
        readAndStoreRegisters();

        settings.is_setup = global_config_.pdn_disable;
        settings.enabled = (chopper_config_.toff > TOFF_DISABLE);
        settings.microsteps_per_step = getMicrostepsPerStep();
        settings.inverse_motor_direction_enabled = global_config_.shaft;
        settings.stealth_chop_enabled = not global_config_.enable_spread_cycle;
        settings.standstill_mode = pwm_config_.freewheel;
        settings.irun_percent = currentSettingToPercent(driver_current_.irun);
        settings.irun_register_value = driver_current_.irun;
        settings.ihold_percent = currentSettingToPercent(driver_current_.ihold);
        settings.ihold_register_value = driver_current_.ihold;
        settings.iholddelay_percent = holdDelaySettingToPercent(driver_current_.iholddelay);
        settings.iholddelay_register_value = driver_current_.iholddelay;
        settings.automatic_current_scaling_enabled = pwm_config_.pwm_autoscale;
        settings.automatic_gradient_adaptation_enabled = pwm_config_.pwm_autograd;
        settings.pwm_offset = pwm_config_.pwm_offset;
        settings.pwm_gradient = pwm_config_.pwm_grad;
        settings.cool_step_enabled = cool_step_enabled_;
        settings.analog_current_scaling_enabled = global_config_.i_scale_analog;
        settings.internal_sense_resistors_enabled = global_config_.internal_rsense;
    } else {
        settings.is_setup = false;
        settings.enabled = false;
        settings.microsteps_per_step = 0;
        settings.inverse_motor_direction_enabled = false;
        settings.stealth_chop_enabled = false;
        settings.standstill_mode = pwm_config_.freewheel;
        settings.irun_percent = 0;
        settings.irun_register_value = 0;
        settings.ihold_percent = 0;
        settings.ihold_register_value = 0;
        settings.iholddelay_percent = 0;
        settings.iholddelay_register_value = 0;
        settings.automatic_current_scaling_enabled = false;
        settings.automatic_gradient_adaptation_enabled = false;
        settings.pwm_offset = 0;
        settings.pwm_gradient = 0;
        settings.cool_step_enabled = false;
        settings.analog_current_scaling_enabled = false;
        settings.internal_sense_resistors_enabled = false;
    }

    return settings;
}

TMC2209::Status TMC2209::getStatus() {
    DriveStatus drive_status;
    drive_status.bytes = 0;
    if (not blocking_) {
        drive_status.bytes = read(ADDRESS_DRV_STATUS);
    }
    return drive_status.status;
}

void TMC2209::enableInverseMotorDirection() {
    if (blocking_) {
        return;
    }
    global_config_.shaft = 1;
    writeStoredGlobalConfig();
}

void TMC2209::disableInverseMotorDirection() {
    if (blocking_) {
        return;
    }
    global_config_.shaft = 0;
    writeStoredGlobalConfig();
}

void TMC2209::setStandstillMode(TMC2209::StandstillMode mode) {
    if (blocking_) {
        return;
    }
    pwm_config_.freewheel = mode;
    writeStoredPwmConfig();
}

void TMC2209::enableAutomaticCurrentScaling() {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
    writeStoredPwmConfig();
}

void TMC2209::disableAutomaticCurrentScaling() {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
    writeStoredPwmConfig();
}

void TMC2209::enableAutomaticGradientAdaptation() {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
    writeStoredPwmConfig();
}

void TMC2209::disableAutomaticGradientAdaptation() {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
    writeStoredPwmConfig();
}

void TMC2209::setPwmOffset(uint8_t pwm_amplitude) {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_offset = pwm_amplitude;
    writeStoredPwmConfig();
}

void TMC2209::setPwmGradient(uint8_t pwm_amplitude) {
    if (blocking_) {
        return;
    }
    pwm_config_.pwm_grad = pwm_amplitude;
    writeStoredPwmConfig();
}

void TMC2209::setPowerDownDelay(uint8_t delay) {
    if (blocking_) {
        return;
    }
    write(ADDRESS_TPOWERDOWN, delay);
}

uint8_t TMC2209::getInterfaceTransmissionCounter() {
    if (blocking_) {
        return 0;
    }
    return read(ADDRESS_IFCNT);
}

void TMC2209::moveAtVelocity(int32_t microsteps_per_period) {
    if (blocking_) {
        return;
    }
    write(ADDRESS_VACTUAL, microsteps_per_period);
}

void TMC2209::moveUsingStepDirInterface() {
    if (blocking_) {
        return;
    }
    write(ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void TMC2209::enableStealthChop() {
    if (blocking_) {
        return;
    }
    global_config_.enable_spread_cycle = 0;
    writeStoredGlobalConfig();
}

void TMC2209::disableStealthChop() {
    if (blocking_) {
        return;
    }
    global_config_.enable_spread_cycle = 1;
    writeStoredGlobalConfig();
}

uint32_t TMC2209::getInterstepDuration() {
    if (blocking_) {
        return 0;
    }
    return read(ADDRESS_TSTEP);
}

void TMC2209::setCoolStepDurationThreshold(uint32_t duration_threshold) {
    if (blocking_) {
        return;
    }
    write(ADDRESS_TCOOLTHRS, duration_threshold);
}

void TMC2209::setStealthChopDurationThreshold(uint32_t duration_threshold) {
    if (blocking_) {
        return;
    }
    write(ADDRESS_TPWMTHRS, duration_threshold);
}

uint16_t TMC2209::getStallGuardResult() {
    if (blocking_) {
        return 0;
    }
    return read(ADDRESS_SG_RESULT);
}

void TMC2209::setStallGuardThreshold(uint8_t stall_guard_threshold) {
    if (blocking_) {
        return;
    }
    write(ADDRESS_SGTHRS, stall_guard_threshold);
}

uint8_t TMC2209::getPwmScaleSum() {
    if (blocking_) {
        return 0;
    }
    PwmScale pwm_scale;
    pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

    return pwm_scale.pwm_scale_sum;
}

int16_t TMC2209::getPwmScaleAuto() {
    if (blocking_) {
        return 0;
    }
    PwmScale pwm_scale;
    pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

    return pwm_scale.pwm_scale_auto;
}

uint8_t TMC2209::getPwmOffsetAuto() {
    if (blocking_) {
        return 0;
    }
    PwmAuto pwm_auto;
    pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

    return pwm_auto.pwm_offset_auto;
}

uint8_t TMC2209::getPwmGradientAuto() {
    if (blocking_) {
        return 0;
    }
    PwmAuto pwm_auto;
    pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

    return pwm_auto.pwm_gradient_auto;
}

void TMC2209::enableCoolStep(uint8_t lower_threshold,
                             uint8_t upper_threshold) {
    if (blocking_) {
        return;
    }
    lower_threshold = constrain(lower_threshold, SEMIN_MIN, SEMIN_MAX);
    cool_config_.semin = lower_threshold;
    upper_threshold = constrain(upper_threshold, SEMAX_MIN, SEMAX_MAX);
    cool_config_.semax = upper_threshold;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
    cool_step_enabled_ = true;
}

void TMC2209::disableCoolStep() {
    if (blocking_) {
        return;
    }
    cool_config_.semin = SEMIN_OFF;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
    cool_step_enabled_ = false;
}

void TMC2209::setCoolStepCurrentIncrement(CurrentIncrement current_increment) {
    if (blocking_) {
        return;
    }
    cool_config_.seup = current_increment;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void TMC2209::setCoolStepMeasurementCount(MeasurementCount measurement_count) {
    if (blocking_) {
        return;
    }
    cool_config_.sedn = measurement_count;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
}

uint16_t TMC2209::getMicrostepCounter() {
    if (blocking_) {
        return 0;
    }
    return read(ADDRESS_MSCNT);
}

void TMC2209::enableAnalogCurrentScaling() {
    if (blocking_) {
        return;
    }
    global_config_.i_scale_analog = 1;
    writeStoredGlobalConfig();
}

void TMC2209::disableAnalogCurrentScaling() {
    if (blocking_) {
        return;
    }
    global_config_.i_scale_analog = 0;
    writeStoredGlobalConfig();
}

void TMC2209::useExternalSenseResistors() {
    if (blocking_) {
        return;
    }
    global_config_.internal_rsense = 0;
    writeStoredGlobalConfig();
}

void TMC2209::useInternalSenseResistors() {
    if (blocking_) {
        return;
    }
    global_config_.internal_rsense = 1;
    writeStoredGlobalConfig();
}

// private
void TMC2209::setOperationModeToSerial(HardwareSerial &serial,
                                       long serial_baud_rate,
                                       SerialAddress serial_address) {
    serial_ptr_ = &serial;
    serial_address_ = serial_address;

    serial_ptr_->begin(serial_baud_rate);

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

uint8_t TMC2209::getVersion() {
    Input input;
    input.bytes = read(ADDRESS_IOIN);

    return input.version;
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

template<typename Datagram>
uint8_t TMC2209::calculateCrc(Datagram &datagram,
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

template<typename Datagram>
void TMC2209::sendDatagram(Datagram &datagram,
                           uint8_t datagram_size) {
    if (not serial_ptr_) {
        return;
    }

    uint8_t byte;

    // clear the serial receive buffer if necessary
    while (serial_ptr_->available() > 0) {
        byte = serial_ptr_->read();
    }

    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        serial_ptr_->write(byte);
    }

    // wait for bytes sent out on TX line to be echoed on RX line
    uint32_t echo_delay = 0;
    while ((serial_ptr_->available() < datagram_size) and
           (echo_delay < ECHO_DELAY_MAX_MICROSECONDS)) {
        sleep_ms(ECHO_DELAY_INC_MICROSECONDS);
        echo_delay += ECHO_DELAY_INC_MICROSECONDS;
    }

    if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS) {
        blocking_ = true;
        return;
    }

    // clear RX buffer of echo bytes
    for (uint8_t i = 0; i < datagram_size; ++i) {
        byte = serial_ptr_->read();
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
    write_datagram.crc = calculateCrc(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    sendDatagram(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

uint32_t TMC2209::read(uint8_t register_address) {
    if (not serial_ptr_) {
        return 0;
    }
    ReadRequestDatagram read_request_datagram;
    read_request_datagram.bytes = 0;
    read_request_datagram.sync = SYNC;
    read_request_datagram.serial_address = serial_address_;
    read_request_datagram.register_address = register_address;
    read_request_datagram.rw = RW_READ;
    read_request_datagram.crc = calculateCrc(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    sendDatagram(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    uint32_t reply_delay = 0;
    while ((serial_ptr_->available() < WRITE_READ_REPLY_DATAGRAM_SIZE) and
           (reply_delay < REPLY_DELAY_MAX_MICROSECONDS)) {
        sleep_ms(REPLY_DELAY_INC_MICROSECONDS);
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
        byte = serial_ptr_->read();
        read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
    }

    // this unfortunate code was found to be necessary after testing on hardware
    uint32_t post_read_delay_repeat = POST_READ_DELAY_NUMERATOR / serial_baud_rate_;
    for (uint8_t i = 0; i < post_read_delay_repeat; ++i) {
        sleep_ms(POST_READ_DELAY_INC_MICROSECONDS);
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

uint8_t TMC2209::currentSettingToPercent(uint8_t current_setting) {
    uint8_t percent = map(current_setting,
                          CURRENT_SETTING_MIN,
                          CURRENT_SETTING_MAX,
                          PERCENT_MIN,
                          PERCENT_MAX);
    return percent;
}

uint8_t TMC2209::percentToHoldDelaySetting(uint8_t percent) {
    uint8_t constrained_percent = constrain(percent,
                                            PERCENT_MIN,
                                            PERCENT_MAX);
    uint8_t hold_delay_setting = map(constrained_percent,
                                     PERCENT_MIN,
                                     PERCENT_MAX,
                                     HOLD_DELAY_MIN,
                                     HOLD_DELAY_MAX);
    return hold_delay_setting;
}

uint8_t TMC2209::holdDelaySettingToPercent(uint8_t hold_delay_setting) {
    uint8_t percent = map(hold_delay_setting,
                          HOLD_DELAY_MIN,
                          HOLD_DELAY_MAX,
                          PERCENT_MIN,
                          PERCENT_MAX);
    return percent;
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
