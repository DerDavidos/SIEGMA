// ----------------------------------------------------------------------------
// Adapted from: https://github.com/peterpolidoro/TMC2209
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#include "pico/time.h"

#include "TMC2209.h"
#include "SerialUART.h"

#include <stdio.h>

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void TMC2209_setup(TMC2209_t *tmc2209, SerialUART_t serial, long serial_baud_rate, SerialAddress_t serial_address) {
    tmc2209->toff_ = TMC2209_TOFF_DEFAULT;

    tmc2209->serial_ptr = NULL;
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

void TMC2209_setMicrostepsPerStep(TMC2209_t *tmc2209, uint16_t microsteps_per_step) {
    if (tmc2209->blocking)
        return;
    uint16_t microsteps_per_step_shifted = constrain(microsteps_per_step,
                                                     MICROSTEPS_PER_STEP_MIN,
                                                     MICROSTEPS_PER_STEP_MAX);
    microsteps_per_step_shifted = microsteps_per_step >> 1;
    uint16_t exponent = 0;
    while (microsteps_per_step_shifted > 0) {
        microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
        ++exponent;
    }
    TMC2209_setMicrostepsPerStepPowerOfTwo(tmc2209, exponent);
}

void TMC2209_setMicrostepsPerStepPowerOfTwo(TMC2209_t *tmc2209, uint8_t exponent) {
    if (tmc2209->blocking)
        return;
    switch (exponent) {
        case 0:
            tmc2209->chopper_config.mres = MRES_001;
            break;
        case 1:
            tmc2209->chopper_config.mres = MRES_002;
            break;
        case 2:
            tmc2209->chopper_config.mres = MRES_004;
            break;
        case 3:
            tmc2209->chopper_config.mres = MRES_008;
            break;
        case 4:
            tmc2209->chopper_config.mres = MRES_016;
            break;
        case 5:
            tmc2209->chopper_config.mres = MRES_032;
            break;
        case 6:
            tmc2209->chopper_config.mres = MRES_064;
            break;
        case 7:
            tmc2209->chopper_config.mres = MRES_128;
            break;
        case 8:
        default:
            tmc2209->chopper_config.mres = MRES_256;
    }
    TMC2209_writeStoredChopperConfig(tmc2209);
}

uint16_t TMC2209_getMicrostepsPerStep(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return 0;
    uint16_t microsteps_per_step_exponent;
    switch (tmc2209->chopper_config.mres) {
        case MRES_001:
            microsteps_per_step_exponent = 0;
            break;
        case MRES_002:
            microsteps_per_step_exponent = 1;
            break;
        case MRES_004:
            microsteps_per_step_exponent = 2;
            break;
        case MRES_008:
            microsteps_per_step_exponent = 3;
            break;
        case MRES_016:
            microsteps_per_step_exponent = 4;
            break;
        case MRES_032:
            microsteps_per_step_exponent = 5;
            break;
        case MRES_064:
            microsteps_per_step_exponent = 6;
            break;
        case MRES_128:
            microsteps_per_step_exponent = 7;
            break;
        case MRES_256:
        default:
            microsteps_per_step_exponent = 8;
    }
    return 1 << microsteps_per_step_exponent;
}

void TMC2209_setRunCurrent(TMC2209_t *tmc2209, uint8_t percent) {
    if (tmc2209->blocking)
        return;
    uint8_t run_current = TMC2209_percentToCurrentSetting(percent);
    tmc2209->driver_current.irun = run_current;
    TMC2209_writeStoredDriverCurrent(tmc2209);
}

void TMC2209_setHoldCurrent(TMC2209_t *tmc2209, uint8_t percent) {
    if (tmc2209->blocking)
        return;
    uint8_t hold_current = TMC2209_percentToCurrentSetting(percent);

    tmc2209->driver_current.ihold = hold_current;
    TMC2209_writeStoredDriverCurrent(tmc2209);
}

void TMC2209_setHoldDelay(TMC2209_t *tmc2209, uint8_t percent) {
    if (tmc2209->blocking)
        return;
    uint8_t hold_delay = TMC2209_percentToHoldDelaySetting(percent);

    tmc2209->driver_current.iholddelay = hold_delay;
    TMC2209_writeStoredDriverCurrent(tmc2209);
}

void TMC2209_setAllCurrentValues(TMC2209_t *tmc2209, uint8_t run_current_percent,
                                 uint8_t hold_current_percent,
                                 uint8_t hold_delay_percent) {
    if (tmc2209->blocking)
        return;
    uint8_t run_current = TMC2209_percentToCurrentSetting(run_current_percent);
    uint8_t hold_current = TMC2209_percentToCurrentSetting(hold_current_percent);
    uint8_t hold_delay = TMC2209_percentToHoldDelaySetting(hold_delay_percent);

    tmc2209->driver_current.irun = run_current;
    tmc2209->driver_current.ihold = hold_current;
    tmc2209->driver_current.iholddelay = hold_delay;
    TMC2209_writeStoredDriverCurrent(tmc2209);
}

TMC2209_Settings_t TMC2209_getSettings(TMC2209_t *tmc2209) {
    TMC2209_Settings_t settings;
    settings.is_communicating = TMC2209_isCommunicating(tmc2209);

    if (settings.is_communicating) {
        TMC2209_readAndStoreRegisters(tmc2209);

        settings.is_setup = tmc2209->global_config.pdn_disable;
        settings.enabled = (tmc2209->chopper_config.toff > TOFF_DISABLE);
        settings.microsteps_per_step = TMC2209_getMicrostepsPerStep(tmc2209);
        settings.inverse_motor_direction_enabled = tmc2209->global_config.shaft;
        settings.stealth_chop_enabled = !
                tmc2209->global_config.enable_spread_cycle;
        settings.standstill_mode = tmc2209->pwm_config.freewheel;
        settings.irun_percent = TMC2209_currentSettingToPercent(tmc2209->driver_current.irun);
        settings.irun_register_value = tmc2209->driver_current.irun;
        settings.ihold_percent = TMC2209_currentSettingToPercent(tmc2209->driver_current.ihold);
        settings.ihold_register_value = tmc2209->driver_current.ihold;
        settings.iholddelay_percent = TMC2209_holdDelaySettingToPercent(tmc2209->driver_current.iholddelay);
        settings.iholddelay_register_value = tmc2209->driver_current.iholddelay;
        settings.automatic_current_scaling_enabled = tmc2209->pwm_config.pwm_autoscale;
        settings.automatic_gradient_adaptation_enabled = tmc2209->pwm_config.pwm_autograd;
        settings.pwm_offset = tmc2209->pwm_config.pwm_offset;
        settings.pwm_gradient = tmc2209->pwm_config.pwm_grad;
        settings.cool_step_enabled = tmc2209->cool_step_enabled;
        settings.analog_current_scaling_enabled = tmc2209->global_config.i_scale_analog;
        settings.internal_sense_resistors_enabled = tmc2209->global_config.internal_rsense;
    } else {
        settings.is_setup = false;

        settings.enabled = false;
        settings.microsteps_per_step = 0;
        settings.inverse_motor_direction_enabled = false;
        settings.stealth_chop_enabled = false;
        settings.standstill_mode = tmc2209->pwm_config.freewheel;
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

Status_t TMC2209_getStatus(TMC2209_t *tmc2209) {
    TMC2209_DriveStatus_t drive_status = {.bytes= 0};
    if (!tmc2209->blocking)
        drive_status.bytes = TMC2209_read(tmc2209, ADDRESS_DRV_STATUS);
    return drive_status.status;
}

void TMC2209_enableInverseMotorDirection(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->global_config.shaft = 1;
    TMC2209_writeStoredGlobalConfig(tmc2209);
}

void TMC2209_disableInverseMotorDirection(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->global_config.shaft = 0;
    TMC2209_writeStoredGlobalConfig(tmc2209);
}

void TMC2209_setStandstillMode(TMC2209_t *tmc2209, TMC2209_StandstillMode_t mode) {
    if (tmc2209->blocking)
        return;
    tmc2209->pwm_config.freewheel = mode;
    TMC2209_writeStoredPwmConfig(tmc2209);
}

void TMC2209_enableAutomaticCurrentScaling(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->pwm_config.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
    TMC2209_writeStoredPwmConfig(tmc2209);
}

void TMC2209_disableAutomaticCurrentScaling(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->pwm_config.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
    TMC2209_writeStoredPwmConfig(tmc2209);
}

void TMC2209_enableAutomaticGradientAdaptation(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->pwm_config.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
    TMC2209_writeStoredPwmConfig(tmc2209);
}

void TMC2209_disableAutomaticGradientAdaptation(TMC2209_t *tmc2209) {
    if (tmc2209->blocking)
        return;
    tmc2209->pwm_config.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
    TMC2209_writeStoredPwmConfig(tmc2209);
}

//void TMC2209_setPwmOffset(TMC2209_t *tmc2209,uint8_t pwm_amplitude) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_pwm_config_.pwm_offset = pwm_amplitude;
//    TMC2209_writeStoredPwmConfig();
//}
//
//void TMC2209_setPwmGradient(TMC2209_t *tmc2209,uint8_t pwm_amplitude) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_pwm_config_.pwm_grad = pwm_amplitude;
//    TMC2209_writeStoredPwmConfig();
//}
//
//void TMC2209_setPowerDownDelay(TMC2209_t *tmc2209,uint8_t delay) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    TMC2209_write(ADDRESS_TPOWERDOWN, delay);
//}

//uint8_t TMC2209_getInterfaceTransmissionCounter(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    return TMC2209_read(ADDRESS_IFCNT);
//}

void TMC2209_moveAtVelocity(TMC2209_t *tmc2209, int32_t microsteps_per_period) {
    if (tmc2209->blocking)
        return;
    TMC2209_write(tmc2209, ADDRESS_VACTUAL, microsteps_per_period);
}

//void TMC2209_moveUsingStepDirInterface(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    TMC2209_write(ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
//}
//
//void TMC2209_enableStealthChop(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_global_config_.enable_spread_cycle = 0;
//    TMC2209_writeStoredGlobalConfig();
//}
//
//void TMC2209_disableStealthChop(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_global_config_.enable_spread_cycle = 1;
//    TMC2209_writeStoredGlobalConfig();
//}
//
//uint32_t TMC2209_getInterstepDuration(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    return TMC2209_read(ADDRESS_TSTEP);
//}
//
//void TMC2209_setCoolStepDurationThreshold(TMC2209_t *tmc2209,uint32_t duration_threshold) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    TMC2209_write(ADDRESS_TCOOLTHRS, duration_threshold);
//}
//
//void TMC2209_setStealthChopDurationThreshold(TMC2209_t *tmc2209,uint32_t duration_threshold) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    TMC2209_write(ADDRESS_TPWMTHRS, duration_threshold);
//}
//
//uint16_t TMC2209_getStallGuardResult(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    return TMC2209_read(ADDRESS_SG_RESULT);
//}
//
//void TMC2209_setStallGuardThreshold(TMC2209_t *tmc2209,uint8_t stall_guard_threshold) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    TMC2209_write(ADDRESS_SGTHRS, stall_guard_threshold);
//}
//
//uint8_t TMC2209_getPwmScaleSum(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    TMC2209_PwmScale_t pwm_scale;
//    pwm_scale.bytes = TMC2209_read(ADDRESS_PWM_SCALE);
//
//    return pwm_scale.pwm_scale_sum;
//}
//
//int16_t TMC2209_getPwmScaleAuto(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    TMC2209_PwmScale_t pwm_scale;
//    pwm_scale.bytes = TMC2209_read(ADDRESS_PWM_SCALE);
//
//    return pwm_scale.pwm_scale_auto;
//}
//
//uint8_t TMC2209_getPwmOffsetAuto(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    TMC2209_PwmAuto_t pwm_auto;
//    pwm_auto.bytes = TMC2209_read(ADDRESS_PWM_AUTO);
//
//    return pwm_auto.pwm_offset_auto;
//}
//
//uint8_t TMC2209_getPwmGradientAuto(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    TMC2209_PwmAuto_t pwm_auto;
//    pwm_auto.bytes = TMC2209_read(ADDRESS_PWM_AUTO);
//
//    return pwm_auto.pwm_gradient_auto;
//}

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

//void TMC2209_disableCoolStep(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_cool_config_.semin = SEMIN_OFF;
//    TMC2209_write(ADDRESS_COOLCONF, tmc2209->TMC2209_cool_config_.bytes);
//    tmc2209->TMC2209_cool_step_enabled_ = false;
//}
//
//void TMC2209_setCoolStepCurrentIncrement(TMC2209_t *tmc2209,TMC2209_CurrentIncrement_t current_increment) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_cool_config_.seup = current_increment;
//    TMC2209_write(ADDRESS_COOLCONF, tmc2209->TMC2209_cool_config_.bytes);
//}
//
//void TMC2209_setCoolStepMeasurementCount(TMC2209_t *tmc2209,TMC2209_MeasurementCount_t measurement_count) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_cool_config_.sedn = measurement_count;
//    TMC2209_write(ADDRESS_COOLCONF, tmc2209->TMC2209_cool_config_.bytes);
//}
//
//uint16_t TMC2209_getMicrostepCounter(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return 0;
//    }
//    return TMC2209_read(ADDRESS_MSCNT);
//}
//
//void TMC2209_enableAnalogCurrentScaling(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_global_config_.i_scale_analog = 1;
//    TMC2209_writeStoredGlobalConfig();
//}
//
//void TMC2209_disableAnalogCurrentScaling(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_global_config_.i_scale_analog = 0;
//    TMC2209_writeStoredGlobalConfig();
//}
//
//void TMC2209_useExternalSenseResistors(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_global_config_.internal_rsense = 0;
//    TMC2209_writeStoredGlobalConfig();
//}
//
//void TMC2209_useInternalSenseResistors(TMC2209_t *tmc2209) {
//    if (tmc2209->TMC2209_blocking_) {
//        return;
//    }
//    tmc2209->TMC2209_global_config_.internal_rsense = 1;
//    TMC2209_writeStoredGlobalConfig();
//}

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
    if (!tmc2209->serial_ptr)
        return;

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
    if (!tmc2209->serial_ptr)
        return;

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
    if (!tmc2209->serial_ptr)
        return 0;

    TMC2209_ReadRequestDatagram_t read_request_datagram;
    read_request_datagram.bytes = 0;
    read_request_datagram.sync = SYNC;
    read_request_datagram.serial_address = tmc2209->serial_address;
    read_request_datagram.register_address = register_address;
    read_request_datagram.rw = RW_READ;
    read_request_datagram.crc = TMC2209_calculateCrcRead(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    TMC2209_sendDatagramRead(tmc2209, read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    uint32_t reply_delay = 0;
    while ((SerialUART_available() < WRITE_READ_REPLY_DATAGRAM_SIZE)
           &&
           (reply_delay < REPLY_DELAY_MAX_MICROSECONDS)) {
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

uint8_t TMC2209_currentSettingToPercent(uint8_t current_setting) {
    uint8_t percent = map(current_setting,
                          CURRENT_SETTING_MIN,
                          CURRENT_SETTING_MAX,
                          PERCENT_MIN,
                          PERCENT_MAX);
    return percent;
}

uint8_t TMC2209_percentToHoldDelaySetting(uint8_t percent) {
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

uint8_t TMC2209_holdDelaySettingToPercent(uint8_t hold_delay_setting) {
    uint8_t percent = map(hold_delay_setting,
                          HOLD_DELAY_MIN,
                          HOLD_DELAY_MAX,
                          PERCENT_MIN,
                          PERCENT_MAX);
    return percent;
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
