// ----------------------------------------------------------------------------
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------


#ifndef TMC2209_H
#define TMC2209_H

#include "SerialUART.h"

typedef enum SerialAddress {
    SERIAL_ADDRESS_0 = 0,
    SERIAL_ADDRESS_1 = 1,
    SERIAL_ADDRESS_2 = 2,
    SERIAL_ADDRESS_3 = 3,
} SerialAddress_t;


typedef struct Settings {
    bool is_communicating;
    bool is_setup;
    bool enabled;
    uint16_t microsteps_per_step;
    bool inverse_motor_direction_enabled;
    bool stealth_chop_enabled;
    uint8_t standstill_mode;
    uint8_t irun_percent;
    uint8_t irun_register_value;
    uint8_t ihold_percent;
    uint8_t ihold_register_value;
    uint8_t iholddelay_percent;
    uint8_t iholddelay_register_value;
    bool automatic_current_scaling_enabled;
    bool automatic_gradient_adaptation_enabled;
    uint8_t pwm_offset;
    uint8_t pwm_gradient;
    bool cool_step_enabled;
    bool analog_current_scaling_enabled;
    bool internal_sense_resistors_enabled;
} TMC2209_Settings_t;



typedef struct Status {
    uint32_t over_temperature_warning: 1;
    uint32_t over_temperature_shutdown: 1;
    uint32_t short_to_ground_a: 1;
    uint32_t short_to_ground_b: 1;
    uint32_t low_side_short_a: 1;
    uint32_t low_side_short_b: 1;
    uint32_t open_load_a: 1;
    uint32_t open_load_b: 1;
    uint32_t over_temperature_120c: 1;
    uint32_t over_temperature_143c: 1;
    uint32_t over_temperature_150c: 1;
    uint32_t over_temperature_157c: 1;
    uint32_t reserved0: 4;
    uint32_t current_scaling: 5;
    uint32_t reserved1: 9;
    uint32_t stealth_chop_mode: 1;
    uint32_t standstill: 1;
} Status_t;
const static uint8_t CURRENT_SCALING_MAX = 31;


typedef enum StandstillMode {
    NORMAL = 0,
    FREEWHEELING = 1,
    STRONG_BRAKING = 2,
    BRAKING = 3,
} TMC2209_StandstillMode_t;


typedef enum CurrentIncrement {
    CURRENT_INCREMENT_1 = 0,
    CURRENT_INCREMENT_2 = 1,
    CURRENT_INCREMENT_4 = 2,
    CURRENT_INCREMENT_8 = 3,
} TMC2209_CurrentIncrement_t;


typedef enum MeasurementCount {
    MEASUREMENT_COUNT_32 = 0,
    MEASUREMENT_COUNT_8 = 1,
    MEASUREMENT_COUNT_2 = 2,
    MEASUREMENT_COUNT_1 = 3,
} TMC2209_MeasurementCount_t;


// Serial Settings
const static uint8_t BYTE_MAX_VALUE = 0xFF;
const static uint8_t BITS_PER_BYTE = 8;

const static uint32_t ECHO_DELAY_INC_MICROSECONDS = 1;
const static uint32_t ECHO_DELAY_MAX_MICROSECONDS = 4000;
const static uint32_t REPLY_DELAY_INC_MICROSECONDS = 1;
const static uint32_t REPLY_DELAY_MAX_MICROSECONDS = 10000;
const static uint32_t POST_READ_DELAY_INC_MICROSECONDS = 10;
const static uint32_t POST_READ_DELAY_NUMERATOR = 500000;

const static uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
const static uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

// Datagrams
const static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
const static uint8_t DATA_SIZE = 4;
typedef union WriteReadReplyDatagram {
    struct {
        uint64_t sync: 4;
        uint64_t reserved: 4;
        uint64_t serial_address: 8;
        uint64_t register_address: 7;
        uint64_t rw: 1;
        uint64_t data: 32;
        uint64_t crc: 8;
    };
    uint64_t bytes;
} TMC2209_WriteReadReplyDatagram_t;

const static uint8_t SYNC = 0b101;
const static uint8_t RW_READ = 0;
const static uint8_t RW_WRITE = 1;
const static uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;

const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
typedef union ReadRequestDatagram {
    struct {
        uint32_t sync: 4;
        uint32_t reserved: 4;
        uint32_t serial_address: 8;
        uint32_t register_address: 7;
        uint32_t rw: 1;
        uint32_t crc: 8;
    };
    uint32_t bytes;
} TMC2209_ReadRequestDatagram_t;

// General Configuration Registers
const static uint8_t ADDRESS_GCONF = 0x00;
typedef union GlobalConfig {
    struct {
        uint32_t i_scale_analog: 1;
        uint32_t internal_rsense: 1;
        uint32_t enable_spread_cycle: 1;
        uint32_t shaft: 1;
        uint32_t index_otpw: 1;
        uint32_t index_step: 1;
        uint32_t pdn_disable: 1;
        uint32_t mstep_reg_select: 1;
        uint32_t multistep_filt: 1;
        uint32_t test_mode: 1;
        uint32_t reserved: 22;
    };
    uint32_t bytes;
} TMC2209_GlobalConfig_t;


const static uint8_t ADDRESS_GSTAT = 0x01;
typedef union GlobalStatus {
    struct {
        uint32_t reset: 1;
        uint32_t drv_err: 1;
        uint32_t uv_cp: 1;
        uint32_t reserved: 29;
    };
    uint32_t bytes;
} TMC2209_GlobalStatus_t;

const static uint8_t ADDRESS_IFCNT = 0x02;

const static uint8_t ADDRESS_SENDDELAY = 0x03;
typedef union SendDelay {
    struct {
        uint32_t reserved_0: 8;
        uint32_t senddelay: 8;
        uint32_t reserved_1: 16;
    };
    uint32_t bytes;
} TMC2209_SendDelay_t;

const static uint8_t ADDRESS_IOIN = 0x06;
typedef union Input {
    struct {
        uint32_t enn: 1;
        uint32_t reserved_0: 1;
        uint32_t ms1: 1;
        uint32_t ms2: 1;
        uint32_t diag: 1;
        uint32_t reserved_1: 1;
        uint32_t pdn_serial: 1;
        uint32_t step: 1;
        uint32_t spread_en: 1;
        uint32_t dir: 1;
        uint32_t reserved_2: 14;
        uint32_t version: 8;
    };
    uint32_t bytes;
} TMC2209_Input_t;
const static uint8_t VERSION = 0x21;


// Velocity Dependent Driver Feature Control Register Set
const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
typedef union DriverCurrent {
    struct {
        uint32_t ihold: 5;
        uint32_t reserved_0: 3;
        uint32_t irun: 5;
        uint32_t reserved_1: 3;
        uint32_t iholddelay: 4;
        uint32_t reserved_2: 12;
    };
    uint32_t bytes;
} TMC2209_DriverCurrent_t;

const static uint8_t PERCENT_MIN = 0;
const static uint8_t PERCENT_MAX = 100;
const static uint8_t CURRENT_SETTING_MIN = 0;
const static uint8_t CURRENT_SETTING_MAX = 31;
const static uint8_t HOLD_DELAY_MIN = 0;
const static uint8_t HOLD_DELAY_MAX = 15;
const static uint8_t IHOLD_DEFAULT = 16;
const static uint8_t IRUN_DEFAULT = 31;
const static uint8_t IHOLDDELAY_DEFAULT = 1;

const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
const static uint8_t TPOWERDOWN_DEFAULT = 20;

const static uint8_t ADDRESS_TSTEP = 0x12;

const static uint8_t ADDRESS_TPWMTHRS = 0x13;
const static uint32_t TPWMTHRS_DEFAULT = 0;

const static uint8_t ADDRESS_VACTUAL = 0x22;
const static int32_t VACTUAL_DEFAULT = 0;
const static int32_t VACTUAL_STEP_DIR_INTERFACE = 0;

// CoolStep and StallGuard Control Register Set
const static uint8_t ADDRESS_TCOOLTHRS = 0x14;
const static uint8_t TCOOLTHRS_DEFAULT = 0;
const static uint8_t ADDRESS_SGTHRS = 0x40;
const static uint8_t SGTHRS_DEFAULT = 0;
const static uint8_t ADDRESS_SG_RESULT = 0x41;

const static uint8_t ADDRESS_COOLCONF = 0x42;
const static uint8_t COOLCONF_DEFAULT = 0;
typedef union CoolConfig {
    struct {
        uint32_t semin: 4;
        uint32_t reserved_0: 1;
        uint32_t seup: 2;
        uint32_t reserved_1: 1;
        uint32_t semax: 4;
        uint32_t reserved_2: 1;
        uint32_t sedn: 2;
        uint32_t seimin: 1;
        uint32_t reserved_3: 16;
    };
    uint32_t bytes;
} TMC2209_CoolConfig_t;

const static uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;
const static uint8_t SEIMIN_LOWER_SETTING = 0;
const static uint8_t SEIMIN_UPPER_SETTING = 1;
const static uint8_t SEMIN_OFF = 0;
const static uint8_t SEMIN_MIN = 1;
const static uint8_t SEMIN_MAX = 15;
const static uint8_t SEMAX_MIN = 0;
const static uint8_t SEMAX_MAX = 15;

// Microstepping Control Register Set
const static uint8_t ADDRESS_MSCNT = 0x6A;
const static uint8_t ADDRESS_MSCURACT = 0x6B;

// Driver Register Set
const static uint8_t ADDRESS_CHOPCONF = 0x6C;
typedef union ChopperConfig {
    struct {
        uint32_t toff: 4;
        uint32_t hstart: 3;
        uint32_t hend: 4;
        uint32_t reserved_0: 4;
        uint32_t tbl: 2;
        uint32_t vsense: 1;
        uint32_t reserved_1: 6;
        uint32_t mres: 4;
        uint32_t interpolation: 1;
        uint32_t double_edge: 1;
        uint32_t diss2g: 1;
        uint32_t diss2vs: 1;
    };
    uint32_t bytes;
} TMC2209_ChopperConfig_t;

const static uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
const static uint8_t TBL_DEFAULT = 0b10;
const static uint8_t HEND_DEFAULT = 0;
const static uint8_t HSTART_DEFAULT = 5;
const static uint8_t TMC2209_TOFF_DEFAULT = 3;
const static uint8_t TOFF_DISABLE = 0;

const static uint8_t MRES_256 = 0b0000;
const static uint8_t MRES_128 = 0b0001;
const static uint8_t MRES_064 = 0b0010;
const static uint8_t MRES_032 = 0b0011;
const static uint8_t MRES_016 = 0b0100;
const static uint8_t MRES_008 = 0b0101;
const static uint8_t MRES_004 = 0b0110;
const static uint8_t MRES_002 = 0b0111;
const static uint8_t MRES_001 = 0b1000;

const static size_t MICROSTEPS_PER_STEP_MIN = 1;
const static size_t MICROSTEPS_PER_STEP_MAX = 256;

const static uint8_t ADDRESS_DRV_STATUS = 0x6F;
typedef union DriveStatus {
    struct {
        Status_t status;
    };
    uint32_t bytes;
} TMC2209_DriveStatus_t;

const static uint8_t ADDRESS_PWMCONF = 0x70;
typedef union PwmConfig {
    struct {
        uint32_t pwm_offset: 8;
        uint32_t pwm_grad: 8;
        uint32_t pwm_freq: 2;
        uint32_t pwm_autoscale: 1;
        uint32_t pwm_autograd: 1;
        uint32_t freewheel: 2;
        uint32_t reserved: 2;
        uint32_t pwm_reg: 4;
        uint32_t pwm_lim: 4;
    };
    uint32_t bytes;
} TMC2209_PwmConfig_t;

const static uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
const static uint8_t PWM_OFFSET_MIN = 0;
const static uint8_t PWM_OFFSET_MAX = 255;
const static uint8_t PWM_OFFSET_DEFAULT = 0x24;
const static uint8_t PWM_GRAD_MIN = 0;
const static uint8_t PWM_GRAD_MAX = 255;
const static uint8_t PWM_GRAD_DEFAULT = 0x14;

typedef union PwmScale {
    struct {
        uint32_t pwm_scale_sum: 8;
        uint32_t reserved_0: 8;
        uint32_t pwm_scale_auto: 9;
        uint32_t reserved_1: 7;
    };
    uint32_t bytes;
} TMC2209_PwmScale_t;
const static uint8_t ADDRESS_PWM_SCALE = 0x71;

typedef union PwmAuto {
    struct {
        uint32_t pwm_offset_auto: 8;
        uint32_t reserved_0: 8;
        uint32_t pwm_gradient_auto: 8;
        uint32_t reserved_1: 8;
    };
    uint32_t bytes;
} TMC2209_PwmAuto_t;
const static uint8_t ADDRESS_PWM_AUTO = 0x72;

typedef struct TMC2209 {
    bool TMC2209_blocking_;
    SerialUART_t *TMC2209_serial_ptr_;
    uint32_t TMC2209_serial_baud_rate_;
    uint8_t TMC2209_serial_address_;

    TMC2209_DriverCurrent_t TMC2209_driver_current_;
    TMC2209_CoolConfig_t TMC2209_cool_config_;
    bool TMC2209_cool_step_enabled_;
    TMC2209_PwmConfig_t TMC2209_pwm_config_;
    uint8_t toff_;
    TMC2209_ChopperConfig_t TMC2209_chopper_config_;
    TMC2209_GlobalConfig_t TMC2209_global_config_;
} TMC2209_t;


void TMC2209_setOperationModeToSerial(TMC2209_t *tmc2209, SerialUART_t seral, long serial_baud_rate, SerialAddress_t serial_address);

void TMC2209_setRegistersToDefaults(TMC2209_t *tmc2209);

void TMC2209_readAndStoreRegisters(TMC2209_t *tmc2209);

uint8_t TMC2209_getVersion(TMC2209_t *tmc2209);

bool TMC2209_serialOperationMode(TMC2209_t *tmc2209);

void TMC2209_minimizeMotorCurrent(TMC2209_t *tmc2209);

uint32_t TMC2209_reverseData(uint32_t data);

uint8_t TMC2209_calculateCrcRead(TMC2209_ReadRequestDatagram_t datagram, uint8_t datagram_size);

void TMC2209_sendDatagramRead(TMC2209_t *tmc2209, TMC2209_ReadRequestDatagram_t datagram, uint8_t datagram_size);

uint8_t TMC2209_calculateCrcWrite(TMC2209_WriteReadReplyDatagram_t datagram, uint8_t datagram_size);

void TMC2209_sendDatagramWrite(TMC2209_t *tmc2209,TMC2209_WriteReadReplyDatagram_t datagram, uint8_t datagram_size);

void TMC2209_write(TMC2209_t *tmc2209,uint8_t register_address, uint32_t data);

uint32_t TMC2209_read(TMC2209_t *tmc2209,uint8_t register_address);

uint8_t TMC2209_percentToCurrentSetting(uint8_t percent);

uint8_t TMC2209_currentSettingToPercent(uint8_t current_setting);

uint8_t TMC2209_percentToHoldDelaySetting(uint8_t percent);

uint8_t TMC2209_holdDelaySettingToPercent(uint8_t hold_delay_setting);

uint8_t TMC2209_pwmAmplitudeToPwmAmpl(TMC2209_t *tmc2209,uint8_t pwm_amplitude);

uint8_t TMC2209_pwmAmplitudeToPwmGrad(TMC2209_t *tmc2209,uint8_t pwm_amplitude);

void TMC2209_writeStoredGlobalConfig(TMC2209_t *tmc2209);

uint32_t TMC2209_readGlobalConfigBytes(TMC2209_t *tmc2209);

void TMC2209_writeStoredDriverCurrent(TMC2209_t *tmc2209);

void TMC2209_writeStoredChopperConfig(TMC2209_t *tmc2209);

uint32_t TMC2209_readChopperConfigBytes(TMC2209_t *tmc2209);

void TMC2209_writeStoredPwmConfig(TMC2209_t *tmc2209);

uint32_t TMC2209_readPwmConfigBytes(TMC2209_t *tmc2209);

TMC2209_Settings_t TMC2209_getSettings(TMC2209_t *tmc2209);

// identify which microcontroller serial port is connected to the TMC2209
// e.g. Serial1, Serial2...
// optionally identify which serial address is assigned to the TMC2209 if not
// the default of SERIAL_ADDRESS_0
void TMC2209_setup(TMC2209_t *tmc2209, SerialUART_t serial, long serial_baud_rate, SerialAddress_t serial_address);

// check to make sure TMC2209 is properly setup and communicating
bool TMC2209_isSetupAndCommunicating(TMC2209_t *tmc2209);

// if driver is not communicating, check power and communication connections
bool TMC2209_isCommunicating(TMC2209_t *tmc2209);

// driver may be communicating but not setup if driver power is lost then
// restored after setup so that defaults are loaded instead of setup options
bool isCommunicatingButNotSetup(TMC2209_t *tmc2209);

// driver must be enabled before use it is disabled by default
void TMC2209_enable(TMC2209_t *tmc2209);

void TMC2209_disable(TMC2209_t *tmc2209);

// driver may also be disabled by the hardware enable input pin
// this pin must be grounded or disconnected before driver may be enabled
bool TMC2209_disabledByInputPin(TMC2209_t *tmc2209);

// valid values = 1,2,4,8,...128,256, other values get rounded down
void TMC2209_setMicrostepsPerStep(TMC2209_t *tmc2209,uint16_t microsteps_per_step);

// valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
// https://en.wikipedia.org/wiki/Power_of_two
void TMC2209_setMicrostepsPerStepPowerOfTwo(TMC2209_t *tmc2209,uint8_t exponent);

uint16_t TMC2209_getMicrostepsPerStep(TMC2209_t *tmc2209);

void TMC2209_setRunCurrent(TMC2209_t *tmc2209,uint8_t percent);

void TMC2209_setHoldCurrent(TMC2209_t *tmc2209,uint8_t percent);

void TMC2209_setHoldDelay(TMC2209_t *tmc2209,uint8_t percent);

void TMC2209_setAllCurrentValues(TMC2209_t *tmc2209,uint8_t run_current_percent, uint8_t hold_current_percent, uint8_t hold_delay_percent);

Status_t TMC2209_getStatus(TMC2209_t *tmc2209);

void TMC2209_enableInverseMotorDirection(TMC2209_t *tmc2209);

void TMC2209_disableInverseMotorDirection(TMC2209_t *tmc2209);

void TMC2209_setStandstillMode(TMC2209_t *tmc2209, TMC2209_StandstillMode_t mode);

void TMC2209_enableAutomaticCurrentScaling(TMC2209_t *tmc2209);

void TMC2209_disableAutomaticCurrentScaling(TMC2209_t *tmc2209);

void TMC2209_enableAutomaticGradientAdaptation(TMC2209_t *tmc2209);

void TMC2209_disableAutomaticGradientAdaptation(TMC2209_t *tmc2209);

void TMC2209_setPwmOffset(uint8_t pwm_amplitude);

void TMC2209_setPwmGradient(uint8_t pwm_amplitude);

// default = 20
// minimum of 2 for StealthChop auto-tuning
void TMC2209_setPowerDownDelay(uint8_t delay);

uint8_t TMC2209_getInterfaceTransmissionCounter();

void TMC2209_moveAtVelocity(TMC2209_t *tmc2209, int32_t microsteps_per_period);

void TMC2209_moveUsingStepDirInterface();

void TMC2209_enableStealthChop();

void TMC2209_disableStealthChop();

uint32_t TMC2209_getInterstepDuration();

void TMC2209_setStealthChopDurationThreshold(uint32_t duration_threshold);

uint16_t TMC2209_getStallGuardResult();

void TMC2209_setStallGuardThreshold(uint8_t stall_guard_threshold);

uint8_t TMC2209_getPwmScaleSum();

int16_t TMC2209_getPwmScaleAuto();

uint8_t TMC2209_getPwmOffsetAuto();

uint8_t TMC2209_getPwmGradientAuto();

// lower_threshold: min = 1, max = 15
// upper_threshold: min = 0, max = 15, 0-2 recommended
void TMC2209_enableCoolStep(TMC2209_t *tmc2209, uint8_t lower_threshold, uint8_t upper_threshold);

void TMC2209_disableCoolStep();

void TMC2209_setCoolStepCurrentIncrement(enum CurrentIncrement current_increment);

void TMC2209_setCoolStepMeasurementCount(enum MeasurementCount measurement_count);

void TMC2209_setCoolStepDurationThreshold(uint32_t duration_threshold);

uint16_t TMC2209_getMicrostepCounter();

void TMC2209_enableAnalogCurrentScaling();

void TMC2209_disableAnalogCurrentScaling();

void TMC2209_useExternalSenseResistors();

void TMC2209_useInternalSenseResistors();

#endif
