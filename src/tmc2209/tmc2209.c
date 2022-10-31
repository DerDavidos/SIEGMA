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




//TODO read()
//getStatus()
//getMicrostepsPerStep()
//setMicrostepsPerStepPowerOfTwo()
//setMictostepsPerStep
//disabledByInputPin()
//disable()
//enable()
//isCommunicatingButNotSetUp()
//isSetupAndCommunicating()
//isCommunicating()
//setup()
//init()



Settings TMC2209_getSettings(TMC2209 *self)
{
    Settings settings;
    settings.is_communicating = isCommunicating();

    if (settings.is_communicating)
    {
        TMC2209_readAndStoreRegisters(self);

        settings.is_setup = self->global_config_.pdn_disable;
        settings.enabled = (self->chopper_config_.toff > TOFF_DISABLE);
        settings.microsteps_per_step = getMicrostepsPerStep();
        settings.inverse_motor_direction_enabled = self->global_config_.shaft;
        settings.stealth_chop_enabled = ! self->global_config_.enable_spread_cycle;
        settings.standstill_mode = self->pwm_config_.freewheel;
        settings.irun_percent = TMC2209_currentSettingToPercent(self->driver_current_.irun);
        settings.irun_register_value = self->driver_current_.irun;
        settings.ihold_percent = TMC2209_currentSettingToPercent(self->driver_current_.ihold);
        settings.ihold_register_value = self->driver_current_.ihold;
        settings.iholddelay_percent = TMC2209_holdDelaySettingToPercent(self->driver_current_.iholddelay);
        settings.iholddelay_register_value = self->driver_current_.iholddelay;
        settings.automatic_current_scaling_enabled = self->pwm_config_.pwm_autoscale;
        settings.automatic_gradient_adaptation_enabled = self->pwm_config_.pwm_autograd;
        settings.pwm_offset = self->pwm_config_.pwm_offset;
        settings.pwm_gradient = self->pwm_config_.pwm_grad;
        settings.cool_step_enabled = self->cool_step_enabled_;
        settings.analog_current_scaling_enabled = self->global_config_.i_scale_analog;
        settings.internal_sense_resistors_enabled = self->global_config_.internal_rsense;
    }
    else
    {
        settings.is_setup = false;
        settings.enabled = false;
        settings.microsteps_per_step = 0;
        settings.inverse_motor_direction_enabled = false;
        settings.stealth_chop_enabled = false;
        settings.standstill_mode = self->pwm_config_.freewheel;
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

void TMC2209_setAllCurrentValues(uint8_t run_current_percent,
                                  uint8_t hold_current_percent,
                                  uint8_t hold_delay_percent,
                                  TMC2209 *self)
{
    if (blocking_)
    {
        return;
    }
    uint8_t run_current = TMC2209_percentToCurrentSetting(run_current_percent);
    uint8_t hold_current = TMC2209_percentToCurrentSetting(hold_current_percent);
    uint8_t hold_delay = TMC2209_percentToHoldDelaySetting(hold_delay_percent);

    self->driver_current_.irun = run_current;
    self->driver_current_.ihold = hold_current;
    self->driver_current_.iholddelay = hold_delay;
    TMC2209_writeStoredDriverCurrent();
}

void TMC2209_setHoldCurrent(uint8_t percent, TMC2209 *self)
{
    if (blocking_)
    {
        return;
    }
    uint8_t hold_current = TMC2209_percentToCurrentSetting(percent);

    self->driver_current_.ihold = hold_current;
    TMC2209_writeStoredDriverCurrent();
}

void TMC2209_setHoldDelay(uint8_t percent, TMC2209 *self)
{
    if (blocking_)
    {
        return;
    }
    uint8_t hold_delay = TMC2209_percentToHoldDelaySetting(percent);

    self->driver_current_.iholddelay = hold_delay;
    TMC2209_writeStoredDriverCurrent();
}

void TMC2209_setRunCurrent(uint8_t percent, TMC2209 *self)
{
    if (blocking_)
    {
        return;
    }
    uint8_t run_current = TMC2209_percentToCurrentSetting(percent);
    self->driver_current_.irun = run_current;
    TMC2209_writeStoredDriverCurrent();
}




void TMC2209_minimizeMotorCurrent(TMC2209 *self){
    self->driver_current_.irun = CURRENT_SETTING_MIN;
    self->driver_current_.ihold = CURRENT_SETTING_MIN;
    TMC2209_writeStoredDriverCurrent();
}

void TMC2209_readAndStoreRegisters(TMC2209 *self){
    self->global_config_.bytes = TMC2209_readGlobalConfigBytes();
    self->chopper_config_.bytes = TMC2209_readChopperConfigBytes();
    self->pwm_config_.bytes = TMC2209_readPwmConfigBytes();
}

void TMC2209_setRegistersToDefaults(TMC2209 *self)
{
    self->driver_current_.bytes = 0;
    self->driver_current_.ihold = IHOLD_DEFAULT;
    self->driver_current_.irun = IRUN_DEFAULT;
    self->driver_current_.iholddelay = IHOLDDELAY_DEFAULT;
    TMC2209_write(ADDRESS_IHOLD_IRUN, self->driver_current_.bytes, self);

    self->chopper_config_.bytes = CHOPPER_CONFIG_DEFAULT;
    self->chopper_config_.tbl = TBL_DEFAULT;
    self->chopper_config_.hend = HEND_DEFAULT;
    self->chopper_config_.hstart = HSTART_DEFAULT;
    self->chopper_config_.toff = TOFF_DEFAULT;
    TMC2209_write(ADDRESS_CHOPCONF,self->chopper_config_.bytes, self);

    self->pwm_config_.bytes = PWM_CONFIG_DEFAULT;
    TMC2209_write(ADDRESS_PWMCONF, self->pwm_config_.bytes, self);

    self->cool_config_.bytes = COOLCONF_DEFAULT;
    TMC2209_write(ADDRESS_COOLCONF,self->cool_config_.bytes, self);

    Tmc2209_write(ADDRESS_TPOWERDOWN,TPOWERDOWN_DEFAULT);
    Tmc2209_write(ADDRESS_TPWMTHRS,TPWMTHRS_DEFAULT);
    Tmc2209_write(ADDRESS_VACTUAL,VACTUAL_DEFAULT);
    Tmc2209_write(ADDRESS_TCOOLTHRS,TCOOLTHRS_DEFAULT);
    Tmc2209_write(ADDRESS_SGTHRS,SGTHRS_DEFAULT);
    Tmc2209_write(ADDRESS_COOLCONF,COOLCONF_DEFAULT);
}

void TMC2209_useExternalSenseResistors(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.internal_rsense = 0;
    TMC2209_writeStoredGlobalConfig(self);
}

void TMC2209_useInternalSenseResistors(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.internal_rsense = 1;
    TMC2209_writeStoredGlobalConfig(self);
}

void TMC2209_enableAnalogCurrentScaling(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.i_scale_analog = 1;
    Tmc2209_writeStoredGlobalConfig(self);
}

void TMC2209_disableAnalogCurrentScaling(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.i_scale_analog = 0;
    Tmc2209_writeStoredGlobalConfig();
}

void TMC2209_setCoolStepMeasurementCount(MeasurementCount measurement_count, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->cool_config_.sedn = measurement_count;
    Tmc2209_write(ADDRESS_COOLCONF,self->cool_config_.bytes);
}

uint16_t TMC2209_getMicrostepCounter(){
    if (blocking_)
    {
        return 0;
    }
    return Tmc2209_read(ADDRESS_MSCNT);
}

void TMC2209_disableCoolStep(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->cool_config_.semin = SEMIN_OFF;
    Tmc2209_write(ADDRESS_COOLCONF,self->cool_config_.bytes);
    self->cool_step_enabled_ = false;
}

void TMC2209_setCoolStepCurrentIncrement(CurrentIncrement current_increment, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->cool_config_.seup = current_increment;
    Tmc2209_write(ADDRESS_COOLCONF,self->cool_config_.bytes);
}

void TMC2209_enableCoolStep(uint8_t lower_threshold,
                             uint8_t upper_threshold,
                             TMC2209 *self)
{
    if (blocking_)
    {
        return;
    }
    lower_threshold = constrain(lower_threshold,SEMIN_MIN,SEMIN_MAX);
    self->cool_config_.semin = lower_threshold;
    upper_threshold = constrain(upper_threshold,SEMAX_MIN,SEMAX_MAX);
    self->cool_config_.semax = upper_threshold;
    TMC2209_write(ADDRESS_COOLCONF,self->cool_config_.bytes);
    self->cool_step_enabled_ = true;
}


void TMC2209_setStallGuardThreshold(uint8_t stall_guard_threshold, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    TMC2209_write(ADDRESS_SGTHRS,stall_guard_threshold, self);
}

void TMC2209_setStealthChopDurationThreshold(uint32_t duration_threshold, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    TMC2209_write(ADDRESS_TPWMTHRS,duration_threshold, self);
}

uint16_t TMC2209_getStallGuardResult(){
    if (blocking_)
    {
        return 0;
    }
    return TMC2209_read(ADDRESS_SG_RESULT);
}

uint32_t TMC2209_getInterstepDuration(){
    if (blocking_)
    {
        return 0;
    }
    return TMC2209_read(ADDRESS_TSTEP);
}

void TMC2209_setCoolStepDurationThreshold(uint32_t duration_threshold, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    TMC2209_write(ADDRESS_TCOOLTHRS,duration_threshold, self);
}

void TMC2209_enableStealthChop(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.enable_spread_cycle = 0;
    TMC2209_writeStoredGlobalConfig(self);
}

void TMC2209_disableStealthChop(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.enable_spread_cycle = 1;
    TMC2209_writeStoredGlobalConfig(self);
}

void TMC2209_setPowerDownDelay(uint8_t delay, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    TMC2209_write(ADDRESS_TPOWERDOWN,delay, self);
}

uint8_t TMC2209_getInterfaceTransmissionCounter(){
    if (blocking_)
    {
        return 0;
    }
    return TMC2209_read(ADDRESS_IFCNT);
}

void TMC2209_setPwmOffset(uint8_t pwm_amplitude, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->pwm_config_.pwm_offset = pwm_amplitude;
    TMC2209_writeStoredPwmConfig(self);
}

void TMC2209_setPwmGradient(uint8_t pwm_amplitude, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->pwm_config_.pwm_grad = pwm_amplitude;
    TMC2209_writeStoredPwmConfig(self);
}

void TMC2209_enableAutomaticGradientAdaptation(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
    TMC2209_writeStoredPwmConfig(self);
}

void TMC2209_disableAutomaticGradientAdaptation(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
    TMC2209_writeStoredPwmConfig(self);
}

void TMC2209_enableAutomaticCurrentScaling(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
    TMC2209_writeStoredPwmConfig(self);
}

void TMC2209_disableAutomaticCurrentScaling(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
    TMC2209_writeStoredPwmConfig(self);
}

void TMC2209_disableInverseMotorDirection(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.shaft = 0;
    TMC2209_writeStoredGlobalConfig(self);
}

void TMC2209_setStandstillMode(StandstillMode mode, TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->pwm_config_.freewheel = mode;
    TMC2209_writeStoredPwmConfig(self);
}

void TMC2209_enableInverseMotorDirection(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    self->global_config_.shaft = 1;
    TMC2209_writeStoredGlobalConfig(self);
}




//getPwmOffsetAuto()
//getPwmGradientAuto()
//getPwmScaleAuto()
//getPwmScaleSum()
// getVersion()
// serialOperationMode()
// setOperationModeToSerial



uint8_t TMC2209_percentToCurrentSetting(uint8_t percent){
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

uint8_t TMC2209_currentSettingToPercent(uint8_t current_setting){
    uint8_t percent = map(current_setting,
                          CURRENT_SETTING_MIN,
                          CURRENT_SETTING_MAX,
                          PERCENT_MIN,
                          PERCENT_MAX);
    return percent;
}


void TMC2209_moveUsingStepDirInterface(TMC2209 *self){
    if (blocking_)
    {
        return;
    }
    TMC2209_write(ADDRESS_VACTUAL,VACTUAL_STEP_DIR_INTERFACE, self);
}


void TMC2209_enable(TMC2209 *self){
    if(blocking_){
        return;
    }
    self->chopper_config_.toff = toff_;
    self->TMC2209_writeStoredChopperConfig = TMC2209_writeStoredChopperConfig();
}

uint8_t TMC2209_percentToHoldDelaySetting(uint8_t percent){
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

uint8_t TMC2209_holdDelaySettingToPercent(uint8_t hold_delay_setting){
    uint8_t percent = map(hold_delay_setting,
                          HOLD_DELAY_MIN,
                          HOLD_DELAY_MAX,
                          PERCENT_MIN,
                          PERCENT_MAX);
    return percent;
}

void TMC2209_writeStoredGlobalConfig(TMC2209 *self){
    TMC2209_write(ADDRESS_GCONF, self->global_config_.bytes, self);
}

uint32_t TMC2209_readGlobalConfigBytes(){
    return TMC2209_read(ADDRESS_GCONF);
}


void TMC2209_writeStoredChopperConfig(TMC2209 *self){
    TMC2209_write(ADDRESS_CHOPCONF, self->chopper_config_.bytes, self);
}

uint32_t TMC2209_readChopperConfigBytes(){
    return TMC2209_read(ADDRESS_CHOPCONF);
}

void TMC2209_writeStoredPwmConfig(TMC2209 *self){
    TMC2209_write(ADDRESS_PWMCONF,self->pwm_config_.bytes, self);
}

uint32_t TMC2209_readPwmConfigBytes(){
    return TMC2209_read(ADDRESS_PWMCONF);
}



