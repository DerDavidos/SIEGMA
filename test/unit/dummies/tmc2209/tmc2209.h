// ----------------------------------------------------------------------------
// Adapted from: https://github.com/peterpolidoro/TMC2209
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC2209_H
#define TMC2209_H

#include "serialUART.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum SerialAddress {
    SERIAL_ADDRESS_0 = 0,
    SERIAL_ADDRESS_1 = 1,
    SERIAL_ADDRESS_2 = 2,
    SERIAL_ADDRESS_3 = 3,
} SerialAddress_t;

typedef struct TMC2209 {
    uint8_t serial_address;
} TMC2209_t;

uint64_t CURRENT_VELOCITY;

void TMC2209_setup(TMC2209_t *tmc2209, SerialUART_t serial, long serial_baud_rate, SerialAddress_t serial_address);

bool TMC2209_isSetupAndCommunicating(TMC2209_t *tmc2209);

void TMC2209_enable(TMC2209_t *tmc2209);

bool TMC2209_disabledByInputPin(TMC2209_t *tmc2209);

void TMC2209_setRunCurrent(TMC2209_t *tmc2209, uint8_t percent);

void TMC2209_moveAtVelocity(TMC2209_t *tmc2209, int32_t microsteps_per_period);

#endif
