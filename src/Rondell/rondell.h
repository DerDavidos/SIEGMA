#ifndef SIEGMA_RONDELL_H
#define SIEGMA_RONDELL_H

#endif //SIEGMA_RONDELL_H

#include <stdint.h>
#include "motor.h"

// A main concept of the rondell is reading out light dependent resistor (=:LDR) values.
typedef uint16_t LDR_VALUE;

// Possible rondell positions
enum RondellPos {
    Pos0 = 0,
    Pos1,
    Pos2,
    Pos3,
    UNDEFINED
};

// possible Rondell states.
enum RondellState {
    RONDELL_SLEEP,
    RONDELL_MOVING_COUNTER_CLOCKWISE,
    RONDELL_IN_KEY_POS,
    RONDELL_MOVING_CLOCKWISE
};

typedef struct Rondell Rondell_t;

// general often needed information of the rondell.
typedef struct Rondell {
    SerialAddress_t address;
    enum RondellState state;
    enum RondellPos position;
    enum RondellPos positionToDriveTo;
    LDR_VALUE max_ldr_value;
    LDR_VALUE min_ldr_value;
    Motor_t motor;
    SerialUART_t uart;
} Rondell_t;

// Sets up the rondell by initializing corresponding struct and setting extreme values of ldr.
// @oaram1 SerialAddress for UART configuration. For further information visit: https://github.com/janelia-arduino/TMC2209
// @param2 Further UART configuration
// @return void
void setUpRondell(SerialAddress_t address, SerialUART_t uart);

// This is the core-function of the rondell; it moves the rondell to the desired dispenser.
// @oaram1 enum RondellPos: Position that should be driven to.
// @return void
void moveToDispenserWithId(enum RondellPos positionToDriveTo);



