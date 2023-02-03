#ifndef SIEGMA_RONDELL_H
#define SIEGMA_RONDELL_H

#endif //SIEGMA_RONDELL_H

#include <stdint.h>
#include "motor.h"

typedef uint16_t LDR_VALUE;

enum RondellPos {
    Pos0 = 0,
    Pos1,
    Pos2,
    Pos3,
    UNDEFINED
};

enum RondellState {
    RONDELL_SLEEP,
    RONDELL_MOVING_COUNTER_CLOCKWISE,
    RONDELL_IN_KEY_POS,
    RONDELL_MOVING_CLOCKWISE
};

typedef struct Rondell Rondell_t;

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

void setUpRondell(SerialAddress_t address, SerialUART_t uart);

void moveToDispenserWithId(enum RondellPos positionToDriveTo);



