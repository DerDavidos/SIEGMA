//
// Created by Baran Demir on 09.01.23.
//

#ifndef SIEGMA_RONDELL_H
#define SIEGMA_RONDELL_H

#endif //SIEGMA_RONDELL_H

#include <stdint.h>
#include "motor.h"

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
    Motor_t motor;
    SerialUART_t uart;
} Rondell_t;

Rondell_t createRondell(SerialAddress_t address, SerialUART_t uart);

void setUpRondell(SerialAddress_t address, SerialUART_t uart);

void moveRondellCounterClockwise(void);

void moveRondellClockwise(void);

void stopRondell(void);

void startRondell(void);

void findLongHoleAndPassIt(void);

void identifyPosition(void);

int8_t moveRondellToKeyPosition(void);

// Probably not needed
void ResetRondell(void);

// The only function really needed ?
void moveToDispenserWithId(enum RondellPos positionToDriveTo);

uint8_t getRondellPosition(void);



