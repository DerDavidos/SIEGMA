#ifndef SIEGMA_DISPENSER_H
#define SIEGMA_DISPENSER_H

#include <stdint.h>
#include "serialUART.h"
#include "tmc2209.h"

#define NUMBER_OF_DISPENSERS 4
#define TIME_DISPENSERS_ARE_MOVING_UP 5000

typedef enum dispenserDirection_t {
    UP,
    DOWN,
    STOP
} dispenserDirection;

typedef struct dispenser_t {
    TMC2209_t *tmc2209;
    dispenserDirection direction;
} Dispenser;

void setUpDispenser(uint8_t id, SerialUART_t uart);

void setUpAllDispensers(SerialUART_t uart);

void moveDispenserUp(uint8_t id);

void moveDispenserDown(uint8_t id);

void stopDispenser(uint8_t id);

dispenserDirection getDispenserDirection(uint8_t id);

#endif //SIEGMA_DISPENSER_H
