#ifndef SIEGMA_DISPENSER_H
#define SIEGMA_DISPENSER_H

#include <stdint.h>
#include "SerialUART.h"

#define NUMBER_OF_DISPENSERS 4

void setUpDispenser(uint8_t id, SerialUART_t uart);

void setUpAllDispensers(SerialUART_t uart);

void moveDispenserUp(uint8_t id);

void moveDispenserDown(uint8_t id);

void stopDispenser(uint8_t id);

#endif //SIEGMA_DISPENSER_H
