#ifndef SIEGMA_TMC2209_INTERN_H
#define SIEGMA_TMC2209_INTERN_H

#include "tmc2209.h"

void TMC2209_setOperationModeToSerial(TMC2209_t *tmc2209, SerialUART_t seral, long serial_baud_rate,
                                      SerialAddress_t serial_address);

void TMC2209_setRegistersToDefaults(TMC2209_t *tmc2209);

void TMC2209_readAndStoreRegisters(TMC2209_t *tmc2209);

uint8_t TMC2209_getVersion(TMC2209_t *tmc2209);

bool TMC2209_serialOperationMode(TMC2209_t *tmc2209);

void TMC2209_minimizeMotorCurrent(TMC2209_t *tmc2209);

uint32_t TMC2209_reverseData(uint32_t data);

uint8_t TMC2209_calculateCrcRead(TMC2209_ReadRequestDatagram_t datagram, uint8_t datagram_size);

void TMC2209_sendDatagramRead(TMC2209_t *tmc2209, TMC2209_ReadRequestDatagram_t datagram, uint8_t datagram_size);

uint8_t TMC2209_calculateCrcWrite(TMC2209_WriteReadReplyDatagram_t datagram, uint8_t datagram_size);

void TMC2209_sendDatagramWrite(TMC2209_t *tmc2209, TMC2209_WriteReadReplyDatagram_t datagram, uint8_t datagram_size);

void TMC2209_write(TMC2209_t *tmc2209, uint8_t register_address, uint32_t data);

uint32_t TMC2209_read(TMC2209_t *tmc2209, uint8_t register_address);

uint8_t TMC2209_percentToCurrentSetting(uint8_t percent);

void TMC2209_writeStoredGlobalConfig(TMC2209_t *tmc2209);

uint32_t TMC2209_readGlobalConfigBytes(TMC2209_t *tmc2209);

void TMC2209_writeStoredDriverCurrent(TMC2209_t *tmc2209);

void TMC2209_writeStoredChopperConfig(TMC2209_t *tmc2209);

uint32_t TMC2209_readChopperConfigBytes(TMC2209_t *tmc2209);

void TMC2209_writeStoredPwmConfig(TMC2209_t *tmc2209);

uint32_t TMC2209_readPwmConfigBytes(TMC2209_t *tmc2209);



void TMC2209_disableAutomaticGradientAdaptation(TMC2209_t *tmc2209);

void TMC2209_disableAutomaticCurrentScaling(TMC2209_t *tmc2209);

void TMC2209_enableCoolStep(TMC2209_t *tmc2209, uint8_t lower_threshold,
                            uint8_t upper_threshold);

#endif //SIEGMA_TMC2209_INTERN_H
