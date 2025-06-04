#ifndef __ECATUTILS_H__
#define __ECATUTILS_H__

#include "osal.h"

int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value);
int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
int drive_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value);
void ecat_delay_ms(uint16_t nms);

#endif /*__ECATUTILS_H__*/