/*
 * flash_mem.h
 *
 *  Created on: 17 sept. 2018
 *      Author: j.solerh
 */

#ifndef FLASH_MEM_INC_FLASH_MEM_H_
#define FLASH_MEM_INC_FLASH_MEM_H_

void Flash_Chip_Erase();
void Flash_Read_Addr(int address);
void Flash_Read_Id();
void Flash_Read_Status_Register();
void Flash_Read_Measure(uint32_t address, BNO_measure *m);
void Flash_Write_Measure(uint32_t address, BNO_measure *m);

#endif /* FLASH_MEM_INC_FLASH_MEM_H_ */
