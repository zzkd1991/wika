#ifndef _inter_flash_h_
#define _inter_flash_h_

#include "main.h"
							
uint8_t flash_write_bytes(uint8_t *buff, uint32_t addr, uint32_t size);

uint8_t erase_flash(uint32_t flash_start_addr, uint32_t length);

int FLASH_If_Read(__IO uint32_t FlashAddr, uint8_t *buf, uint32_t DataLength);

void FLASH_If_Init(void);


#endif



