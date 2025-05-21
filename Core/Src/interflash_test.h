#ifndef _inter_flash_test_h_
#define _inter_flash_test_h_

#include "main.h"
							
uint8_t flash_write_bytes(uint8_t *buff, uint32_t addr, uint32_t size);

uint8_t erase_flash(uint32_t flash_start_addr, uint32_t length);

uint8_t interflash_test(void);

#endif



