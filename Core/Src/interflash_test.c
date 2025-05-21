#include "interflash.h"
#include "interflash_test.h"
#include "led.h"
#include <string.h>

#define READ_WRITE_CNT		104
#define INTER_FLASH_TEST_ADDRESS	0x8020000

uint8_t write_cont[READ_WRITE_CNT] MEM_ALIGNED(8) = {0};
uint8_t read_cont[READ_WRITE_CNT] MEM_ALIGNED(4) = {0};

uint8_t interflash_test(void)
{	
	uint8_t ret;
	ret = erase_flash(INTER_FLASH_TEST_ADDRESS, 0x10000);
	if(ret != 0)
	{
		return 1;
	}
	
	memset(write_cont, 'a', READ_WRITE_CNT);
	ret = flash_write_bytes(write_cont, INTER_FLASH_TEST_ADDRESS, READ_WRITE_CNT);
	if(ret)
		return ret;
	FLASH_If_Read(INTER_FLASH_TEST_ADDRESS, read_cont, READ_WRITE_CNT);

	if(memcmp(write_cont, read_cont, READ_WRITE_CNT) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}



