#include "interflash.h"
#include "interflash_test.h"
#include "led.h"

uint8_t interflash_ret1 = 0;
uint8_t interflash_ret2 = 0;
uint8_t interflash_ret3 = 0;

uint8_t write_cont[30] MEM_ALIGNED(8) = "111111111111111";
uint8_t read_cont[20] MEM_ALIGNED(4) = {0};

uint32_t read_addr = 0x8003000;



void interflash_test(void)
{	
	interflash_ret2 = flash_write_bytes(write_cont, 0x8003000, sizeof(write_cont));

	//if(interflash_ret2 == 1)
	//{
	//	LEDG_ON;
	//}
	//HAL_Delay(2000);
	interflash_ret3 = FLASH_If_Read(read_addr, read_cont, sizeof(read_cont));
}



