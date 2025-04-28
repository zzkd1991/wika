#include "interflash.h"
#include <string.h>

uint8_t flash_write_bytes(uint8_t *buff, uint32_t addr, uint32_t size)
{
	uint32_t ki;
	HAL_FLASH_Unlock();
	for(ki = 0; ki < size /8; ki++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, *(uint64_t*)buff) != HAL_OK)
		{
			return 1;
		}
		addr = addr + 8;
		buff += 8;
	}
	HAL_FLASH_Lock();
	
	return 0;
}

static uint32_t GetBank(uint32_t Addr)
{
	return FLASH_BANK_1;
}

static uint32_t GetPage(uint32_t Addr)
{
	uint32_t page = 0;
	
	if(Addr < (FLASH_BASE + FLASH_BANK_SIZE))//FLASH_BANK_SIZE = 0x40000
	{
		//Bank 1
		page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
	}
	else
	{
		//Bank 2
		page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
	}
	
	return page;
}

uint8_t erase_flash(uint32_t flash_start_addr, uint32_t length)
{
	uint32_t flashdestination;
	uint32_t filesize;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t Address = 0, PAGEError = 0;

	filesize = length;
	if(flash_start_addr + filesize >= 0x8040000)
	{
		return 1;
	}

	flashdestination = flash_start_addr;

	HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = GetBank(flash_start_addr);
	EraseInitStruct.Page = GetPage(flash_start_addr);
	EraseInitStruct.NbPages = GetPage(flash_start_addr + filesize) - EraseInitStruct.Page + 1;

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		return 2;
	}

	return 0;
}

int FLASH_If_Read(__IO uint32_t FlashAddr, uint8_t *buf, uint32_t DataLength)
{
	while(DataLength > 0)
	{
		//memcpy(buf, (uint32_t *)FlashAddr, 4);
		
		*(uint32_t *)buf = *(__IO uint32_t*)FlashAddr;
		FlashAddr += 4;
		DataLength -= 4;
		buf += 4;
	}

	
	return 0;
}

void FLASH_If_Init(void)
{
	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
							FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
}



