#include "fm25cl64.h"

static uint8_t SPI2_Read_Write(uint8_t data)
{
	uint8_t RxData;
	if(HAL_SPI_TransmitReceive(&hspi2, &data, &RxData, 1, 1000) != HAL_OK)
	{
		;
	}
	return RxData;
}

static void SPIFRAM_SetCS(uint8_t level)
{
	if(!level)
	{
		FRAM_CS_0();
	}
	else
	{
		FRAM_CS_1();
	}
}

void SPIFRAM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = FRAM_PIN_NSS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(FRAM_PORT_NSS, &GPIO_InitStruct);
	SPIFRAM_SetCS(1);
}

static void Delay(uint32_t uiDly)
{
	uint32_t i;

	for(; uiDly > 0; uiDly--)
	{
		for(i = 0; i < 10; i++);
	}
}

static void FM25CL64_WREN(void)
{
	SPIFRAM_SetCS(0);
	SPI2_Read_Write(WREN);
	Delay(1);
	SPIFRAM_SetCS(1);
}

void FM35CL64_WRSR(uint8_t Reg_Status)
{
	FM25CL64_WREN();
	Delay(1);
	SPIFRAM_SetCS(0);
	SPI2_Read_Write(WRSR);
	SPI2_Read_Write(Reg_Status);
	Delay(1);
	SPIFRAM_SetCS(1);
}

uint8_t FM25CL64_RDSR(void)
{
	uint8_t Reg_Status;

	SPIFRAM_SetCS(0);
	SPI2_Read_Write(RDSR);
	Reg_Status = SPI2_Read_Write(0);
	Delay(1);
	SPIFRAM_SetCS(1);

	return Reg_Status;
}

uint8_t SPIFRAM_Read_Byte(uint32_t addr)
{
	uint8_t data;
	SPIFRAM_SetCS(0);
	SPI2_Read_Write(READ);
	SPI2_Read_Write((addr >> 8) & 0xff);
	SPI2_Read_Write(addr & 0xff);
	data = SPI2_Read_Write(DUMY);
	SPIFRAM_SetCS(1);
	return data;
}

void SPIFRAM_Write_Byte(uint8_t data, uint16_t addr)
{
	FM25CL64_WREN();
	SPIFRAM_SetCS(0);
	SPI2_Read_Write(WRITE);
	SPI2_Read_Write((addr >> 8) & 0xff);
	SPI2_Read_Write(addr & 0xff);
	SPI2_Read_Write(data);
	SPIFRAM_SetCS(1);
}

void SPIFRAM_Read_Bytes(uint32_t addr, uint8_t *data, uint32_t len)
{
	uint32_t i;
	for(i = 0; i < len; i++)
	{
		data[i] = SPIFRAM_Read_Byte(addr + i);
	}
}

void SPIFRAM_Write_Bytes(uint32_t addr, uint8_t *data, uint32_t len)
{
	uint32_t i;
	for(i = 0; i < len; i++)
	{
		SPIFRAM_Write_Byte(data[i], addr + i);
	}
}

uint32_t TestFRAM(void)
{
	uint32_t i;
	uint32_t err;
	uint8_t ByteBuf_Write[1024] = {0};
	uint8_t ByteBuf_Read[1024] = {0};

	for(i = 0; i < 1024; i++)
	{
		ByteBuf_Write[i] = i % 255;
	}

	err = 0;
	SPIFRAM_Write_Bytes(0, ByteBuf_Write, 1024);
	SPIFRAM_Read_Bytes(0, ByteBuf_Read, 1024);

	for(i = 0; i < 1024; i++)
	{
		if(ByteBuf_Write[i] != ByteBuf_Read[i])
		{
			err++;
		}
	}

	if(err > 0)
	{
		return err;
	}

	return 0;
}

void framTest(void)
{
	uint8_t buf[200];
	uint8_t buf1[200];
	memset(buf, 8, sizeof(buf));
	memset(buf1, 0, sizeof(buf1));

	SPIFRAM_Write_Bytes(10, buf, 200);
	SPIFRAM_Read_Bytes(10, buf, 200);
	while(1);
}


