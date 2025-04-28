#include "gpio_i2c.h"

//i2c3_sda------>PC1
//i2c3_scl------>PC0

#define IIC3_SCL_GPIO_PORT		GPIOC
#define IIC3_SCL_GPIO_PIN		GPIO_PIN_0
#define IIC3_SCL_GPIO_CLK_ENABLE()		do{__HAL_RCC_GPIOC_CLK_ENABLE();}while(0)

#define IIC3_SDA_GPIO_PORT		GPIOC
#define IIC3_SDA_GPIO_PIN		GPIO_PIN_1
#define IIC3_SDA_GPIO_CLK_ENABLE()		do{__HAL_RCC_GPIOC_CLK_ENABLE();}while(0)


#define I2C_WR	0
#define I2C_RD	1

/*
	i2c2_sda----->PB11
	i2c2_scl----->PB10
*/
#define IIC2_SCL_GPIO_PORT		GPIOB
#define IIC2_SCL_GPIO_PIN		GPIO_PIN_10
#define IIC2_SCL_GPIO_CLK_ENABLE()	do{__HAL_RCC_GPIOB_CLK_ENABLE();}while(0)

#define IIC2_SDA_GPIO_PORT		GPIOB
#define IIC2_SDA_GPIO_PIN		GPIO_PIN_11
#define IIC2_SDA_GPIO_CLK_ENABLE()	do{__HAL_RCC_GPIOB_CLK_ENABLE();}while(0)

//i2c1_sda------>PA10
//i2c1_scl------>PA9
#define IIC1_SCL_GPIO_PORT	GPIOA
#define IIC1_SCL_GPIO_PIN	GPIO_PIN_9
#define IIC1_SCL_GPIO_CLK_ENABLE()	do{__HAL_RCC_GPIOA_CLK_ENABLE();}while(0)

#define IIC1_SDA_GPIO_PORT	GPIOA
#define IIC1_SDA_GPIO_PIN	GPIO_PIN_10
#define IIC1_SDA_GPIO_CLK_ENABLE() do{__HAL_RCC_GPIOA_CLK_ENABLE();}while(0)

void i2c_init(uint8_t port)
{
	GPIO_InitTypeDef gpio_init_struct;
	HAL_GPIO_WritePin(IIC3_SDA_GPIO_PORT, IIC3_SDA_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IIC3_SCL_GPIO_PORT, IIC3_SCL_GPIO_PIN, GPIO_PIN_SET);	
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;	
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		
	if(port == 1)
	{
		/*
		I2C1_SDA------>PA10
		I2C1_SCL------>PA9
		*/
		IIC1_SCL_GPIO_CLK_ENABLE();
		IIC1_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC1_SCL_GPIO_PIN;
		HAL_GPIO_Init(IIC1_SCL_GPIO_PORT, &gpio_init_struct);
		gpio_init_struct.Pin = IIC1_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC1_SDA_GPIO_PORT, &gpio_init_struct);
		i2c_stop(port);		
	}
	else if(port == 2)
	{
		/*
		I2C2_SDA----->PB11
		I2C2_SCL----->PB10
		*/
		IIC2_SCL_GPIO_CLK_ENABLE();
		IIC2_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC2_SCL_GPIO_PIN;
		HAL_GPIO_Init(IIC2_SCL_GPIO_PORT, &gpio_init_struct);
		gpio_init_struct.Pin = IIC2_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC2_SDA_GPIO_PORT, &gpio_init_struct);
		i2c_stop(port);
	}
	else if(port == 3)
	{
		//i2c3_sda----->PC1
		//i2c3_scl----->PC0
		IIC3_SCL_GPIO_CLK_ENABLE();
		IIC3_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC3_SCL_GPIO_PIN;
		HAL_GPIO_Init(IIC3_SCL_GPIO_PORT, &gpio_init_struct);
		gpio_init_struct.Pin = IIC3_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC3_SDA_GPIO_PORT, &gpio_init_struct);

	}
}

void i2c_sda_out(uint8_t port)
{
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_struct.Pull = GPIO_PULLUP;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;	
	if(port == 1)
	{
		IIC1_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC1_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC1_SDA_GPIO_PORT, &gpio_init_struct);	
	}
	else if(port == 2)
	{
		IIC2_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC2_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC2_SDA_GPIO_PORT, &gpio_init_struct);		
	}
	else if(port == 3)
	{
		IIC3_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC3_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC3_SDA_GPIO_PORT, &gpio_init_struct);
	}
}

void i2c_sda_in(uint8_t port)
{		
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.Mode = GPIO_MODE_INPUT;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;	
	if(port == 1)
	{
		IIC1_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC1_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC1_SDA_GPIO_PORT, &gpio_init_struct);		
	}
	else if(port == 2)
	{
		IIC2_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC2_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC2_SDA_GPIO_PORT, &gpio_init_struct);		
	}
	else if(port == 3)
	{
		IIC3_SDA_GPIO_CLK_ENABLE();
		gpio_init_struct.Pin = IIC3_SDA_GPIO_PIN;
		HAL_GPIO_Init(IIC3_SDA_GPIO_PORT, &gpio_init_struct);
	}
}

void i2c_scl_set(uint8_t port, uint8_t value)
{
	if(port == 1)
	{
		if(value == 1)
		{
			HAL_GPIO_WritePin(IIC1_SCL_GPIO_PORT, IIC1_SCL_GPIO_PIN, GPIO_PIN_SET);
		}
		else if(value == 0)
		{
			HAL_GPIO_WritePin(IIC1_SCL_GPIO_PORT, IIC1_SCL_GPIO_PIN, GPIO_PIN_RESET);
		}
	}
	else if(port == 2)
	{
		if(value == 1)
		{
			HAL_GPIO_WritePin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN, GPIO_PIN_SET);
		}
		else if(value == 0)
		{
			HAL_GPIO_WritePin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN, GPIO_PIN_RESET);
		}
	}
	else if(port == 3)
	{
		if(value == 1)
		{
			HAL_GPIO_WritePin(IIC3_SCL_GPIO_PORT, IIC3_SCL_GPIO_PIN, GPIO_PIN_SET);
		}
		else if(value == 0)
		{
			HAL_GPIO_WritePin(IIC3_SCL_GPIO_PORT, IIC3_SCL_GPIO_PIN, GPIO_PIN_RESET);
		}
	}
}

void i2c_sda_set(uint8_t port, uint8_t value)
{
	if(port == 1)
	{
		if(value == 1)
		{
			HAL_GPIO_WritePin(IIC1_SDA_GPIO_PORT, IIC1_SDA_GPIO_PIN, GPIO_PIN_SET);
		}
		else if(value == 0)
		{
			HAL_GPIO_WritePin(IIC1_SDA_GPIO_PORT, IIC1_SDA_GPIO_PIN, GPIO_PIN_RESET);
		}
	}
	else if(port == 2)
	{
		if(value == 1)
		{
			HAL_GPIO_WritePin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN, GPIO_PIN_SET);
		}
		else if(value == 0)
		{
			HAL_GPIO_WritePin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN, GPIO_PIN_RESET);
		}
	}
	else if(port == 3)
	{
		if(value == 1)
		{
			HAL_GPIO_WritePin(IIC3_SDA_GPIO_PORT, IIC3_SDA_GPIO_PIN, GPIO_PIN_SET);
		}
		else if(value == 0)
		{
			HAL_GPIO_WritePin(IIC3_SDA_GPIO_PORT, IIC3_SDA_GPIO_PIN, GPIO_PIN_RESET);
		}
	}
}

uint8_t i2c_scl_read(uint8_t port)
{
	uint8_t ret_value;
	if(port == 1)
	{
		ret_value =  HAL_GPIO_ReadPin(IIC1_SCL_GPIO_PORT, IIC1_SCL_GPIO_PIN);
	}
	else if(port == 2)
	{
		ret_value = HAL_GPIO_ReadPin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN);
	}
	else if(port == 3)
	{
		ret_value = HAL_GPIO_ReadPin(IIC3_SCL_GPIO_PORT, IIC3_SCL_GPIO_PIN);
	}
	
	return ret_value;
}

uint8_t i2c_sda_read(uint8_t port)
{
	uint8_t ret_value;
	if(port == 1)
	{
		ret_value = HAL_GPIO_ReadPin(IIC1_SDA_GPIO_PORT, IIC1_SDA_GPIO_PIN);
	}
	else if(port == 2)
	{
		ret_value =  HAL_GPIO_ReadPin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN);
	}
	else if(port == 3)
	{
		ret_value = HAL_GPIO_ReadPin(IIC3_SDA_GPIO_PORT, IIC3_SDA_GPIO_PIN);
	}
	
	return ret_value;
}

static void i2c_delay(void)
{
	uint32_t i;
	
	//for (i = 0; i < 10000; i++);
	for (i = 0; i < 60; i++);
}

static void i2c_delay_cnt(uint32_t u32cnt)
{
    while(u32cnt--) ;
}

void i2c_start(uint8_t port)
{
	i2c_sda_set(port, 1);
	i2c_scl_set(port, 1);
	i2c_delay();
	i2c_sda_set(port, 0);
	i2c_delay();
	
	i2c_scl_set(port, 0);
	i2c_delay();
}

void i2c_stop(uint8_t port)
{
	i2c_sda_set(port, 0);
	i2c_scl_set(port, 1);
	i2c_delay();
	i2c_sda_set(port, 1);
	i2c_delay();
}

void i2c_sendbyte(uint8_t port, uint8_t _ucByte)
{
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		if(_ucByte & 0x80)
		{
			i2c_sda_set(port,1);
		}
		else
		{
			i2c_sda_set(port, 0);
		}
		i2c_delay();
		i2c_scl_set(port, 1);
		i2c_delay();
		i2c_scl_set(port, 0);
		if(i == 7)
		{
			i2c_sda_set(port, 1);
		}
		_ucByte <<= 1;
		i2c_delay();
	}
}

uint8_t i2c_readbyte(uint8_t port)
{
	uint8_t i;
	uint8_t value;
	value = 0;
	for(i = 0; i < 8; i++)
	{
		value <<= 1;
		i2c_scl_set(port, 1);
		i2c_delay();
		if(i2c_sda_read(port))
		{
			value++;
		}
		i2c_scl_set(port, 0);
		i2c_delay();
	}
	
	return value;
}

uint8_t i2c_waitack(uint8_t port)
{
	uint8_t ret;
	i2c_sda_set(port, 1);
	i2c_delay();
	i2c_scl_set(port, 1);
	i2c_delay();
	if(i2c_sda_read(port))
	{
			ret = 1;
	}
	else
	{
			ret = 0;
	}
	i2c_scl_set(port, 0);
	i2c_delay();
	return ret;
}

void i2c_ack(uint8_t port)
{
	i2c_sda_set(port, 0);
	i2c_delay();
	i2c_scl_set(port, 1);
	i2c_delay();
	i2c_scl_set(port, 0);
	i2c_delay();
	i2c_sda_set(port, 1);
}

void i2c_nack(uint8_t port)
{
	i2c_sda_set(port, 1);
	i2c_delay();
	i2c_scl_set(port, 1);
	i2c_delay();
	i2c_scl_set(port, 0);
	i2c_delay();
}

static void i2c_gpio_mode_init(uint8_t port)
{
	i2c_scl_set(port, 0);
	i2c_sda_set(port, 1);
}

static void i2c_recover_send_pulse(uint8_t port)
{
	uint8_t cnt;
	cnt = 0;
	while((!i2c_sda_read(port)) && (cnt <= 9))
	{
		cnt++;
		i2c_scl_set(port, 1);
		i2c_delay_cnt(8000);
		i2c_scl_set(port, 0);
		i2c_delay_cnt(8000);
	}
}

uint8_t i2c_powerup_recover(uint8_t port)
{
	uint8_t ret;

	i2c_gpio_mode_init(port);
	i2c_recover_send_pulse(port);

	ret = i2c_sda_read(port);

	return ret;
}

