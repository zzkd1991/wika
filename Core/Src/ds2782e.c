#include "gpio_i2c.h"
#include "ds2782e.h"
#include <string.h>

int my_current_uA;
int my_voltage_uV;
int mY_capacity;
int my_error = 0;
short my_raw = 0;
uint8_t global_capacity = 0;

#define DS278x_SLAVE_ADDRESS 0x68

static int ds278x_read_regs(uint8_t port, int reg, uint8_t *data, uint16_t data_size)
{
	uint8_t slave_addr;
	uint8_t i;

	slave_addr = DS278x_SLAVE_ADDRESS;
	i2c_start(port);
	i2c_sendbyte(port, slave_addr);

	if(i2c_waitack(port) != 0)
	{
		goto cmd_fail;
	}

	i2c_sendbyte(port, reg);
	if(i2c_waitack(port) != 0)
	{
		goto cmd_fail;
	}

	i2c_start(port);
	slave_addr |= 1;
	i2c_sendbyte(port, slave_addr);

	if(i2c_waitack(port) != 0)
	{
		goto cmd_fail;
	}

	for(i = 0; i < data_size; i++)
	{
		data[i] = i2c_readbyte(port);
		
		if(i != data_size - 1)
		{
			i2c_ack(port);
		}
		else
		{
			i2c_nack(port);
		}
	}
	i2c_stop(port);
	return 0;

cmd_fail:
	i2c_stop(port);
	return 1;	
}

int ds278x_write_regs(uint8_t port, uint8_t reg, uint8_t *data, uint16_t data_size)
{
	int i;
	uint8_t slave_addr = DS278x_SLAVE_ADDRESS;
	i2c_stop(port);
	i2c_start(port);
	i2c_sendbyte(port, slave_addr);
	if(i2c_waitack(port) != 0)
	{
		goto cmd_fail;
	}
	i2c_sendbyte(port, reg);
	if(i2c_waitack(port) != 0)
	{
		goto cmd_fail;
	}

	for(i = 0; i < data_size; i++)
	{
		i2c_sendbyte(port, *(data + i));
		if(i2c_waitack(port) != 0)
		{
			goto cmd_fail;
		}
	}

	i2c_stop(port);
	return 0;
cmd_fail:
	i2c_stop(port);
	return 1;
}


int ds278x_read_reg(int reg, uint8_t *data)
{
	return ds278x_read_regs(1, reg, data, 1);
}

int ds278x_write_reg16(int reg, unsigned short val)
{
	uint8_t write_buf[2];
	write_buf[0] = val & 0xff;
	write_buf[1] = (val & 0xff00) >> 8;
	return ds278x_write_regs(1, reg, write_buf, 2);
}

int ds278x_read_reg16(int reg, short *val)
{
	int ret;
	uint8_t ret_data[2];
	ret = ds278x_read_regs(1, reg, ret_data, 2);
	if(ret == 1)
		return ret;
	*val = ret_data[0] << 8 | ret_data[1];
	return 0;
}

int ds278x_write_reg(int reg, uint8_t *data)
{
	return ds278x_write_regs(1, reg, data, 1);
}

typedef struct ds278x_def_val_t {
	int fixed;
	int reg;
	uint32_t val;
}ds278x_def_val_t;

ds278x_def_val_t ds2782_ini[] = {
	{1, DS2782_REG_CTRL,	DS2782_DEF_CTRL		},
	{1, DS2782_REG_AB,		DS2782_DEF_AB		},
	{1,	DS2782_REG_AE40,	DS2782_DEF_AE40		},
	{1, DS2782_REG_RSNSP,	DS2782_DEF_RSNSP	},
	{1, DS2782_REG_FULL40_MSB,	((uint16_t)DS2782_DEF_ACR >> 8) & 0xff},
	{1, DS2782_REG_FULL40_LSB, (uint16_t)DS2782_DEF_ACR & 0xff },
	{1, DS2782_REG_FULL3040,	DS2782_DEF_FULL3040},
	{1, DS2782_REG_FULL2030,	DS2782_DEF_FULL2030},
	{1, DS2782_REG_FULL1020,	DS2782_DEF_FULL1020},
	{1, DS2782_REG_FULL0010,	DS2782_DEF_FULL0010},
	{1, DS2782_REG_SE3040,		DS2782_DEF_SE3040},
	{1, DS2782_REG_SE2030,		DS2782_DEF_SE2030},
	{1, DS2782_REG_SE1020,		DS2782_DEF_SE1020},
	{1, DS2782_REG_SE0010,		DS2782_DEF_SE0010},
	{1, DS2782_REG_RSGAIN_MSB,	DS2782_DEF_RSGAIN_MSB},
	{1, DS2782_REG_RSGAIN_LSB,  DS2782_DEF_RSGAIN_LSB},
	{1, DS2782_REG_RSTC,		DS2782_DEF_RSTC},


	{1, DS2782_REG_AC_MSB,		((uint16_t)DS2782_DEF_ACR >> 8) & 0xff},
	{1, DS2782_REG_AC_LSB,		(uint16_t)DS2782_DEF_ACR & 0xff},
	{0, DS2782_REG_ACR_MSB,		((uint16_t)(DS2782_DEF_ACR * 0.75) >> 8) & 0xff},
	{0, DS2782_REG_ACR_LSB,		(uint16_t)(DS2782_DEF_ACR * 0.75) & 0xff},
	{1, DS2782_REG_VCHG,		ICM7100_BATTERY_VCHG / 2 * 100 / 1952},
	{1, DS2782_REG_IMIN,		ICM7100_BATTERY_IMIN * ICM7100_SENSE_RESISTOR / 50},
	{1, DS2782_REG_VAE,			ICM7100_BATTERY_EV / 2 * 100 / 1952},
	{1, DS2782_REG_IAE,			ICM7100_BATTERY_EC * ICM7100_SENSE_RESISTOR / 200},
	{0, DS2782_REG_SFR,			DS2782_DEF_SFR},
};

void ds278x_cfg_check(void)
{
	static int s_inited = 0;
	int i, err, flag = 0, commit = 0;
	uint8_t val = 0;
	uint8_t temp;

	if(s_inited)
		return;

	for(i = 0; i < sizeof(ds2782_ini) / sizeof(ds2782_ini[0]); i++)
	{
		err = ds278x_read_reg(ds2782_ini[i].reg, &val);
		if(err)
		{
			flag |= err;
			continue;
		}

		if((ds2782_ini[i].fixed && ds2782_ini[i].val != val) || (!ds2782_ini[i].fixed && ds2782_ini[i].val == 0))
		{
			err = ds278x_write_reg(ds2782_ini[i].reg, (uint8_t *)&ds2782_ini[i].val);
			HAL_Delay(8);
			commit = 1;
		}
	}

	//write to EEPROM
	if(commit)
	{
		temp = DS2782_FC_COPY;
		ds278x_write_reg(DS2782_REG_FCR, &temp);
	}
	if(!flag)
		s_inited = 1;
}

int ds278x_get_temp(int *temp)
{
	short raw;
	int err;

	err = ds278x_read_reg16(DS278x_REG_TEMP_MSB, &raw);
	if(err)
		return err;

	*temp = ((raw / 32) * 125) / 100;
	return 0;
}

static int ds2782_get_current(int *current_uA)
{
	int sense_res;
	int err;
	uint8_t sense_res_raw;
	short raw;

	err = ds278x_read_reg(DS2782_REG_RSNSP, &sense_res_raw);
	if(err)
		return err;


	if(sense_res_raw == 0)
	{
		return -1;
	}

	sense_res = 1000 / sense_res_raw;

	err = ds278x_read_reg16(DS278x_REG_CURRENT_MSB, &raw);
	if(err)
		return err;

	my_raw = raw;
	*current_uA = raw * (DS2782_CURRENT_UNITS / sense_res);
	return 0;
}

static int ds2782_get_voltage(int *voltage_uV)
{
	short raw;
	int err;

	err = ds278x_read_reg16(DS278x_REG_VOLT_MSB, &raw);
	if(err)
		return err;

	*voltage_uV = (raw / 32) * 4880;
	return 0;
}

static int ds2782_estimate_capacity(int *capacity)
{
	int uV = 0, uA = 0, val = 0;
	int err;
	uint8_t temp;

	err = ds2782_get_voltage(&uV);
	if(err)
		return err;

	if(uV <= 3600000)
	{
		*capacity = (uV - 3000000) * 7 / (3600000 - 3000000);
	}
	else if(uV < 4200000)
	{
		*capacity = 7 + (uV - 3600000) * 92 / (4200000 - 3600000);
	}
	else 
	{
		*capacity = 99;
	}

	err = ds2782_get_current(&uA);
	if(err == 0 && uA > ICM7100_BATTERY_IMIN / 2 / 2 * 1000)
	{
		val = DS2782_DEF_ACR;
		val = val * (*capacity) / 100;
		temp = ((uint16_t)val >> 8) & 0xff;
		ds278x_write_reg(DS2782_REG_ACR_MSB, &temp);
		temp = (uint16_t)val & 0xff;
		ds278x_write_reg(DS2782_REG_ACR_LSB, &temp);
	}

	return 0;
}



int ds2782_get_capacity(int *capacity)
{
	int err;
	uint8_t raw;

	err = ds278x_read_reg(DS2782_REG_RARC, &raw);
	//my_error = err;
	if(err || raw == 0)
	{
		err = ds2782_estimate_capacity(capacity);
		if(err)
		{
			return err;
		}
		return 0;
	}

	global_capacity = raw;
	*capacity = raw;
	return 0;
}

int ds278x_get_status(struct ds278x_info *info)
{
	int err;
	int current_uA;
	int voltage_uV;
	int capacity;

	ds278x_cfg_check();

	err = ds2782_get_voltage(&voltage_uV);
	if(err)
		return err;

	err = ds2782_get_current(&current_uA);
	if(err)
		return err;

	err = ds2782_get_capacity(&capacity);
	if(err)
		return err;

	info->capacity = capacity;
	info->present = voltage_uV >= (ICM7100_BATTERY_EV / 2 / 2* 1000);

	my_voltage_uV = voltage_uV;
	my_current_uA = current_uA;
	if(my_current_uA != 0)
		return 0;
	return 0;
}





