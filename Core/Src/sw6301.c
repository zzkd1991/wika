#include "gpio_i2c.h"
#include "sw6301.h"
#include <string.h>

#define SW6301_SLAVE_ADDRESS		0x78

force_control global_force_control = {0};
mode_set mode_set_value = {0};


static int sw6301_read_regs(uint8_t port, int reg, uint8_t *data, uint16_t data_size)
{
	uint8_t slave_addr;
	uint8_t i;

	slave_addr = SW6301_SLAVE_ADDRESS;
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

static int sw6301_write_regs(uint8_t port, uint16_t reg, uint8_t *data, uint16_t data_size)
{
	int i;
	uint8_t slave_addr = SW6301_SLAVE_ADDRESS;
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

static int sw6301_unlock(void)
{
	int ret;
	uint8_t write_value;
	write_value = 0x20;
	ret = i2c_sw6301_write(0x24, &write_value);
	if(ret)
		return ret;
	write_value = 0x40;
	ret = i2c_sw6301_write(0x24, &write_value);
	if(ret)
		return ret;
	write_value = 0x80;
	ret = i2c_sw6301_write(0x24, &write_value);
	if(ret)
		return ret;
	
	return 0;
}

static int sw6301_lock(void)
{
	int ret;
	uint8_t data = 0;

	ret = sw6301_write_regs(2, 0x24, &data, 1);
	return ret;
}

int sw6301_switch_high_reg(void)
{
	uint8_t write_value;
	int ret;

	write_value = 0x81;
	ret = i2c_sw6301_write(0x24, &write_value);
	return ret;
}

int sw6301_switch_low_reg(void)
{
	uint8_t write_value = 0;
	int ret;

	ret = i2c_sw6301_write(0xff, &write_value);
	return ret;
}

int sw6301_write(uint16_t reg, uint8_t *value)
{
	uint8_t real_reg;
	int ret;

	sw6301_unlock();

	if(reg >= 0x100 && reg <= 0x156)
	{
		ret = sw6301_switch_high_reg();
		if(ret)
			goto lock_flow;
		real_reg = reg - 0x100;
		ret = i2c_sw6301_write(0x24, &real_reg);
		if(ret)
			goto lock_flow;
	}
	else if(reg < 0x100)
	{
		ret = sw6301_switch_low_reg();
		if(ret)
			goto lock_flow;
		ret = i2c_sw6301_write(reg, value);
		if(ret)
			goto lock_flow;
	}

lock_flow:
	sw6301_lock();
	return ret;
}

static int adc_config_func(adc_config adc_config_value)
{
	int ret;
	ret = sw6301_write(0x30, (uint8_t *)&adc_config_value);
	return ret;
}


static int i2c_sw6301_read(uint8_t reg, uint8_t *value)
{
	int ret;
	ret = sw6301_read_regs(2, reg, value, 1);
	return ret;
}

int i2c_sw6301_write(uint8_t reg, uint8_t* value)
{
	int ret;
	ret = sw6301_write_regs(2, reg, value, 1);
	return ret;
}

int get_chip_version(chip_version *version)
{
	int ret;
	ret = i2c_sw6301_read(0x01, (uint8_t *)version);
	return ret;
}

int get_system_status(system_status* sys_state)
{
	int ret;
	ret = i2c_sw6301_read(0x18, (uint8_t *)sys_state);
	return ret;
}

int get_mode_status(mode_status *mode_state)
{
	int ret;
	ret = i2c_sw6301_read(0x12, (uint8_t *)mode_state);
	return ret;
}

int get_c_no_load_status(c_no_load_status *load_status)
{
	int ret;
	ret = i2c_sw6301_read(0x13, (uint8_t *)load_status);
	return ret;
}

static int mode_set_func(mode_set mode_set_value)
{
	int ret;
	ret = sw6301_write(0x28, (uint8_t *)&mode_set_value);
	return ret;
}

int event_int_enable_func(event_int_enable int_value)
{
	int ret;
	ret = sw6301_write(0x25, (uint8_t *)&int_value);
	return ret;
}

int low_power_func(uint8_t low_power)
{
	int ret;
	low_power_enable set_value;
	set_value.low_power = low_power;
	ret = sw6301_write(0x23, (uint8_t *)&set_value);
	return ret;
}

int get_typec_status_func(typec_status *typec_status)
{
	int ret;
	ret = i2c_sw6301_read(0x19, (uint8_t *)typec_status);
	return ret;
}

int discharge_config0_func(discharge_config0 config_value)
{
	int ret;
	ret = sw6301_write(0x100, (uint8_t *)&config_value);
	return ret;
}

int charge_config0_func(charge_config0 config_value)
{
	int ret;
	ret = sw6301_write(0x107, (uint8_t *)&config_value);
	return ret;
}

int force_control_func(force_control force_value)
{
	int ret;
	ret = sw6301_write(0x40, (uint8_t *)&force_value);
	return ret;
}

int set_charge_ibus_curr_limit_value(float ibus_value)
{
	uint8_t reg_value;
	int ret;
	reg_value = (uint8_t)(ibus_value / 50);

	ret = sw6301_write(0x49, &reg_value);
	if(ret != 0)
		return ret;

	global_force_control.force_ibus_current = 1;
	ret = sw6301_write(0x40, (uint8_t *)&global_force_control);

	return ret;
}

int set_discharge_ibus_curr_limit_value(float ibus_value)
{
	uint8_t reg_value;
	int ret;
	reg_value = (uint8_t)(ibus_value / 50);

	ret = sw6301_write(0x43, &reg_value);
	if(ret != 0)
		return ret;

	global_force_control.force_ibus_current = 1;
	ret = sw6301_write(0x40, (uint8_t *)&global_force_control);

	return ret;

}

int set_charge_ibat_current_limit_value(float ibat_value)
{
	uint8_t reg_value;
	int ret;
	reg_value = (uint8_t)(ibat_value / 100);

	ret = sw6301_write(0x4A, &reg_value);
	if(ret != 0)
		return ret;

	global_force_control.force_ibat_current = 1;
	ret = sw6301_write(0x40, (uint8_t *)&global_force_control);

	return ret;
}

int set_discharge_ibat_current_limit_value(float ibat_value)
{
	uint8_t reg_value;
	int ret;
	reg_value = (uint8_t)(ibat_value / 100);

	ret = sw6301_write(0x44, &reg_value);
	if(ret != 0)
		return ret;

	global_force_control.force_ibat_current = 1;
	ret = sw6301_write(0x40, (uint8_t *)&global_force_control);

	return ret;
}

int set_vbus_vol_limit_value(float vbus_value)
{
	uint8_t reg_low;
	uint8_t reg_high;
	uint16_t reg_value;
	int ret;

	reg_value = (uint16_t)(vbus_value / 10);

	reg_low = reg_value & 0xff;

	ret = sw6301_write(0x41, &reg_low);
	if(ret != 0)
		return ret;

	reg_high = (reg_value >> 8) & 0x0f;
	ret = sw6301_write(0x42, &reg_high);
	if(ret != 0)
		return ret;

	global_force_control.force_contr_vbus_vol = 1;
	ret = sw6301_write(0x40, (uint8_t *)&global_force_control);

	return ret;
}


#if 0
int set_charge_goal_vol(float value)//3.3-26该值不用设置，硬件电阻设置好了
{
	uint16_t real_value;
	uint8_t lower_value;
	uint8_t higher_value;
	charge_goal_vol_lower goal_lower;	
	charge_goal_vol_higher goal_higher;
	int ret;
	

	real_value = (uint16_t)(value * 100);
	lower_value = (real_value & 0xff);
	higher_value = (real_value >> 8);

	goal_lower.charge_vol_lower = lower_value;
	goal_higher.charge_vol_higher = higher_value;
	ret = sw6301_write(0x46, (uint8_t *)&goal_lower);
	if(ret)
		return ret;
	ret = sw6301_write(0x47, (uint8_t *)&goal_higher);
	return ret;
}
#endif

int sw6301_read_adc(uint8_t channel, float *value)
{
	adc_config adc_config_value;
	int ret;
	uint8_t low_value;
	uint8_t high_value;

	adc_config_value.adc_choice = channel;
	ret = adc_config_func(adc_config_value);
	if(ret != 0)
		return ret;
	
	ret = i2c_sw6301_read(0x30, (uint8_t *)&adc_config_value);
	if(ret != 0)
		return ret;

	ret = i2c_sw6301_read(0x31, (uint8_t *)&low_value);
	if(ret != 0)
		return ret;

	ret = i2c_sw6301_read(0x32, (uint8_t *)&high_value);
	if(ret != 0)
		return ret;

	if(adc_config_value.adc_choice == (uint8_t)SW6301_ADC_CHANNEL_VBUS)
	{
		*value = (low_value | ((high_value & 0x0f) << 8)) * 8;
	}
	else if(adc_config_value.adc_choice == (uint8_t)SW6301_ADC_CHANNEL_IBUS)
	{
		*value = (((high_value & 0x0f) << 8) | low_value) * 4;
	}
	else if(adc_config_value.adc_choice == (uint8_t)SW6301_ADC_CHANNEL_VBAT)
	{
		*value = (((high_value & 0x0f) << 8) | low_value) * 7;
	}
	else if(adc_config_value.adc_choice == (uint8_t)SW6301_ADC_CHANNEL_IBAT)
	{
		*value = (((high_value & 0x0f) << 8) | low_value) * 5;
	}
	else if(adc_config_value.adc_choice == (uint8_t)SW6301_ADC_CHANNEL_TDIE)
	{
		*value = ((((high_value & 0x0f) << 8) | low_value) - 1838) / 6.82;
	}
	else if (adc_config_value.adc_choice == (uint8_t)SW6301_ADC_CHANNEL_VNTC)
  {
     *value = (((high_value & 0x0f) << 8) | low_value) * 1.1;
  }

	return ret;
}

int charge_switch_func(uint8_t charge_switch)
{
	int ret;
	mode_set_value.charge_switch = charge_switch;
	ret = mode_set_func(mode_set_value);
	return ret;
}

int quick_charge_indication(quick_charge_sign *value)
{
	int ret;
	ret = i2c_sw6301_read(0x0F, (uint8_t *)value);
	return ret;	
}

int quick_charge_switch(quick_charge_conf value)
{
	int ret;
	ret = sw6301_write(0x11F, (uint8_t *)&value);
	return ret;
}


