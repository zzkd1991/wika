#include "gpio_i2c.h"
#include "ds3232.h"
#include <string.h>

#define DS3232_SLAVE_ADDRESS 0xD0

unsigned int bcd2bin(unsigned char val)
{
	return (val & 0x0f) + (val >> 4) * 10;
}

unsigned char bin2bcd(unsigned val)
{
	return ((val / 10) << 4) + val % 10;
}

void ds3232_reset_flow(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);	
}

int ds3232_write_regs(uint8_t port, uint8_t reg, uint8_t *data, uint16_t data_size)
{
	int i;
	uint8_t slave_addr = DS3232_SLAVE_ADDRESS;
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

int ds3232_read_regs(uint8_t port, uint8_t reg, uint8_t *data, uint16_t data_size)
{
	uint8_t slave_addr;
	uint8_t i;

	slave_addr = DS3232_SLAVE_ADDRESS;
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

static int i2c_ds3232_read(uint8_t reg, uint8_t *value)
{
	int ret;
	ret = ds3232_read_regs(3, reg, value, 1);
	return ret;
}

static int i2c_ds3232_write(uint8_t reg, uint8_t* value)
{
	int ret;
	ret = ds3232_write_regs(3, reg, value, 1);
	return ret;
}

static int i2c_bulk_read(uint8_t reg, uint8_t *val, int len)
{
	int i;
	int ret;
	uint8_t ret_value;
	for(i = 0; i < len; i++)
	{
		ret = i2c_ds3232_read(reg, &ret_value);
		if(ret == 1)
			return 1;
		val[i] = ret_value;
		reg++;
		HAL_Delay(10);
	}

	return 0;
}

static int i2c_bulk_write(uint8_t reg, uint8_t *value, int len)
{
	int i;
	int ret;

	for(i = 0; i < len; i++)
	{
		ret = i2c_ds3232_write(reg, value + i);
		if(ret == 1)
			return ret;
		reg++;
	}

	return 0;
}

int ds3232_check_rtc_status(void)
{
	int ret;
	uint8_t control, stat;

	ret = i2c_ds3232_read(DS3232_REG_SR, &stat);
	if(ret)
		return ret;

	if(stat & DS3232_REG_SR_OSF)
	{
		DS3232_DEBUG("oscillator discontinuity flagged, time unreliable\n");
	}

	stat &= ~(DS3232_REG_SR_OSF | DS3232_REG_SR_A1F | DS3232_REG_SR_A2F);

	ret = i2c_ds3232_write(DS3232_REG_SR, &stat);
	if(ret)
		return ret;

	ret = i2c_ds3232_read(DS3232_REG_CR, &control);
	if(ret)
		return ret;

	control &= ~(DS3232_REG_CR_A1IE | DS3232_REG_CR_A2IE);
	control |= DS3232_REG_CR_INTCN;

	return i2c_ds3232_write(DS3232_REG_CR, &control);
	
}

int ds3232_read_time(struct rtc_time *time)
{
	int ret;
	uint8_t buf[7];
	unsigned int year, month, day, hour, minute, second;
	unsigned int week, twelve_hr, am_pm;
	unsigned int century, add_century = 0;
	ret = i2c_bulk_read(DS3232_REG_SECONDS, buf, 7);
	if(ret)
		return ret;
	
	second = buf[0];
	minute = buf[1];
	hour = buf[2];
	week = buf[3];
	day = buf[4];
	month = buf[5];
	year = buf[6];

	twelve_hr = hour & 0x40;
	am_pm = hour & 0x20;
	century = month & 0x80;

	time->tm_sec = bcd2bin(second);
	time->tm_min = bcd2bin(minute);
	if(twelve_hr)
	{
		//convert to 24 hr
		if(am_pm)
			time->tm_hour = bcd2bin(hour & 0x1F) + 12;
		else
			time->tm_hour = bcd2bin(hour & 0x1F);
	}
	else
	{
		time->tm_hour = bcd2bin(hour);
	}

	//Day of the week in linux range is 0~6 while 1~7 in RTC chip
	time->tm_wday = bcd2bin(week)  -1;
	time->tm_mday = bcd2bin(day);
	//linux tm_mon range:0~11,while month range is 1~12 in RTC chip
	time->tm_mon = bcd2bin(month & 0x7F) - 1;
	if(century)
		add_century = 100;

	time->tm_year = bcd2bin(year) + add_century;

	return 0;
}


int ds3232_set_time(struct rtc_time *time)
{
	uint8_t buf[7];

	//extract time from rtc_time and load into ds3232

	buf[0] = bin2bcd(time->tm_sec);
	buf[1] = bin2bcd(time->tm_min);
	buf[2] = bin2bcd(time->tm_hour);
	//Day of the week in linux range is 0~6 while 1~7 in RTC chip
	buf[3] = bin2bcd(time->tm_wday + 1);
	buf[4] = bin2bcd(time->tm_mday); //date
	//Linux tm_mon range:0~11, while month range is 1~12 in RTC chip
	buf[5] = bin2bcd(time->tm_mon + 1);
	if(time->tm_year >= 100)
	{
		buf[5] |= 0x80;
		buf[6] = bin2bcd(time->tm_year - 100);
	}
	else
	{
		buf[6] = bin2bcd(time->tm_year);
	}

	return i2c_bulk_write(DS3232_REG_SECONDS, buf, 7);
}

static int ds3232_read_alarm(struct rtc_wkalrm *alarm)
{
	uint8_t control, stat;
	int ret;
	uint8_t buf[4];

	ret = i2c_ds3232_read(DS3232_REG_SR, &stat);
	if(ret)
		goto out;
	ret = i2c_ds3232_read(DS3232_REG_CR, &control);
	if(ret)
		goto out;
	ret = i2c_bulk_read(DS3232_REG_ALARM1, buf, 4);
	if(ret)
		goto out;

	alarm->time.tm_sec = bcd2bin(buf[0] & 0x7F);
	alarm->time.tm_min = bcd2bin(buf[1] & 0x7F);
	alarm->time.tm_hour = bcd2bin(buf[2] & 0x7F);
	alarm->time.tm_mday = bcd2bin(buf[3] & 0x7F);

	alarm->enabled = !!(control & DS3232_REG_CR_A1IE);
	alarm->pending = !!(stat & DS3232_REG_SR_A1F);

	ret = 0;
out:
	return ret;
}

static int ds3232_set_alarm(struct rtc_wkalrm *alarm)
{
	uint8_t control, stat;
	int ret;
	uint8_t buf[4];

	buf[0] = bin2bcd(alarm->time.tm_sec);
	buf[1] = bin2bcd(alarm->time.tm_min);
	buf[2] = bin2bcd(alarm->time.tm_hour);
	buf[3] = bin2bcd(alarm->time.tm_mday);

	//clear alarm interrupt enable bit
	ret = i2c_ds3232_read(DS3232_REG_CR, &control);
	if(ret)
		goto out;

	control &= ~(DS3232_REG_CR_A1IE | DS3232_REG_CR_A2IE);
	ret = i2c_ds3232_write(DS3232_REG_CR, &control);
	if(ret)
		goto out;

	//clear any pending alarm flag
	ret = i2c_ds3232_read(DS3232_REG_SR, &stat);
	if(ret)
		goto out;

	stat &= ~(DS3232_REG_SR_A1F | DS3232_REG_SR_A2F);
	ret = i2c_ds3232_write(DS3232_REG_SR, &stat);
	if(ret)
		goto out;

	ret = i2c_bulk_write(DS3232_REG_ALARM1, buf, 4);
	if(ret)
		goto out;

	if(alarm->enabled)
	{
		control |= DS3232_REG_CR_A1IE;
		ret = i2c_ds3232_write(DS3232_REG_CR, &control);
	}

out:
	return ret;
}

static int ds3232_update_alarm(unsigned int enabled)
{
	uint8_t control;
	int ret;

	ret = i2c_ds3232_read(DS3232_REG_CR, &control);
	if(ret)
		return ret;

	if(enabled)
		//enable alarm1 interrupt
		control |= DS3232_REG_CR_A1IE;
	else
		//disable alarm1 interrupt
		control &= ~(DS3232_REG_CR_A1IE);

	ret = i2c_ds3232_write(DS3232_REG_CR, &control);

	return ret;
}

static int ds3232_hwmon_read_temp(long int *mC)
{
	uint8_t temp_buf[2];
	short temp;
	int ret;

	ret = i2c_bulk_read(DS3232_REG_TEMPERATURE, temp_buf, sizeof(temp_buf));

	if(ret == 1)
		return ret;

	temp = (temp_buf[0] << 8) | temp_buf[1];
	temp >>= 6;
	*mC = temp * 250;

	return 0;
}

static int ds3232_hwmon_read(long *temp)
{
	int err;
	
	err = ds3232_hwmon_read_temp(temp);

	return err;
}

static int ds3232_alarm_irq_enable(unsigned int enabled)
{
	return ds3232_update_alarm(enabled);
}

static int ds3232_irq(void)
{
	int ret;
	uint8_t stat, control;

	ret = i2c_ds3232_read(DS3232_REG_SR, &stat);
	if(ret)
		return 1;

	if(stat & DS3232_REG_SR_A1F)
	{
		ret = i2c_ds3232_read(DS3232_REG_CR, &control);
		if(ret)
		{
			DS3232_DEBUG("Read Control Register error %d\n", ret);
		}
		else
		{
			//disable alarm1 interrupt
			control &= ~(DS3232_REG_CR_A1IE);
			ret = i2c_ds3232_write(DS3232_REG_CR, &control);
			if(ret)
			{
				DS3232_DEBUG("Write Control Register error %d\n", ret);
				return 1;
			}

			//clear the alarm pend flag
			stat &= ~DS3232_REG_SR_A2F;
			ret = i2c_ds3232_write(DS3232_REG_SR, &stat);
			if(ret)
			{
				DS3232_DEBUG("Write Status Register error %d\n", ret);
				return 1;
			}

		//	rtc_update_irq(1, RTC_AF );
		}
	}

	return 0;
}


int ds3232_probe(void)
{
	int ret;

	ret = ds3232_check_rtc_status();

	return ret;

}


