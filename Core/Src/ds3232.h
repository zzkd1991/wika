#ifndef _ds3232_h_
#define _ds3232_h_

#include "main.h"
#include <stdio.h>

struct rtc_time{
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
	int tm_yday;
	int tm_isdst;
};

struct rtc_wkalrm {
	unsigned char enabled;
	unsigned char pending;
	struct rtc_time time;
};

#define DS3232_REG_SECONDS      0x00
#define DS3232_REG_MINUTES      0x01
#define DS3232_REG_HOURS        0x02
#define DS3232_REG_AMPM         0x02
#define DS3232_REG_DAY          0x03
#define DS3232_REG_DATE         0x04
#define DS3232_REG_MONTH        0x05
#define DS3232_REG_CENTURY      0x05
#define DS3232_REG_YEAR         0x06
#define DS3232_REG_ALARM1       0x07       /* Alarm 1 BASE */
#define DS3232_REG_ALARM2       0x0B       /* Alarm 2 BASE */
#define DS3232_REG_CR           0x0E       /* Control register */
#       define DS3232_REG_CR_nEOSC   0x80
#       define DS3232_REG_CR_INTCN   0x04
#       define DS3232_REG_CR_A2IE    0x02
#       define DS3232_REG_CR_A1IE    0x01

#define DS3232_REG_SR           0x0F       /* control/status register */
#       define DS3232_REG_SR_OSF     0x80
#       define DS3232_REG_SR_BSY     0x04
#       define DS3232_REG_SR_A2F     0x02
#       define DS3232_REG_SR_A1F     0x01

#define DS3232_REG_TEMPERATURE	0x11
#define DS3232_REG_SRAM_START   0x14
#define DS3232_REG_SRAM_END     0xFF


int ds3232_set_time(struct rtc_time *time);
int ds3232_read_time(struct rtc_time *time);
int ds3232_check_rtc_status(void);
void ds3232_reset_flow(void);
int ds3232_probe(void);

#define DS3232_DEBUG_ON			0

#define DS3232_DEBUG(fmt,arg...)          do{\
                                         if(DS3232_DEBUG_ON)\
                                         printf("<<-DS3232-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)

#endif
