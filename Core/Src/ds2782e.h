#ifndef _ds2782e_h_
#define _ds2782e_h_

#include "main.h"
#include <stdio.h>

#define DS2782_REG_RARC	0x06 /*Remaining active relative capacity*/

#define DS278x_REG_VOLT_MSB	0x0c
#define DS278x_REG_TEMP_MSB	0x0a
#define DS278x_REG_CURRENT_MSB	0x0e

/* ICM7100 board battery info */
#define ICM7100_SENSE_RESISTOR	20 //O
#define ICM7100_BATTERY_mWH		29600	//Wh
#define ICM7100_BATTERY_mAH	4000 //mAh
#define ICM7100_BATTERY_mVOLT	7400	//mV
#define ICM7100_BATTERY_VCHG	8400 //7950 //mv
#define ICM7100_BATTERY_IMIN	80 //mA
#define ICM7100_BATTERY_EV	6000 //mV
#define ICM7100_BATTERY_EC	420 //mA
#define ICM7100_DISCHG_IMIN	(-20) //mA


//EEPROM Block
#define DS2782_REG_ACR_MSB	0x10
#define DS2782_REG_ACR_LSB	0x11
#define DS2782_REG_SFR		0x15
#define DS2782_REG_CTRL		0x60
#define DS2782_REG_AB		0x61
#define DS2782_REG_AC_MSB	0x62
#define DS2782_REG_AC_LSB	0x63
#define DS2782_REG_VCHG		0x64
#define DS2782_REG_IMIN		0x65
#define DS2782_REG_VAE		0x66
#define DS2782_REG_IAE		0x67
#define DS2782_REG_AE40		0x68
#define DS2782_REG_RSNSP	0x69
#define DS2782_REG_FULL40_MSB	0x6a
#define DS2782_REG_FULL40_LSB	0x6b
#define DS2782_REG_FULL3040	0x6c
#define DS2782_REG_FULL2030	0x6d
#define DS2782_REG_FULL1020		0x6e
#define DS2782_REG_FULL0010	0x6f
#define DS2782_REG_SE3040	0x74
#define DS2782_REG_SE2030	0x75
#define DS2782_REG_SE1020	0x76
#define DS2782_REG_SE0010	0x77
#define DS2782_REG_RSGAIN_MSB	0x78
#define DS2782_REG_RSGAIN_LSB	0x79
#define DS2782_REG_RSTC			0x7a
#define DS2782_REG_FCR		0xfe


#define DS2782_DEF_SFR		(122)
#define DS2782_DEF_CTRL		0
#define DS2782_DEF_AB		0x05
#define DS2782_DEF_AE40		0x06
#define DS2782_DEF_ACR		(ICM7100_BATTERY_mAH * ICM7100_SENSE_RESISTOR/6.25) //mAh
#define DS2782_DEF_RSNSP	(1000 / ICM7100_SENSE_RESISTOR)
#define DS2782_DEF_FULL3040	0x10
#define DS2782_DEF_FULL2030	0x17
#define DS2782_DEF_FULL1020	0x3c
#define DS2782_DEF_FULL0010	0x49
#define DS2782_DEF_SE3040	0x03
#define DS2782_DEF_SE2030	0x05
#define DS2782_DEF_SE1020	0x08
#define DS2782_DEF_SE0010	0x1b
#define DS2782_DEF_RSGAIN_MSB	0x04
#define DS2782_DEF_RSGAIN_LSB	0x00
#define DS2782_DEF_RSTC		0x00
#define DS2782_FC_COPY		0x44
#define DS2782_FC_RECALL	0xb4
#define DS2782_FC_LOCK	0x66

//current unit measurement in uA for a 1 milli-ohm sense resistor
#define DS2782_CURRENT_UNITS	1563
#define DS2786_REG_RARC		0x02		//Remaining active relateive capacity

#define DS278x_DELAY		1000//ms

struct ds278x_info {
	int capacity;
	int present;
};


void ds278x_cfg_check(void);
int ds278x_get_status(struct ds278x_info *info);
int ds2782_get_capacity(int *capacity);
int ds278x_get_temp(int *temp);



/**/




#endif
