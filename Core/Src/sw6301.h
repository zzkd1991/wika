#ifndef _sw6301_h_
#define _sw6301_h_

#include "main.h"
#include <stdio.h>

typedef struct
{
	uint8_t verion :4;
	uint8_t reserved :4;
}chip_version;

typedef struct 
{
	uint8_t reserved1 :2;
	uint8_t c_status :1;
	uint8_t reserved2 :1;
	uint8_t discharge_status :1;
	uint8_t charge_status :1;
	uint8_t discharge_abnor :1;
	uint8_t charge_abnor :1;
}system_status;

typedef struct
{
	uint8_t mppt_charge_mode :1;
	uint8_t small_current :1;
	uint8_t reserved :6;
}mode_status;

typedef struct
{
	uint8_t c_no_load :1;
	uint8_t reserved :7;
}c_no_load_status;

typedef struct
{
	uint8_t mppt_switch :1;
	uint8_t small_current_switch :1;
	uint8_t charge_switch :1;
	uint8_t output_switch :1;
	uint8_t wled_switch :1;
	uint8_t reserved :3;
}mode_set;

typedef struct
{
	uint8_t reserved :1;
	uint8_t button_event :1;
	uint8_t discharge_abn_event :1;
	uint8_t charge_abn_event :1;
	uint8_t batt_uvlo_event :1;
	uint8_t ele_mete_complete :1;
	uint8_t reserved2 :2;
}event_int_enable;

typedef struct
{
	uint8_t low_power :1;
	uint8_t reserved :7;
}low_power_enable;

typedef struct
{
	uint8_t reserved1 :4;
	uint8_t led_status :1;
	uint8_t wled_status :1;
	uint8_t reserved2 :2;
}display_status;

typedef struct
{
	uint8_t cc2_status :1;
	uint8_t cc1_status :1;
	uint8_t reserved1 :2;
	uint8_t c_role :2;
	uint8_t reserved :2;
}typec_status;

typedef struct
{
	uint8_t reg_addr :1;
	uint8_t reserved :4;
	uint8_t write_enable_reg :3;
}i2c_enable;

typedef struct
{
	uint8_t output_power :3;
	uint8_t max_power_autodet :1;
	uint8_t charge_con_temp_ring_thre :3;
	uint8_t charge_con_temp_ring :1;
}discharge_config0;

typedef struct
{
	uint8_t input_power :3;
	uint8_t max_power_autodet :1;
	uint8_t const_power_charge :1;
	uint8_t reserved :3;
}charge_config0;

typedef struct
{
	uint8_t force_contr_vbus_vol :1;
	uint8_t force_contr_charge_vol :1;
	uint8_t force_contr_max_input_power :1;
	uint8_t force_contr_charge_hold :1;
	uint8_t reserved :1;
	uint8_t force_ibat_current :1;
	uint8_t force_ibus_current :1;
	uint8_t force_i2c_output_power :1;
}force_control;

typedef struct
{
	uint8_t dischg_ibat_limit :7;
	uint8_t reserved :1;
}output_ibat_value;

typedef struct
{
	uint8_t charge_vol_lower;
}charge_goal_vol_lower;

typedef struct
{
	uint8_t charge_vol_higher;
}charge_goal_vol_higher;

typedef struct 
{
	uint8_t adc_choice :5;
	uint8_t reserved :3;
}adc_config;

enum
{
	SW6301_ADC_CHANNEL_VBUS = 0,
	SW6301_ADC_CHANNEL_IBUS = 1,
	SW6301_ADC_CHANNEL_VBAT = 2,
	SW6301_ADC_CHANNEL_IBAT = 3,
	SW6301_ADC_CHANNEL_TDIE = 9,
	SW6301_ADC_CHANNEL_VNTC = 10,
};

int get_chip_version(chip_version *version);
int i2c_sw6301_read(uint8_t reg, uint8_t *value);
int i2c_sw6301_write(uint8_t reg, uint8_t* value);
int get_system_status(system_status* sys_state);
int get_mode_status(mode_status *mode_state);
int get_c_no_load_status(c_no_load_status *load_status);
int mode_set_func(mode_set mode_set_value);
int event_int_enable_func(event_int_enable int_value);
int display_status_func(display_status* display_value);
int get_typec_status_func(typec_status *typec_status);
void low_power_disable_func(void);
void low_power_enable_func(void);
int sw6301_write(uint16_t reg, uint8_t *value);
void sw6301_write_enable(void);
int discharge_config0_func(discharge_config0 config_value);
int charge_config0_func(charge_config0 config_value);
int force_control_func(force_control force_value);
int set_charge_goal_vol(float value);//3.3-26
int adc_config_func(adc_config adc_config_value);
int sw6301_read_adc(uint8_t channel, float *value);
int set_charge_ibus_curr_limit_value(float ibus_value);
int set_discharge_ibus_curr_limit_value(float ibus_value);
int set_discharge_ibat_current_limit_value(float ibat_value);
int set_charge_ibat_current_limit_value(float ibat_value);
int set_vbus_vol_limit_value(float vbus_value);
int charge_switch_func(uint8_t charge_switch);

#endif
