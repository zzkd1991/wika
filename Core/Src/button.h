#ifndef _button_h_
#define _button_h_

#include "main.h"

#define KEY_GPIO_PORT		GPIOC
#define KEY_PIN		GPIO_PIN_13

typedef struct key_scan_state_t {
	uint32_t key_curr_tick;
	uint8_t key_scan_tick_flag;
	uint8_t soc_onoff_state;
	uint8_t key_scan_flag;
	uint8_t onoff_action_flag;
}key_scan_state;

uint8_t Key_Scan(void);
extern key_scan_state key_state;

#endif



