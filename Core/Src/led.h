#ifndef _led_h_
#define _led_h_

#define led_digitalHi(p, i)		{p->BSRR = i;}
#define led_digitalLo(p, i)		{p->BSRR = (uint32_t)i << 16;}
#define led_digitalToggle(p, i)	{p->ODR ^= i};

#define LEDG_GPIO_PORT		GPIOC
#define LEDG_PIN		GPIO_PIN_6

#define LEDR_GPIO_PORT		GPIOC
#define LEDR_PIN		GPIO_PIN_7

#define LEDG_ON		led_digitalHi(LEDG_GPIO_PORT, LEDG_PIN)
#define LEDG_OFF	led_digitalLo(LEDG_GPIO_PORT, LEDG_PIN)

#define LEDR_ON		led_digitalHi(LEDR_GPIO_PORT, LEDR_PIN)
#define LEDR_OFF	led_digitalLo(LEDR_GPIO_PORT, LEDR_PIN)

#endif



