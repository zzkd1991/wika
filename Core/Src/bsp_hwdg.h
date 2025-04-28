#ifndef HIC_WDI_H
#define HIC_WDI_H

#include "main.h"

#define WDI_Pin  GPIO_PIN_2
#define WDI_GPIO GPIOC

uint8_t API_WatchDog_Enable(uint8_t Statue_u8);
uint8_t API_WatchDog_FeedDog(void);

#endif



