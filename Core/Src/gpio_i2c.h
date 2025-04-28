#ifndef _user_i2c_h_
#define _user_i2c_h_

#include "main.h"
							
void i2c_init(uint8_t port);
void i2c_start(uint8_t port);
void i2c_stop(uint8_t port);
void i2c_sendbyte(uint8_t port, uint8_t _ucByte);
void i2c_send9bit(uint8_t port, unsigned short value);
uint8_t i2c_readbyte(uint8_t port);
uint8_t i2c_waitack(uint8_t port);
void i2c_ack(uint8_t port);
void i2c_nack(uint8_t port);


#endif



