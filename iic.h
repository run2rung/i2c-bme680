/*
 * iic.h
 *
 *  Created on: Nov 23, 2018
 *      Author: piyatap
 */

#ifndef SRC_CONFIG_IIC_H_
#define SRC_CONFIG_IIC_H_

int iic_read(XIic *iicConfig, uint8_t ADDR, ConfigTable* data);
int iic_write(XIic *iicConfig, uint8_t ADDR, ConfigTable* data);
int iic_multi_read(XIic *iicConfig, uint8_t ADDR, uint8_t offset, uint8_t ByteCount, uint8_t* data);
int iic_multi_write(XIic *iicConfig, uint8_t ADDR, uint8_t offset, uint8_t ByteCount, uint8_t* data);

#endif /* SRC_CONFIG_IIC_H_ */
