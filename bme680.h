/*
 * bme680.h

 *
 *  Created on: Jun 21, 2019
 *      Author: RnD_Client
 */

#include "config/config.h"
#include "config/iic.h"
#include "bme680_defs.h"

#ifndef SRC_BME680_H_
#define SRC_BME680_H_

bool BME680_begin(uint8_t addr, bool initSettings);

bool BME680_setTemperatureOversampling(uint8_t oversample);
bool BME680_setHumidityOversampling(uint8_t oversample);
bool BME680_setPressureOversampling(uint8_t oversample);
bool BME680_setIIRFilterSize(uint8_t filtersize);
bool BME680_setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);

float BME680_readTemperature();
float BME680_readPressure();
float BME680_readHumidity();
uint32_t BME680_readGas();
float BME680_readAltitude(float seaLevel);

bool BME680_performReading(void);
bool BME680_endReading(void);
unsigned long BME680_beginReading(void);

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

int8_t bme680_init (struct bme680_dev *dev);
int8_t bme680_soft_reset(struct bme680_dev *dev);

int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme680_dev *dev);
int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme680_dev *dev);

int8_t bme680_set_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev);
int8_t bme680_set_sensor_mode(struct bme680_dev *dev);
int8_t bme680_get_sensor_data(struct bme680_field_data *data, struct bme680_dev *dev);



void bme680_get_profile_dur(uint16_t *duration, const struct bme680_dev *dev);



#endif /* SRC_BME680_H_ */


