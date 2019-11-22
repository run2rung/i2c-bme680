/*
 * bme680e.c
 *
 *  Created on: Jun 21, 2019
 *      Author: RnD_Client
 */

#include "bme680.h"
#include "math.h"

bool _filterEnabled, _tempEnabled, _humEnabled, _presEnabled, _gasEnabled;
uint8_t _i2caddr;
int32_t _sensorID;

unsigned long _meas_start;
uint16_t _meas_period;

uint8_t spixfer(uint8_t x);

struct bme680_dev gas_sensor;

static int8_t null_ptr_check(const struct bme680_dev *dev);
static void delay_msec(uint32_t ms);

static int8_t get_mem_page(struct bme680_dev *dev);
static int8_t set_mem_page(uint8_t reg_addr, struct bme680_dev *dev);

static int8_t get_calib_data(struct bme680_dev *dev);
static int8_t set_gas_config(struct bme680_dev *dev);
static uint8_t calc_heater_res(uint16_t temp, const struct bme680_dev *dev);
static uint8_t calc_heater_dur(uint16_t dur);
static int8_t read_field_data(struct bme680_field_data *data, struct bme680_dev *dev);
static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme680_dev *dev);
static int16_t calc_temperature(uint32_t temp_adc, struct bme680_dev *dev);
static uint32_t calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev);
static uint32_t calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev);
static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev);

/** Temperature (Celsius) assigned after calling performReading() or endReading() **/
float temperature;
/** Pressure (Pascals) assigned after calling performReading() or endReading() **/
uint32_t pressure;
/** Humidity (RH %) assigned after calling performReading() or endReading() **/
float humidity;
/** Gas resistor (ohms) assigned after calling performReading() or endReading() **/
uint32_t gas_resistance;

bool BME680_begin(uint8_t addr, bool initSettings)
{
  _i2caddr = addr;

  gas_sensor.dev_id = addr;
  gas_sensor.intf = BME680_I2C_INTF;
  gas_sensor.read = &i2c_read;
  gas_sensor.write = &i2c_write;

  gas_sensor.delay_ms = delay_msec;

  int8_t rslt = bme680_init(&gas_sensor);

  if (rslt != BME680_OK)
    return false;

  if (initSettings)
  {
	  BME680_setTemperatureOversampling(BME680_OS_8X);
	  BME680_setHumidityOversampling(BME680_OS_2X);
	  BME680_setPressureOversampling(BME680_OS_4X);
	  BME680_setIIRFilterSize(BME680_FILTER_SIZE_3);
	  BME680_setGasHeater(320, 150); // 320*C for 150 ms
  }
  else
  {
	  BME680_setGasHeater(0, 0);
  }
  // don't do anything till we request a reading
  gas_sensor.power_mode = BME680_FORCED_MODE;

  return true;
}

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	iic_multi_read(&iic, dev_id, reg_addr, len, reg_data);

	return 0;
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  iic_multi_write(&iic, dev_id, reg_addr, len, reg_data);

  return 0;
}

static void delay_msec(uint32_t ms)
{
  usleep(ms * 1000);
}

int8_t bme680_init(struct bme680_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK)
	{
		/* Soft reset to restore it to default values*/
		rslt = bme680_soft_reset(dev);
		if (rslt == BME680_OK)
		{
			rslt = bme680_get_regs(BME680_CHIP_ID_ADDR, &dev->chip_id, 1, dev);
			if (rslt == BME680_OK)
			{
				if (dev->chip_id == BME680_CHIP_ID)
				{
					/* Get the Calibration data */
					rslt = get_calib_data(dev);
				}
				else
				{
					rslt = BME680_E_DEV_NOT_FOUND;
				}
			}
		}
	}

	return rslt;
}

static int8_t null_ptr_check(const struct bme680_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
	{
		/* Device structure pointer is not valid */
		rslt = BME680_E_NULL_PTR;
	}
	else
	{
		/* Device structure is fine */
		rslt = BME680_OK;
	}

	return rslt;
}

int8_t bme680_soft_reset(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = BME680_SOFT_RESET_ADDR;
	/* 0xb6 is the soft reset command */
	uint8_t soft_rst_cmd = BME680_SOFT_RESET_CMD;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK) {
		if (dev->intf == BME680_SPI_INTF)
			rslt = get_mem_page(dev);

		/* Reset the device */
		if (rslt == BME680_OK) {
			rslt = bme680_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);
			/* Wait for 5ms */
			dev->delay_ms(BME680_RESET_PERIOD);

			if (rslt == BME680_OK) {
				/* After reset get the memory page */
				if (dev->intf == BME680_SPI_INTF)
					rslt = get_mem_page(dev);
			}
		}
	}

	return rslt;
}

static int8_t get_mem_page(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK)
	{
		dev->com_rslt = dev->read(dev->dev_id, BME680_MEM_PAGE_ADDR | BME680_SPI_RD_MSK, &reg, 1);
		if (dev->com_rslt != 0)
			rslt = BME680_E_COM_FAIL;
		else
			dev->mem_page = reg & BME680_MEM_PAGE_MSK;
	}

	return rslt;
}

int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme680_dev *dev)
{
	int8_t rslt;
	/* Length of the temporary buffer is 2*(length of register)*/
	uint8_t tmp_buff[BME680_TMP_BUFFER_LENGTH] = { 0 };
	uint16_t index;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK)
	{
		if ((len > 0) && (len < BME680_TMP_BUFFER_LENGTH / 2))
		{
			/* Interleave the 2 arrays */
			for (index = 0; index < len; index++)
			{
				if (dev->intf == BME680_SPI_INTF)
				{
					/* Set the memory page */
					rslt = set_mem_page(reg_addr[index], dev);
					tmp_buff[(2 * index)] = reg_addr[index] & BME680_SPI_WR_MSK;
				}
				else
				{
					tmp_buff[(2 * index)] = reg_addr[index];
				}
				tmp_buff[(2 * index) + 1] = reg_data[index];
			}
			/* Write the interleaved array */
			if (rslt == BME680_OK)
			{
				dev->com_rslt = dev->write(dev->dev_id, tmp_buff[0], &tmp_buff[1], (2 * len) - 1);
				if (dev->com_rslt != 0)
					rslt = BME680_E_COM_FAIL;
			}
		}
		else
		{
			rslt = BME680_E_INVALID_LENGTH;
		}
	}

	return rslt;
}

static int8_t set_mem_page(uint8_t reg_addr, struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg;
	uint8_t mem_page;

	/* Check for null pointers in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK) {
		if (reg_addr > 0x7f)
			mem_page = BME680_MEM_PAGE1;
		else
			mem_page = BME680_MEM_PAGE0;

		if (mem_page != dev->mem_page)
		{
			dev->mem_page = mem_page;

			dev->com_rslt = dev->read(dev->dev_id, BME680_MEM_PAGE_ADDR | BME680_SPI_RD_MSK, &reg, 1);
			if (dev->com_rslt != 0)
				rslt = BME680_E_COM_FAIL;

			if (rslt == BME680_OK)
			{
				reg = reg & (~BME680_MEM_PAGE_MSK);
				reg = reg | (dev->mem_page & BME680_MEM_PAGE_MSK);

				dev->com_rslt = dev->write(dev->dev_id, BME680_MEM_PAGE_ADDR & BME680_SPI_WR_MSK,
					&reg, 1);
				if (dev->com_rslt != 0)
					rslt = BME680_E_COM_FAIL;
			}
		}
	}

	return rslt;
}

int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme680_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK)
	{
		if (dev->intf == BME680_SPI_INTF)
		{
			/* Set the memory page */
			rslt = set_mem_page(reg_addr, dev);
			if (rslt == BME680_OK)
				reg_addr = reg_addr | BME680_SPI_RD_MSK;
		}
		dev->com_rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);
		if (dev->com_rslt != 0)
			rslt = BME680_E_COM_FAIL;
	}

	return rslt;
}

static int8_t get_calib_data(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t coeff_array[BME680_COEFF_SIZE] = { 0 };
	uint8_t temp_var = 0; /* Temporary variable */

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK) {
		rslt = bme680_get_regs(BME680_COEFF_ADDR1, coeff_array, BME680_COEFF_ADDR1_LEN, dev);
		/* Append the second half in the same array */
		if (rslt == BME680_OK)
			rslt = bme680_get_regs(BME680_COEFF_ADDR2, &coeff_array[BME680_COEFF_ADDR1_LEN]
			, BME680_COEFF_ADDR2_LEN, dev);

		/* Temperature related coefficients */
		dev->calib.par_t1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T1_MSB_REG],
			coeff_array[BME680_T1_LSB_REG]));
		dev->calib.par_t2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG],
			coeff_array[BME680_T2_LSB_REG]));
		dev->calib.par_t3 = (int8_t) (coeff_array[BME680_T3_REG]);

		/* Pressure related coefficients */
		dev->calib.par_p1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P1_MSB_REG],
			coeff_array[BME680_P1_LSB_REG]));
		dev->calib.par_p2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG],
			coeff_array[BME680_P2_LSB_REG]));
		dev->calib.par_p3 = (int8_t) coeff_array[BME680_P3_REG];
		dev->calib.par_p4 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG],
			coeff_array[BME680_P4_LSB_REG]));
		dev->calib.par_p5 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG],
			coeff_array[BME680_P5_LSB_REG]));
		dev->calib.par_p6 = (int8_t) (coeff_array[BME680_P6_REG]);
		dev->calib.par_p7 = (int8_t) (coeff_array[BME680_P7_REG]);
		dev->calib.par_p8 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG],
			coeff_array[BME680_P8_LSB_REG]));
		dev->calib.par_p9 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG],
			coeff_array[BME680_P9_LSB_REG]));
		dev->calib.par_p10 = (uint8_t) (coeff_array[BME680_P10_REG]);

		/* Humidity related coefficients */
		dev->calib.par_h1 = (uint16_t) (((uint16_t) coeff_array[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
			| (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
		dev->calib.par_h2 = (uint16_t) (((uint16_t) coeff_array[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
			| ((coeff_array[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
		dev->calib.par_h3 = (int8_t) coeff_array[BME680_H3_REG];
		dev->calib.par_h4 = (int8_t) coeff_array[BME680_H4_REG];
		dev->calib.par_h5 = (int8_t) coeff_array[BME680_H5_REG];
		dev->calib.par_h6 = (uint8_t) coeff_array[BME680_H6_REG];
		dev->calib.par_h7 = (int8_t) coeff_array[BME680_H7_REG];

		/* Gas heater related coefficients */
		dev->calib.par_gh1 = (int8_t) coeff_array[BME680_GH1_REG];
		dev->calib.par_gh2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_GH2_MSB_REG],
			coeff_array[BME680_GH2_LSB_REG]));
		dev->calib.par_gh3 = (int8_t) coeff_array[BME680_GH3_REG];

		/* Other coefficients */
		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_RANGE_ADDR, &temp_var, 1, dev);

			dev->calib.res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);
			if (rslt == BME680_OK) {
				rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_VAL_ADDR, &temp_var, 1, dev);

				dev->calib.res_heat_val = (int8_t) temp_var;
				if (rslt == BME680_OK)
					rslt = bme680_get_regs(BME680_ADDR_RANGE_SW_ERR_ADDR, &temp_var, 1, dev);
			}
		}
		dev->calib.range_sw_err = ((int8_t) temp_var & (int8_t) BME680_RSERROR_MSK) / 16;
	}

	return rslt;
}

bool BME680_setTemperatureOversampling(uint8_t oversample)
{
  if (oversample > BME680_OS_16X) return false;

  gas_sensor.tph_sett.os_temp = oversample;

  if (oversample == BME680_OS_NONE)
    _tempEnabled = false;
  else
    _tempEnabled = true;

  return true;
}

bool BME680_setHumidityOversampling(uint8_t oversample)
{
  if (oversample > BME680_OS_16X) return false;

  gas_sensor.tph_sett.os_hum = oversample;

  if (oversample == BME680_OS_NONE)
    _humEnabled = false;
  else
    _humEnabled = true;

  return true;
}

bool BME680_setPressureOversampling(uint8_t oversample)
{
  if (oversample > BME680_OS_16X) return false;

  gas_sensor.tph_sett.os_pres = oversample;

  if (oversample == BME680_OS_NONE)
    _presEnabled = false;
  else
    _presEnabled = true;

  return true;
}

bool BME680_setIIRFilterSize(uint8_t filtersize)
{
  if (filtersize > BME680_FILTER_SIZE_127) return false;

  gas_sensor.tph_sett.filter = filtersize;

  if (filtersize == BME680_FILTER_SIZE_0)
    _filterEnabled = false;
  else
    _filterEnabled = true;

  return true;
}

bool BME680_setGasHeater(uint16_t heaterTemp, uint16_t heaterTime)
{
  gas_sensor.gas_sett.heatr_temp = heaterTemp;
  gas_sensor.gas_sett.heatr_dur = heaterTime;

  if ( (heaterTemp == 0) || (heaterTime == 0) ) {
    // disabled!
    gas_sensor.gas_sett.heatr_ctrl = BME680_DISABLE_HEATER;
    gas_sensor.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
    _gasEnabled = false;
  } else {
    gas_sensor.gas_sett.heatr_ctrl = BME680_ENABLE_HEATER;
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    _gasEnabled = true;
  }
  return true;
}

bool BME680_performReading(void)
{
  return BME680_endReading();
}

bool BME680_endReading(void)
{
  unsigned long meas_end = BME680_beginReading();
  if (meas_end == 0)
  {
    return false;
  }

   usleep((meas_end * 1000) * 2); /* Delay till the measurement is ready */

  _meas_start = 0; /* Allow new measurement to begin */
  _meas_period = 0;

  struct bme680_field_data data;

  //Serial.println("Getting sensor data");
  int8_t rslt = bme680_get_sensor_data(&data, &gas_sensor);
  if (rslt != BME680_OK)
    return false;

  if (_tempEnabled)
  {
    temperature = data.temperature / 100.0;
  }
  else
  {
    temperature = -9999;
  }

  if (_humEnabled)
  {
    humidity = data.humidity / 1000.0;
  }
  else
  {
    humidity = -9999;
  }

  if (_presEnabled)
  {
    pressure = data.pressure;
  }
  else
  {
    pressure = -9999;
  }

  /* Avoid using measurements from an unstable heating setup */
  if (_gasEnabled)
  {
    if (data.status & BME680_HEAT_STAB_MSK)
    {
      //Serial.print("Gas resistance: "); Serial.println(data.gas_resistance);
      gas_resistance = data.gas_resistance;
    }
    else
    {
      gas_resistance = 0;
      //Serial.println("Gas reading unstable!");
    }
  } else {
    gas_resistance = -9999;
  }

  return true;
}

unsigned long BME680_beginReading(void)
{
  if (_meas_start != 0) {
	/* A measurement is already in progress */
	return _meas_start + _meas_period;
  }

  uint8_t set_required_settings = 0;
  int8_t rslt;

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  gas_sensor.power_mode = BME680_FORCED_MODE;

  /* Set the required sensor settings needed */
  if (_tempEnabled)
	set_required_settings |= BME680_OST_SEL;
  if (_humEnabled)
	set_required_settings |= BME680_OSH_SEL;
  if (_presEnabled)
	set_required_settings |= BME680_OSP_SEL;
  if (_filterEnabled)
	set_required_settings |= BME680_FILTER_SEL;
  if (_gasEnabled)
	set_required_settings |= BME680_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  rslt = bme680_set_sensor_settings(set_required_settings, &gas_sensor);
  if (rslt != BME680_OK)
	return 0;

  /* Set the power mode */
  rslt = bme680_set_sensor_mode(&gas_sensor);
  if (rslt != BME680_OK)
	return 0;

  /* Get the total measurement duration so as to sleep or wait till the
   * measurement is complete */
  uint16_t meas_period;
  bme680_get_profile_dur(&meas_period, &gas_sensor);
  _meas_period = meas_period;
  return _meas_period;
}

int8_t bme680_set_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr;
	uint8_t data = 0;
	uint8_t count = 0;
	uint8_t reg_array[BME680_REG_BUFFER_LENGTH] = { 0 };
	uint8_t data_array[BME680_REG_BUFFER_LENGTH] = { 0 };
	uint8_t intended_power_mode = dev->power_mode; /* Save intended power mode */

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK) {
		if (desired_settings & BME680_GAS_MEAS_SEL)
			rslt = set_gas_config(dev);

		dev->power_mode = BME680_SLEEP_MODE;
		if (rslt == BME680_OK)
			rslt = bme680_set_sensor_mode(dev);

		/* Selecting the filter */
		if (desired_settings & BME680_FILTER_SEL) {
			rslt = boundary_check(&dev->tph_sett.filter, BME680_FILTER_SIZE_0, BME680_FILTER_SIZE_127, dev);
			reg_addr = BME680_CONF_ODR_FILT_ADDR;

			if (rslt == BME680_OK)
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & BME680_FILTER_SEL)
				data = BME680_SET_BITS(data, BME680_FILTER, dev->tph_sett.filter);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
		}

		/* Selecting heater control for the sensor */
		if (desired_settings & BME680_HCNTRL_SEL) {
			rslt = boundary_check(&dev->gas_sett.heatr_ctrl, BME680_ENABLE_HEATER,
				BME680_DISABLE_HEATER, dev);
			reg_addr = BME680_CONF_HEAT_CTRL_ADDR;

			if (rslt == BME680_OK)
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);
			data = BME680_SET_BITS_POS_0(data, BME680_HCTRL, dev->gas_sett.heatr_ctrl);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
		}

		/* Selecting heater T,P oversampling for the sensor */
		if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
			rslt = boundary_check(&dev->tph_sett.os_temp, BME680_OS_NONE, BME680_OS_16X, dev);
			reg_addr = BME680_CONF_T_P_MODE_ADDR;

			if (rslt == BME680_OK)
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & BME680_OST_SEL)
				data = BME680_SET_BITS(data, BME680_OST, dev->tph_sett.os_temp);

			if (desired_settings & BME680_OSP_SEL)
				data = BME680_SET_BITS(data, BME680_OSP, dev->tph_sett.os_pres);

			reg_array[count] = reg_addr;
			data_array[count] = data;
			count++;
		}

		/* Selecting humidity oversampling for the sensor */
		if (desired_settings & BME680_OSH_SEL) {
			rslt = boundary_check(&dev->tph_sett.os_hum, BME680_OS_NONE, BME680_OS_16X, dev);
			reg_addr = BME680_CONF_OS_H_ADDR;

			if (rslt == BME680_OK)
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);
			data = BME680_SET_BITS_POS_0(data, BME680_OSH, dev->tph_sett.os_hum);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
		}

		/* Selecting the runGas and NB conversion settings for the sensor */
		if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
			rslt = boundary_check(&dev->gas_sett.run_gas, BME680_RUN_GAS_DISABLE,
				BME680_RUN_GAS_ENABLE, dev);
			if (rslt == BME680_OK) {
				/* Validate boundary conditions */
				rslt = boundary_check(&dev->gas_sett.nb_conv, BME680_NBCONV_MIN,
					BME680_NBCONV_MAX, dev);
			}

			reg_addr = BME680_CONF_ODR_RUN_GAS_NBC_ADDR;

			if (rslt == BME680_OK)
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & BME680_RUN_GAS_SEL)
				data = BME680_SET_BITS(data, BME680_RUN_GAS, dev->gas_sett.run_gas);

			if (desired_settings & BME680_NBCONV_SEL)
				data = BME680_SET_BITS_POS_0(data, BME680_NBCONV, dev->gas_sett.nb_conv);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
		}

		if (rslt == BME680_OK)
			rslt = bme680_set_regs(reg_array, data_array, count, dev);

		/* Restore previous intended power mode */
		dev->power_mode = intended_power_mode;
	}

	return rslt;
}

static int8_t set_gas_config(struct bme680_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK) {

		uint8_t reg_addr[2] = {0};
		uint8_t reg_data[2] = {0};

		if (dev->power_mode == BME680_FORCED_MODE) {
			reg_addr[0] = BME680_RES_HEAT0_ADDR;
			reg_data[0] = calc_heater_res(dev->gas_sett.heatr_temp, dev);
			reg_addr[1] = BME680_GAS_WAIT0_ADDR;
			reg_data[1] = calc_heater_dur(dev->gas_sett.heatr_dur);
			dev->gas_sett.nb_conv = 0;
		} else {
			rslt = BME680_W_DEFINE_PWR_MODE;
		}
		if (rslt == BME680_OK)
			rslt = bme680_set_regs(reg_addr, reg_data, 2, dev);
	}

	return rslt;
}

int8_t bme680_set_sensor_mode(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t tmp_pow_mode;
	uint8_t pow_mode = 0;
	uint8_t reg_addr = BME680_CONF_T_P_MODE_ADDR;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK)
	{
		/* Call repeatedly until in sleep */
		do
		{
			rslt = bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &tmp_pow_mode, 1, dev);
			if (rslt == BME680_OK)
			{
				/* Put to sleep before changing mode */
				pow_mode = (tmp_pow_mode & BME680_MODE_MSK);

				if (pow_mode != BME680_SLEEP_MODE)
				{
					tmp_pow_mode = tmp_pow_mode & (~BME680_MODE_MSK); /* Set to sleep */
					rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
					dev->delay_ms(BME680_POLL_PERIOD_MS);
				}
			}
		} while (pow_mode != BME680_SLEEP_MODE);

		/* Already in sleep */
		if (dev->power_mode != BME680_SLEEP_MODE)
		{
			tmp_pow_mode = (tmp_pow_mode & ~BME680_MODE_MSK) | (dev->power_mode & BME680_MODE_MSK);
			if (rslt == BME680_OK)
				rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
		}
	}

	return rslt;
}

static uint8_t calc_heater_res(uint16_t temp, const struct bme680_dev *dev)
{
	uint8_t heatr_res;
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t heatr_res_x100;

	if (temp > 400) /* Cap temperature */
		temp = 400;

	var1 = (((int32_t) dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
	var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (dev->calib.res_heat_range + 4));
	var5 = (131 * dev->calib.res_heat_val) + 65536;
	heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
	heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

	return heatr_res;
}

static uint8_t calc_heater_dur(uint16_t dur)
{
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		durval = (uint8_t) (dur + (factor * 64));
	}

	return durval;
}

void bme680_get_profile_dur(uint16_t *duration, const struct bme680_dev *dev)
{
	uint32_t tph_dur; /* Calculate in us */
	uint32_t meas_cycles;
	uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};

	meas_cycles = os_to_meas_cycles[dev->tph_sett.os_temp];
	meas_cycles += os_to_meas_cycles[dev->tph_sett.os_pres];
	meas_cycles += os_to_meas_cycles[dev->tph_sett.os_hum];

	/* TPH measurement duration */
	tph_dur = meas_cycles * UINT32_C(1963);
	tph_dur += UINT32_C(477 * 4); /* TPH switching duration */
	tph_dur += UINT32_C(477 * 5); /* Gas measurement duration */
	tph_dur += UINT32_C(500); /* Get it to the closest whole number.*/
	tph_dur /= UINT32_C(1000); /* Convert to ms */

	tph_dur += UINT32_C(1); /* Wake up duration of 1ms */

	*duration = (uint16_t) tph_dur;

	/* Get the gas duration only when the run gas is enabled */
	if (dev->gas_sett.run_gas) {
		/* The remaining time should be used for heating */
		*duration += dev->gas_sett.heatr_dur;
	}
}

int8_t bme680_get_sensor_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME680_OK) {
		/* Reading the sensor data in forced mode only */
		rslt = read_field_data(data, dev);
		if (rslt == BME680_OK) {
			if (data->status & BME680_NEW_DATA_MSK)
				dev->new_fields = 1;
			else
				dev->new_fields = 0;
		}
	}

	return rslt;
}

static int8_t read_field_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t buff[BME680_FIELD_LENGTH] = { 0 };
	uint8_t gas_range;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res;
	uint8_t tries = 10;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	do {
		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(((uint8_t) (BME680_FIELD0_ADDR)), buff, (uint16_t) BME680_FIELD_LENGTH,
				dev);

			data->status = buff[0] & BME680_NEW_DATA_MSK;
			data->gas_index = buff[0] & BME680_GAS_INDEX_MSK;
			data->meas_index = buff[1];

			/* read the raw data from the sensor */
			adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16)
				| ((uint32_t) buff[4] / 16));
			adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16)
				| ((uint32_t) buff[7] / 16));
			adc_hum = (uint16_t) (((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);
			adc_gas_res = (uint16_t) ((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
			gas_range = buff[14] & BME680_GAS_RANGE_MSK;

			data->status |= buff[14] & BME680_GASM_VALID_MSK;
			data->status |= buff[14] & BME680_HEAT_STAB_MSK;

			if (data->status & BME680_NEW_DATA_MSK) {
				data->temperature = calc_temperature(adc_temp, dev);
				data->pressure = calc_pressure(adc_pres, dev);
				data->humidity = calc_humidity(adc_hum, dev);
				data->gas_resistance = calc_gas_resistance(adc_gas_res, gas_range, dev);
				break;
			}
			/* Delay to poll the data */
			dev->delay_ms(BME680_POLL_PERIOD_MS);
		}
		tries--;
	} while (tries);

	if (!tries)
		rslt = BME680_W_NO_NEW_DATA;

	return rslt;
}

static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme680_dev *dev)
{
	int8_t rslt = BME680_OK;

	if (value != NULL) {
		/* Check if value is below minimum value */
		if (*value < min) {
			/* Auto correct the invalid value to minimum value */
			*value = min;
			dev->info_msg |= BME680_I_MIN_CORRECTION;
		}
		/* Check if value is above maximum value */
		if (*value > max) {
			/* Auto correct the invalid value to maximum value */
			*value = max;
			dev->info_msg |= BME680_I_MAX_CORRECTION;
		}
	} else {
		rslt = BME680_E_NULL_PTR;
	}

	return rslt;
}

static int16_t calc_temperature(uint32_t temp_adc, struct bme680_dev *dev)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int16_t calc_temp;

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t) dev->calib.par_t1 << 1);
	var2 = (var1 * (int32_t) dev->calib.par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) dev->calib.par_t3 << 4)) >> 14;
	dev->calib.t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((dev->calib.t_fine * 5) + 128) >> 8);

	return calc_temp;
}

static uint32_t calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t pressure_comp;

	var1 = (((int32_t)dev->calib.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)dev->calib.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)dev->calib.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)dev->calib.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		((int32_t)dev->calib.par_p3 << 5)) >> 3) +
		(((int32_t)dev->calib.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)dev->calib.par_p1) >> 15;
	pressure_comp = 1048576 - pres_adc;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
	if (pressure_comp >= BME680_MAX_OVERFLOW_VAL)
		pressure_comp = ((pressure_comp / var1) << 1);
	else
		pressure_comp = ((pressure_comp << 1) / var1);
	var1 = ((int32_t)dev->calib.par_p9 * (int32_t)(((pressure_comp >> 3) *
		(pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) *
		(int32_t)dev->calib.par_p8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
		(int32_t)(pressure_comp >> 8) *
		(int32_t)dev->calib.par_p10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
		((int32_t)dev->calib.par_p7 << 7)) >> 4);

	return (uint32_t)pressure_comp;

}

static uint32_t calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	temp_scaled = (((int32_t) dev->calib.t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) dev->calib.par_h1 * 16)))
		- (((temp_scaled * (int32_t) dev->calib.par_h3) / ((int32_t) 100)) >> 1);
	var2 = ((int32_t) dev->calib.par_h2
		* (((temp_scaled * (int32_t) dev->calib.par_h4) / ((int32_t) 100))
			+ (((temp_scaled * ((temp_scaled * (int32_t) dev->calib.par_h5) / ((int32_t) 100))) >> 6)
				/ ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t) dev->calib.par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t) dev->calib.par_h7) / ((int32_t) 100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 100000) /* Cap at 100%rH */
		calc_hum = 100000;
	else if (calc_hum < 0)
		calc_hum = 0;

	return (uint32_t) calc_hum;
}

static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev)
{
	int64_t var1;
	uint64_t var2;
	int64_t var3;
	uint32_t calc_gas_res;
	/**Look up table 1 for the possible gas range values */
	uint32_t lookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
		UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };
	/**Look up table 2 for the possible gas range values */
	uint32_t lookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
		UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016),
		UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000),
		UINT32_C(250000), UINT32_C(125000) };

	var1 = (int64_t) ((1340 + (5 * (int64_t) dev->calib.range_sw_err)) *
		((int64_t) lookupTable1[gas_range])) >> 16;
	var2 = (((int64_t) ((int64_t) gas_res_adc << 15) - (int64_t) (16777216)) + var1);
	var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) >> 9);
	calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

	return calc_gas_res;
}

float BME680_readTemperature(void) {
	BME680_performReading();
  return temperature;
}

float BME680_readPressure(void) {
	BME680_performReading();
  return pressure;
}

float BME680_readHumidity(void) {
	BME680_performReading();
  return humidity;
}

uint32_t BME680_readGas(void) {
	BME680_performReading();
  return gas_resistance;
}

float BME680_readAltitude(float seaLevel)
{
    float atmospheric = BME680_readPressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}
