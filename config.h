/*
 * config.h
 *
 *  Created on: Mar 6, 2018
 *      Author: petalinux
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#include <assert.h>
#include <sleep.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "xparameters_ps.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "sleep.h"
#include "xiic.h"

#define IIC_CAMERA0								XPAR_IIC_0_DEVICE_ID

#define DS90UB954_ADDR 							0x30
#define DS90UB953_REMOTE_SLAVE_ALIAS_ADDR 		0x21

typedef struct
{
    uint8_t addr;
    uint8_t data;
} ConfigTable;

extern XIic iic;

#endif /* SRC_CONFIG_H_ */
