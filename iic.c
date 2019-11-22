/*
 * iic.c
 *
 *  Created on: Nov 23, 2018
 *      Author: piyatap
 */

#include "config.h"

int iic_write(XIic *iicPtr, uint8_t ADDR, ConfigTable* data)
{
	uint32_t i, rv;
    uint8_t send_data[2];

    for ( i = 0 ; i < (sizeof(*data)/sizeof(ConfigTable)); i++, data++ )
    {
        send_data[0] = data->addr;
        send_data[1] = data->data;

        rv = XIic_Send(iicPtr->BaseAddress, ADDR, send_data, 2, XIIC_STOP);
        if(rv != 2)
        	return XST_FAILURE;

        usleep(10 * 1000);
    }

    return XST_SUCCESS;
}

int iic_read(XIic *iicPtr, uint8_t ADDR, ConfigTable* data)
{
	uint32_t i, rv;
	uint8_t read_data;

	for ( i = 0 ; i < (sizeof(*data)/sizeof(ConfigTable)); i++, data++ )
	{
		read_data = data->addr;

		rv = XIic_Send(iicPtr->BaseAddress, ADDR, &read_data, 1, XIIC_REPEATED_START);
		if(rv != 1)
			return XST_FAILURE;

		rv = XIic_Recv(iicPtr->BaseAddress, ADDR, &read_data, 1, XIIC_STOP);
		if(rv != 1)
			return XST_FAILURE;

		data->data = read_data;
		usleep(10 * 1000);
	}

	return XST_SUCCESS;
}

int iic_multi_read(XIic *iicPtr, uint8_t ADDR, uint8_t offset, uint8_t ByteCount, uint8_t* data)
{
	uint32_t rv;
	uint8_t read_data;

	read_data = offset;

	rv = XIic_Send(iicPtr->BaseAddress, ADDR, &read_data, 1, XIIC_REPEATED_START);
	if(rv != 1)
		return XST_FAILURE;

	rv = XIic_Recv(iicPtr->BaseAddress, ADDR, data, ByteCount, XIIC_STOP);
	if(rv != ByteCount)
		return XST_FAILURE;

	usleep(10 * 1000);

	return XST_SUCCESS;
}

int iic_multi_write(XIic *iicPtr, uint8_t ADDR, uint8_t offset, uint8_t ByteCount, uint8_t* data)
{
	uint32_t rv, i;
	uint8_t *read_data = (uint8_t *)malloc(sizeof(uint8_t)*(ByteCount+1));

	for (i=0; i<ByteCount+1; i++)
	{
		if (i == 0)
			read_data[0] = offset;
		else
			read_data[i] = data[i-1];
	}

	rv = XIic_Send(iicPtr->BaseAddress, ADDR, read_data, ByteCount+1, XIIC_STOP);
	if(rv != ByteCount+1)
		return XST_FAILURE;

	usleep(10 * 1000);

	return XST_SUCCESS;
}




