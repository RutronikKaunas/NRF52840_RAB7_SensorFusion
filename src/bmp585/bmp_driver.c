/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// k_usleep
#include <zephyr/kernel.h>

#include "bmp5_defs.h"
#include "bmp_driver.h"

/******************************************************************************/
/*!                         Macro definitions                                 */

/*! BMP5 shuttle id */
#define BMP5_SHUTTLE_ID_PRIM  UINT16_C(0x1B3)
#define BMP5_SHUTTLE_ID_SEC   UINT16_C(0x1D3)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

static const struct i2c_dt_spec *i2c_spec;

void bmp585_set_i2c_spec(const struct i2c_dt_spec *spec)
{
    i2c_spec = spec;
}

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    /*uint8_t dev_addr = *(uint8_t*)intf_ptr;
    int8_t err;

    err = hal_i2c_read_register(dev_addr, reg_addr, reg_data, length);
    if(err < 0)
    {
    	return BMP5_E_COM_FAIL;
    }*/

    if (i2c_write_read_dt(i2c_spec, &reg_addr, sizeof(reg_addr), reg_data, length) != 0) return -1;

	return BMP5_OK;
}

/*!
 * I2C write function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    /*uint8_t dev_addr = *(uint8_t*)intf_ptr;
    int8_t err;*/

    /*err = hal_i2c_write_register(dev_addr, reg_addr, (uint8_t*)reg_data, length);
    if(err < 0)
    {
    	return BMP5_E_COM_FAIL;
    }*/

    uint8_t* i2c_data = malloc(length + 1);
    i2c_data[0] = reg_addr;
    memcpy(&i2c_data[1], reg_data, length);

    int ret = i2c_write_dt(i2c_spec, i2c_data, length + 1);

    free(i2c_data);

    if (ret != 0) return -1;


    return BMP5_OK;
}

/*!
 * SPI read function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	return BMP5_E_COM_FAIL;
}

/*!
 * SPI write function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	return BMP5_E_COM_FAIL;
}

/*!
 * Delay function map to COINES platform
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    k_usleep(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmp5_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP5_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP5_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMP5_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_CHIP_ID)
        {
            printf("Error [%d] : Invalid chip id\r\n", rslt);
        }
        else if (rslt == BMP5_E_POWER_UP)
        {
            printf("Error [%d] : Power up error\r\n", rslt);
        }
        else if (rslt == BMP5_E_POR_SOFTRESET)
        {
            printf("Error [%d] : Power-on reset/softreset failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_POWERMODE)
        {
            printf("Error [%d] : Invalid powermode\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmp5_interface_init(struct bmp5_dev *bmp5_dev, uint8_t intf)
{
	if (bmp5_dev != NULL)
	{
	    /* Bus configuration : I2C */
	    if (intf == BMP5_I2C_INTF)
	    {
	        printf("BMP5xx I2C Interface Initialized.\r\n");
	        dev_addr = BMP5_I2C_ADDR_SEC;
	        bmp5_dev->read = bmp5_i2c_read;
	        bmp5_dev->write = bmp5_i2c_write;
	        bmp5_dev->intf = BMP5_I2C_INTF;
	    }
	    /* Bus configuration : SPI */
	    else if (intf == BMP5_SPI_INTF)
	    {
	        printf("SPI Interface not supported.\r\n");
	        return BMP5_E_DEV_NOT_FOUND;
	    }

	    bmp5_dev->delay_us = bmp5_delay_us;
	    bmp5_dev->intf_ptr = &dev_addr;
	}
	else
	{
		return BMP5_E_COM_FAIL;
	}

    return BMP5_OK;
}

void bmp5_coines_deinit(void)
{
	return;
}
