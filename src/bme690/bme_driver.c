/**
 * Copyright (C) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bme69x.h"
#include "bme_driver.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME69X shuttle board ID */
#define BME69X_SHUTTLE_ID  0x93

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;
/******************************************************************************/
/*!                User interface functions                                   */


static const struct i2c_dt_spec *i2c_spec;

void bme690_set_i2c_spec(const struct i2c_dt_spec *spec)
{
    i2c_spec = spec;
}

/*!
 * I2C read function map to RDK2 platform
 */
BME69X_INTF_RET_TYPE bme69x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /*uint8_t device_addr = *(uint8_t*)intf_ptr;
    int8_t err;

    err = hal_i2c_read_register(device_addr, reg_addr, reg_data, len);
    if(err < 0)
    {
    	return BME69X_E_COM_FAIL;
    }*/

    if (i2c_write_read_dt(i2c_spec, &reg_addr, sizeof(reg_addr), reg_data, len) != 0) return -1;

	return BME69X_OK;
}

/*!
 * I2C write function map to RDK2 platform
 */
BME69X_INTF_RET_TYPE bme69x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /*uint8_t device_addr = *(uint8_t*)intf_ptr;
    int8_t err;

    err = hal_i2c_write_register(device_addr, reg_addr, (uint8_t*)reg_data, len);
    if(err < 0)
    {
    	return BME69X_E_COM_FAIL;
    }*/

    uint8_t* i2c_data = malloc(len + 1);
    i2c_data[0] = reg_addr;
    memcpy(&i2c_data[1], reg_data, len);

    int ret = i2c_write_dt(i2c_spec, i2c_data, len + 1);

    free(i2c_data);

    if (ret != 0) return -1;

    return BME69X_OK;
}

/*!
 * SPI read function map to RDK2 platform
 */
BME69X_INTF_RET_TYPE bme69x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	return BME69X_E_COM_FAIL;
}

/*!
 * SPI write function map to RDK2 platform
 */
BME69X_INTF_RET_TYPE bme69x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	return BME69X_E_COM_FAIL;
}

/*!
 * Delay function map to RDK2 platform
 */
void bme69x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    k_usleep(period);
}

void bme69x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME69X_OK:

            /* Do nothing */
            break;
        case BME69X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME69X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME69X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME69X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME69X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME69X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme69x_interface_init(struct bme69x_dev *bme, uint8_t intf)
{
    if (bme != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BME69X_I2C_INTF)
        {
        	printf("BME69x I2C Interface Initialized.\r\n");
            dev_addr = BME69X_I2C_ADDR_LOW;
            bme->read = bme69x_i2c_read;
            bme->write = bme69x_i2c_write;
            bme->intf = BME69X_I2C_INTF;
        }
        /* Bus configuration : SPI */
        else if (intf == BME69X_SPI_INTF)
        {
	        printf("SPI Interface not supported.\r\n");
	        return BME69X_E_DEV_NOT_FOUND;
        }

        bme->delay_us = bme69x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        return BME69X_E_NULL_PTR;
    }

    return BME69X_OK;
}

void bme69x_coines_deinit(void)
{
	return;
}
