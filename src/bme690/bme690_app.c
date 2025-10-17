/*
 * bme690_app.c
 *
 *  Created on: 2025-04-03
 *      Author: GDR
 */


#include "bme690_app.h"

struct bme69x_dev bme;
struct bme69x_conf conf;
struct bme69x_heatr_conf heatr_conf;
static uint32_t measurement_period_ms = 5000;
struct bme69x_data data[3];
uint8_t n_fields;
/* Heater temperature in degree Celsius */
uint16_t temp_prof[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };
/* Multiplier to the shared heater duration */
uint16_t mul_prof[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };

int8_t bme690_app_init(void)
{
	int8_t rslt;

    rslt = bme69x_interface_init(&bme, BME69X_I2C_INTF);
    bme69x_check_rslt("bme69x_interface_init", rslt);
    if(rslt != BME69X_OK){return rslt;}

    rslt = bme69x_init(&bme);
    bme69x_check_rslt("bme69x_init", rslt);
    if(rslt != BME69X_OK){return rslt;}

    rslt = bme69x_get_conf(&conf, &bme);
    bme69x_check_rslt("bme69x_get_conf", rslt);
    if(rslt != BME69X_OK){return rslt;};

    conf.filter = BME69X_FILTER_OFF;
    conf.odr = BME69X_ODR_NONE;
    conf.os_hum = BME69X_OS_1X;
    conf.os_pres = BME69X_OS_16X;
    conf.os_temp = BME69X_OS_2X;
    rslt = bme69x_set_conf(&conf, &bme);
    bme69x_check_rslt("bme69x_set_conf", rslt);
    if(rslt != BME69X_OK){return rslt;}

    heatr_conf.enable = BME69X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = mul_prof;

    /* Shared heating duration in milliseconds */
    heatr_conf.shared_heatr_dur = (uint16_t)(140 - (bme69x_get_meas_dur(BME69X_PARALLEL_MODE, &conf, &bme) / 1000));

    heatr_conf.profile_len = 10;
    rslt = bme69x_set_heatr_conf(BME69X_PARALLEL_MODE, &heatr_conf, &bme);
    bme69x_check_rslt("bme69x_set_heatr_conf", rslt);
    if(rslt != BME69X_OK){return rslt;}

    /* Check if rslt == BME69X_OK, report or handle if otherwise */
    rslt = bme69x_set_op_mode(BME69X_PARALLEL_MODE, &bme);
    bme69x_check_rslt("bme69x_set_op_mode", rslt);
    if(rslt != BME69X_OK){return rslt;}

    /* Calculate delay period in milliseconds */
    measurement_period_ms = bme69x_get_meas_dur(BME69X_PARALLEL_MODE, &conf, &bme) + (heatr_conf.shared_heatr_dur * 1000);
    measurement_period_ms = measurement_period_ms / 1000;

	return rslt;
}

uint32_t bme690_app_get_measurement_period()
{
	return measurement_period_ms;
}

int8_t bme690_data_available(uint8_t *data_fields)
{
	int8_t rslt;

	rslt = bme69x_get_data(BME69X_PARALLEL_MODE, data, &n_fields, &bme);
	if(rslt != BME69X_OK){return rslt;}

	*data_fields = n_fields;

	return rslt;
}

int8_t bme690_data_read(bme69x_data_t *bme_data, uint8_t index)
{
	if(data[index].status != BME69X_VALID_DATA)
	{
		return 0;
	}

	bme_data->temperature = data[index].temperature;
	bme_data->pressure = data[index].pressure;
	bme_data->humidity = data[index].humidity;
	bme_data->gas_resistance = data[index].gas_resistance;
	bme_data->status = data[index].status;
	bme_data->gas_index = data[index].gas_index;
	bme_data->meas_index = data[index].meas_index;

	// Data is valid
	return 1;
}

