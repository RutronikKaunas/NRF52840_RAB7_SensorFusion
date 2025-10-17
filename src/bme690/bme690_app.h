/*
 * bme690_app.h
 *
 *  Created on: 2025-04-03
 *      Author: GDR
 */

#ifndef BME690_BME690_APP_H_
#define BME690_BME690_APP_H_

#include "bme69x.h"
#include "bme_driver.h"

#define BME69X_VALID_DATA  	UINT8_C(0xB0)

typedef struct bme69x_data bme69x_data_t;

int8_t bme690_app_init(void);
int8_t bme690_data_available(uint8_t *data_fields);
int8_t bme690_data_read(bme69x_data_t *bme_data, uint8_t index);

/**
 * @brief Get the measurement period in milliseconds
 *
 * Remark: to be valid, first call bme690_app_init
 */
uint32_t bme690_app_get_measurement_period();

#endif /* BME690_BME690_APP_H_ */
