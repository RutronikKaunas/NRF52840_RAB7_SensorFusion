/*
 * bmp585_app.h
 *
 *  Created on: 2025-04-03
 *      Author: GDR
 */

#ifndef BMP585_BMP585_APP_H_
#define BMP585_BMP585_APP_H_

#include "bmp5_defs.h"
#include "bmp_driver.h"
#include "bmp5.h"

int8_t bmp585_app_init(void);
int8_t bmp585_read_data(float *temp, float *press);


#endif /* BMP585_BMP585_APP_H_ */
