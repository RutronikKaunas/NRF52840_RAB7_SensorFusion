/*
 * bmp585_app.c
 *
 *  Created on: 2025-04-03
 *      Author: GDR
 */



#include "bmp585_app.h"

struct bmp5_dev bmp = { 0 };
struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };
struct bmp5_iir_config set_iir_cfg;
struct bmp5_int_source_select int_source_select;
uint8_t bmp_int_status;
struct bmp5_sensor_data bmp_data;

int8_t bmp585_app_init(void)
{
	int8_t rslt;

    rslt = bmp5_interface_init(&bmp, BMP5_I2C_INTF);
    bmp5_error_codes_print_result("bmp5_interface_init", rslt);
    if(rslt != BMP5_OK){return rslt;}

    rslt = bmp5_soft_reset(&bmp);
    bmp5_error_codes_print_result("bmp5_soft_reset", rslt);
    if(rslt != BMP5_OK){return rslt;}
    bmp5_delay_us(5000, &bmp);

    rslt = bmp5_init(&bmp);
    bmp5_error_codes_print_result("bmp5_init", rslt);
    if(rslt != BMP5_OK){return rslt;}

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmp);
    bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);
    if(rslt != BMP5_OK){return rslt;}

    rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, &bmp);
    bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);
    if(rslt != BMP5_OK){return rslt;}

    /* Set ODR as 50Hz */
    osr_odr_press_cfg.odr = BMP5_ODR_50_HZ;

    /* Enable pressure */
    osr_odr_press_cfg.press_en = BMP5_ENABLE;

    /* Set Over-sampling rate with respect to odr */
    osr_odr_press_cfg.osr_t = BMP5_OVERSAMPLING_64X;
    osr_odr_press_cfg.osr_p = BMP5_OVERSAMPLING_4X;

    rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, &bmp);
    bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
    if(rslt != BMP5_OK){return rslt;}

    set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
    set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
    set_iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
    set_iir_cfg.shdw_set_iir_p = BMP5_ENABLE;

    rslt = bmp5_set_iir_config(&set_iir_cfg, &bmp);
    bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
    if(rslt != BMP5_OK){return rslt;}

    rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, &bmp);
    bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);
    if(rslt != BMP5_OK){return rslt;}

    int_source_select.drdy_en = BMP5_ENABLE;
    rslt = bmp5_int_source_select(&int_source_select, &bmp);
    bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
    if(rslt != BMP5_OK){return rslt;}

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp);
    bmp5_error_codes_print_result("bmp5_set_power_mode", rslt);
    if(rslt != BMP5_OK){return rslt;}

	return rslt;
}

int8_t bmp585_read_data(float *temp, float *press)
{
	int8_t rslt;
	bmp_int_status = 0;

    rslt = bmp5_get_interrupt_status(&bmp_int_status, &bmp);
    bmp5_error_codes_print_result("bmp5_get_interrupt_status", rslt);
    if(rslt != BMP5_OK){return rslt;}

    if (bmp_int_status & BMP5_INT_ASSERTED_DRDY)
    {
        rslt = bmp5_get_sensor_data(&bmp_data, &osr_odr_press_cfg, &bmp);
        bmp5_error_codes_print_result("bmp5_get_sensor_data", rslt);
        if(rslt != BMP5_OK){return rslt;}

        *press = bmp_data.pressure;
        *temp = bmp_data.temperature;
    }

	return rslt;
}
