#include "dps368.h"

#define RAW_VALUE_ADDR      0x00
#define PRS_CFG_ADDR        0x06
#define TMP_CFG_ADDR        0x07
#define MEAS_CFG_ADDR       0x08
#define CFG_REG_ADDR        0x09
#define COEF_REG_ADDR       0x10
#define TMP_COEF_SRCE_ADDR  0x28

#define COEF_LENGTH         18
#define RAW_VALUES_LENGTH   6

#define POW_2_11_MINUS_1                            (0x7FF)
#define POW_2_12                                    (0x1000)
#define POW_2_19_MINUS_1                            (0x7FFFF)
#define POW_2_20                                    (0x100000)
#define POW_2_15_MINUS_1                            (0x7FFF)
#define POW_2_16                                    (0x10000)
#define POW_2_23_MINUS_1                            (0x7FFFFF)
#define POW_2_24                                    (0x1000000)

static const struct i2c_dt_spec *i2c_spec;

static int16_t C0;
static int16_t C1;
static int32_t C00;
static int32_t C10;
static int32_t C01;
static int32_t C11;
static int32_t C20;
static int32_t C21;
static int32_t C30;

#define TEMP_SRC_INTERNAL   0
#define TEMP_SRC_EXTERNAL   1

#define OSR_8_TIMES 3
#define MODE_BACKGROUND_ALL 7

static uint8_t fifo_enable = 0;
static uint8_t interrupt_triggers = 0;
static uint8_t temperature_coeff_src = 0;

void dps368_set_i2c_spec(const struct i2c_dt_spec *spec)
{
    i2c_spec = spec;
}

static int dps368_read_calibration_reg()
{
    // Set read pointer
    uint8_t reg_addr = COEF_REG_ADDR;
    int ret = i2c_write_dt(i2c_spec, &reg_addr, sizeof(uint8_t));
	if(ret != 0) return -1;

    uint8_t coeff_raw[COEF_LENGTH] = {0};
    ret = i2c_read_dt(i2c_spec, coeff_raw, sizeof(coeff_raw));
	if(ret != 0) return -2;

    C0 = ((int32_t)coeff_raw[0] << 4u) + (((int32_t)coeff_raw[1] >> 4u) & 0x0F);
    if (C0 > POW_2_11_MINUS_1)
    {
        C0 = C0 - POW_2_12;
    }

    C1 = (coeff_raw[2] + ((coeff_raw[1] & 0x0F) << 8u));
    if (C1 > POW_2_11_MINUS_1)
    {
        C1 = C1 - POW_2_12;
    }

    C00 = (((int32_t)coeff_raw[4] << 4u)
                             + ((int32_t)coeff_raw[3] << 12u)) + (((int32_t)coeff_raw[5] >> 4) & 0x0F);
    if (C00 > POW_2_19_MINUS_1)
    {
        C00 = C00 - POW_2_20;
    }

    C10 = (((int32_t)coeff_raw[5] & (int32_t)0x0F) << 16u)
                            + ((int32_t)coeff_raw[6] << 8u) + coeff_raw[7];
    if (C10 > POW_2_19_MINUS_1)
    {
        C10 = C10 - POW_2_20;
    }

    C01 = (coeff_raw[9] + ((int32_t)coeff_raw[8] << 8u));
    if (C01 > POW_2_15_MINUS_1)
    {
        C01 = C01 - POW_2_16;
    }

    C11 = (coeff_raw[11] + ((int32_t)coeff_raw[10] << 8u));
    if (C11 > POW_2_15_MINUS_1)
    {
        C11 = C11 - POW_2_16;
    }

    C20 = (coeff_raw[13] + ((int32_t)coeff_raw[12] << 8u));
    if (C20 > POW_2_15_MINUS_1)
    {
        C20 = C20 - POW_2_16;
    }

    C21 = (coeff_raw[15] + ((int32_t)coeff_raw[14] << 8u));
    if (C21 > POW_2_15_MINUS_1)
    {
        C21 = C21 - POW_2_16;
    }

    C30 = (coeff_raw[17] + ((int32_t)coeff_raw[16] << 8u));
    if (C30 > POW_2_15_MINUS_1)
    {
        C30 = C30 - POW_2_16;
    }

    return 0;
}

/*
 * function to fix a hardware problem on some devices
 * you have this bug if you measure around 60°C when temperature is around 20°C
 * call dps368_hw_bug_fix() directly in the init() function to fix this issue
 */
static int dps368_hw_bug_fix()
{
    uint8_t reg_value[2] = { 0x0E, 0xA5 };
    int ret = i2c_write_dt(i2c_spec, reg_value, sizeof(reg_value));
	if(ret != 0) return -1;

    reg_value[0] = 0x0F;
    reg_value[1] = 0x96;
    ret = i2c_write_dt(i2c_spec, reg_value, sizeof(reg_value));
	if(ret != 0) return -2;

    reg_value[0] = 0x62;
    reg_value[1] = 0x02;
    ret = i2c_write_dt(i2c_spec, reg_value, sizeof(reg_value));
	if(ret != 0) return -2;

    reg_value[0] = 0x0E;
    reg_value[1] = 0x00;
    ret = i2c_write_dt(i2c_spec, reg_value, sizeof(reg_value));
	if(ret != 0) return -2;

    reg_value[0] = 0x0F;
    reg_value[1] = 0x00;
    ret = i2c_write_dt(i2c_spec, reg_value, sizeof(reg_value));
	if(ret != 0) return -2;

    return 0;
}

/**
 * @brief Set temperature measurement configuration
 * 
 * @param [in] temp_measurement Source - 0: internal / 1: external
 * @param [in] rate Measurement rate 0: 1 meas/sec, 1: 2 meas/sec, ..., 7: 128 meas/sec
 * @param [in] osr Oversampling (precision) 0: single, 1: 2 times, ..., 7: 128 times
 */
static int dps368_set_temperature_config(uint8_t temp_measurement, uint8_t rate, uint8_t osr)
{
    uint8_t reg_value = (temp_measurement << 7) | (rate << 4) | osr;
    uint8_t to_write[2] = { TMP_CFG_ADDR, reg_value };

    int ret = i2c_write_dt(i2c_spec, to_write, sizeof(to_write));
	if(ret != 0) return -1;

    // OSR > 8 times
    if (osr > OSR_8_TIMES)
    {
        // TODO
    }

    return 0;
}

/**
 * @brief Set pressure measurement configuration
 * 
 * @param [in] rate Measurement rate 0: 1 meas/sec, 1: 2 meas/sec, ..., 7: 128 meas/sec
 * @param [in] osr Oversampling (precision) 0: single, 1: 2 times, ..., 7: 128 times
 */
static int dps368_set_pressure_config(uint8_t rate, uint8_t osr)
{
    uint8_t reg_value = (rate << 4) | osr;
    uint8_t to_write[2] = { PRS_CFG_ADDR, reg_value };

    int ret = i2c_write_dt(i2c_spec, to_write, sizeof(to_write));
	if(ret != 0) return -1;

    // OSR > 8 times
    if (osr > OSR_8_TIMES)
    {
        // TODO
    }

    return 0;
}

/**
 * @brief Set the measurement mode
 * 
 * @param[in] measurement_mode  
 *              0: stop measurement
 *              1: command mode - pressure measurement
 *              2: command mode - temperature measurement
 *              5: continuous pressure measurement
 *              6: continuous temperature measurement
 *              7: continuous pressure and temperature measurement
 */
static int dps368_set_measurement_mode(uint8_t measurement_mode)
{
    uint8_t to_write[2] = { MEAS_CFG_ADDR, measurement_mode };

    int ret = i2c_write_dt(i2c_spec, to_write, sizeof(to_write));
	if(ret != 0) return -1;

    return 0;
}

static int dps368_read_raw_values(uint8_t* raw_values)
{
    // Set read pointer
    uint8_t reg_addr = RAW_VALUE_ADDR;
    int ret = i2c_write_dt(i2c_spec, &reg_addr, sizeof(uint8_t));
	if(ret != 0) return -1;

    // Read the 6 values
    ret = i2c_read_dt(i2c_spec, raw_values, RAW_VALUES_LENGTH);
	if(ret != 0) return -2;

    return 0;
}

static float dps368_calc_temp_scaled(int32_t temp_raw)
{
    if (temp_raw > POW_2_23_MINUS_1)
    {
        temp_raw -= POW_2_24;
    }

    // TODO: use selected OSR coeff
    float temp_scaled = (float)temp_raw / (float)524288;
    return temp_scaled;
}

static float dps368_calc_temperature(int32_t temp_raw)
{
    if (temp_raw > POW_2_23_MINUS_1)
    {
        temp_raw -= POW_2_24;
    }

    // TODO: use selected OSR coeff
    float temp_scaled = (float)temp_raw / (float)524288;

    int64_t c0 = C0;
    int64_t c1 = C1;
    return (c0 / 2.0f) + (c1 * temp_scaled);
}

static float dps368_calc_pressure(int32_t press_raw, float temp_scaled)
{
    if (press_raw > POW_2_23_MINUS_1)
    {
        press_raw -= POW_2_24;
    }

    // TODO: use selected OSR coeff
    float press_scaled = (float)press_raw / (float)524288;
    int64_t c00 = C00;
    int64_t c10 = C10;
    int64_t c20 = C20;
    int64_t c30 = C30;
    int64_t c01 = C01;
    int64_t c11 = C11;
    int64_t c21 = C21;

    float Pcal = c00
                 + press_scaled * (c10 + press_scaled * (c20 + press_scaled * c30))
                 + (temp_scaled * c01)
                 + (temp_scaled * press_scaled * (c11 + press_scaled * c21));

    return Pcal * 0.01f;
}

int dps368_get_values(float* pressure, float* temperature)
{
    uint8_t raw_values[RAW_VALUES_LENGTH] = {0};
    if (dps368_read_raw_values(raw_values) != 0) return -1;

    // TODO: make it for all measurement modes
    int32_t press_raw = (int32_t)(raw_values[2]) + (raw_values[1] << 8) + (raw_values[0] << 16);
    int32_t temp_raw = (int32_t)(raw_values[5]) + (raw_values[4] << 8) + (raw_values[3] << 16);

    float temp_scaled = dps368_calc_temp_scaled(temp_raw);
    *temperature = dps368_calc_temperature(temp_raw);
    *pressure = dps368_calc_pressure(press_raw, temp_scaled);

    return 0;
}

int dps368_init()
{
    uint8_t reg_value = 0;
    int timeout_counter = 0;
    // Wait until ready (COEF_RDY and SENSOR_RDY)
    for(;;)
    {
        if (i2c_reg_read_byte_dt(i2c_spec, MEAS_CFG_ADDR, &reg_value) != 0) return -1;

        printf("Reg value = %d\n", reg_value);

        if ((reg_value & 0xC0) == 0xC0) break;

        k_msleep(10);
        timeout_counter++;
        if (timeout_counter > 10) return -2;
    }

    // Read coefficients
    if (dps368_read_calibration_reg() != 0) return -3;

    // Read configuration
    if (i2c_reg_read_byte_dt(i2c_spec, CFG_REG_ADDR, &reg_value) != 0) return -4;

    fifo_enable = ((reg_value & 0x2) != 0);
    interrupt_triggers = (reg_value & 0xF0);

    printf("fifo_enable %d \n", fifo_enable);
    printf("interrupt_triggers %d \n", interrupt_triggers);

    // Read temperature source
    if (i2c_reg_read_byte_dt(i2c_spec, TMP_COEF_SRCE_ADDR, &reg_value) != 0) return -5;

    temperature_coeff_src = ((reg_value >> 7) & 0x1) ? TEMP_SRC_EXTERNAL : TEMP_SRC_INTERNAL;
    printf("temperature_coeff_src %d \n", temperature_coeff_src);

    if (dps368_hw_bug_fix() != 0) return -6;

    // 1 measurement per sec, OSR = single
    if (dps368_set_temperature_config(temperature_coeff_src, 0, 0) != 0) return -7;

    // 1 measurement per sec, OSR = single (low precision)
    if (dps368_set_pressure_config(0, 0) != 0) return -8;

    // Measure all in background
    if (dps368_set_measurement_mode(MODE_BACKGROUND_ALL) != 0) return -9;

    return 0;
}