#include "bmi323.h"

static const struct i2c_dt_spec *i2c_spec;

void bmi323_set_i2c_spec(const struct i2c_dt_spec *spec)
{
    i2c_spec = spec;
}

int bmi323_get_chip_id(uint8_t* chip_id)
{
    uint8_t addr = 0;
    uint8_t read_buffer[3] = {0};
    
    // 2 dummy bytes (in I2C mode)

    if (i2c_write_read_dt(i2c_spec, &addr, sizeof(addr), read_buffer, sizeof(read_buffer)) != 0) return -1;

    *chip_id = read_buffer[2];

    return 0;
}

int bmi323_do_configuration()
{
    // TODO: make it with parameters

    // 0x4000: normal mode
    // 0x0000: no averaging
    // 0x0000: filtering to ODR/2
    // Range 8g: 0x0020
    // ODR 50Hz: 0x0007
    uint8_t acc_config[3] = {0x20, 0x27, 0x40};

    uint8_t gyr_config[3] = {0x21, 0x4B, 0x40};

    if (i2c_write_dt(i2c_spec, acc_config, sizeof(acc_config)) != 0) return -1;
    if (i2c_write_dt(i2c_spec, gyr_config, sizeof(gyr_config)) != 0) return -2;

    return 0;
}

int bmi323_get_data(bmi323_values_t* values)
{
    uint8_t addr = 3;
    uint8_t read_buffer[14] = {0};
    
    // 2 dummy bytes (in I2C mode)

    if (i2c_write_read_dt(i2c_spec, &addr, sizeof(addr), read_buffer, sizeof(read_buffer)) != 0) return -1;

    // Convert
    values->acc_x = *((int16_t*)&read_buffer[2]);
    values->acc_y = *((int16_t*)&read_buffer[4]);
    values->acc_z = *((int16_t*)&read_buffer[6]);
    values->gyr_x = *((int16_t*)&read_buffer[8]);
    values->gyr_y = *((int16_t*)&read_buffer[10]);
    values->gyr_z = *((int16_t*)&read_buffer[12]);

    return 0;
}