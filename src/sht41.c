#include "sht41.h"

static const struct i2c_dt_spec *i2c_spec;

#define SHT4X_CMD_READ_SERIAL           0x89
#define SHT4X_CMD_READ_HIGH_PRECISION   0xFD

void sht41_set_i2c_spec(const struct i2c_dt_spec *spec)
{
    i2c_spec = spec;
}

int sht41_get_serial_number(uint32_t* serial_number)
{
    uint8_t addr = SHT4X_CMD_READ_SERIAL;
    uint8_t read_buffer[6] = {0};
    
    // Write addr
    if (i2c_write_dt(i2c_spec, &addr, sizeof(addr)) != 0) return -1;

    // Small wait
    k_msleep(1);

    if (i2c_read_dt(i2c_spec, read_buffer, sizeof(read_buffer)) != 0) return -2;

    uint16_t msb = *((uint16_t*)&read_buffer[0]);
    uint16_t lsb = *((uint16_t*)&read_buffer[3]);

    *serial_number = ((uint32_t)msb) << 16 | (uint32_t)lsb;
    return 0;
}

int sht41_get_raw_measurement(uint16_t* temperature, uint16_t* humidity)
{
    uint8_t addr = SHT4X_CMD_READ_HIGH_PRECISION;
    uint8_t read_buffer[6] = {0};
    
    // Write addr
    if (i2c_write_dt(i2c_spec, &addr, sizeof(addr)) != 0) return -1;

    // Small wait
    k_msleep(9);

    if (i2c_read_dt(i2c_spec, read_buffer, sizeof(read_buffer)) != 0) return -2;

    *temperature = ((uint16_t)read_buffer[0] << 8) | read_buffer[1];
    *humidity = ((uint16_t)read_buffer[3] << 8) | read_buffer[4];

    return 0;
}

int sht41_get_temperature_and_humidity(float* temperature, float* humidity)
{
    uint16_t raw_temperature = 0;
    uint16_t raw_humidity = 0;
    if (sht41_get_raw_measurement(&raw_temperature, &raw_humidity) != 0) return -1;

    // Convert
    *temperature = -45.f + 175.f * ((float)raw_temperature / 65535.f);
    *humidity = -6.f + 125.f * ((float)raw_humidity / 65535.f);

    return 0;
}