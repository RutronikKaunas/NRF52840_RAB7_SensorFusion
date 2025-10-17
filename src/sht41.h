#include <zephyr/drivers/i2c.h>
#include <stdint.h>

void sht41_set_i2c_spec(const struct i2c_dt_spec *spec);

// Takes 1ms
int sht41_get_serial_number(uint32_t* serial_number);

// Takes 9ms
int sht41_get_raw_measurement(uint16_t* temperature, uint16_t* humidity);

// Takes 9ms
int sht41_get_temperature_and_humidity(float* temperature, float* humidity);