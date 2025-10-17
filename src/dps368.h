#include <zephyr/drivers/i2c.h>
#include <stdint.h>

void dps368_set_i2c_spec(const struct i2c_dt_spec *spec);

int dps368_init();

int dps368_get_values(float* pressure, float* temperature);