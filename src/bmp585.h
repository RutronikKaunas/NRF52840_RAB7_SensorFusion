#include <zephyr/drivers/i2c.h>
#include <stdint.h>

void bmp585_set_i2c_spec(const struct i2c_dt_spec *spec);