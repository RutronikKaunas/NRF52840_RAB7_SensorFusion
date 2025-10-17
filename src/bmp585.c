#include "bmp585.h"

static const struct i2c_dt_spec *i2c_spec;

void bmp585_set_i2c_spec(const struct i2c_dt_spec *spec)
{
    i2c_spec = spec;
}