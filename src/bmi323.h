#include <zephyr/drivers/i2c.h>
#include <stdint.h>

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
} bmi323_values_t;

void bmi323_set_i2c_spec(const struct i2c_dt_spec *spec);

int bmi323_get_chip_id(uint8_t* chip_id);

int bmi323_do_configuration();

int bmi323_get_data(bmi323_values_t* values);