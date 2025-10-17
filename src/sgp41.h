#include <zephyr/drivers/i2c.h>
#include <stdint.h>

#define SGP41_SERIAL_NUMBER_LENGTH 6

#define SGP41_DEFAULT_RH		0x8000
#define SGP41_DEFAULT_T			0x6666

typedef struct
{
    uint8_t data[SGP41_SERIAL_NUMBER_LENGTH];
} sgp41_serial_number_t;


void sgp41_set_i2c_spec(const struct i2c_dt_spec *spec);

int sgp41_get_serial_number(sgp41_serial_number_t* serial_number);

int sgp41_execute_conditioning(uint16_t humidity, uint16_t temperature);

/**
 * @brief Start a raw signal measurement
 * You can then call sgp41_read_raw_signals after 45 to 50ms
 * 
 * @retval 0 Success
 */
int sgp41_measure_raw_signals(uint16_t humidity, uint16_t temperature);

int sgp41_read_raw_signals(uint16_t* voc, uint16_t* nox);