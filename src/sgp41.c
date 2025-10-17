#include "sgp41.h"

static const struct i2c_dt_spec *i2c_spec;

#define SGP41_WORD_SIZE 2

static uint8_t crc(uint8_t data[2]) 
{
    uint8_t crc = 0xFF;
    for(int i = 0; i < 2; i++) {
        crc ^= data[i];
        for(uint8_t bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31u;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

static uint16_t add_command_to_buffer(uint8_t* buffer, uint16_t offset, uint16_t command)
{
    buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
    buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
    return offset;
}

static uint16_t add_data_to_buffer(uint8_t* buffer, uint16_t offset, uint16_t data)
{
    buffer[offset++] = (uint8_t)((data & 0xFF00) >> 8);
    buffer[offset++] = (uint8_t)((data & 0x00FF) >> 0);
    buffer[offset] = crc(&buffer[offset - SGP41_WORD_SIZE]);
    offset++;
    return offset;
}

static uint16_t bytes_to_uint16_t(uint8_t* bytes)
{
    return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

void sgp41_set_i2c_spec(const struct i2c_dt_spec *spec)
{
    i2c_spec = spec;
}

int sgp41_get_serial_number(sgp41_serial_number_t* serial_number)
{
    uint8_t buffer[9] = {0};
    
    uint16_t cmd_length = add_command_to_buffer(buffer, 0, 0x3682);

    int ret = i2c_write_dt(i2c_spec, buffer, cmd_length);
	if(ret != 0) return -1;

    ret = i2c_read_dt(i2c_spec, buffer, sizeof(buffer));
	if(ret != 0) return -2;

    // Check that the CRC matches
    uint8_t expected_crc = crc(&buffer[0]);
    if (expected_crc != buffer[2]) return -3;
    
    expected_crc = crc(&buffer[3]);
    if (expected_crc != buffer[5]) return -4;

    expected_crc = crc(&buffer[6]);
    if (expected_crc != buffer[8]) return -5;

    // Copy
    serial_number->data[0] = buffer[0];
    serial_number->data[1] = buffer[1];
    serial_number->data[2] = buffer[3];
    serial_number->data[3] = buffer[4];
    serial_number->data[4] = buffer[6];
    serial_number->data[5] = buffer[7];

    return 0;
}

int sgp41_execute_conditioning(uint16_t humidity, uint16_t temperature)
{
    uint8_t buffer[8] = {0};
    uint16_t offset = add_command_to_buffer(buffer, 0, 0x2612);
    offset = add_data_to_buffer(buffer, offset, humidity);
    offset = add_data_to_buffer(buffer, offset, temperature);

    int ret = i2c_write_dt(i2c_spec, buffer, offset);
	if(ret != 0) return -1;

    return 0;
}

int sgp41_measure_raw_signals(uint16_t humidity, uint16_t temperature)
{
    uint8_t buffer[8] = {0};
    uint16_t offset = add_command_to_buffer(buffer, 0, 0x2619);
    offset = add_data_to_buffer(buffer, offset, humidity);
    offset = add_data_to_buffer(buffer, offset, temperature);

    int ret = i2c_write_dt(i2c_spec, buffer, offset);
	if(ret != 0) return -1;

    return 0;
}

int sgp41_read_raw_signals(uint16_t* voc, uint16_t* nox)
{
    uint8_t buffer[6] = {0};

    int ret = i2c_read_dt(i2c_spec, buffer, sizeof(buffer));
	if(ret != 0) return -1;

    // Check that the CRC matches
    uint8_t expected_crc = crc(&buffer[0]);
    if (expected_crc != buffer[2]) return -2;
    
    expected_crc = crc(&buffer[3]);
    if (expected_crc != buffer[5]) return -3;

    // Convertbytes_to_uint16_t    
    *voc = bytes_to_uint16_t(&buffer[0]);
    *nox = bytes_to_uint16_t(&buffer[3]);

    return 0;
}