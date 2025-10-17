/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/drivers/i2c.h>

#include "sgp41.h"
#include "dps368.h"
#include "bmi323.h"
#include "sht41.h"
#include "bmp585/bmp_driver.h"
#include "bmp585/bmp585_app.h"
#include "bme690/bme_driver.h"
#include "bme690/bme690_app.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/drivers/sensor.h>

#include "sensirion_gas_index_algorithm.h"

/**
 * Service UUID (same as RDK3)
 */
#define BT_UUID_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x76B6758B, 0xFEDC, 0x45F0, 0xA5A4, 0x4F49559BD71D)

#define BT_UUID_CHAR_VAL \
	BT_UUID_128_ENCODE(0x9DC72F3C, 0x9623, 0x479B, 0xB2E3, 0xCA83316A0A0F)

#define BT_UUID_SERVICE	BT_UUID_DECLARE_128(BT_UUID_SERVICE_VAL)
#define BT_UUID_CHAR	BT_UUID_DECLARE_128(BT_UUID_CHAR_VAL)

static bool notify_mysensor_enabled = false;

static ssize_t write_char(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags);

/* STEP 13 - Define the configuration change callback function for the MYSENSOR characteristic */
static void mylbsbc_ccc_mysensor_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_mysensor_enabled = (value == BT_GATT_CCC_NOTIFY);

	printk("Notify enabled: %d \n", (int)notify_mysensor_enabled);
}

BT_GATT_SERVICE_DEFINE(my_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_SERVICE),
				BT_GATT_CHARACTERISTIC(BT_UUID_CHAR, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_WRITE, NULL, write_char, NULL),
				BT_GATT_CCC(mylbsbc_ccc_mysensor_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static ssize_t write_char(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	printk("write_char - len is %d \n", len);

	uint8_t cmd = *((uint8_t*) buf);

	printk("CMD is : %d \n", (int)cmd);

	/*const char *value = attr->user_data;

	some_data = some_data + 1;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));*/

	if (cmd == 0)
	{
		uint32_t sensors = (1 << 6); // RAB7
		int retval = bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], &sensors, sizeof(sensors));
		printk("Write char: %d \n", retval);
	}

	return len;
}

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

// Get access to SGP41 sensor
#define I2C_SGP41_NODE DT_NODELABEL(sgp41sensor)

// Get access to the DPS368 sensor
#define I2C_DPS368_NODE DT_NODELABEL(dps368sensor)

// Get access to the BMI323 sensor
#define I2C_BMI323_NODE DT_NODELABEL(bmi323sensor)

// Get access to the SHT41 sensor
#define I2C_SHT41_NODE DT_NODELABEL(sht41sensor)

// Get access to the BMP585
#define I2C_BMP585_NODE DT_NODELABEL(bmp585sensor)

// Get access to the BME690
#define I2C_BME690_NODE DT_NODELABEL(bme690sensor)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// BT initialization handler
static void bt_ready(int err) 
{
	printf("bt_ready: %d\n", err);
    if (err) {
        return;
    }
}

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

/**
 * Advertising data
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/**
 * Scan response packet
 */

static unsigned char url_data[] ={0x17,'/','/','r','u','t','r','o','n','i','k','.','c','o','m'};

/*static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b,
                  0x86, 0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
};*/
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_URI, url_data,sizeof(url_data)),
};

struct bt_conn *my_conn;

static void connected(struct bt_conn *conn, uint8_t err) {
    printk("BLE connected: err = %d\n", err);

	if(err){
		printk("Error, return...\n");
		return;
	}

	my_conn = bt_conn_ref(conn);

	// gpio_pin_set_dt(&led_ble_connection, 1);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("BLE disconnected: reason = %d\n", reason);

	bt_conn_unref(my_conn);

	// gpio_pin_set_dt(&led_ble_connection, 0);
}

void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval*1.25;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms
    printk("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms\n", connection_interval, latency, supervision_timeout);
}

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM((BT_LE_ADV_OPT_CONN|BT_LE_ADV_OPT_USE_IDENTITY), 
                800, /*Min Advertising Interval 500ms (800*0.625ms) */
                801, /*Max Advertising Interval 500.625ms (801*0.625ms)*/
                NULL); /* Set to NULL for undirected advertising*/

static struct k_work adv_work;

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}
static void recycled_cb(void)
{
	printk("Connection object available from previous conn. Disconnect is complete!\n");
	advertising_start();
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
	.le_param_updated = on_le_param_updated,
	.recycled = recycled_cb
};

static void notify_sgp41_value(uint16_t voc_raw, uint16_t nox_raw, int32_t voc_index, int32_t nox_index)
{
	uint8_t data_packet[16] = {0};
	const uint16_t sensor_id = 0x10;
	const uint8_t data_size = 12;
	const uint8_t notification_size = data_size + 4;
	
	data_packet[0] = (uint8_t) (sensor_id & 0xFF);
	data_packet[1] = (uint8_t) (sensor_id >> 8);

	data_packet[2] = data_size;

	*((uint16_t*) &data_packet[3]) = voc_raw;
	*((uint16_t*) &data_packet[5]) = nox_raw;
	*((int32_t*) &data_packet[7]) = voc_index;
	*((int32_t*) &data_packet[11]) = nox_index;

	data_packet[notification_size - 1] = 0x3;

	if (notify_mysensor_enabled)
	{
		bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], data_packet, sizeof(data_packet));
	}
}

static void notify_bmm350_value(float temperature, float mag_x, float mag_y, float mag_z)
{
	uint8_t data_packet[20] = {0};
	const uint16_t sensor_id = 0x11;
	const uint8_t data_size = 16;
	const uint8_t notification_size = data_size + 4;
	
	data_packet[0] = (uint8_t) (sensor_id & 0xFF);
	data_packet[1] = (uint8_t) (sensor_id >> 8);

	data_packet[2] = data_size;

	*((float*) &data_packet[3]) = temperature;
	*((float*) &data_packet[7]) = mag_x;
	*((float*) &data_packet[11]) = mag_y;
	*((float*) &data_packet[15]) = mag_z;

	data_packet[notification_size - 1] = 0x3;

	if (notify_mysensor_enabled)
	{
		bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], data_packet, sizeof(data_packet));
	}
}

static void notify_dps368_value(float pressure, float temperature)
{
	uint8_t data_packet[12] = {0};
	const uint16_t sensor_id = 0x14;
	const uint8_t data_size = 8;
	const uint8_t notification_size = data_size + 4;
	
	data_packet[0] = (uint8_t) (sensor_id & 0xFF);
	data_packet[1] = (uint8_t) (sensor_id >> 8);

	data_packet[2] = data_size;

	*((float*) &data_packet[3]) = pressure;
	*((float*) &data_packet[7]) = temperature;

	data_packet[notification_size - 1] = 0x3;

	if (notify_mysensor_enabled)
	{
		bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], data_packet, sizeof(data_packet));
	}
}

static void notify_bmi323_value(bmi323_values_t value)
{
	uint8_t data_packet[16] = {0};
	const uint16_t sensor_id = 0x15;
	const uint8_t data_size = 12;
	const uint8_t notification_size = data_size + 4;
	
	data_packet[0] = (uint8_t) (sensor_id & 0xFF);
	data_packet[1] = (uint8_t) (sensor_id >> 8);

	data_packet[2] = data_size;

	*((int16_t*) &data_packet[3]) = value.acc_x;
	*((int16_t*) &data_packet[5]) = value.acc_y;
	*((int16_t*) &data_packet[7]) = value.acc_z;
	*((int16_t*) &data_packet[9]) = value.gyr_x;
	*((int16_t*) &data_packet[11]) = value.gyr_y;
	*((int16_t*) &data_packet[13]) = value.gyr_z;

	data_packet[notification_size - 1] = 0x3;

	if (notify_mysensor_enabled)
	{
		bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], data_packet, sizeof(data_packet));
	}
}

static void notify_sht41_value(float temperature, float humidity)
{
	uint8_t data_packet[12] = {0};
	const uint16_t sensor_id = 0x1;
	const uint8_t data_size = 8;
	const uint8_t notification_size = data_size + 4;
	
	data_packet[0] = (uint8_t) (sensor_id & 0xFF);
	data_packet[1] = (uint8_t) (sensor_id >> 8);

	data_packet[2] = data_size;

	*((float*) &data_packet[3]) = temperature;
	*((float*) &data_packet[7]) = humidity;

	data_packet[notification_size - 1] = 0x3;

	if (notify_mysensor_enabled)
	{
		bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], data_packet, sizeof(data_packet));
	}
}


static void notify_bmp585_value(float pressure, float temperature)
{
	uint8_t data_packet[12] = {0};
	const uint16_t sensor_id = 0x13;
	const uint8_t data_size = 8;
	const uint8_t notification_size = data_size + 4;
	
	data_packet[0] = (uint8_t) (sensor_id & 0xFF);
	data_packet[1] = (uint8_t) (sensor_id >> 8);

	data_packet[2] = data_size;

	*((float*) &data_packet[3]) = pressure;
	*((float*) &data_packet[7]) = temperature;

	data_packet[notification_size - 1] = 0x3;

	if (notify_mysensor_enabled)
	{
		bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], data_packet, sizeof(data_packet));
	}
}

static void notify_bme690_value(bme69x_data_t* bme_data)
{
	uint8_t data_packet[26] = {0};
	const uint16_t sensor_id = 0x12;
	const uint8_t data_size = 22;
	const uint8_t notification_size = data_size + 4;
	
	data_packet[0] = (uint8_t) (sensor_id & 0xFF);
	data_packet[1] = (uint8_t) (sensor_id >> 8);

	data_packet[2] = data_size;

	data_packet[3] = bme_data->status;
	data_packet[4] = bme_data->gas_index;
	data_packet[5] = bme_data->meas_index;
	data_packet[6] = bme_data->res_heat;
	data_packet[7] = bme_data->idac;
	data_packet[8] = bme_data->gas_wait;

	*((float*) &data_packet[9]) = bme_data->temperature;
	*((float*) &data_packet[13]) = bme_data->pressure;
	*((float*) &data_packet[17]) = bme_data->humidity;
	*((float*) &data_packet[21]) = bme_data->gas_resistance;

	data_packet[notification_size - 1] = 0x3;

	if (notify_mysensor_enabled)
	{
		bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], data_packet, sizeof(data_packet));
	}
}


const struct device *bmm350_dev = DEVICE_DT_GET_ANY(bosch_bmm350);

static struct sensor_value simple;

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	printf("Test\n");

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	// BLE stuff
	int ble_result = bt_enable(bt_ready);
    printk("bt_enable: %d \n", ble_result);

	ble_result = bt_conn_cb_register(&conn_callbacks);
	printk("bt_conn_cb_register: %d \n", ble_result);

	//ble_result = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	// printk("bt_le_adv_start: %d \n", ble_result);
	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	if (!device_is_ready(bmm350_dev))
	{
		printf("BMM350 is not ready...\n");
		return 0;
	}

	ret = sensor_sample_fetch(bmm350_dev);
	if (ret != 0)
	{
		printf("Cannot sensor_sample_fetch\n");
	}

	ret = sensor_channel_get(bmm350_dev, SENSOR_CHAN_MAGN_X, &simple);
	printf("ret = %d Value x: %f\n", ret, sensor_value_to_double(&simple));

	ret = sensor_channel_get(bmm350_dev, SENSOR_CHAN_MAGN_Y, &simple);
	printf("ret = %d Value y: %f\n", ret, sensor_value_to_double(&simple));

	ret = sensor_channel_get(bmm350_dev, SENSOR_CHAN_MAGN_Z, &simple);
	printf("ret = %d Value z: %f\n", ret, sensor_value_to_double(&simple));

	// I2C stuff
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_SGP41_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printf("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return 0;
	}
	printf("SGP41 I2C Initialized\n");
	sgp41_set_i2c_spec(&dev_i2c);

	static const struct i2c_dt_spec dev_i2c_dps368 = I2C_DT_SPEC_GET(I2C_DPS368_NODE);
	if (!device_is_ready(dev_i2c_dps368.bus)) {
		printf("I2C bus %s is not ready!\n\r",dev_i2c_dps368.bus->name);
		return 0;
	}
	dps368_set_i2c_spec(&dev_i2c_dps368);

	printf("dsp368 init: %d\n", dps368_init());


	// BMI 323
	static const struct i2c_dt_spec dev_i2c_bmi323 = I2C_DT_SPEC_GET(I2C_BMI323_NODE);
	if (!device_is_ready(dev_i2c_bmi323.bus)) {
		printf("I2C bus %s is not ready!\n\r",dev_i2c_bmi323.bus->name);
		return 0;
	}
	bmi323_set_i2c_spec(&dev_i2c_bmi323);

	uint8_t bmi323_chip = 0;
	if (bmi323_get_chip_id(&bmi323_chip) != 0)
	{
		printf("Bmi323 cannot get id\n");
		return 0;
	}
	printf("Bmi323 id: %d \n", bmi323_chip);

	if (bmi323_do_configuration() != 0)
	{
		printf("Bmi323 cannot do configuration\n");
		return 0;
	}


	// SHT41
	static const struct i2c_dt_spec dev_i2c_sht41 = I2C_DT_SPEC_GET(I2C_SHT41_NODE);
	if (!device_is_ready(dev_i2c_sht41.bus)) {
		printf("I2C bus %s is not ready!\n\r",dev_i2c_sht41.bus->name);
		return 0;
	}
	sht41_set_i2c_spec(&dev_i2c_sht41);

	uint32_t sht41_serial = 0;
	if (sht41_get_serial_number(&sht41_serial) != 0)
	{
		printf("Error getting sht41 serial number\n");
		return 0;
	}

	printf("Serial number is :0x%x\n", sht41_serial);

	// BMP585
	static const struct i2c_dt_spec dev_i2c_bmp585 = I2C_DT_SPEC_GET(I2C_BMP585_NODE);
	if (!device_is_ready(dev_i2c_bmp585.bus)) {
		printf("I2C bus %s is not ready!\n\r",dev_i2c_bmp585.bus->name);
		return 0;
	}
	bmp585_set_i2c_spec(&dev_i2c_bmp585);

	if (bmp585_app_init() != 0)
	{
		printf("Cannot init bmp585 app\n");
		return 0;
	}

	// BME690
	static const struct i2c_dt_spec dev_i2c_bme690 = I2C_DT_SPEC_GET(I2C_BME690_NODE);
	if (!device_is_ready(dev_i2c_bme690.bus)) {
		printf("I2C bus %s is not ready!\n\r",dev_i2c_bme690.bus->name);
		return 0;
	}
	bme690_set_i2c_spec(&dev_i2c_bme690);

	if (bme690_app_init() != 0)
	{
		printf("Cannot init bme690 app\n");
		return 0;	
	}


	// SGP41
	sgp41_serial_number_t serial_number;
	ret = sgp41_get_serial_number(&serial_number);
	if (ret != 0){
		printf("Cannot get serial: %d \n", ret);
		return 0;
	}

	printf("Serial: %d %d %d %d %d %d\n",
		serial_number.data[0],
		serial_number.data[1],
		serial_number.data[2],
		serial_number.data[3],
		serial_number.data[4],
		serial_number.data[5]);

	// Start conditioning
	ret = sgp41_execute_conditioning(SGP41_DEFAULT_RH, SGP41_DEFAULT_T);
	if (ret != 0){
		printf("Cannot do conditioning: %d \n", ret);
		return 0;
	}

	printf("Conditioning done\n");

	k_msleep(500);

	int counter = 0;

	float comp_temperature = 25;
	float comp_humidity = 50;

	GasIndexAlgorithmParams gas_index_voc_params;
	GasIndexAlgorithmParams gas_index_nox_params;

	GasIndexAlgorithm_init(&gas_index_voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
	GasIndexAlgorithm_init(&gas_index_nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

	// Do measurement
	for(;;)
	{
		if (counter == 0)
		{
			uint16_t comp_rh = (uint16_t)comp_humidity * 65535 / 100;
			uint16_t comp_t = (uint16_t)(comp_temperature + 45) * 65535 / 175;

			ret = sgp41_measure_raw_signals(comp_rh, comp_t);
			if (ret != 0){
				printf("Cannot do measurement: %d \n", ret);
				return 0;
			}
		}

		ret = sensor_sample_fetch(bmm350_dev);
		if (ret != 0)
		{
			printf("Cannot sensor_sample_fetch\n");
			return 0;
		}

		k_msleep(100);

		if (counter == 0)
		{
			uint16_t nox = 0;
			uint16_t voc = 0;

			ret = sgp41_read_raw_signals(&voc, & nox);
			if (ret != 0){
				printf("Cannot read measurement: %d \n", ret);
				return 0;
			}

			int32_t voc_index = 0;
			int32_t nox_index = 0;

			GasIndexAlgorithm_process(&gas_index_voc_params, voc, &voc_index);
			GasIndexAlgorithm_process(&gas_index_nox_params, nox, &nox_index);

			notify_sgp41_value(voc, nox, voc_index, nox_index);

			float temperature = 0;
			float pressure = 0;
			ret = dps368_get_values(&pressure, &temperature);
			if (ret != 0){
				printf("dps368_get_values: %d \n", ret);
				return 0;
			}

			// convert to Pa
			pressure = pressure * 100;
			notify_dps368_value(pressure, temperature);

			float humidity = 0;
			ret = sht41_get_temperature_and_humidity(&temperature, &humidity);
			if (ret != 0)
			{
				printf("sht41 get value error: %d\n", ret);
				return 0;
			}

			notify_sht41_value(temperature, humidity);

			// Store for compensation of SGP41 values
			comp_humidity = humidity;
			comp_temperature = temperature;

			if (bmp585_read_data(&temperature, &pressure) != 0)
			{
				printf("CAnnot read bmp585 data\n");
				return 0;
			}

			notify_bmp585_value(pressure, temperature);
		}

		counter++;
		if (counter >= 9) counter = 0;

		float mag_x = 0;
		float mag_y = 0;
		float mag_z = 0;
		sensor_channel_get(bmm350_dev, SENSOR_CHAN_MAGN_X, &simple);
		mag_x = sensor_value_to_float(&simple);

		sensor_channel_get(bmm350_dev, SENSOR_CHAN_MAGN_Y, &simple);
		mag_y = sensor_value_to_float(&simple);

		sensor_channel_get(bmm350_dev, SENSOR_CHAN_MAGN_Z, &simple);
		mag_z = sensor_value_to_float(&simple);

		// printf("VOC = %d \t NOX = %d\n", voc, nox);
		
		notify_bmm350_value(0, mag_x, mag_y, mag_z);

		bmi323_values_t bmi323_values;
		if (bmi323_get_data(&bmi323_values) != 0)
		{
			printf("Bmi323 get data error\n");
			return 0;
		}
		notify_bmi323_value(bmi323_values);

		// Data available?
		uint8_t fields_count = 0;
		if (bme690_data_available(&fields_count) == 0)
		{
			for(uint8_t field_idx = 0; field_idx < fields_count; ++field_idx)
			{
				bme69x_data_t bme690_data = {0};
				if (bme690_data_read(&bme690_data, field_idx))
				{
					notify_bme690_value(&bme690_data);
				}
			}
		}
	}

/*
	// Get serial number
	uint8_t config[2] = {0x36,0x82};
	uint8_t serial_number[9] = {0};

	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printf("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,config[0]);
		return 0;
	}

	ret = i2c_read_dt(&dev_i2c, serial_number, sizeof(serial_number));
	if(ret != 0){
		printf("Failed to read from I2C device address %x at Reg. %x \n\r", dev_i2c.addr,config[0]);
		return 0;
	}


	// Execute conditioning
	uint8_t conditioning_cmd[2] = {0x26,0x12};


	// Start measurement


	for (int i = 0; i <9; ++i){
		printf("%d \n", serial_number[i]);
	}

*/

	for(;;){
		k_msleep(SLEEP_TIME_MS);
	}


	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);

	}
	return 0;
}
