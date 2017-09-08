/*
 * sgpxx.c - Support for Sensirion SGP Gas Sensors
 *
 * Copyright (C) 2017 Andreas Brauchli <andreas.brauchli@sensirion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

/* Sensirion SGP30 Preliminary Datasheet:
 * https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/
 * Dokumente/9_Gas_Sensors/Sensirion_Gas_Sensors_SGP_Preliminary_Datasheet_EN.pdf
 */

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define SGP_WORD_LEN			2
#define SGP_CRC8_POLYNOMIAL		0x31
#define SGP_CRC8_INIT			0xff
#define SGP_CRC8_LEN			1
#define SGP_CMD(cmd_word)		cpu_to_be16(cmd_word)
#define SGP_CMD_DURATION_US		10000
#define SGP_CMD_LEN			SGP_WORD_LEN
#define SGP_MEASUREMENT_LEN		6


DECLARE_CRC8_TABLE(sgp_crc8_table);

enum sgp_channel_idx {
	SGP_IAQ_TVOC_IDX,
	SGP_IAQ_CO2EQ_IDX,
	SGP_SIG_ETOH_IDX,
	SGP_SIG_H2_IDX,
};

enum sgp_cmd {
	SGP_CMD_IAQ_INIT		= SGP_CMD(0x2003),
	SGP_CMD_GET_BASELINE		= SGP_CMD(0x2015),
	SGP_CMD_SET_BASELINE		= SGP_CMD(0x201e),
	SGP_CMD_GET_FEATURE_SET		= SGP_CMD(0x202f),
	SGP_CMD_MEASURE_TEST		= SGP_CMD(0x2032),

	SGP30_CMD_SET_ABSOLUTE_HUMIDITY = SGP_CMD(0x2061),
};

struct sgp_crc_word {
	__be16 value;
	u8 crc8;
} __attribute__((__packed__));

struct sgp_iaq_reading {
	struct sgp_crc_word tvoc_ppb;
	struct sgp_crc_word co2eq_ppm;
} __attribute__((__packed__));

struct sgp_gas_signal_reading {
	struct sgp_crc_word etoh_scaled_ticks;
	struct sgp_crc_word h2_scaled_ticks;
} __attribute__((__packed__));

union sgp_reading {
	u8 start;
	struct sgp_crc_word raw_word;
	struct sgp_iaq_reading iaq_reading;
	struct sgp_gas_signal_reading gas_signal_reading;
};

struct sgp_data {
	struct i2c_client *client;
	struct mutex data_lock;
	struct mutex i2c_lock;
	unsigned long last_update;

	union sgp_reading buffer;
	struct iio_buffer_setup_ops setup_ops;
	u16 feature_set;
};

static const struct iio_chan_spec sgp_channels[] = {
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_VOC,
		.datasheet_name = "TVOC signal",
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGP_IAQ_TVOC_IDX,
	},
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_CO2,
		.datasheet_name = "CO2eq signal",
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGP_IAQ_CO2EQ_IDX,
	},
	{
		.type = IIO_CONCENTRATION, /* IIO_CONCENTRATION_RATIO */
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					BIT(IIO_CHAN_INFO_SCALE),
		.address = SGP_SIG_ETOH_IDX,
		.extend_name = "ethanol",
		.datasheet_name = "Ethanol signal",
		.scan_index = 0,
		.scan_type = {
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_CONCENTRATION, /* IIO_CONCENTRATION_RATIO */
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					BIT(IIO_CHAN_INFO_SCALE),
		.address = SGP_SIG_H2_IDX,
		.extend_name = "h2",
		.datasheet_name = "H2 signal",
		.scan_index = 1,
		.scan_type = {
			.endianness = IIO_BE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static ssize_t sgp_selftest_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	//struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	strncpy(buf, "OK", 3);
	return 3;
}

static IIO_CONST_ATTR(in_gas_signal_ratio_scale, "0.001953125"); /* 1/512 */
static IIO_DEVICE_ATTR(in_selftest, S_IRUGO, sgp_selftest_show,
			NULL, 0);

static struct attribute *sgp_attributes[] = {
	&iio_dev_attr_in_selftest.dev_attr.attr,
	&iio_const_attr_in_gas_signal_ratio_scale.dev_attr.attr,
	NULL
};

static const struct attribute_group sgp_attr_group = {
	.attrs = sgp_attributes,
};

/**
 * sgp_i2c_read() - reads data from SGP sensor
 *
 * Note that since this function uses data->buffer, data->data_lock must be held
 * by the caller throughout the duration of the call.
 *
 * @client:     I2C client device
 * @data:       SGP data
 * @word_buf:   Word buffer to store the read data
 * @word_count: Num words to read, excluding CRC bytes
 *
 * Return:      0 on success, negative error code otherwise
 */
static int sgp_i2c_read(struct i2c_client *client,
					struct sgp_data *data,
					u16 *word_buf,
					size_t word_count)
{
	int ret;
	int i, w;
	size_t size;
	u8 crc;
	u8 *data_buf = &data->buffer.start;

	size = word_count * (SGP_WORD_LEN + SGP_CRC8_LEN);
	mutex_lock(&data->i2c_lock);
	ret = i2c_master_recv(client, data_buf, size);
	if (ret < 0) {
		ret = -ETXTBSY;
		goto fail_unlock;
	}
	if (ret != size) {
		ret = -EINTR;
		goto fail_unlock;
	}
	mutex_unlock(&data->i2c_lock);

	for (i = 0, w = 0; i < size;
		i += SGP_WORD_LEN + SGP_CRC8_LEN, w += 1)
	{
		crc = crc8(sgp_crc8_table, &data_buf[i],
				SGP_WORD_LEN,
				SGP_CRC8_INIT);
		if (crc != data_buf[i + SGP_WORD_LEN]) {
			dev_err(&client->dev, "CRC error\n");
			return -EIO;
		}

		word_buf[w] = be16_to_cpup((__be16 *) &data_buf[i]);
	}

	return 0;

fail_unlock:
    mutex_unlock(&data->i2c_lock);
    return ret;
}

/**
 * sgp_i2c_read_from_cmd() - reads data from SGP sensor after issuing a command
 * @client:     I2C client device
 * @data:       SGP data
 * @cmd:        SGP Command to issue
 * @buf:        Buffer to store the read data
 * @word_count: Num words to read, excluding CRC bytes
 *
 * Return:      0 on success, negative error otherwise.
 */
static int sgp_read_from_cmd(struct i2c_client *client,
					struct sgp_data *data,
					enum sgp_cmd cmd,
					u16 *word_buf,
					size_t word_count)
{
	int ret;

	mutex_lock(&data->i2c_lock);
	ret = i2c_master_send(client, (const char *) &cmd,
				SGP_CMD_LEN);
	if (ret != SGP_CMD_LEN) {
		mutex_unlock(&data->i2c_lock);
		return -EIO;
	}
	/* Wait inside lock to ensure the chip is ready before next command */
	usleep_range(SGP_CMD_DURATION_US,
			SGP_CMD_DURATION_US + 1000);
	mutex_unlock(&data->i2c_lock);

	mutex_lock(&data->data_lock);
	ret = sgp_i2c_read(client, data, word_buf, word_count);
	mutex_unlock(&data->data_lock);

	return ret;
}


static int sgp_read_measurement(struct sgp_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = client->flags | I2C_M_RD,
		.len = SGP_MEASUREMENT_LEN,
		.buf = (char *) &data->buffer,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	return (ret == SGP_MEASUREMENT_LEN) ? 0 : ret;
}

static int sgp_preenable_buffer(struct iio_dev *indio_dev)
{
	// Run IAQ_INIT if needed
	dev_err(&indio_dev->dev, "preenable\n");
	return 0;
}

static int sgp_get_measurement(struct sgp_data *data)
{
	int ret;

	/* sensor can only be polled once a second max per datasheet */
	if (!time_after(jiffies, data->last_update + HZ))
		return 0;

	ret = sgp_read_measurement(data);
	if (ret < 0)
		return ret;

	data->last_update = jiffies;

	return 0;
}

static int sgp_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan, int *val,
			int *val2, long mask)
{
	struct sgp_data *data = iio_priv(indio_dev);
	int ret;

	if (mask != IIO_CHAN_INFO_PROCESSED)
		return -EINVAL;

	mutex_lock(&data->data_lock);
	dev_err(&indio_dev->dev, "sgp_read_raw: skipping measurement...\n");
	// ret = sgp_get_measurement(data);

	// if (ret)
	// 	goto err_out;

	switch (chan->address) {
	case SGP_IAQ_TVOC_IDX:
		dev_err(&indio_dev->dev, "reading tvoc\n");
		*val = 0;
		*val2 = be16_to_cpu(data->buffer.iaq_reading.tvoc_ppb.value);
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	case SGP_IAQ_CO2EQ_IDX:
		dev_err(&indio_dev->dev, "reading tvoc\n");
		*val = 0;
		*val2 = be16_to_cpu(data->buffer.iaq_reading.co2eq_ppm.value);
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	case SGP_SIG_ETOH_IDX:
		dev_err(&indio_dev->dev, "reading ethanol\n");
		*val = 0;
		break;
	case SGP_SIG_H2_IDX:
		dev_err(&indio_dev->dev, "reading h2\n");
		*val = 0;
		break;
	default:
		ret = -EINVAL;
	}

err_out:
	mutex_unlock(&data->data_lock);

	return ret;
}

static const struct iio_info sgp_info = {
	.attrs		= &sgp_attr_group,
	.driver_module	= THIS_MODULE,
	.read_raw	= sgp_read_raw,
};

static int sgp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct sgp_data *data;
	int ret;

	dev_err(&client->dev, "probe\n");

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	crc8_populate_msb(sgp_crc8_table, SGP_CRC8_POLYNOMIAL);
	mutex_init(&data->data_lock);
	mutex_init(&data->i2c_lock);

	ret = sgp_read_from_cmd(client, data, SGP_CMD_GET_FEATURE_SET,
				&data->feature_set, 1);
	if (ret != 0)
		return ret;

	// TODO: Check feature set compatibility

	/* so initial reading will complete */
	data->last_update = jiffies - HZ;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &sgp_info;
	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = sgp_channels;
	indio_dev->num_channels = ARRAY_SIZE(sgp_channels);

	data->setup_ops = (struct iio_buffer_setup_ops) {
		.preenable = sgp_preenable_buffer
	};
	indio_dev->setup_ops = &data->setup_ops;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id sgp_id[] = {
	{ "sgpxx", 0 },
	{ "sgp30", 0 },
	{ "sgpc3", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sgp_id);

static const struct of_device_id sgp_dt_ids[] = {
	{ .compatible = "sensirion,sgpxx" },
	{ .compatible = "sensirion,sgpc3" },
	{ .compatible = "sensirion,sgpc3" },
	{ }
};
MODULE_DEVICE_TABLE(of, sgp_dt_ids);

static struct i2c_driver sgp_driver = {
	.driver = {
		.name	= "sgpxx",
		.of_match_table = of_match_ptr(sgp_dt_ids),
	},
	.probe = sgp_probe,
	.id_table = sgp_id,
};
module_i2c_driver(sgp_driver);

MODULE_AUTHOR("Andreas Brauchli <andreas.brauchli@sensirion.com>");
MODULE_DESCRIPTION("Sensirion SGPxx gas sensors");
MODULE_LICENSE("GPL v2");
