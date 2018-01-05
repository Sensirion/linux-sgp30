/*
 * sgp30.c - Support for Sensirion SGP Gas Sensors
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
 * Datasheets:
 * https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Sensirion_Gas_Sensors_SGP30_Datasheet_EN.pdf
 * https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Sensirion_Gas_Sensors_SGPC3_Datasheet_EN.pdf
 */

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/version.h>

#define SGP_WORD_LEN			2
#define SGP_CRC8_POLYNOMIAL		0x31
#define SGP_CRC8_INIT			0xff
#define SGP_CRC8_LEN			1
#define SGP_CMD(cmd_word)		cpu_to_be16(cmd_word)
#define SGP_CMD_DURATION_US		12000
#define SGP_MEASUREMENT_DURATION_US	50000
#define SGP_SELFTEST_DURATION_US	220000
#define SGP_CMD_HANDLING_DURATION_US	10000
#define SGP_CMD_LEN			SGP_WORD_LEN
#define SGP30_MEASUREMENT_LEN		2
#define SGPC3_MEASUREMENT_LEN		2
#define SGP30_MEASURE_INTERVAL_HZ	1
#define SGPC3_MEASURE_INTERVAL_HZ	2
#define SGP_SELFTEST_OK			0xd400

DECLARE_CRC8_TABLE(sgp_crc8_table);

enum sgp_product_id {
	SGP30 = 0,
	SGPC3
};

enum sgp30_channel_idx {
	SGP30_IAQ_TVOC_IDX = 0,
	SGP30_IAQ_CO2EQ_IDX,
	SGP30_SIG_ETOH_IDX,
	SGP30_SIG_H2_IDX,
};

enum sgpc3_channel_idx {
	SGPC3_IAQ_TVOC_IDX = 10,
	SGPC3_SIG_ETOH_IDX,
};

enum sgp_cmd {
	SGP_CMD_IAQ_INIT		= SGP_CMD(0x2003),
	SGP_CMD_IAQ_MEASURE		= SGP_CMD(0x2008),
	SGP_CMD_GET_BASELINE		= SGP_CMD(0x2015),
	SGP_CMD_SET_BASELINE		= SGP_CMD(0x201e),
	SGP_CMD_GET_FEATURE_SET		= SGP_CMD(0x202f),
	SGP_CMD_GET_SERIAL_ID		= SGP_CMD(0x3682),
	SGP_CMD_SELFTEST		= SGP_CMD(0x2032),

	SGP30_CMD_MEASURE_SIGNAL	= SGP_CMD(0x2050),
	SGP30_CMD_SET_ABSOLUTE_HUMIDITY = SGP_CMD(0x2061),

	SGPC3_CMD_IAQ_INIT0		= SGP_CMD(0x2089),
	SGPC3_CMD_IAQ_INIT16		= SGP_CMD(0x2024),
	SGPC3_CMD_IAQ_INIT64		= SGP_CMD(0x2003),
	SGPC3_CMD_IAQ_INIT184		= SGP_CMD(0x206a),
	SGPC3_CMD_MEASURE_RAW		= SGP_CMD(0x2046),
};

struct sgp_version {
	u8 major;
	u8 minor;
};

struct sgp_crc_word {
	__be16 value;
	u8 crc8;
} __attribute__((__packed__));

union sgp_reading {
	u8 start;
	struct sgp_crc_word raw_words[4];
};

struct sgp_data {
	struct i2c_client *client;
	struct task_struct *iaq_thread;
	struct mutex data_lock;
	unsigned long iaq_init_jiffies;
	unsigned long iaq_init_skip_jiffies;
	unsigned long last_update;
	u64 serial_id;
	u16 product_id;
	u16 feature_set;
	u16 measurement_len;
	int measure_interval_hz;
	enum sgp_cmd iaq_init_cmd;
	enum sgp_cmd last_cmd;
	enum sgp_cmd measure_iaq_cmd;
	enum sgp_cmd measure_signal_cmd;
	char *baseline_format;
	u8 baseline_len;
	u16 set_baseline[2];
	union sgp_reading buffer;
};

struct sgp_device {
	const struct iio_chan_spec *channels;
	int num_channels;
};

static const struct sgp_version supported_versions_sgp30[] = {
	{
		.major = 1,
		.minor = 0,
	}
};

static const struct sgp_version supported_versions_sgpc3[] = {
	{
		.major = 0,
		.minor = 4,
	}
};

static const struct iio_chan_spec sgp30_channels[] = {
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_VOC,
		.datasheet_name = "TVOC signal",
		.scan_index = 0,
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGP30_IAQ_TVOC_IDX,
	},
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_CO2,
		.datasheet_name = "CO2eq signal",
		.scan_index = 1,
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGP30_IAQ_CO2EQ_IDX,
	},
	{
		.type = IIO_CONCENTRATION,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.address = SGP30_SIG_ETOH_IDX,
		.extend_name = "ethanol",
		.datasheet_name = "Ethanol signal",
		.scan_index = 2,
		.scan_type = {
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_CONCENTRATION,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.address = SGP30_SIG_H2_IDX,
		.extend_name = "h2",
		.datasheet_name = "H2 signal",
		.scan_index = 3,
		.scan_type = {
			.endianness = IIO_BE,
		},
	},
};

static const struct iio_chan_spec sgpc3_channels[] = {
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_VOC,
		.datasheet_name = "TVOC signal",
		.scan_index = 0,
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGPC3_IAQ_TVOC_IDX,
	},
	{
		.type = IIO_CONCENTRATION,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.address = SGPC3_SIG_ETOH_IDX,
		.extend_name = "ethanol",
		.datasheet_name = "Ethanol signal",
		.scan_index = 1,
		.scan_type = {
			.endianness = IIO_BE,
		},
	},
};

static struct sgp_device sgp_devices[] = {
	[SGP30] = {
		.channels = sgp30_channels,
		.num_channels = ARRAY_SIZE(sgp30_channels),
	},
	[SGPC3] = {
		.channels = sgpc3_channels,
		.num_channels = ARRAY_SIZE(sgpc3_channels),
	},
};

/**
 * sgp_verify_buffer() - verify the checksums of the data buffer words
 *
 * @data:       SGP data containing the raw buffer
 * @word_count: Num data words stored in the buffer, excluding CRC bytes
 *
 * Return:      0 on success, negative error otherwise.
 */
static int sgp_verify_buffer(struct sgp_data *data, size_t word_count)
{
	size_t size = word_count * (SGP_WORD_LEN + SGP_CRC8_LEN);
	int i;
	u8 crc;
	u8 *data_buf = &data->buffer.start;

	for (i = 0; i < size; i += SGP_WORD_LEN + SGP_CRC8_LEN) {
		crc = crc8(sgp_crc8_table, &data_buf[i], SGP_WORD_LEN,
			   SGP_CRC8_INIT);
		if (crc != data_buf[i + SGP_WORD_LEN]) {
			dev_err(&data->client->dev, "CRC error\n");
			return -EIO;
		}
	}
	return 0;
}

/**
 * sgp_read_cmd() - reads data from sensor after issuing a command
 * The caller must hold data->data_lock for the duration of the call.
 * @data:        SGP data
 * @cmd:         SGP Command to issue
 * @word_count:  Num words to read, excluding CRC bytes
 *
 * Return:       0 on success, negative error otherwise.
 */
static int sgp_read_cmd(struct sgp_data *data, enum sgp_cmd cmd,
			size_t word_count, unsigned long duration_us)
{
	int ret;
	struct i2c_client *client = data->client;
	size_t size = word_count * (SGP_WORD_LEN + SGP_CRC8_LEN);
	u8 *data_buf = &data->buffer.start;

	ret = i2c_master_send(client, (const char *)&cmd, SGP_CMD_LEN);
	if (ret != SGP_CMD_LEN)
		return -EIO;
	usleep_range(duration_us, duration_us + 1000);

	if (word_count == 0)
		return 0;

	ret = i2c_master_recv(client, data_buf, size);

	if (ret < 0)
		ret = -ETXTBSY;
	else if (ret != size)
		ret = -EINTR;
	else
		ret = sgp_verify_buffer(data, word_count);

	return ret;
}

/**
 * sgp_i2c_write_cmd() - write data to SGP sensor with a command
 * @data:       SGP data
 * @cmd:        SGP Command to issue
 * @buf:        Data to write
 * @buf_size:   Data size of the buffer
 *
 * Return:      0 on success, negative error otherwise.
 */
static int sgp_write_cmd(struct sgp_data *data, enum sgp_cmd cmd, u16 *buf,
			 size_t buf_size, unsigned long duration_us)
{
	int ret, ix;
	u16 buf_idx = 0;
	u16 buffer_size = SGP_CMD_LEN + buf_size *
		(SGP_WORD_LEN + SGP_CRC8_LEN);
	u8 buffer[buffer_size];

	/* assemble buffer */
	*((u16 *)&buffer[0]) = cmd;
	buf_idx += SGP_CMD_LEN;
	for (ix = 0; ix < buf_size; ix++) {
		*((u16 *)&buffer[buf_idx]) = ntohs(buf[ix] & 0xffff);
		buf_idx += SGP_WORD_LEN;
		buffer[buf_idx] = crc8(sgp_crc8_table,
				       &buffer[buf_idx - SGP_WORD_LEN],
				       SGP_WORD_LEN, SGP_CRC8_INIT);
		buf_idx += SGP_CRC8_LEN;
	}
	ret = i2c_master_send(data->client, buffer, buffer_size);
	if (ret != buffer_size)
		return -EIO;
	ret = 0;
	usleep_range(duration_us, duration_us + 1000);

	return ret;
}

/**
 * sgp_get_measurement() - retrieve measurement result from sensor
 * The measurement is skipped if the last measurement is still valid.
 * The caller must hold data->data_lock for the duration of the call.
 * @data:       SGP data
 * @cmd:        SGP Command to issue
 *
 * Return:      0 on success, negative error otherwise.
 */
static int sgp_get_measurement(struct sgp_data *data, enum sgp_cmd cmd)
{
	int ret;

	/* Only measure if measure command changed or the data expired */
	if (data->last_cmd == cmd &&
	    !time_after(jiffies,
			data->last_update + data->measure_interval_hz * HZ)) {
		return 0;
	}

	ret = sgp_read_cmd(data, cmd, data->measurement_len,
			   SGP_MEASUREMENT_DURATION_US);
	if (ret < 0)
		return ret;

	if (cmd == data->measure_iaq_cmd &&
	    !time_after(jiffies,
			data->iaq_init_jiffies + data->iaq_init_skip_jiffies)) {
		/* data contains default values */
		return -EBUSY;
	}

	data->last_cmd = cmd;
	data->last_update = jiffies;

	return 0;
}

static int sgp_iaq_threadfn(void *p)
{
	struct sgp_data *data = (struct sgp_data *)p;
	long intv_low = data->measure_interval_hz * 1000000;
	long intv_high = intv_low + 1000;
	int ret;
	bool expect_busy;

	while(!kthread_should_stop()) {
		mutex_lock(&data->data_lock);
		if (data->iaq_init_jiffies == 0) {
			ret = sgp_read_cmd(data, data->iaq_init_cmd, 0,
					   SGP_CMD_DURATION_US);
			if (ret < 0)
				goto unlock_sleep_continue;
			data->iaq_init_jiffies = jiffies;
			if (data->set_baseline) {
				ret = sgp_write_cmd(data, SGP_CMD_SET_BASELINE,
						    data->set_baseline,
						    data->baseline_len,
						    SGP_CMD_DURATION_US);
				if (ret < 0)
					goto unlock_sleep_continue;
			}
		}

		expect_busy = !time_after(jiffies, data->iaq_init_jiffies +
						   data->iaq_init_skip_jiffies);
		ret = sgp_get_measurement(data, data->measure_iaq_cmd);
		if (ret && !(expect_busy && ret == -EBUSY)) {
			dev_warn(&data->client->dev, "measurement error [%d]\n",
				 ret);
		}
unlock_sleep_continue:
		mutex_unlock(&data->data_lock);
		usleep_range(intv_low, intv_high);
	}
	return 0;
}

static ssize_t sgp_absolute_humidity_store(struct device *dev,
					   __attribute__((unused))struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	u32 ah;
	u16 ah_scaled;
	int ret;

	ret = sscanf(buf, "%u", &ah);
	if (ret != 1 || ah < 0 || ah > 256000)
		return -EINVAL;

	/* ah_scaled = (u16)((ah / 1000.0) * 256.0) */
	ah_scaled = (u16)(((u64)ah * 256 * 16777) >> 24);

	/* don't disable AH compensation due to rounding */
	if (ah > 0 && ah_scaled == 0)
		ah_scaled = 1;

	ret = sgp_write_cmd(data, SGP30_CMD_SET_ABSOLUTE_HUMIDITY,
			    &ah_scaled, 1, SGP_CMD_DURATION_US);
	if (ret)
		return ret;

	return count;
}

static int sgp_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan, int *val,
			int *val2, long mask)
{
	struct sgp_data *data = iio_priv(indio_dev);
	struct sgp_crc_word *words;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&data->data_lock);
		if (data->iaq_init_jiffies == 0) {
			dev_warn(&data->client->dev,
				 "IAQ potentially uninitialized\n");
		}
		ret = sgp_get_measurement(data, data->measure_iaq_cmd);
		if (ret)
			goto unlock_fail;
		words = data->buffer.raw_words;
		switch (chan->address) {
		case SGP30_IAQ_TVOC_IDX:
		case SGPC3_IAQ_TVOC_IDX:
			*val = 0;
			*val2 = be16_to_cpu(words[1].value);
			ret = IIO_VAL_INT_PLUS_NANO;
			break;
		case SGP30_IAQ_CO2EQ_IDX:
			*val = 0;
			*val2 = be16_to_cpu(words[0].value);
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		mutex_unlock(&data->data_lock);
		break;
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->data_lock);
		ret = sgp_get_measurement(data, data->measure_signal_cmd);
		if (ret)
			goto unlock_fail;
		words = data->buffer.raw_words;
		switch (chan->address) {
		case SGP30_SIG_ETOH_IDX:
			*val = be16_to_cpu(words[1].value);
			ret = IIO_VAL_INT;
			break;
		case SGPC3_SIG_ETOH_IDX:
		case SGP30_SIG_H2_IDX:
			*val = be16_to_cpu(words[0].value);
			ret = IIO_VAL_INT;
			break;
		}
unlock_fail:
		mutex_unlock(&data->data_lock);
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->address) {
		case SGP30_SIG_ETOH_IDX:
		case SGPC3_SIG_ETOH_IDX:
		case SGP30_SIG_H2_IDX:
			*val = 0;
			*val2 = 1953125;
			ret = IIO_VAL_INT_PLUS_NANO;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static void sgp_start_iaq_thread(struct sgp_data *data)
{
	mutex_lock(&data->data_lock);
	if (!data->iaq_thread) {
		data->iaq_thread = kthread_run(sgp_iaq_threadfn, data,
					       "sgp-iaq");
	}
	mutex_unlock(&data->data_lock);
}

static ssize_t sgp_iaq_init_store(struct device *dev,
				  __attribute__((unused))struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	u32 init_time;
	int ret;

	mutex_lock(&data->data_lock);
	data->iaq_init_cmd = SGP_CMD_IAQ_INIT;
	data->iaq_init_jiffies = 0;
	data->iaq_init_skip_jiffies = 15 * HZ;
	data->set_baseline[0] = 0;
	data->set_baseline[1] = 0;
	if (data->product_id == SGPC3) {
		ret = kstrtou32(buf, 10, &init_time);
		if (ret)
			return -EINVAL;

		switch (init_time) {
		case 0:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT0;
			break;
		case 16:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT16;
			break;
		case 64:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT64;
			break;
		case 184:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT184;
			break;
		default:
			return -EINVAL;
		}
		data->iaq_init_skip_jiffies = (21 + init_time) * HZ;
	}
	mutex_unlock(&data->data_lock);

	sgp_start_iaq_thread(data);
	return count;
}

static int sgp_get_baseline(struct sgp_data *data, u16 *baseline_words)
{
	u16 baseline_word;
	int ret, ix;

	mutex_lock(&data->data_lock);
	ret = sgp_read_cmd(data, SGP_CMD_GET_BASELINE, data->baseline_len,
			   SGP_CMD_DURATION_US);
	if (ret < 0)
		goto unlock_fail;

	for (ix = 0; ix < data->baseline_len; ix++) {
		baseline_word = be16_to_cpu(data->buffer.raw_words[ix].value);
		baseline_words[ix] = baseline_word;
	}

unlock_fail:
	mutex_unlock(&data->data_lock);
	return ret;
}

static ssize_t sgp_iaq_baseline_show(struct device *dev,
				     __attribute__((unused))struct device_attribute *attr,
				     char *buf)
{
	u16 baseline_words[2];
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	int ret = sgp_get_baseline(data, baseline_words);
	if (ret < 0)
		return ret;

	return sprintf(buf, data->baseline_format, baseline_words);
}

/**
 * Retrieve the sensor's baseline and report whether it's valid for persistance
 * @baseline:   sensor's current baseline
 *
 * Return:      1 if the baseline was set manually or the sensor has been
 *              operating for at least 12h, 0 if the baseline is not yet valid,
 *              a negative error otherwise.
 */
static int sgp_is_baseline_valid(struct sgp_data *data, u16 *baseline_words)
{
	int ret = sgp_get_baseline(data, baseline_words);
	if (ret < 0)
		return ret;

	mutex_lock(&data->data_lock);
	if (data->set_baseline[0] ||
	    time_after(jiffies, data->iaq_init_jiffies + 60 * 60 * 12 * HZ)) {
		ret = baseline_words[0] > 0;
	}
	mutex_unlock(&data->data_lock);

	return ret;
}

static void sgp_set_baseline(struct sgp_data *data, u16 *baseline_words)
{
	mutex_lock(&data->data_lock);
	data->set_baseline[0] = baseline_words[0];
	if (data->baseline_len == 2)
		data->set_baseline[1] = baseline_words[1];
	data->iaq_init_jiffies = 0;
	mutex_unlock(&data->data_lock);
}

static ssize_t sgp_iaq_baseline_store(struct device *dev,
				      __attribute__((unused))struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	int newline = (count > 0 && buf[count - 1] == '\n');
	u16 words[2];
	int ret = 0;

	/* 1 word (4 chars) per signal */
	if (count - newline == (data->baseline_len * 4)) {
		if (data->baseline_len == 1)
			ret = sscanf(buf, "%04hx", &words[0]);
		else if (data->baseline_len == 2)
			ret = sscanf(buf, "%04hx%04hx", &words[0], &words[1]);
		else
			return -EIO;
	}

	/* Check if baseline format is correct */
	if (ret != data->baseline_len) {
		dev_err(&data->client->dev, "invalid baseline format\n");
		return -EIO;
	}

	sgp_set_baseline(data, words);
	return count;
}

static ssize_t sgp_selftest_show(struct device *dev,
				 __attribute__((unused))struct device_attribute *attr,
				 char *buf)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	u16 baseline_words[2];
	u16 measure_test;
	int baseline_valid = sgp_is_baseline_valid(data, baseline_words);
	int ret;

	mutex_lock(&data->data_lock);
	data->iaq_init_jiffies = 0;
	ret = sgp_read_cmd(data, SGP_CMD_SELFTEST, 1, SGP_SELFTEST_DURATION_US);
	if (ret != 0)
		goto unlock_fail;

	measure_test = be16_to_cpu(data->buffer.raw_words[0].value);
	mutex_unlock(&data->data_lock);

	if (baseline_valid > 0)
		sgp_set_baseline(data, baseline_words);


	return sprintf(buf, "%s\n",
		       measure_test ^ SGP_SELFTEST_OK ? "FAILED" : "OK");

unlock_fail:
	mutex_unlock(&data->data_lock);
	return ret;
}

static ssize_t sgp_serial_id_show(struct device *dev,
				 __attribute__((unused))struct device_attribute *attr,
				 char *buf)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%llu\n", data->serial_id);
}

static ssize_t sgp_feature_set_version_show(struct device *dev,
					    __attribute__((unused))struct device_attribute *attr,
					    char *buf)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%hu.%hu\n", (data->feature_set & 0x00e0) >> 5,
		       data->feature_set & 0x001f);
}

static int sgp_get_serial_id(struct sgp_data *data)
{
	int ret;
	struct sgp_crc_word *words;

	mutex_lock(&data->data_lock);
	ret = sgp_read_cmd(data, SGP_CMD_GET_SERIAL_ID, 3, SGP_CMD_DURATION_US);
	if (ret != 0)
		goto unlock_fail;

	words = data->buffer.raw_words;
	data->serial_id = (u64)(be16_to_cpu(words[2].value) & 0xffff)       |
			  (u64)(be16_to_cpu(words[1].value) & 0xffff) << 16 |
			  (u64)(be16_to_cpu(words[0].value) & 0xffff) << 32;

unlock_fail:
	mutex_unlock(&data->data_lock);
	return ret;
}

static int setup_and_check_sgp_data(struct sgp_data *data,
				    unsigned int product_id)
{
	u16 minor, major, product, eng, ix, num_fs, reserved;
	struct sgp_version *supported_versions;

	product = (data->feature_set & 0xf000) >> 12;
	reserved = (data->feature_set & 0x0e00) >> 9;
	eng = (data->feature_set & 0x0100) >> 8;
	major = (data->feature_set & 0x00e0) >> 5;
	minor = data->feature_set & 0x001f;

	/* driver does not match product */
	if (product != product_id) {
		dev_err(&data->client->dev,
			"sensor reports a different product: 0x%04hx\n",
			product);
		return -ENODEV;
	}
	if (reserved != 0)
		dev_warn(&data->client->dev, "reserved bits set: 0x%04hx\n",
			 reserved);
	/* engineering samples are not supported */
	if (eng != 0)
		return -ENODEV;

	data->iaq_init_cmd = SGP_CMD_IAQ_INIT;
	data->iaq_init_jiffies = 0;
	switch (product) {
	case SGP30:
		supported_versions =
			(struct sgp_version *)supported_versions_sgp30;
		num_fs = ARRAY_SIZE(supported_versions_sgp30);
		data->measurement_len = SGP30_MEASUREMENT_LEN;
		data->measure_interval_hz = SGP30_MEASURE_INTERVAL_HZ;
		data->measure_iaq_cmd = SGP_CMD_IAQ_MEASURE;
		data->measure_signal_cmd = SGP30_CMD_MEASURE_SIGNAL;
		data->product_id = SGP30;
		data->baseline_len = 2;
		data->set_baseline[0] = 0;
		data->set_baseline[1] = 0;
		data->baseline_format = "%08x\n";
		break;
	case SGPC3:
		supported_versions =
			(struct sgp_version *)supported_versions_sgpc3;
		num_fs = ARRAY_SIZE(supported_versions_sgpc3);
		data->measurement_len = SGPC3_MEASUREMENT_LEN;
		data->measure_interval_hz = SGPC3_MEASURE_INTERVAL_HZ;
		data->measure_iaq_cmd = SGPC3_CMD_MEASURE_RAW;
		data->measure_signal_cmd = SGPC3_CMD_MEASURE_RAW;
		data->product_id = SGPC3;
		data->baseline_len = 1;
		data->set_baseline[0] = 0;
		data->baseline_format = "%04x\n";
		break;
	default:
		return -ENODEV;
	};

	for (ix = 0; ix < num_fs; ix++) {
		if (major == supported_versions[ix].major &&
		    minor >= supported_versions[ix].minor)
			return 0;
	}

	dev_err(&data->client->dev, "unsupported sgp version: %d.%d\n",
		major, minor);
	return -ENODEV;
}

static IIO_DEVICE_ATTR(in_serial_id, 0444, sgp_serial_id_show, NULL, 0);
static IIO_DEVICE_ATTR(in_feature_set_version, 0444,
		       sgp_feature_set_version_show, NULL, 0);
static IIO_DEVICE_ATTR(in_selftest, 0444, sgp_selftest_show, NULL, 0);
static IIO_DEVICE_ATTR(do_iaq_init, 0220, NULL, sgp_iaq_init_store, 0);
static IIO_DEVICE_ATTR(in_iaq_baseline, 0444, sgp_iaq_baseline_show, NULL, 0);
static IIO_DEVICE_ATTR(set_iaq_baseline, 0220, NULL, sgp_iaq_baseline_store, 0);
static IIO_DEVICE_ATTR(set_absolute_humidity, 0220, NULL,
		       sgp_absolute_humidity_store, 0);

static struct attribute *sgp_attributes[] = {
	&iio_dev_attr_in_serial_id.dev_attr.attr,
	&iio_dev_attr_in_feature_set_version.dev_attr.attr,
	&iio_dev_attr_in_selftest.dev_attr.attr,
	&iio_dev_attr_do_iaq_init.dev_attr.attr,
	&iio_dev_attr_in_iaq_baseline.dev_attr.attr,
	&iio_dev_attr_set_iaq_baseline.dev_attr.attr,
	&iio_dev_attr_set_absolute_humidity.dev_attr.attr,
	NULL
};

static const struct attribute_group sgp_attr_group = {
	.attrs = sgp_attributes,
};

static const struct iio_info sgp_info = {
	.attrs		= &sgp_attr_group,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	.driver_module	= THIS_MODULE,
#endif /* LINUX_VERSION_CODE */
	.read_raw	= sgp_read_raw,
};

static const struct of_device_id sgp_dt_ids[] = {
	{ .compatible = "sensirion,sgp30", .data = (void *)SGP30 },
	{ .compatible = "sensirion,sgpc3", .data = (void *)SGPC3 },
	{ }
};

static int sgp_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct sgp_data *data;
	struct sgp_device *chip;
	const struct of_device_id *of_id;
	unsigned long product_id;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	of_id = of_match_device(sgp_dt_ids, &client->dev);
	if (!of_id)
		product_id = id->driver_data;
	else
		product_id = (unsigned long)of_id->data;

	chip = &sgp_devices[product_id];
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	crc8_populate_msb(sgp_crc8_table, SGP_CRC8_POLYNOMIAL);
	mutex_init(&data->data_lock);

	/* get serial id and write it to client data */
	ret = sgp_get_serial_id(data);
	if (ret != 0)
		return ret;

	/* get feature set version and write it to client data */
	ret = sgp_read_cmd(data, SGP_CMD_GET_FEATURE_SET, 1,
			   SGP_CMD_DURATION_US);
	if (ret != 0)
		return ret;

	data->feature_set = be16_to_cpu(data->buffer.raw_words[0].value);

	ret = setup_and_check_sgp_data(data, product_id);
	if (ret < 0)
		goto fail_free;

	/* so initial reading will complete */
	data->last_update = jiffies - data->measure_interval_hz * HZ;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &sgp_info;
	indio_dev->name = id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = chip->channels;
	indio_dev->num_channels = chip->num_channels;

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register iio device\n");
		goto fail_free;
	}

	sgp_start_iaq_thread(data);
	return ret;

fail_free:
	mutex_destroy(&data->data_lock);
	iio_device_free(indio_dev);
	return ret;
}

static int sgp_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct sgp_data *data = iio_priv(indio_dev);

	if (data->iaq_thread) {
		kthread_stop(data->iaq_thread);
		data->iaq_thread = NULL;
	}
	devm_iio_device_unregister(&client->dev, indio_dev);
	return 0;
}

static const struct i2c_device_id sgp_id[] = {
	{ "sgp30", SGP30 },
	{ "sgpc3", SGPC3 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sgp_id);
MODULE_DEVICE_TABLE(of, sgp_dt_ids);

static struct i2c_driver sgp_driver = {
	.driver = {
		.name	= "sgp30",
		.of_match_table = of_match_ptr(sgp_dt_ids),
	},
	.probe = sgp_probe,
	.remove = sgp_remove,
	.id_table = sgp_id,
};
module_i2c_driver(sgp_driver);

MODULE_AUTHOR("Andreas Brauchli <andreas.brauchli@sensirion.com>");
MODULE_AUTHOR("Pascal Sachs <pascal.sachs@sensirion.com>");
MODULE_DESCRIPTION("Sensirion SGP gas sensors");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.5.0");
