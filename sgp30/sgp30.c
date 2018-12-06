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
 * https://www.sensirion.com/file/datasheet_sgp30
 * https://www.sensirion.com/file/datasheet_sgpc3
 */

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/string.h>

#define SGP_WORD_LEN				2
#define SGP_CRC8_POLYNOMIAL			0x31
#define SGP_CRC8_INIT				0xff
#define SGP_CRC8_LEN				1
#define SGP_CMD(cmd_word)			cpu_to_be16(cmd_word)
#define SGP_CMD_DURATION_US			12000
#define SGP_MEASUREMENT_DURATION_US		50000
#define SGP_SELFTEST_DURATION_US		220000
#define SGP_CMD_HANDLING_DURATION_US		10000
#define SGP_CMD_LEN				SGP_WORD_LEN
#define SGP_CMD_MAX_BUF_SIZE			(SGP_CMD_LEN + 2 * SGP_WORD_LEN)
#define SGP_MEASUREMENT_LEN			2
#define SGP30_MEASURE_INTERVAL_HZ		1
#define SGPC3_MEASURE_INTERVAL_HZ		2
#define SGPC3_MEASURE_ULP_INTERVAL_HZ		30
#define SGPC3_POWER_MODE_ULTRA_LOW_POWER	0
#define SGPC3_POWER_MODE_LOW_POWER		1
#define SGPC3_DEFAULT_IAQ_INIT_DURATION_HZ	64
#define SGP_SELFTEST_OK				0xd400
#define SGP_VERS_PRODUCT(data)	((((data)->feature_set) & 0xf000) >> 12)
#define SGP_VERS_RESERVED(data)	((((data)->feature_set) & 0x0800) >> 11)
#define SGP_VERS_GEN(data)	((((data)->feature_set) & 0x0600) >> 9)
#define SGP_VERS_ENG_BIT(data)	((((data)->feature_set) & 0x0100) >> 8)
#define SGP_VERS_MAJOR(data)	((((data)->feature_set) & 0x00e0) >> 5)
#define SGP_VERS_MINOR(data)	(((data)->feature_set) & 0x001f)

DECLARE_CRC8_TABLE(sgp_crc8_table);

enum sgp_product_id {
	SGP30 = 0,
	SGPC3,
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
	SGP_CMD_IAQ_INIT			= SGP_CMD(0x2003),
	SGP_CMD_IAQ_MEASURE			= SGP_CMD(0x2008),
	SGP_CMD_GET_BASELINE			= SGP_CMD(0x2015),
	SGP_CMD_SET_BASELINE			= SGP_CMD(0x201e),
	SGP_CMD_GET_TVOC_FACTORY_BASELINE	= SGP_CMD(0x20b3),
	SGP_CMD_GET_FEATURE_SET			= SGP_CMD(0x202f),
	SGP_CMD_GET_SERIAL_ID			= SGP_CMD(0x3682),
	SGP_CMD_SELFTEST			= SGP_CMD(0x2032),
	SGP_CMD_SET_ABSOLUTE_HUMIDITY		= SGP_CMD(0x2061),

	SGP30_CMD_MEASURE_SIGNAL		= SGP_CMD(0x2050),
	SGP30_CMD_SET_TVOC_BASELINE		= SGP_CMD(0x2077),

	SGPC3_CMD_IAQ_INIT_0			= SGP_CMD(0x2089),
	SGPC3_CMD_IAQ_INIT_16			= SGP_CMD(0x2024),
	SGPC3_CMD_IAQ_INIT_64			= SGP_CMD(0x2003),
	SGPC3_CMD_IAQ_INIT_184			= SGP_CMD(0x206a),
	SGPC3_CMD_IAQ_INIT_CONTINUOUS		= SGP_CMD(0x20ae),
	SGPC3_CMD_MEASURE_RAW			= SGP_CMD(0x2046),
	SGPC3_CMD_SET_POWER_MODE		= SGP_CMD(0x209f),
};

enum sgp_baseline_type {
	SGP_BASELINE_TYPE_IAQ,
	SGP_BASELINE_TYPE_TVOC,
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

enum _iaq_buffer_state {
	IAQ_BUFFER_EMPTY = 0,
	IAQ_BUFFER_DEFAULT_VALS,
	IAQ_BUFFER_VALID,
};

enum sgp_features {
	SGP_FEATURE_NONE			= 0,
	SGP_FEATURE_SET_ABSOLUTE_HUMIDITY	= 1 << 0,
	SGP_FEATURE_SET_POWER_MODE		= 1 << 1,
	SGP_FEATURE_TVOC_FACTORY_BASELINE	= 1 << 2,
};

struct sgp_data {
	struct i2c_client *client;
	struct task_struct *iaq_thread;
	struct mutex data_lock;
	unsigned long iaq_init_start_jiffies;
	unsigned long iaq_init_duration_jiffies;
	unsigned long iaq_defval_skip_jiffies;
	u64 serial_id;
	void (*iaq_init_fn)(struct sgp_data *data);
	u32 iaq_init_user_duration;
	u16 product_id;
	u16 feature_set;
	unsigned long measure_interval_jiffies;
	enum sgp_cmd iaq_init_cmd;
	enum sgp_cmd measure_iaq_cmd;
	enum sgp_cmd measure_gas_signals_cmd;
	enum sgp_cmd set_tvoc_baseline_cmd;
	u8 num_baseline_words;
	u16 user_baseline[2];
	enum sgp_baseline_type user_baseline_type;
	u16 power_mode;
	union sgp_reading buffer;
	union sgp_reading iaq_buffer;
	enum _iaq_buffer_state iaq_buffer_state;
	enum sgp_features sgp_feature_mask;
};

struct sgp_device {
	const struct iio_chan_spec *channels;
	int num_channels;
};

static const struct sgp_version supported_versions_sgp30[] = {
	{
		.major = 1,
		.minor = 0,
	},
};

static const struct sgp_version supported_versions_sgpc3[] = {
	{
		.major = 0,
		.minor = 4,
	},
};

static const struct iio_chan_spec sgp30_channels[] = {
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_VOC,
		.datasheet_name = "TVOC signal",
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGP30_IAQ_TVOC_IDX,
	},
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_CO2,
		.datasheet_name = "CO2eq signal",
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGP30_IAQ_CO2EQ_IDX,
	},
	{
		.type = IIO_CONCENTRATION,
#ifdef IIO_MOD_ETHANOL
		.channel2 = IIO_MOD_ETHANOL,
		.modified = 1,
#else /* IIO_MOD_ETHANOL */
		.extend_name = "ethanol",
#endif /* IIO_MOD_ETHANOL */
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW),
		.address = SGP30_SIG_ETOH_IDX,
		.datasheet_name = "Ethanol signal",
		.scan_type = {
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_CONCENTRATION,
#ifdef IIO_MOD_H2
		.channel2 = IIO_MOD_H2,
		.modified = 1,
#else /* IIO_MOD_H2 */
		.extend_name = "h2",
#endif /* IIO_MOD_H2 */
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW),
		.address = SGP30_SIG_H2_IDX,
		.datasheet_name = "H2 signal",
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
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SGPC3_IAQ_TVOC_IDX,
	},
	{
		.type = IIO_CONCENTRATION,
#ifdef IIO_MOD_ETHANOL
		.channel2 = IIO_MOD_ETHANOL,
		.modified = 1,
#else /* IIO_MOD_ETHANOL */
		.extend_name = "ethanol",
#endif /* IIO_MOD_ETHANOL */
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW),
		.address = SGPC3_SIG_ETOH_IDX,
		.datasheet_name = "Ethanol signal",
		.scan_type = {
			.endianness = IIO_BE,
		},
	},
};

static const struct sgp_device sgp_devices[] = {
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
 * @data:       SGP data
 * @buf:        Raw data buffer
 * @word_count: Num data words stored in the buffer, excluding CRC bytes
 *
 * Return:      0 on success, negative error otherwise.
 */
static int sgp_verify_buffer(const struct sgp_data *data,
			     union sgp_reading *buf, size_t word_count)
{
	size_t size = word_count * (SGP_WORD_LEN + SGP_CRC8_LEN);
	int i;
	u8 crc;
	u8 *data_buf = &buf->start;

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
 * @buf:         Raw data buffer to use
 * @word_count:  Num words to read, excluding CRC bytes
 *
 * Return:       0 on success, negative error otherwise.
 */
static int sgp_read_cmd(struct sgp_data *data, enum sgp_cmd cmd,
			union sgp_reading *buf, size_t word_count,
			unsigned long duration_us)
{
	int ret;
	struct i2c_client *client = data->client;
	size_t size = word_count * (SGP_WORD_LEN + SGP_CRC8_LEN);
	u8 *data_buf;

	ret = i2c_master_send(client, (const char *)&cmd, SGP_CMD_LEN);
	if (ret != SGP_CMD_LEN)
		return -EIO;
	usleep_range(duration_us, duration_us + 1000);

	if (word_count == 0)
		return 0;

	data_buf = &buf->start;
	ret = i2c_master_recv(client, data_buf, size);
	if (ret < 0)
		return ret;
	if (ret != size)
		return -EIO;

	return sgp_verify_buffer(data, buf, word_count);
}

/**
 * sgp_i2c_write_cmd() - write data to SGP sensor with a command
 * @data:       SGP data
 * @cmd:        SGP Command to issue
 * @buf:        Data to write
 * @buf_word_size:   Data size of the buffer in words
 *
 * Return:      0 on success, negative error otherwise.
 */
static int sgp_write_cmd(struct sgp_data *data, enum sgp_cmd cmd, u16 *buf,
			 size_t buf_word_size, unsigned long duration_us)
{
	int ret, ix;
	u16 buf_idx;
	u8 buffer[SGP_CMD_MAX_BUF_SIZE];

	/* assemble buffer */
	*((__be16 *)&buffer[0]) = cmd;
	buf_idx = SGP_CMD_LEN;
	for (ix = 0; ix < buf_word_size; ix++) {
		*((__be16 *)&buffer[buf_idx]) = cpu_to_be16(buf[ix]);
		buf_idx += SGP_WORD_LEN;
		buffer[buf_idx] = crc8(sgp_crc8_table,
				       &buffer[buf_idx - SGP_WORD_LEN],
				       SGP_WORD_LEN, SGP_CRC8_INIT);
		buf_idx += SGP_CRC8_LEN;
	}
	ret = i2c_master_send(data->client, buffer, buf_idx);
	if (ret != buf_idx)
		return -EIO;
	ret = 0;
	usleep_range(duration_us, duration_us + 1000);

	return ret;
}

/**
 * sgp_measure_iaq() - measure and retrieve IAQ values from sensor
 * The caller must hold data->data_lock for the duration of the call.
 * @data:       SGP data
 *
 * Return:      0 on success, -EBUSY on default values, negative error
 *              otherwise.
 */

static int sgp_measure_iaq(struct sgp_data *data)
{
	int ret;
	/* data contains default values */
	bool default_vals = !time_after(jiffies, data->iaq_init_start_jiffies +
						 data->iaq_defval_skip_jiffies);

	ret = sgp_read_cmd(data, data->measure_iaq_cmd, &data->iaq_buffer,
			   SGP_MEASUREMENT_LEN, SGP_MEASUREMENT_DURATION_US);
	if (ret < 0)
		return ret;

	data->iaq_buffer_state = IAQ_BUFFER_DEFAULT_VALS;

	if (default_vals)
		return -EBUSY;

	data->iaq_buffer_state = IAQ_BUFFER_VALID;
	return 0;
}

static void sgp_iaq_thread_sleep_until(const struct sgp_data *data,
				       unsigned long sleep_jiffies)
{
	const long IAQ_POLL = 50000;

	while (!time_after(jiffies, sleep_jiffies)) {
		usleep_range(IAQ_POLL, IAQ_POLL + 10000);
		if (kthread_should_stop() || data->iaq_init_start_jiffies == 0)
			return;
	}
}

static int sgp_iaq_thread_set_baseline(struct sgp_data *data)
{
	size_t num_baseline_words;
	enum sgp_cmd set_baseline_cmd;

	switch (data->user_baseline_type) {
	case SGP_BASELINE_TYPE_TVOC:
		set_baseline_cmd = data->set_tvoc_baseline_cmd;
		num_baseline_words = 1;
		break;
	case SGP_BASELINE_TYPE_IAQ:
	default:
		set_baseline_cmd = SGP_CMD_SET_BASELINE;
		num_baseline_words = data->num_baseline_words;
		break;
	}

	return sgp_write_cmd(data, set_baseline_cmd, data->user_baseline,
			     num_baseline_words, SGP_CMD_DURATION_US);
}

static int sgp_iaq_threadfn(void *p)
{
	struct sgp_data *data = (struct sgp_data *)p;
	unsigned long next_update_jiffies;
	int ret;

	while (!kthread_should_stop()) {
		mutex_lock(&data->data_lock);
		if (data->iaq_init_start_jiffies == 0) {
			if (data->iaq_init_fn)
				data->iaq_init_fn(data);
			ret = sgp_read_cmd(data, data->iaq_init_cmd, NULL, 0,
					   SGP_CMD_DURATION_US);
			if (ret < 0)
				goto unlock_sleep_continue;
			data->iaq_init_start_jiffies = jiffies;
			if (data->user_baseline[0]) {
				ret = sgp_iaq_thread_set_baseline(data);
				if (ret < 0) {
					dev_err(&data->client->dev,
						"IAQ set baseline failed [%d]\n",
						ret);
					goto unlock_sleep_continue;
				}
			}
			if (data->iaq_init_duration_jiffies) {
				mutex_unlock(&data->data_lock);
				sgp_iaq_thread_sleep_until(data,
					    data->iaq_init_start_jiffies +
					    data->iaq_init_duration_jiffies);
				continue;
			}
		}

		ret = sgp_measure_iaq(data);
		if (ret && ret != -EBUSY) {
			dev_warn(&data->client->dev,
				 "IAQ measurement error [%d]\n",
				 ret);
		}
unlock_sleep_continue:
		next_update_jiffies = jiffies + data->measure_interval_jiffies;
		mutex_unlock(&data->data_lock);
		sgp_iaq_thread_sleep_until(data, next_update_jiffies);
	}
	return 0;
}

static ssize_t sgp_absolute_humidity_store(struct device *dev,
					   __attribute__((unused))
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	u32 ah;
	u16 ah_scaled;
	int ret;

	ret = kstrtou32(buf, 10, &ah);
	if (ret)
		return ret;
	if (ah > 256000)
		return -EINVAL;

	/* ah_scaled = (u16)((ah / 1000.0) * 256.0) */
	ah_scaled = (u16)(((u64)ah * 256 * 16777) >> 24);

	/* don't disable AH compensation due to rounding */
	if (ah > 0 && ah_scaled == 0)
		ah_scaled = 1;

	mutex_lock(&data->data_lock);
	ret = sgp_write_cmd(data, SGP_CMD_SET_ABSOLUTE_HUMIDITY,
			    &ah_scaled, 1, SGP_CMD_DURATION_US);
	mutex_unlock(&data->data_lock);
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
		if (data->iaq_buffer_state != IAQ_BUFFER_VALID) {
			mutex_unlock(&data->data_lock);
			return -EBUSY;
		}
		words = data->iaq_buffer.raw_words;
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
		if (chan->address == SGPC3_SIG_ETOH_IDX) {
			if (data->iaq_buffer_state == IAQ_BUFFER_EMPTY)
				ret = -EBUSY;
			else
				ret = 0;
			words = data->iaq_buffer.raw_words;
		} else {
			ret = sgp_read_cmd(data, data->measure_gas_signals_cmd,
					   &data->buffer, SGP_MEASUREMENT_LEN,
					   SGP_MEASUREMENT_DURATION_US);
			words = data->buffer.raw_words;
		}
		if (ret) {
			mutex_unlock(&data->data_lock);
			return ret;
		}

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
		mutex_unlock(&data->data_lock);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static void sgp_restart_iaq_thread(struct sgp_data *data)
{
	mutex_lock(&data->data_lock);
	data->iaq_buffer_state = IAQ_BUFFER_EMPTY;
	data->iaq_init_start_jiffies = 0;
	mutex_unlock(&data->data_lock);
}

static void sgpc3_iaq_init(struct sgp_data *data)
{
	int skip_cycles;

	data->iaq_init_duration_jiffies = data->iaq_init_user_duration * HZ;
	if (data->sgp_feature_mask & SGP_FEATURE_SET_POWER_MODE) {
		data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT_CONTINUOUS;

		if (data->power_mode == SGPC3_POWER_MODE_ULTRA_LOW_POWER) {
			data->measure_interval_jiffies =
				SGPC3_MEASURE_ULP_INTERVAL_HZ * HZ;
		} else {
			data->measure_interval_jiffies =
				SGPC3_MEASURE_INTERVAL_HZ * HZ;
		}

		if (data->user_baseline[0])
			skip_cycles = 1;
		else if (data->power_mode == SGPC3_POWER_MODE_ULTRA_LOW_POWER)
			skip_cycles = 2;
		else
			skip_cycles = 11;
	} else {
		switch (data->iaq_init_user_duration) {
		case 0:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT_0;
			skip_cycles = 1;
			break;
		case 16:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT_16;
			skip_cycles = 9;
			break;
		case 64:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT_64;
			skip_cycles = 33;
			break;
		case 184:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT_184;
			skip_cycles = 93;
			break;
		default:
			data->iaq_init_cmd = SGPC3_CMD_IAQ_INIT_64;
			skip_cycles = 33;
			break;
		}
		if (!data->user_baseline[0])
			skip_cycles += 10;
	}
	data->iaq_defval_skip_jiffies = data->iaq_init_duration_jiffies +
					(skip_cycles *
					 data->measure_interval_jiffies);
}

static ssize_t sgp_iaq_preheat_store(struct device *dev,
				     __attribute__((unused))
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	u32 init_duration;
	int ret;

	if (SGP_VERS_PRODUCT(data) != SGPC3)
		return -EINVAL;

	mutex_lock(&data->data_lock);
	ret = kstrtou32(buf, 10, &init_duration);
	if (ret)
		goto unlock_fail;

	if (data->sgp_feature_mask & SGP_FEATURE_SET_POWER_MODE) {
		if (init_duration > 300) {
			ret = -EINVAL;
			goto unlock_fail;
		}
	} else {
		if (init_duration == 0) {
		} else if (init_duration <= 16) {
			init_duration = 16;
		} else if (init_duration <= 64) {
			init_duration = 64;
		} else if (init_duration <= 184) {
			init_duration = 184;
		} else {
			ret = -EINVAL;
			goto unlock_fail;
		}
	}

	data->iaq_init_user_duration = init_duration;
	mutex_unlock(&data->data_lock);

	return count;

unlock_fail:
	mutex_unlock(&data->data_lock);
	return ret;
}

static ssize_t sgp_power_mode_store(struct device *dev,
				    __attribute__((unused))
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	u16 power_mode;
	int ret;
	const char ULTRA_LOW[] = "ultra-low";
	const char LOW[] = "low";

	if (strncmp(buf, ULTRA_LOW, sizeof(ULTRA_LOW) - 1) == 0)
		power_mode = SGPC3_POWER_MODE_ULTRA_LOW_POWER;
	else if (strncmp(buf, LOW, sizeof(LOW) - 1) == 0)
		power_mode = SGPC3_POWER_MODE_LOW_POWER;
	else
		return -EINVAL;

	mutex_lock(&data->data_lock);
	if (power_mode == data->power_mode) {
		mutex_unlock(&data->data_lock);
		return count;
	}

	/* Switching power mode invalidates any set baselines */
	if (power_mode != data->power_mode) {
		data->user_baseline[0] = 0;
		data->user_baseline[1] = 0;
	}

	ret = sgp_write_cmd(data, SGPC3_CMD_SET_POWER_MODE,
			    &power_mode, 1, SGP_CMD_DURATION_US);
	if (ret) {
		mutex_unlock(&data->data_lock);
		return ret;
	}
	data->power_mode = power_mode;
	mutex_unlock(&data->data_lock);

	sgp_restart_iaq_thread(data);
	return count;
}

static int sgp_get_baseline(struct sgp_data *data, u16 *baseline_words)
{
	u16 baseline_word;
	int ret, ix, jx;

	mutex_lock(&data->data_lock);
	ret = sgp_read_cmd(data, SGP_CMD_GET_BASELINE, &data->buffer,
			   data->num_baseline_words, SGP_CMD_DURATION_US);
	if (ret < 0)
		goto unlock_fail;

	for (ix = 0, jx = data->num_baseline_words - 1;
	     ix < data->num_baseline_words;
	     ix++, jx--) {
		baseline_word = be16_to_cpu(data->buffer.raw_words[jx].value);
		baseline_words[ix] = baseline_word;
	}

	if (!baseline_words[0])
		ret = -EBUSY;

unlock_fail:
	mutex_unlock(&data->data_lock);
	return ret;
}

/**
 * Retrieve the sensor's baseline and report whether it's valid for persistence
 * @baseline:   sensor's current baseline
 *
 * Return:      1 if the baseline was set manually or the sensor has been
 *              operating for at least 12h, -EBUSY if the baseline is not yet
 *              valid, another negative error otherwise.
 */
static int sgp_get_valid_baseline(struct sgp_data *data, u16 *baseline_words)
{
	int ret;

	ret = sgp_get_baseline(data, baseline_words);
	if (ret < 0)
		return ret;

	mutex_lock(&data->data_lock);
	ret = (data->user_baseline[0] ||
	       time_after(jiffies,
			  data->iaq_init_start_jiffies + 60 * 60 * 12 * HZ));
	mutex_unlock(&data->data_lock);

	return ret;
}

static ssize_t sgp_iaq_baseline_show(struct device *dev,
				     __attribute__((unused))
				     struct device_attribute *attr,
				     char *buf)
{
	int ret;
	u16 baseline_words[2];
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));

	ret = sgp_get_valid_baseline(data, baseline_words);
	if (ret < 0)
		return ret;

	if (data->num_baseline_words == 1)
		return sprintf(buf, "%04hx\n", baseline_words[0]);

	return sprintf(buf, "%04hx%04hx\n", baseline_words[0],
		       baseline_words[1]);
}

static void sgp_set_baseline(struct sgp_data *data, u16 *baseline_words,
			     enum sgp_baseline_type type)
{
	mutex_lock(&data->data_lock);
	data->user_baseline_type = type;
	data->user_baseline[0] = baseline_words[0];
	if (type == SGP_BASELINE_TYPE_IAQ && data->num_baseline_words == 2)
		data->user_baseline[1] = baseline_words[1];
	mutex_unlock(&data->data_lock);
	sgp_restart_iaq_thread(data);
}

static ssize_t sgp_iaq_baseline_store(struct device *dev,
				      __attribute__((unused))
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	int newline = (count > 0 && buf[count - 1] == '\n');
	u16 words[2];
	int ret = 0;

	if (count - newline == (data->num_baseline_words * 4)) {
		if (data->num_baseline_words == 1)
			ret = sscanf(buf, "%04hx", &words[0]);
		else
			ret = sscanf(buf, "%04hx%04hx", &words[0], &words[1]);
	}

	if (ret != data->num_baseline_words) {
		dev_err(&data->client->dev, "invalid baseline format\n");
		return -EINVAL;
	}

	sgp_set_baseline(data, words, SGP_BASELINE_TYPE_IAQ);
	return count;
}

static ssize_t sgp_tvoc_factory_baseline_show(struct device *dev,
					      __attribute__((unused))
					      struct device_attribute *attr,
					      char *buf)
{
	int ret;
	u16 baseline_word;
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));

	mutex_lock(&data->data_lock);
	ret = sgp_read_cmd(data, SGP_CMD_GET_TVOC_FACTORY_BASELINE,
			   &data->buffer, 1, SGP_CMD_DURATION_US);
	if (ret < 0) {
		mutex_unlock(&data->data_lock);
		return ret;
	}

	baseline_word = be16_to_cpu(data->buffer.raw_words[0].value);
	mutex_unlock(&data->data_lock);

	return sprintf(buf, "%04hx\n", baseline_word);
}

static ssize_t sgp_tvoc_baseline_store(struct device *dev,
				       __attribute__((unused))
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	int newline = (count > 0 && buf[count - 1] == '\n');
	u16 word;
	int ret = 0;

	if (count - newline == 4)
		ret = sscanf(buf, "%04hx", &word);

	if (ret != 1) {
		dev_err(&data->client->dev, "invalid tVOC baseline format\n");
		return -EINVAL;
	}

	sgp_set_baseline(data, &word, SGP_BASELINE_TYPE_TVOC);
	return count;
}

static ssize_t sgp_selftest_show(struct device *dev,
				 __attribute__((unused))
				 struct device_attribute *attr,
				 char *buf)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	u16 baseline_words[2];
	u16 measure_test;
	int baseline_valid = 0;
	int ret;

	if (data->product_id == SGP30) {
		/* On the SGP30, the self-test interferes with iaq_init */
		baseline_valid = sgp_get_valid_baseline(data, baseline_words);
	}

	mutex_lock(&data->data_lock);
	ret = sgp_read_cmd(data, SGP_CMD_SELFTEST, &data->buffer, 1,
			   SGP_SELFTEST_DURATION_US);
	if (ret < 0) {
		mutex_unlock(&data->data_lock);
		return ret;
	}

	measure_test = be16_to_cpu(data->buffer.raw_words[0].value);
	mutex_unlock(&data->data_lock);

	if (data->product_id == SGP30) {
		if (baseline_valid > 0) {
			sgp_set_baseline(data, baseline_words,
					 SGP_BASELINE_TYPE_IAQ);
		} else {
			sgp_restart_iaq_thread(data);
		}
	}

	return sprintf(buf, "%s\n",
		       measure_test ^ SGP_SELFTEST_OK ? "FAILED" : "OK");
}

static ssize_t sgp_serial_id_show(struct device *dev,
				 __attribute__((unused))
				 struct device_attribute *attr,
				 char *buf)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%llu\n", data->serial_id);
}

static ssize_t sgp_feature_set_version_show(struct device *dev,
					    __attribute__((unused))
					    struct device_attribute *attr,
					    char *buf)
{
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%hu.%hu\n", SGP_VERS_MAJOR(data),
		       SGP_VERS_MINOR(data));
}

static int sgp_get_serial_id(struct sgp_data *data)
{
	int ret;
	struct sgp_crc_word *words;

	mutex_lock(&data->data_lock);
	ret = sgp_read_cmd(data, SGP_CMD_GET_SERIAL_ID, &data->buffer, 3,
			   SGP_CMD_DURATION_US);
	if (ret < 0)
		goto unlock_fail;

	words = data->buffer.raw_words;
	data->serial_id = (u64)(be16_to_cpu(words[2].value) & 0xffff)       |
			  (u64)(be16_to_cpu(words[1].value) & 0xffff) << 16 |
			  (u64)(be16_to_cpu(words[0].value) & 0xffff) << 32;

unlock_fail:
	mutex_unlock(&data->data_lock);
	return ret;
}

static int sgp_check_compat(struct sgp_data *data,
			    unsigned int product_id)
{
	struct sgp_version *supported_versions;
	u16 ix, num_fs;
	u16 product = SGP_VERS_PRODUCT(data);
	u16 reserved = SGP_VERS_RESERVED(data);
	u16 generation = SGP_VERS_GEN(data);
	u16 major = SGP_VERS_MAJOR(data);
	u16 minor = SGP_VERS_MINOR(data);

	/* driver does not match product */
	if (generation != 0) {
		dev_err(&data->client->dev,
			"incompatible product generation %d != 0", generation);
		return -ENODEV;
	}

	if (product != product_id) {
		dev_err(&data->client->dev,
			"sensor reports a different product: 0x%04hx\n",
			product);
		return -ENODEV;
	}

	if (reserved)
		dev_warn(&data->client->dev, "reserved bit is set\n");

	/* engineering samples are not supported: no interface guarantees */
	if (SGP_VERS_ENG_BIT(data))
		return -ENODEV;

	switch (product) {
	case SGP30:
		supported_versions =
			(struct sgp_version *)supported_versions_sgp30;
		num_fs = ARRAY_SIZE(supported_versions_sgp30);
		break;
	case SGPC3:
		supported_versions =
			(struct sgp_version *)supported_versions_sgpc3;
		num_fs = ARRAY_SIZE(supported_versions_sgpc3);
		break;
	default:
		return -ENODEV;
	}

	for (ix = 0; ix < num_fs; ix++) {
		if (major == supported_versions[ix].major &&
		    minor >= supported_versions[ix].minor)
			return 0;
	}
	dev_err(&data->client->dev, "unsupported sgp version: %d.%d\n",
		major, minor);
	return -ENODEV;
}

static void sgp_init(struct sgp_data *data)
{
	data->iaq_init_cmd = SGP_CMD_IAQ_INIT;
	data->iaq_init_start_jiffies = 0;
	data->iaq_init_duration_jiffies = 0;
	data->iaq_init_user_duration = 0;
	data->iaq_init_fn = NULL;
	data->iaq_buffer_state = IAQ_BUFFER_EMPTY;
	data->user_baseline[0] = 0;
	data->user_baseline[1] = 0;
	data->sgp_feature_mask = SGP_FEATURE_NONE;
	switch (SGP_VERS_PRODUCT(data)) {
	case SGP30:
		data->measure_interval_jiffies = SGP30_MEASURE_INTERVAL_HZ * HZ;
		data->measure_iaq_cmd = SGP_CMD_IAQ_MEASURE;
		data->measure_gas_signals_cmd = SGP30_CMD_MEASURE_SIGNAL;
		data->set_tvoc_baseline_cmd = SGP30_CMD_SET_TVOC_BASELINE;
		data->product_id = SGP30;
		data->num_baseline_words = 2;
		data->iaq_defval_skip_jiffies = 15 * HZ;
		data->sgp_feature_mask |= SGP_FEATURE_SET_ABSOLUTE_HUMIDITY;
		if (SGP_VERS_MAJOR(data) == 1 && SGP_VERS_MINOR(data) >= 1) {
			data->sgp_feature_mask |=
				SGP_FEATURE_TVOC_FACTORY_BASELINE;
		}
		break;
	case SGPC3:
		data->measure_interval_jiffies = SGPC3_MEASURE_INTERVAL_HZ * HZ;
		data->measure_iaq_cmd = SGPC3_CMD_MEASURE_RAW;
		data->measure_gas_signals_cmd = SGPC3_CMD_MEASURE_RAW;
		data->set_tvoc_baseline_cmd = SGP_CMD_SET_BASELINE;
		data->product_id = SGPC3;
		data->num_baseline_words = 1;
		data->power_mode = SGPC3_POWER_MODE_LOW_POWER;
		data->iaq_init_user_duration =
			SGPC3_DEFAULT_IAQ_INIT_DURATION_HZ;
		data->iaq_init_fn = &sgpc3_iaq_init;
		if (SGP_VERS_MAJOR(data) == 0 && SGP_VERS_MINOR(data) >= 5) {
			data->sgp_feature_mask |=
				SGP_FEATURE_TVOC_FACTORY_BASELINE;
		}
		if (SGP_VERS_MAJOR(data) == 0 && SGP_VERS_MINOR(data) >= 6) {
			data->sgp_feature_mask |=
				SGP_FEATURE_SET_ABSOLUTE_HUMIDITY |
				SGP_FEATURE_SET_POWER_MODE;
		}
		break;
	};
}

static IIO_DEVICE_ATTR(in_serial_id, 0444, sgp_serial_id_show, NULL, 0);
static IIO_DEVICE_ATTR(in_feature_set_version, 0444,
		       sgp_feature_set_version_show, NULL, 0);
static IIO_DEVICE_ATTR(in_selftest, 0444, sgp_selftest_show, NULL, 0);
static IIO_DEVICE_ATTR(set_iaq_preheat_seconds, 0220, NULL,
		       sgp_iaq_preheat_store, 0);
static IIO_DEVICE_ATTR(iaq_baseline, 0664, sgp_iaq_baseline_show,
		       sgp_iaq_baseline_store, 0);
static IIO_DEVICE_ATTR(tvoc_factory_baseline, 0664,
		       sgp_tvoc_factory_baseline_show,
		       sgp_tvoc_baseline_store, 0);
static IIO_DEVICE_ATTR(set_absolute_humidity, 0220, NULL,
		       sgp_absolute_humidity_store, 0);
static IIO_DEVICE_ATTR(set_power_mode, 0220, NULL,
		       sgp_power_mode_store, 0);

static struct attribute *sgp_attributes[] = {
	&iio_dev_attr_in_serial_id.dev_attr.attr,
	&iio_dev_attr_in_feature_set_version.dev_attr.attr,
	&iio_dev_attr_in_selftest.dev_attr.attr,
	&iio_dev_attr_iaq_baseline.dev_attr.attr,
	&iio_dev_attr_tvoc_factory_baseline.dev_attr.attr,
	&iio_dev_attr_set_iaq_preheat_seconds.dev_attr.attr,
	&iio_dev_attr_set_absolute_humidity.dev_attr.attr,
	&iio_dev_attr_set_power_mode.dev_attr.attr,
	NULL
};

umode_t sgp_attributes_visible(struct kobject *kobj, struct attribute *attr,
                               int index)
{
	struct device *dev = kobj_to_dev(kobj);
	struct sgp_data *data = iio_priv(dev_to_iio_dev(dev));
	umode_t effective_mode = attr->mode;
	u16 product = SGP_VERS_PRODUCT(data);

	if (product == SGP30 &&
	    attr == &iio_dev_attr_set_iaq_preheat_seconds.dev_attr.attr)
		return 0;

	if (!(data->sgp_feature_mask & SGP_FEATURE_SET_ABSOLUTE_HUMIDITY) &&
	    attr == &iio_dev_attr_set_absolute_humidity.dev_attr.attr)
		return 0;

	if (!(data->sgp_feature_mask & SGP_FEATURE_SET_POWER_MODE) &&
	    attr == &iio_dev_attr_set_power_mode.dev_attr.attr)
		return 0;

	return effective_mode;
}

static const struct attribute_group sgp_attr_group = {
	.attrs = sgp_attributes,
};

static const struct iio_info sgp_info = {
	.attrs		= &sgp_attr_group,
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
	const struct of_device_id *of_id;
	unsigned long product_id;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	of_id = of_match_device(sgp_dt_ids, &client->dev);
	if (of_id)
		product_id = (unsigned long)of_id->data;
	else
		product_id = id->driver_data;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	crc8_populate_msb(sgp_crc8_table, SGP_CRC8_POLYNOMIAL);
	mutex_init(&data->data_lock);

	/* get serial id and write it to client data */
	ret = sgp_get_serial_id(data);
	if (ret < 0)
		return ret;

	/* get feature set version and write it to client data */
	ret = sgp_read_cmd(data, SGP_CMD_GET_FEATURE_SET, &data->buffer, 1,
			   SGP_CMD_DURATION_US);
	if (ret < 0)
		return ret;

	data->feature_set = be16_to_cpu(data->buffer.raw_words[0].value);

	ret = sgp_check_compat(data, product_id);
	if (ret)
		return ret;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &sgp_info;
	indio_dev->chan_attr_group.is_visible = sgp_attributes_visible;
	indio_dev->name = id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = sgp_devices[product_id].channels;
	indio_dev->num_channels = sgp_devices[product_id].num_channels;

	sgp_init(data);

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register iio device\n");
		return ret;
	}

	data->iaq_thread = kthread_run(sgp_iaq_threadfn, data,
				       "%s-iaq", data->client->name);
	return 0;
}

static int sgp_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct sgp_data *data = iio_priv(indio_dev);

	if (data->iaq_thread)
		kthread_stop(data->iaq_thread);
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
		.name = "sgp30",
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
MODULE_VERSION("0.7.0");
