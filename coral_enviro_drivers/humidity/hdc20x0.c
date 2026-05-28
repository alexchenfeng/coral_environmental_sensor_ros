/*
 * hdc20x0.c - Support for the TI HDC20x0 temperature + humidity sensors
 *
 * Copyright (C) 2019 Google LLC
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define HDC20X0_REG_TEMP_LOW 0x00
#define HDC20X0_REG_TEMP_HIGH 0x01
#define HDC20X0_REG_HUMID_LOW 0x02
#define HDC20X0_REG_HUMID_HIGH 0x03
#define HDC20X0_REG_INTERRUPT_DRDY 0x04
#define HDC20X0_REG_TEMP_MAX 0x05
#define HDC20X0_REG_HUMID_MAX 0x06
#define HDC20X0_REG_INTERRUPT_CONFIG 0x07
#define HDC20X0_REG_TEMP_OFFSET_ADJUST 0x08
#define HDC20X0_REG_HUM_OFFSET_ADJUST 0x09
#define HDC20X0_REG_TEMP_THR_L 0x0A
#define HDC20X0_REG_TEMP_THR_H 0x0B
#define HDC20X0_REG_HUMID_THR_L 0x0C
#define HDC20X0_REG_HUMID_THR_H 0x0D
#define HDC20X0_REG_CONFIG 0x0E
#define HDC20X0_REG_MEASUREMENT_CONFIG 0x0F
#define HDC20X0_REG_MID_L 0xFC
#define HDC20X0_REG_MID_H 0xFD
#define HDC20X0_REG_DEVICE_ID_L 0xFE
#define HDC20X0_REG_DEVICE_ID_H 0xFF

#define HDC20X0_REG_CONFIG_HEATER_EN 0x8
#define HDC20X0_REG_MEASUREMENT_CONFIG_TRIGGER 0x1

#define HDC20X0_DRV_NAME "hdc20x0"

#define HDC20X0_TEMP_SCALE_NUM 165
#define HDC20X0_TEMP_SCALE_DENOM (1 << 16)
#define HDC20X0_TEMP_OFFSET_VAL -40
#define HDC20X0_TEMP_OFFSET_INT -15887
#define HDC20X0_TEMP_OFFSET_MICRO 515151
#define HDC20X0_HUMIDITY_SCALE_NUM 100
#define HDC20X0_HUMIDITY_SCALE_DENOM (1 << 16)


struct hdc20x0_data {
	struct regmap *regmap;
	/* Mutex should be used on read/write operations to prevent bus
	 * contention */
	struct mutex lock;
	u16 config;
};

static const struct regmap_config hdc20x0_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = HDC20X0_REG_DEVICE_ID_H,
};

static IIO_CONST_ATTR(out_current_heater_raw_available, "0 1");

static struct attribute *hdc20x0_attributes[] = {
	&iio_const_attr_out_current_heater_raw_available.dev_attr.attr, NULL
};

static struct attribute_group hdc20x0_attribute_group = {
	.attrs = hdc20x0_attributes,
};

static const struct iio_chan_spec hdc20x0_channels[] = {
	{
		.type = IIO_TEMP,
		.address = HDC20X0_REG_TEMP_LOW,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |BIT(IIO_CHAN_INFO_SCALE)
			| BIT(IIO_CHAN_INFO_OFFSET) | BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_HUMIDITYRELATIVE,
		.address = HDC20X0_REG_HUMID_LOW,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE)
			| BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_CURRENT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.extend_name = "heater",
		.output = 1,
	},
};

static int hdc20x0_update_config(struct hdc20x0_data *data, int mask, int val)
{
	int tmp = (~mask & data->config) | val;
	int ret;

	ret = regmap_update_bits(data->regmap, HDC20X0_REG_CONFIG, mask, val);
	if (!ret)
		data->config = tmp;

	return ret;
}

static int hdc20x0_get_measurement(
	struct hdc20x0_data *data, struct iio_chan_spec const *chan)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;
	int val, reg;

	/* Trigger measurement by setting bit in measurement config */
	ret = regmap_update_bits(data->regmap, HDC20X0_REG_MEASUREMENT_CONFIG,
		HDC20X0_REG_MEASUREMENT_CONFIG_TRIGGER, 1);

	/* Read low byte of measurement */
	ret = regmap_read(data->regmap, chan->address, &reg);
	if (ret < 0) {
		dev_err(dev, "cannot read low byte");
		return ret;
	}
	val = reg;

	/* High byte is address + 1 */
	ret = regmap_read(data->regmap, chan->address + 1, &reg);
	if (ret < 0) {
		dev_err(dev, "cannot read high byte");
		return ret;
	}
	val |= reg << 8;

	return val;
}

static int hdc20x0_get_processed_data(
	struct hdc20x0_data *data, struct iio_chan_spec const *chan, int *val, int *val2)
{
	int raw;
	raw = hdc20x0_get_measurement(data, chan);
	if (raw < 0) {
		return raw;
	}
	switch(chan->type) {
	case IIO_TEMP:
		raw *= HDC20X0_TEMP_SCALE_NUM;
		*val = raw / HDC20X0_TEMP_SCALE_DENOM;
		/* Keep 3 digits of precision, but multiply by 1000 for micros */
		*val2 = (raw - (*val * HDC20X0_TEMP_SCALE_DENOM)) * 1000 / HDC20X0_TEMP_SCALE_DENOM;
		*val2 *= 1000;
		*val += HDC20X0_TEMP_OFFSET_VAL;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_HUMIDITYRELATIVE:
		raw *= HDC20X0_HUMIDITY_SCALE_NUM;
		*val = raw / HDC20X0_HUMIDITY_SCALE_DENOM;
		*val2 = (raw - (*val * HDC20X0_HUMIDITY_SCALE_DENOM)) * 1000 / HDC20X0_HUMIDITY_SCALE_DENOM;
		*val2 *= 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int hdc20x0_get_heater_status(struct hdc20x0_data *data)
{
	return !!(data->config & HDC20X0_REG_CONFIG_HEATER_EN);
}

static int hdc20x0_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct hdc20x0_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&data->lock);
		ret = hdc20x0_get_processed_data(data, chan, val, val2);
		mutex_unlock(&data->lock);
		return ret;
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);
		if (chan->type == IIO_CURRENT) {
			*val = hdc20x0_get_heater_status(data);
			ret = IIO_VAL_INT;
		} else {
			ret = hdc20x0_get_measurement(data, chan);
			if (ret >= 0) {
				*val = ret;
				ret = IIO_VAL_INT;
			}
		}
		mutex_unlock(&data->lock);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP) {
			*val = HDC20X0_TEMP_SCALE_NUM;
			*val2 = HDC20X0_TEMP_SCALE_DENOM;
			return IIO_VAL_FRACTIONAL;
		} else if (chan->type == IIO_HUMIDITYRELATIVE) {
			*val = HDC20X0_HUMIDITY_SCALE_NUM;
			*val2 = HDC20X0_HUMIDITY_SCALE_DENOM;
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP) {
			/* IIO offset happens before scaling so values are -40 *
			 * scaling */
			*val = HDC20X0_TEMP_OFFSET_INT;
			*val2 = HDC20X0_TEMP_OFFSET_MICRO;
			return IIO_VAL_INT_PLUS_MICRO;
		} else {
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int hdc20x0_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct hdc20x0_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (mask == IIO_CHAN_INFO_RAW) {
		if (chan->type != IIO_CURRENT || val2 != 0)
			return -EINVAL;

		mutex_lock(&data->lock);
		ret = hdc20x0_update_config(data, HDC20X0_REG_CONFIG_HEATER_EN,
			val ? HDC20X0_REG_CONFIG_HEATER_EN : 0);
		mutex_unlock(&data->lock);
		return ret;
	} else {
		return -EINVAL;
	}
}

static const struct iio_info hdc20x0_info = {
	.read_raw = hdc20x0_read_raw,
	.write_raw = hdc20x0_write_raw,
	.attrs = &hdc20x0_attribute_group,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0))
	.driver_module = THIS_MODULE,
#endif
};

static int hdc20x0_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct hdc20x0_data *data;

	if (!i2c_check_functionality(client->adapter,
		    I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_BYTE))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->regmap = devm_regmap_init_i2c(client, &hdc20x0_regmap_config);
	mutex_init(&data->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = HDC20X0_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &hdc20x0_info;

	indio_dev->channels = hdc20x0_channels;
	indio_dev->num_channels = ARRAY_SIZE(hdc20x0_channels);

	return devm_iio_device_register(&client->dev, indio_dev);
}

static int hdc20x0_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void hdc20x0_remove_void(struct i2c_client *client)
{
	hdc20x0_remove(client);
}
#endif

static const struct i2c_device_id hdc20x0_id[] = { { HDC20X0_DRV_NAME, 0 },
	{} };
MODULE_DEVICE_TABLE(i2c, hdc20x0_id);

static const struct of_device_id hdc20x0_of_match[] = {
	{ .compatible = "ti,hdc20x0" },
	{ .compatible = "hdc20x0" },
	{}
};
MODULE_DEVICE_TABLE(of, hdc20x0_of_match);

static struct i2c_driver hdc20x0_driver = {
	.driver = {
		.name	= HDC20X0_DRV_NAME,
		.of_match_table = of_match_ptr(hdc20x0_of_match),
	},
	.probe = hdc20x0_probe,
	.remove = hdc20x0_remove_void,
	.id_table = hdc20x0_id,
};

module_i2c_driver(hdc20x0_driver);

MODULE_AUTHOR("Michael Brooks <mrbrooks@google.com>");
MODULE_DESCRIPTION("TI HDC20x0 Humidity and Temperature Sensor Driver");
MODULE_LICENSE("GPL");
