// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADI ADC emulator driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

static int adi_emu_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->channel)
			*val = 10;
		else
			*val = 22;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info adi_emu_info = {
	.read_raw = &adi_emu_read_raw,
};

static const struct iio_chan_spec adi_emu_channel[] = {
	{
		.type = IIO_VOLTAGE,
		.channel = 0,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 1,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int adi_emu_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, 0);
	if (!indio_dev)
		return -ENOMEM;
	
	indio_dev->name = "iio-adi-emu";
	indio_dev->info = &adi_emu_info;
	indio_dev->channels = adi_emu_channel;
	indio_dev->num_channels = ARRAY_SIZE(adi_emu_channel);

	return devm_iio_device_register(&spi->dev,indio_dev);
}

static struct spi_driver adi_emu_driver = {
	.driver = {
		.name = "iio-adi-emu",
	},
	.probe = adi_emu_probe,
};
module_spi_driver(adi_emu_driver);

MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADC Emulator Driver");
MODULE_LICENSE("GPL v2");
