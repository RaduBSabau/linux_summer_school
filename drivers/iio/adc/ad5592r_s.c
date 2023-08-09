// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD5592R SPI ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

struct ad5592r_s_state {
	bool en;
	u16 tmp_chan0;
	u16 tmp_chan1;
	u16 tmp_chan2;
	u16 tmp_chan3;
	u16 tmp_chan4;
	u16 tmp_chan5;
};

static int ad5592r_s_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask)
{
	struct ad5592r_s_state *st = iio_priv(indio_dev);
	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		switch(chan->channel){
			case 0:
				*val = st->tmp_chan0;
				return IIO_VAL_INT;
			case 1:
		 		*val = st->tmp_chan1;
				return IIO_VAL_INT;
			case 2:
				*val = st->tmp_chan2;
				return IIO_VAL_INT;
			case 3:
				*val = st->tmp_chan3;
				return IIO_VAL_INT;
			case 4:
				*val = st->tmp_chan4;
				return IIO_VAL_INT;
			case 5:
				*val = st->tmp_chan5;
				return IIO_VAL_INT;
			default :
				return -EINVAL;
		}
	case IIO_CHAN_INFO_ENABLE:
		*val = st->en;
	default :
		return -EINVAL;
	
	}
}

static int ad5592r_s_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ad5592r_s_state *st = iio_priv(indio_dev);

	switch(mask)
	{
		case IIO_CHAN_INFO_RAW:
			switch(chan->channel)
			{
				case 0:
					st->tmp_chan0 = val;
					return 0;
				case 1:
					st->tmp_chan1 = val;
					return 0;
				case 2:
					st->tmp_chan2 = val;
					return 0;
				case 3:
					st->tmp_chan3 = val;
					return 0;
				case 4:
					st->tmp_chan4 = val;
					return 0;
				case 5:
					st->tmp_chan5 = val;
					return 0;
				default:
					return -EINVAL;
			}
		case IIO_CHAN_INFO_ENABLE:
			st->en = val;
			return 0;
		default:
			return -EINVAL;
	}
}

static const struct iio_info ad5592r_s_info = {
	.read_raw = &ad5592r_s_read_raw,
	.write_raw = &ad5592r_s_write_raw,
};

static const struct iio_chan_spec ad5592r_s_channel[] = {
	{
		.type = IIO_VOLTAGE,
		.channel = 0,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 1,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 2,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 3,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 4,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 5,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int ad5592r_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad5592r_s_state *st;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if(!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->en = 0;
	st->tmp_chan0 = 0;
	st->tmp_chan1 = 0;
	st->tmp_chan2 = 0;
	st->tmp_chan3 = 0;
	st->tmp_chan4 = 0;
	st->tmp_chan5 = 0;
	
	indio_dev->name = "ad5592r_s";
	indio_dev->info = &ad5592r_s_info;
	indio_dev->channels = ad5592r_s_channel;
	indio_dev->num_channels = ARRAY_SIZE(ad5592r_s_channel);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad5592r_s_driver = {
	.driver = {
		.name = "ad5592r_s",
	},
	.probe = ad5592r_probe,
};

module_spi_driver (ad5592r_s_driver);

MODULE_AUTHOR("Radu bogdan Sabau <Radu.Sabau@analog.com");
MODULE_DESCRIPTION("Analog Devices AD5592R ADC Driver");
MODULE_LICENSE("GPL v2");