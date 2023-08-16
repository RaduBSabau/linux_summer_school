// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices AD5592R SPI ADC driver
 *
 * Copyright (C) 2023 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

#include <asm/unaligned.h>
#include <linux/bitfield.h>

#define AD5592R_REG_READBACK	0x7
#define AD5592R_RDB_MASK_EN	BIT(6)
#define AD5592R_RDB_MASK_REG	GENMASK(5,2)

#define AD5592R_ADDR_MASK	GENMASK(14,11)
#define AD5592R_VAL_MASK	GENMASK(10,0)


struct ad5592r_state{
	bool en;
	u16 tmp_chan0;
	u16 tmp_chan1;
	u16 tmp_chan2;
	u16 tmp_chan3;
	u16 tmp_chan4;
	u16 tmp_chan5;

	struct spi_device *spi;
};

static int ad5592r_spi_nop(struct ad5592r_state *st, u16 *val)
{

	struct spi_transfer xfer = {
		.tx_buf = 0,
		.rx_buf = val,
		.len = 2,		
	};

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad5592r_spi_read_ctl(struct ad5592r_state *st, u8 reg, u16 *val)
{
	u16 tx = 0;
	u16 rx = 0;
	u16 msg = 0;
	int ret;

	struct spi_transfer xfer = {
		.tx_buf = &tx,
		.len = 2,
	};

	msg |= FIELD_PREP(AD5592R_ADDR_MASK, AD5592R_REG_READBACK);
	msg |= AD5592R_RDB_MASK_EN;
	msg |= FIELD_PREP(AD5592R_RDB_MASK_REG, reg);

	put_unaligned_be16(msg, &tx);
	
	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if(ret)
	{
		dev_err(&st->spi->dev, "Failed at SPI WR transfer");
		return ret;
	}

	ret = ad5592r_spi_nop(st, &rx);
	if(ret)
	{
		dev_err(&st->spi->dev, "Failed at SPI WR NOP transfer");
		return ret;
	}

	*val = get_unaligned_be16(&rx);

	return 0;
}

static int ad5592r_spi_write(struct ad5592r_state *st, u16 reg, u16 val)
{
	u16 msg = 0;
	u16 tx = 0;

	struct spi_transfer xfer  = {
		.tx_buf = &tx,
		.len = 2,
	};

	msg |= FIELD_PREP(AD5592R_ADDR_MASK, reg);
	msg |= FIELD_PREP(AD5592R_VAL_MASK, val);

	dev_info(&st->spi->dev, "msg at write = 0%x", msg);

	put_unaligned_be16(msg, &tx);

	dev_info(&st->spi->dev, "tx at write = 0%x", tx);

	return spi_sync_transfer(st->spi, &xfer, 1);
}


static int ad5592r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct ad5592r_state *st = iio_priv(indio_dev);	
	switch (mask)
	{
		case IIO_CHAN_INFO_RAW:
			switch (chan->channel)
			{
				case 0:
					st->tmp_chan0 = val;
					break;
				case 1:
					st->tmp_chan1 = val;
					break;

				case 2:
					st->tmp_chan2 = val;
					break;

				case 3:
					st->tmp_chan3 = val;
					break;

				case 4:
					st->tmp_chan4 = val;
					break;

				case 5:
					st->tmp_chan5 = val;
					break;
				default:
					return -EINVAL;
			}
			return 0;
		case IIO_CHAN_INFO_ENABLE:
			st->en = val;
			return 0;
		default:
			return -EINVAL;
	}
};

static int ad5592r_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask)
{
	struct ad5592r_state *st = iio_priv(indio_dev);
	switch(mask){
		case IIO_CHAN_INFO_RAW:
			switch (chan->channel)
			{
				case 0:
					*val = st->tmp_chan0;
					break;
				case 1:
					*val = st->tmp_chan1;
					break;

				case 2:
					*val = st->tmp_chan2;
					break;

				case 3:
					*val = st->tmp_chan3;
					break;

				case 4:
					*val = st->tmp_chan4;
					break;

				case 5:
					*val = st->tmp_chan5;
					break;
				
				default:
					return -EINVAL;
			}
			return IIO_VAL_INT;

		case IIO_CHAN_INFO_ENABLE:
			*val = st->en;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
	}
};


static int ad5592r_reg_access(struct iio_dev *indio_dev,
			       unsigned reg, unsigned writeval,
			       unsigned *readval)
{
	struct ad5592r_state *st = iio_priv(indio_dev);

	if(readval)
	{
		return ad5592r_spi_read_ctl(st, reg, (u16 *)readval);
	}

	return ad5592r_spi_write(st, reg, writeval);
}

static int ad5592r_adc_init(struct iio_dev *indio_dev, // return by iio 
			        unsigned int reg,
			        unsigned int writeval)
{
	struct ad5592r_state *st = iio_priv(indio_dev);
	return ad5592r_spi_write(st, reg, writeval);
}

static const struct iio_info ad5592r_s_info = {
	.read_raw = &ad5592r_read_raw,
	.write_raw = &ad5592r_write_raw,
	.debugfs_reg_access = &ad5592r_reg_access,
};

static const struct iio_chan_spec ad5592r_channel[] ={
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
	struct ad5592r_state *st;

	indio_dev = devm_iio_device_alloc(&spi->dev, 0);
	if(!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	//spi->mode = 3;

	st->spi = spi;
	st->en = 0;
	st->tmp_chan0 = 0;
	st->tmp_chan1 = 0;
	st->tmp_chan2 = 0;
	st->tmp_chan3 = 0;
	st->tmp_chan4 = 0;
	st->tmp_chan5 = 0;

	// int ret;
	// ret = ad5592r_adc_init(indio_dev, ADC_CONFIG, ADI_ADC_EN_MASK);
	// if(ret)
	// 	return ret;
	
	indio_dev->name = "ad5592r_s";
	indio_dev->info = &ad5592r_s_info;
	indio_dev->channels = ad5592r_channel;
	indio_dev->num_channels = ARRAY_SIZE(ad5592r_channel);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad5592r_driver = {
	.driver = {
		.name = "ad5592r_s",
	},
	.probe = ad5592r_probe,
};

module_spi_driver (ad5592r_driver);

MODULE_AUTHOR("Radu bogdan Sabau <Radu.Sabau@analog.com");
MODULE_DESCRIPTION("Analog Devices AD5592R ADC Driver");
MODULE_LICENSE("GPL v2");
