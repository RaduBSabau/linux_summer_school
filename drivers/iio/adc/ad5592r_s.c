// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD5592R SPI ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

#define ADC_SEQ 0x2

#define AD5592R_RD_MASK 	BIT(15)
#define AD5592R_RD_ADDR_MASK	GENMASK(14,11)
#define AD5592R_RD_ADDR3_MASK	GENMASK(14,12)
#define AD5592R_RD_ADC_MASK	GENMASK(11,0)
#define ADI_EMU_ADDR_MASK	GENMASK(14,8)
#define ADI_EMU_VAL_MASK	GENMASK(7,0)
#define AD5592R_WR_ADDR_MASK	GENMASK(14,11)
#define AD5592R_WR_VAL_MASK	GENMASK(8,0)

struct ad5592r_s_state {
	struct spi_device *spi;
	bool en;
	u16 tmp_chan0;
	u16 tmp_chan1;
	u16 tmp_chan2;
	u16 tmp_chan3;
	u16 tmp_chan4;
	u16 tmp_chan5;
};

static int ad5592r_s_read(struct ad5592r_s_state *st, u16 reg, u16 *val)
{
	u16 tx = 0;
	u16 rx = 0;
	u16 msg = 0;
	int ret;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = NULL,
			.rx_buf = NULL,
			.len = 2,
			.cs_change = 1
		},
		{
			.tx_buf = NULL,
			.rx_buf = NULL,
			.len = 2,
		},
		{
			.tx_buf = NULL,
			.rx_buf = NULL,
			.len = 2,
		},
	};

	//prima secventa

	msg |= FIELD_PREP(AD5592R_RD_ADDR_MASK, ADC_SEQ);
	msg |= BIT(reg);

	put_unaligned_be16(msg, &tx);

	xfer[0].tx_buf = &tx;

	//a doua secventa

	u16 tx_null = 0;
	xfer[1].tx_buf = &tx_null;

	//a treia secventa
	//0adrdatabyte12b

	xfer[2].rx_buf = &rx;

	rx &= AD5592R_RD_ADC_MASK;

	ret = spi_sync_transfer(st->spi, xfer, 3);

	if(ret)
		return ret;
	
	*val = rx;

	return 0;
}

static int ad5592r_s_write(struct ad5592r_s_state *st, u16 reg, u16 val)
{
	u16 msg = 0;
	u16 tx = 0;

	struct spi_transfer xfer  = {
		.tx_buf = NULL,
		.len = 2,
	};
	msg |= FIELD_PREP(AD5592R_WR_ADDR_MASK, reg);
	msg |= FIELD_PREP(AD5592R_WR_VAL_MASK, val);

	put_unaligned_be16(msg, &tx);
	xfer.tx_buf = &tx;

	return spi_sync_transfer(st->spi, &xfer, 1);
}

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

static int ad5592r_s_reg_acces(struct iio_dev *indio_dev,
			       unsigned reg, unsigned writeval,
			       unsigned *readval)
{
	struct ad5592r_s_state *st = iio_priv(indio_dev);

	if(readval)
	{
		return ad5592r_s_read(st, reg, (u8 *)readval);
	}

	return ad5592r_s_write(st, reg, writeval);
}

static const struct iio_info ad5592r_s_info = {
	.read_raw = &ad5592r_s_read_raw,
	.write_raw = &ad5592r_s_write_raw,
	.debugfs_reg_access = &ad5592r_s_reg_acces,
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

	//configurare ca adc

	st->spi = spi;
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