
// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADI AD5592R ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define AD5592r_S_REG_ADC_SEQ		0x2
#define  AD559R_S_MASK_REP		BIT(9)
#define AD5592R_S_REG_ADC_CONFIG	0x4
#define  AD5592R_S_MASK_ADC_CH_EN	GENMASK(5,0)
#define AD5592R_S_REG_READBACK		0x7
#define  AD5592R_S_MASK_RDB_EN		BIT(6)
#define  AD5592R_S_MASK_RDB_REG		GENMASK(5, 2)

#define AD5592R_S_ADDR_MASK		GENMASK(14, 11)
#define AD5592R_S_VAL_MASK		GENMASK(10, 0)
#define AD5592R_S_ADC_READ_GENMASK	GENMASK(11, 0)
#define AD5592r_S_ADC_CHAN(x)		BIT(x)
#define AD5592R_S_ADC_ADDR_MASK		GENMASK(14, 12)
#define AD5592R_S_REG_PD_REF_CTRL	0xB
#define AD5592R_S_MASK_EN_REF		BIT(9)
#define AD5592R_S_REG_SW_RESET		0xF
#define AD5592R_S_SW_REST_CMD		0b10110101100

struct ad5592r_s_state {
	struct spi_device *spi;
	bool en;
};

static int ad5592r_s_spi_write(struct ad5592r_s_state *st, u8 reg, u16 val)
{
	u16 msg = 0;
	u16 tx = 0;
	struct spi_transfer xfer = {
		.tx_buf = &tx,
		.len = 2,
	};

	msg |= FIELD_PREP(AD5592R_S_ADDR_MASK, reg);
	msg |= FIELD_PREP(AD5592R_S_VAL_MASK, val);

	put_unaligned_be16(msg, &tx);

	// dev_info(&st->spi->dev, "spi wwrite msg =  %d ", msg);
	// dev_info(&st->spi->dev, "spi wwrite tx=  %d ", tx);

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad5592r_s_spi_nop(struct ad5592r_s_state *st, u16 *val)
{
	struct spi_transfer xfer = {
		.tx_buf = 0,
		.rx_buf = val,
		.len = 2,
	};

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad5592r_s_spi_read_ctl(struct ad5592r_s_state *st, u8 reg, u16 *val)
{
	u16 tx = 0;
	u16 msg = 0;
	u16 rx = 0;
	int ret;

	struct spi_transfer xfer = {
		.tx_buf = &tx,
		.len = 2,
	};

	msg |= FIELD_PREP(AD5592R_S_ADDR_MASK, AD5592R_S_REG_READBACK);
	msg |= AD5592R_S_MASK_RDB_EN;
	msg |= FIELD_PREP(AD5592R_S_MASK_RDB_REG, reg);

	put_unaligned_be16(msg, &tx);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret){
		dev_err(&st->spi->dev, "Failed at SPI WR transfer");
		return ret;
	}

	ret = ad5592r_s_spi_nop(st, &rx);
	if (ret) {
		dev_err(&st->spi->dev, "Failed at SPI WR NOP transfer");
		return ret;
	}
	
	*val = get_unaligned_be16(&rx);
	return 0;
}

static int ad5592r_s_read_adc_chan(struct ad5592r_s_state* st, int chan, int* val)
{
	int ret = 0;
	u16 tmp = 0;
	u8 addr = 0;
	// Select 1 channel for conversion
	// dev_info(&st->spi->dev, "read_adc, chan = %d ", chan);
	// dev_info(&st->spi->dev, "read_adc, chan = %lx ", AD5592r_S_ADC_CHAN(chan));

	ret = ad5592r_s_spi_write(st, AD5592r_S_REG_ADC_SEQ, AD5592r_S_ADC_CHAN(chan));
	if (ret) 
	{
		dev_err(&st->spi->dev, "Writing to ADC_SEQ reg failed ");
		return ret;
	}
	// Wait 1 cycle
	ret = ad5592r_s_spi_nop(st, &tmp);
	if (ret) 
	{
		dev_err(&st->spi->dev, "Intermediate noop failed ");
		return ret;	
	}
	// Result available afterwards
	ret = ad5592r_s_spi_nop(st, &tmp);
	if (ret) 
	{
		dev_err(&st->spi->dev, "Final conversion read failed");
		return ret;	
	}
	
	tmp = get_unaligned_be16(&tmp);

	addr = (tmp & AD5592R_S_ADC_ADDR_MASK) >> 12;
	// dev_info(&st->spi->dev, "req chan = 0x%x, \trecvd addr = 0x%x", chan, addr);
	// dev_info(&st->spi->dev, "tmp val  %x ", tmp);
	if(addr != chan) 
	{
		dev_err(&st->spi->dev, "Requested channel doesn't match read channel");
		return -EINVAL;
	}
	
	tmp &= AD5592R_S_ADC_READ_GENMASK;
	*val = tmp;
	return 0;
}

static int ad5592r_s_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask)
{
	struct ad5592r_s_state *st = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->channel)
		{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
			ret = ad5592r_s_read_adc_chan(st, chan->channel, val);
			if (ret) 
			{
				dev_err(&st->spi->dev, "Error ar read ADC_CH%d", chan->channel);
				return ret;	
			}
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
}

static int ad5592r_s_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ad5592r_s_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		st->en = val;
		return 0;
	default:
		return -EINVAL;
	}
}

// static int ad5592r_s_preenable(struct iio_dev *indio_dev)
// {
// 	struct ad5592r_s_state *st = iio_priv(indio_dev);
// 	int ret;
// 	u16 tmp = 0;

// 	tmp |= *indio_dev->active_scan_mask;
// 	tmp |= AD559R_S_MASK_REP;

// 	ret = ad5592r_s_spi_write(st, AD5592r_S_REG_ADC_SEQ, tmp);

// 	if (ret) {
// 		dev_err(&st->spi->dev, "Failed to set up ADC seq register");
// 		return ret;
// 	}

// 	return ad5592r_s_spi_nop(st, &tmp);
// }


// static int ad5592r_s_predisable(struct iio_dev *indio_dev)
// {
// 	struct ad5592r_s_state *st = iio_priv(indio_dev);
// 	return ad5592r_s_spi_write(st, AD5592r_S_REG_ADC_SEQ, 0x0);
// }


static irqreturn_t ad5592r_s_trigger_handler (int irq, void* p)
{
        struct iio_poll_func *pf = p;
        struct iio_dev *indio_dev = pf->indio_dev;
        struct ad5592r_s_state *st = iio_priv(indio_dev);
        u16 buff[6];
        int bit = 0;
        int i = 0;
        int val;
        u16 tmp;
        int ret;
        for_each_set_bit(bit, indio_dev->active_scan_mask, indio_dev->num_channels) {
                ret = ad5592r_s_read_adc_chan(st, bit, &val);
                if (ret)
                {
                        dev_err(&st->spi->dev, "Error ar read ADC_CH%d", bit);
                        return ret;
                }
                buff[i++] = (u16) val;
        }
        iio_push_to_buffers(indio_dev, buff);
        iio_trigger_notify_done(indio_dev->trig);
        return IRQ_HANDLED;
}

// static irqreturn_t ad5592r_s_trigger_handler (int irq, void* p)
// {
// 	struct iio_poll_func *pf = p;
// 	struct iio_dev *indio_dev = pf->indio_dev;
// 	struct ad5592r_s_state *st = iio_priv(indio_dev);
// 	u16 buff[6];
// 	int bit = 0;
// 	int i = 0;
// 	u16 val;
// 	u16 tmp;
// 	int ret;

// 	for_each_set_bit(bit, indio_dev->active_scan_mask, indio_dev->num_channels) {
// 		ret = ad5592r_s_spi_nop(st, &val);
// 		if (ret) {
// 			dev_err(&st->spi->dev, "Buffer read error at CH %d", bit);
// 			return ret;
// 		}
// 		tmp = get_unaligned_be16(&val);
// 		tmp &= AD5592R_S_ADC_READ_GENMASK;

// 		buff[i++] = tmp;
// 	}

// 	iio_push_to_buffers(indio_dev, buff);
// 	iio_trigger_notify_done(indio_dev->trig);

// 	return IRQ_HANDLED;
// }






static int ad5592r_s_reg_access(struct iio_dev *indio_dev,
				unsigned reg, unsigned writeval,
				unsigned *readval)
{
	struct ad5592r_s_state *st = iio_priv(indio_dev);

	if(readval)
		return ad5592r_s_spi_read_ctl(st, reg, (u16 *)readval);

	return ad5592r_s_spi_write(st, reg, writeval);
}

static const struct iio_info ad5592r_s_info = {
	.read_raw = &ad5592r_s_read_raw,
	.write_raw = &ad5592r_s_write_raw,
	.debugfs_reg_access = &ad5592r_s_reg_access,
};


static const struct iio_buffer_setup_ops ad5592r_s_setup_ops = {
	// .preenable = &ad5592r_s_preenable,
	// .predisable = &ad5592r_s_predisable,
};



static const struct iio_chan_spec ad5592r_s_channel[] = {
	{
		.type = IIO_VOLTAGE,
		.channel = 0,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},

	},
	{
		.type = IIO_VOLTAGE,
		.channel = 1,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 1,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 2,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 2,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 3,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 3,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 4,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 4,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.channel = 5,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 5,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
};


static int ad5592r_s_init(struct iio_dev *indio_dev)
{
	struct ad5592r_s_state *st = iio_priv(indio_dev);
	int ret = 0;
	// enable ADC channels 5 to 0

	ret = ad5592r_s_spi_write(st, AD5592R_S_REG_SW_RESET, AD5592R_S_SW_REST_CMD);
	if (ret)
	{
		return ret;
	}
	usleep_range(250, 300);

	ret = ad5592r_s_spi_write(st, AD5592R_S_REG_ADC_CONFIG, 
				  AD5592R_S_MASK_ADC_CH_EN);
	if (ret)
	{
		return ret;
	}
	//  enable internal Vref source
	ret = ad5592r_s_spi_write(st, AD5592R_S_REG_PD_REF_CTRL, 
				  AD5592R_S_MASK_EN_REF);
	if (ret)
	{
		return ret;
	}

	return 0;
}

static int ad5592r_s_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad5592r_s_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	
	st = iio_priv(indio_dev);
	st->spi = spi;
	st->en = 0;

	indio_dev->name = "ad5592r_s";
	indio_dev->info = &ad5592r_s_info;
	indio_dev->channels = ad5592r_s_channel;
	indio_dev->num_channels = ARRAY_SIZE(ad5592r_s_channel);

	ret = ad5592r_s_init(indio_dev); //set pins as ADC
	if (ret)
	{
		dev_err(&st->spi->dev, "Init failed");
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, NULL, 
					&ad5592r_s_trigger_handler, 
					&ad5592r_s_setup_ops);

	if (ret) {
		dev_err(&st->spi->dev, "Error at triggered buffer setup");
		return ret;
	}

	return devm_iio_device_register(&spi->dev,indio_dev);
}

static struct spi_driver ad5592r_s_driver = {
	.driver = {
		.name = "ad5592r_s",
	},
	.probe = ad5592r_s_probe,
};
module_spi_driver(ad5592r_s_driver);

MODULE_AUTHOR("Radu bogdan Sabau <Radu.Sabau@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R ADC Driver");
MODULE_LICENSE("GPL v2");
