/*
 * AD777X ADC
 *
 * Copyright 2022 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/clk.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "cf_axi_adc.h"

#define AD777X_SPI_READ_CMD			0x80
#define AD777X_SPI_WRITE_CMD			0x00

#define AD777X_ENABLE_CRC			0x3F
#define AD777X_ENABLE_SAR			0x29
#define AD777X_DISABLE_SAR			0x09
#define AD777X_PDB_SAR				0x2C
#define AD777X_PDB_SAR_OFF			0x24
#define AD777X_ENABLE_SD			0x90
#define AD777X_DISABLE_SD			0x80


#define AD777X_REG_CH_CONFIG(ch)		(0x00 + (ch))		// Channel Configuration
#define AD777X_REG_CH_DISABLE			0x08			// Disable clocks to ADC channel
#define AD777X_REG_CH_SYNC_OFFSET(ch)		(0x09 + (ch))		// Channel SYNC Offset
#define AD777X_REG_GENERAL_USER_CONFIG_1	0x11			// General User Config 1
#define AD777X_REG_GENERAL_USER_CONFIG_2	0x12			// General User Config 2
#define AD777X_REG_GENERAL_USER_CONFIG_3	0x13			// General User Config 3
#define AD777X_REG_DOUT_FORMAT			0x14			// Data out format
#define AD777X_REG_ADC_MUX_CONFIG		0x15			// Main ADC meter and reference Mux control
#define AD777X_REG_GLOBAL_MUX_CONFIG		0x16			// Global diagnostics mux
#define AD777X_REG_GPIO_CONFIG			0x17			// GPIO config
#define AD777X_REG_GPIO_DATA			0x18			// GPIO Data
#define AD777X_REG_BUFFER_CONFIG_1		0x19			// Buffer Config 1
#define AD777X_REG_BUFFER_CONFIG_2		0x1A			// Buffer Config 2
#define AD777X_REG_CH_OFFSET_UPPER_BYTE(ch)	(0x1C + (ch) * 6)	// Channel offset upper byte
#define AD777X_REG_CH_OFFSET_MID_BYTE(ch)	(0x1D + (ch) * 6)	// Channel offset middle byte
#define AD777X_REG_CH_OFFSET_LOWER_BYTE(ch)	(0x1E + (ch) * 6)	// Channel offset lower byte
#define AD777X_REG_CH_GAIN_UPPER_BYTE(ch)	(0x1F + (ch) * 6)	// Channel gain upper byte
#define AD777X_REG_CH_GAIN_MID_BYTE(ch)		(0x20 + (ch) * 6)	// Channel gain middle byte
#define AD777X_REG_CH_GAIN_LOWER_BYTE(ch)	(0x21 + (ch) * 6)	// Channel gain lower byte
#define AD777X_REG_CH_ERR_REG(ch)		(0x4C + (ch))		// Channel Status Register
#define AD777X_REG_CH0_1_SAT_ERR		0x54			// Channel 0/1 DSP errors
#define AD777X_REG_CH2_3_SAT_ERR		0x55			// Channel 2/3 DSP errors
#define AD777X_REG_CH4_5_SAT_ERR		0x56			// Channel 4/5 DSP errors
#define AD777X_REG_CH6_7_SAT_ERR		0x57			// Channel 6/7 DSP errors
#define AD777X_REG_CHX_ERR_REG_EN		0x58			// Channel 0-7 Error Reg Enable
#define AD777X_REG_GEN_ERR_REG_1		0x59			// General Errors Register 1
#define AD777X_REG_GEN_ERR_REG_1_EN		0x5A			// General Errors Register 1 Enable
#define AD777X_REG_GEN_ERR_REG_2		0x5B			// General Errors Register 2
#define AD777X_REG_GEN_ERR_REG_2_EN		0x5C			// General Errors Register 2 Enable
#define AD777X_REG_STATUS_REG_1			0x5D			// Error Status Register 1
#define AD777X_REG_STATUS_REG_2			0x5E			// Error Status Register 2
#define AD777X_REG_STATUS_REG_3			0x5F			// Error Status Register 3
#define AD777X_REG_SRC_N_MSB			0x60			// Decimation Rate (N) MSB
#define AD777X_REG_SRC_N_LSB			0x61			// Decimation Rate (N) LSB
#define AD777X_REG_SRC_IF_MSB			0x62			// Decimation Rate (IF) MSB
#define AD777X_REG_SRC_IF_LSB			0x63			// Decimation Rate (IF) LSB
#define AD777X_REG_SRC_UPDATE			0x64			// SRC load source and load update

#define AD777X_CH_GAIN(x)			(((x) & 0x3) << 6)
#define AD777X_CH_RX				(1 << 4)

#define AD777X_POWER_MODE_HIGH			(1 << 6)
#define AD777X_POWER_MODE_LOW			0x10111111

#define AD777X_FILTER_SINC5			BIT(6)
#define AD777X_FILTER_SINC3			0x10111111

#define AD777X_LOW_POWER_MODE			0
#define AD777X_HIGH_POWER_MODE			1

/* AD777X_REG_DOUT_FORMAT */
#define AD777X_DOUT_FORMAT(x)			(((x) & 0x3) << 6)
#define AD777X_DOUT_HEADER_FORMAT		(1 << 5)
#define AD777X_DCLK_CLK_DIV(x)			(((x) & 0x7) << 1)

#define AD777X_DOUT_FORMAT_00			2
#define AD777X_DOUT_FORMAT_01			4
#define AD777X_DOUT_FORMAT_10			8

#define AD777X_DCLK_DIV_HIGH			8
#define AD777X_DCLK_DIV_LOW			4
#define AD777X_MAX_DCLK_DIV			128
#define AD777X_MIN_DCLK_DIV_4_LINES		4
#define AD777X_MIN_DCLK_DIV_2_LINES		2
#define AD777X_MIN_DCLK_DIV_1_LINE		1

#define AD777X_CLKRATE_DIV_HIGH			4
#define AD777X_CLKRATE_DIV_LOW			8

#define CLK_MAX_RATE				8192000
#define AD777X_NUM_CHANNELS			8

/* AXI CONTROL REGS VALUES FOR DATA LINES */
#define AXI_CTRL_4_LINES			0x400
#define AXI_CTRL_2_LINES			0x200
#define AXI_CTRL_1_LINE				0x100

#define SHIFT_16				65536
#define DEC3					1000

#define GAIN_REL				0x555555

#define HEADER_0				0
#define HEADER_1				1
#define HEADER_2				2

#define AD777X_CRC8_POLY	0x07
DECLARE_CRC8_TABLE(ad777x_crc8_table);

enum ad777x_data_lines {
	AD777x_4LINES,
	AD777x_2LINES,
	AD777x_1LINE,

};

enum ad777x_filter {
	AD777X_SINC3,
	AD777X_SINC5,
};

enum ad777x_variant {
	ad7770,
	ad7771,
	ad7779,
};

enum ad777x_power_mode {
	AD777X_LOW_POWER,
	AD777X_HIGH_POWER,
};

struct ad777x_state {
	struct spi_device 	*spi;
	struct clk 		*mclk;
	struct regulator	*vref;
	struct gpio_chip	gpiochip;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*start_gpio;
	unsigned int		sampling_freq;
	enum ad777x_power_mode 	power_mode;
	unsigned int 		decimation;
	enum ad777x_data_lines 	data_lines;
	enum ad777x_filter	filter_enabled;
	struct iio_trigger 	*dready_trig;
	
	unsigned int 		crc_enabled;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			reg_rx_buf[3] ____cacheline_aligned;
	u8			reg_tx_buf[3];
	u8			spidata_buf_rx[32];
	u8			spidata_buf_tx[32];
};

static const char * const ad777x_filter_type[] = {
	[AD777X_SINC3] = "sinc3_filter",
	[AD777X_SINC5] = "sinc5_filter",
};

static const char * const ad777x_data_lines_modes[] = {
	[AD777x_4LINES] = "4_data_lines",
	[AD777x_2LINES] = "2_data_lines",
	[AD777x_1LINE]  = "1_data_line",
};


static const char * const ad777x_power_modes[] = {
        [AD777X_LOW_POWER] = "low_power_mode",
        [AD777X_HIGH_POWER] = "high_power_mode",
};

static bool ad777x_has_axi_adc(struct device *dev)
{
	return device_property_present(dev, "spibus-connected");
}

static struct ad777x_state *ad777x_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	if(ad777x_has_axi_adc(&indio_dev->dev)) {
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

static int ad777x_spi_read(struct iio_dev *indio_dev, u8 reg, u8 *rbuf) {
	struct ad777x_state *spi_st = ad777x_get_data(indio_dev);
	int ret;
	int length = 2;
	u8 crc_buf[2];
	u8 exp_crc = 0;
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
			.rx_buf = spi_st->reg_rx_buf,
		},
	};

	if(spi_st->crc_enabled)
		length = 3;
	reg_read_tr[0].len = length;	

	spi_st->reg_tx_buf[0] = AD777X_SPI_READ_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = 0;
	spi_st->reg_tx_buf[2] = crc8(ad777x_crc8_table, 
					spi_st->reg_tx_buf, 2, 0);

	ret = spi_sync_transfer(spi_st->spi, reg_read_tr, 
				ARRAY_SIZE(reg_read_tr));
	
	crc_buf[0] = AD777X_SPI_READ_CMD | (reg & 0x7F);
	crc_buf[1] = spi_st->reg_rx_buf[1];
	exp_crc = crc8(ad777x_crc8_table, crc_buf, 2, 0);
	if(spi_st->crc_enabled && (exp_crc != spi_st->reg_rx_buf[2])) {
		dev_err(&spi_st->spi->dev, "Bad CRC %x, expected %x",
			spi_st->reg_rx_buf[2], exp_crc);
		return -EINVAL;
	}
	*rbuf = spi_st->reg_rx_buf[1];

	return ret;
} 

int ad777x_spi_write(struct iio_dev *indio_dev, u8 reg, u8 val) {
	struct ad777x_state *spi_st = ad777x_get_data(indio_dev);
	int length = 2;
	struct spi_transfer reg_write_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
		},
	};

	if(spi_st->crc_enabled)
		length = 3;
	reg_write_tr[0].len = length;

	spi_st->reg_tx_buf[0] = AD777X_SPI_WRITE_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = val;
	spi_st->reg_tx_buf[2] = crc8(ad777x_crc8_table,
					spi_st->reg_tx_buf, 2, 0);
	
	return spi_sync_transfer(spi_st->spi, reg_write_tr,
				ARRAY_SIZE(reg_write_tr));
}

static int ad777x_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int writeval,
				unsigned int *readval) {
	int ret;

	if(readval)
		ret = ad777x_spi_read(indio_dev, reg, (u8 *) readval);
	else
		ret = ad777x_spi_write(indio_dev, reg, writeval);

	return ret;
}

static int ad777x_set_sampling_frequency(struct iio_dev *indio_dev,
					unsigned int sampling_freq)
{
	struct ad777x_state *st = ad777x_get_data(indio_dev);
	int ret;
	unsigned long dec;
	unsigned int dclk;
	unsigned int dec_div, clk_div;
	unsigned int div;
	int temp;
	int decimal;

	u8 msb, lsb;

	if (st->power_mode) {
		clk_div = AD777X_CLKRATE_DIV_HIGH;
		dec_div = AD777X_DCLK_DIV_HIGH;
	} else {
		clk_div = AD777X_CLKRATE_DIV_LOW;
		dec_div = AD777X_DCLK_DIV_LOW;
	}
	dec = (unsigned long) clk_get_rate(st->mclk) / 
			(clk_div * sampling_freq);

	lsb = (int) dec & 0xFF;
	msb = ((int) dec >> 8) & 0xFF;
	switch (st->data_lines) {
	case AD777x_4LINES:
		div = dec / (dec_div * AD777X_DOUT_FORMAT_00);
		if ( div < AD777X_MIN_DCLK_DIV_4_LINES)
			return -EINVAL;
		break;
	case AD777x_2LINES:
		div = dec / (dec_div * AD777X_DOUT_FORMAT_01);
		if ( div < AD777X_MIN_DCLK_DIV_2_LINES)
			return -EINVAL;
		break;
	case AD777x_1LINE:
		div = dec / (dec_div * AD777X_DOUT_FORMAT_10);
		if ( div < AD777X_MIN_DCLK_DIV_1_LINE)
			return -EINVAL;
		break;
	}

	if ( div > AD777X_MAX_DCLK_DIV )
		return -EINVAL;

	// switch (div)
	// {
	// case 1:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x20);
	// 	break;
	// case 2:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x22);
	// 	break;
	// case 4:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x24);
	// 	break;
	// case 8:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x26);
	// 	break;
	// case 16:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x28);
	// 	break;
	// case 32:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x2A);
	// 	break;
	// case 64:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x2C);
	// 	break;
	// case 128:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x2E);
	// 	break;
	
	// default:
	// 	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 
	// 		0x24);
	// 	break;
	// }
	dev_info(&st->spi->dev, "The mclk value from device is %x", clk_get_rate(st->mclk));
	dev_info(&st->spi->dev, "The calculated decimation is %x", dec);
	dev_info(&st->spi->dev, "The calculated div is %x", div);
	dev_info(&st->spi->dev, "The msb calculated value was msb %x and lsb %x", msb, lsb);
	
	ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_N_LSB, lsb);
	ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_N_MSB, msb);

	dev_info(&st->spi->dev, "The calculated decimal is %x", dec);
	if (clk_get_rate(st->mclk) % (clk_div * sampling_freq) != 0) {
		temp = clk_get_rate(st->mclk) * DEC3 
			/ (clk_div * sampling_freq);
		decimal = ((temp -  dec * DEC3) * SHIFT_16) / DEC3;
		dev_info(&st->spi->dev, "The calculated decimal is %x", decimal);
		lsb = (int) decimal & 0xFF;
		msb = ((int) decimal >> 8) & 0xFF;
		dev_info(&st->spi->dev, "The msb calculated IF msb %x and lsb %x", msb, lsb);
		ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_IF_LSB, lsb);
		ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_IF_MSB, msb);
	} else {
		ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_IF_LSB, 0x0);
		ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_IF_MSB, 0x0);
	}
	ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_UPDATE, 0x1);
	ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_UPDATE, 0x0);

	if(ret)
		return ret;
	else
		st->sampling_freq = sampling_freq;
		// TODO: also set decimation???
	return 0;
}

static int ad7779_set_power_mode(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan,
                                unsigned int power_mode)
{
        struct ad777x_state *st = ad777x_get_data(indio_dev);
        int ret;
        u8 temp;
        if(power_mode) {
                ret = ad777x_spi_read(indio_dev, AD777X_REG_GENERAL_USER_CONFIG_1, &temp);
                temp |= AD777X_POWER_MODE_HIGH;
                ret = ad777x_spi_write(indio_dev, AD777X_REG_GENERAL_USER_CONFIG_1, temp);
        } else {
                ret = ad777x_spi_read(indio_dev, AD777X_REG_GENERAL_USER_CONFIG_1, &temp);
                temp |= AD777X_POWER_MODE_LOW;
                ret = ad777x_spi_write(indio_dev, AD777X_REG_GENERAL_USER_CONFIG_1, temp);
        }
        if (ret)
                return -EINVAL;
        st->power_mode = power_mode;
        return 0;
}
static int ad7779_get_power_mode(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan)
{
        struct ad777x_state *st = ad777x_get_data(indio_dev);
        return st->power_mode;
}

static int ad777x_set_data_lines(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				unsigned int mode)
{
	struct ad777x_state *st = ad777x_get_data(indio_dev);
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);

	switch (mode) {
	case AD777x_4LINES:
		ad777x_set_sampling_frequency(indio_dev, 8000);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL, AXI_CTRL_4_LINES);
		break;
	case AD777x_2LINES:
		ad777x_set_sampling_frequency(indio_dev, 4000);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL, AXI_CTRL_2_LINES);	
		break;
	case AD777x_1LINE:
		ad777x_set_sampling_frequency(indio_dev, 2000);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL, AXI_CTRL_1_LINE);
		break;
	default:
		return -EINVAL;
	}

	ad777x_spi_write(indio_dev, AD777X_REG_SRC_UPDATE, 0x01);
	ad777x_spi_write(indio_dev, AD777X_REG_SRC_UPDATE, 0x00);
  


	st->data_lines = mode;

	return 0;
}

static int ad777x_get_data_lines(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan)
{
	struct ad777x_state *st = ad777x_get_data(indio_dev);

	return st->data_lines;
}

static int ad777x_get_filter(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan)
{
	struct ad777x_state *st = ad777x_get_data(indio_dev);

	return st->filter_enabled;
}

static int ad777x_set_filter(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				unsigned int mode)
{
	struct ad777x_state *st = ad777x_get_data(indio_dev);
	int ret = 0;
	u8 temp;

	if (spi_get_device_id(st->spi)->driver_data == ad7771) {
		ad777x_spi_read(indio_dev, AD777X_REG_GENERAL_USER_CONFIG_2,
				&temp);
		if (mode) {
			temp |= AD777X_FILTER_SINC5;
		} else {
			temp &= AD777X_FILTER_SINC3;
		}
		ret = ad777x_spi_write(indio_dev,
				AD777X_REG_GENERAL_USER_CONFIG_2,
				temp);
		st->filter_enabled = mode;
	}

	return ret;
}

static int ad777x_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask) 
{
	int ret;
	struct ad777x_state *st = ad777x_get_data(indio_dev);
	u8 temp;
	u8 low, mid, high;
	int gain;
	
	if(iio_buffer_enabled(indio_dev)) {
		dev_info(&st->spi->dev, "iio_buffer_enabled_detected");
		return -EBUSY;
	}

	switch(mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = ad777x_spi_read(indio_dev,
					AD777X_REG_CH_GAIN_LOWER_BYTE(chan->channel),
					&low);
		ret = ad777x_spi_read(indio_dev,
					AD777X_REG_CH_GAIN_MID_BYTE(chan->channel),
					&mid);
		ret = ad777x_spi_read(indio_dev,
					AD777X_REG_CH_GAIN_UPPER_BYTE(chan->channel),
					&high);
		dev_info(&st->spi->dev, "The gain bytes are %x %x %x ",high, mid, low);
		if(ret)
			return ret;
		dev_info(&st->spi->dev, "The gain bytes are %x %x %x ",high, mid, low);
		gain = (high << 16) | (mid << 8) | low;
		*val = gain;
		*val2 = GAIN_REL;
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_CALIBBIAS:
		dev_info(&st->spi->dev, "in calibbias");
		ret = ad777x_spi_read(indio_dev,
					AD777X_REG_CH_OFFSET_LOWER_BYTE(chan->channel),
					&low);
		ret = ad777x_spi_read(indio_dev,
					AD777X_REG_CH_OFFSET_MID_BYTE(chan->channel),
					&mid);
		ret = ad777x_spi_read(indio_dev,
					AD777X_REG_CH_OFFSET_UPPER_BYTE(chan->channel),
					&high);
		if(ret)
			return ret;
		*val = (high << 16) | (mid << 8) | low;
		dev_info(&st->spi->dev, "The offset bytes are %x %x %x ",high, mid, low);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad777x_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val,
			int val2,
			long mask) 
{
	int ret;
	u8 msb, mid, lsb;

	switch(mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = ad777x_spi_write(indio_dev,
					AD777X_CH_GAIN(chan->channel),
					val);
		return ret;
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = ad777x_spi_write(indio_dev,
					AD777X_CH_GAIN(chan->channel),
					val);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ad777x_set_sampling_frequency(indio_dev, val);
		return ret;
	}

	return -EINVAL;
}

static int ad777x_buffer_preenable(struct iio_dev *indio_dev)
{	int ret;
	struct ad777x_state *st = ad777x_get_data(indio_dev);

	ret = ad777x_spi_write(indio_dev,
				AD777X_REG_GENERAL_USER_CONFIG_3,
				AD777X_ENABLE_SD);
	return ret;
}

static int ad777x_buffer_postdisable(struct iio_dev *indio_dev)
{
	int ret;
	struct ad777x_state *st = ad777x_get_data(indio_dev);

	ret = ad777x_spi_write(indio_dev,
				AD777X_REG_GENERAL_USER_CONFIG_3,
				AD777X_DISABLE_SD);
	return ret;
}

static irqreturn_t ad777x_irq_handler(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ad777x_state *st = ad777x_get_data(indio_dev);
	int ret;
	int bit;
	int i;

	struct spi_transfer sd_readback_tr[] = {
		{
			.rx_buf = st->spidata_buf_rx,
			.tx_buf = st->spidata_buf_tx,
			.len = 32,
		}
	};

	if (iio_buffer_enabled(indio_dev)) {
		st->spidata_buf_tx[0] = AD777X_SPI_READ_CMD;
		ret = spi_sync_transfer(st->spi, sd_readback_tr,
					ARRAY_SIZE(sd_readback_tr));
	
		if (ret) {
			dev_err(&st->spi->dev, "Spi fail in irq handler");
			return IRQ_HANDLED;
		}
		iio_push_to_buffers(indio_dev, st->spidata_buf_rx);
	}

	return IRQ_HANDLED;
}

static irqreturn_t ad777x_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad777x_state *st = ad777x_get_data(indio_dev);
	int ret;
	int bit = 0;
	u8 data_buf[3];

	struct spi_transfer sd_readback_tr[] = {
		{
			.rx_buf = st->reg_rx_buf,
			.tx_buf = st->reg_tx_buf,
			.len = 4,
		}
	};

	st->reg_tx_buf[0] = AD777X_SPI_READ_CMD;
	st->reg_tx_buf[1] = 0;
	st->reg_tx_buf[2] = 0;
	st->reg_tx_buf[3] = 0;

	
	for_each_set_bit(bit ,indio_dev->active_scan_mask, 
				indio_dev->num_channels) {
		ret = spi_sync_transfer(st->spi, sd_readback_tr,
			ARRAY_SIZE(sd_readback_tr));
		if(ret)
			return ret;
		data_buf[0] = st->reg_rx_buf[1];
		data_buf[1] = st->reg_rx_buf[2];
		data_buf[2] = st->reg_rx_buf[3];
	}
	iio_push_to_buffers_with_timestamp(indio_dev, data_buf,
					iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ad777x_dready_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ad777x_state *st = ad777x_get_data(indio_dev);

	// if(state)
	// 	st->trigcnt = state;
	
	return 0;
}

static int ad777x_soft_reset(struct iio_dev *indio_dev)
{
	struct ad777x_state *spi_st = iio_priv(indio_dev);
	u8 reset_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = &reset_buf[0],
			.len = 8,
		},
	};

	return spi_sync_transfer(spi_st->spi, reg_read_tr,
				ARRAY_SIZE(reg_read_tr));
}

static const struct iio_info ad777x_info = {
	.read_raw = ad777x_read_raw,
	.write_raw = ad777x_write_raw,
	.debugfs_reg_access = &ad777x_reg_access,
};

static const struct iio_enum ad777x_data_lines_enum = {
	.items = ad777x_data_lines_modes,
	.num_items = ARRAY_SIZE(ad777x_data_lines_modes),
	.get = ad777x_get_data_lines,
	.set = ad777x_set_data_lines,
};

static const struct iio_enum ad777x_filter_enum = {
	.items = ad777x_filter_type,
	.num_items = ARRAY_SIZE(ad777x_filter_type),
	.get = ad777x_get_filter,
	.set = ad777x_set_filter,
};

static const struct iio_enum ad777x_power_mode_enum = {
        .items = ad777x_power_modes,
        .num_items = ARRAY_SIZE(ad777x_power_modes),
        .get = ad7779_get_power_mode,
        .set = ad7779_set_power_mode,
};

static const struct iio_chan_spec_ext_info ad777x_ext_info[] = {
	IIO_ENUM("data_lines", IIO_SHARED_BY_ALL, &ad777x_data_lines_enum),
	IIO_ENUM_AVAILABLE_SHARED("data_lines", IIO_SHARED_BY_ALL, &ad777x_data_lines_enum),
	IIO_ENUM("filter_type", IIO_SHARED_BY_ALL, &ad777x_filter_enum),
	IIO_ENUM_AVAILABLE_SHARED("filter_type", IIO_SHARED_BY_ALL, &ad777x_filter_enum),
	IIO_ENUM("power_mode", IIO_SHARED_BY_ALL, &ad777x_power_mode_enum),
        IIO_ENUM_AVAILABLE_SHARED("power_mode", IIO_SHARED_BY_ALL, &ad777x_power_mode_enum),
	{ },
};

#define AD777x_CHAN(index)							\
	{									\
		.type = IIO_VOLTAGE,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE)  |		\
					BIT(IIO_CHAN_INFO_CALIBBIAS),		\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.address = index,						\
		.indexed = 1,							\
		.channel = index,						\
		.scan_index = index,						\
		.ext_info = ad777x_ext_info,					\
		.scan_type = {							\
			.sign = 's',						\
			.realbits = 24,						\
			.storagebits = 32,					\
		},								\
	}


static const struct iio_chan_spec ad777x_channels[] = {
	AD777x_CHAN(0),
	AD777x_CHAN(1),
	AD777x_CHAN(2),
	AD777x_CHAN(3),
	AD777x_CHAN(4),
	AD777x_CHAN(5),
	AD777x_CHAN(6),
	AD777x_CHAN(7),
};

const struct iio_buffer_setup_ops ad777x_buffer_setup_ops = {
	.preenable = ad777x_buffer_preenable,
	.postdisable = ad777x_buffer_postdisable,
};

static const struct axiadc_chip_info conv_chip_info = {
	.name = "ad777x_axi_adc",
	.max_rate = 4096000UL,
	.num_channels = 8,
	.channel[0] = AD777x_CHAN(0),
	.channel[1] = AD777x_CHAN(1),
	.channel[2] = AD777x_CHAN(2),
	.channel[3] = AD777x_CHAN(3),
	.channel[4] = AD777x_CHAN(4),
	.channel[5] = AD777x_CHAN(5),
	.channel[6] = AD777x_CHAN(6),
	.channel[7] = AD777x_CHAN(7),
};


static void ad777x_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void ad777x_reg_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

int ad777x_gpio_init(struct ad777x_state *st) 
{
	st->gpiochip.label = "ad777x_gpios";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 2;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.owner = THIS_MODULE;

	return gpiochip_add_data(&st->gpiochip, st);
}

static int ad777x_register_axi(struct ad777x_state *st)
{
	struct axiadc_converter *conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->clk = st->mclk;
	conv->chip_info = &conv_chip_info;
	conv->reg_access = &ad777x_reg_access;
	conv->write_raw = &ad777x_write_raw;
	conv->read_raw = &ad777x_read_raw;
	//conv->read_label = &ad777x_read_label;
	conv->phy = st;
	spi_set_drvdata(st->spi, conv);

	return 0;
}

static int ad777x_register(struct ad777x_state *st, struct iio_dev *indio_dev)
{
	int ret;
	int irq;
	struct iio_buffer *buffer;

	indio_dev->dev.parent = &st->spi->dev;
	indio_dev->name = spi_get_device_id(st->spi)->name;
	indio_dev->info = &ad777x_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = ad777x_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad777x_channels);

	irq = of_irq_get((&st->spi->dev)->of_node, 0);
	if (irq <= 0) {
		dev_err(&st->spi->dev, "irq trigger fail\n");
		return -EINVAL;
	}
	

	ret = devm_request_threaded_irq(&st->spi->dev, irq, NULL,
					ad777x_irq_handler, IRQF_ONESHOT,
					indio_dev->name, indio_dev);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret, "request irq %d fail\n",
				irq);

	buffer = devm_iio_kfifo_allocate(&st->spi->dev);
	if (!buffer)
		return -ENOMEM;


	indio_dev->setup_ops = &ad777x_buffer_setup_ops;
	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static int ad777x_powerup(struct iio_dev *indio_dev)
{
	int ret;
	struct ad777x_state *st = ad777x_get_data(indio_dev);

	ret = ad777x_spi_write(indio_dev, AD777X_REG_GENERAL_USER_CONFIG_1, 0x74);
	ret = ad777x_spi_write(indio_dev, AD777X_REG_DOUT_FORMAT, 0x24);
	ret = ad777x_spi_write(indio_dev, AD777X_REG_ADC_MUX_CONFIG, 0x40);
	ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_N_LSB, 0x40);
	ret = ad777x_spi_write(indio_dev, AD777X_REG_GEN_ERR_REG_1_EN, 0x3F);
	st->crc_enabled = true;
	st->sampling_freq = 8000;
	st->power_mode = 1;
	st->data_lines = AD777x_4LINES;

	if(ret)
		return ret;

	ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_UPDATE, 0x1);
	usleep_range(10, 15);
	ret = ad777x_spi_write(indio_dev, AD777X_REG_SRC_UPDATE, 0x0);
	usleep_range(10, 15);

	gpiod_set_value(st->start_gpio, 0);
	usleep_range(10, 15);
	gpiod_set_value(st->start_gpio, 1);
	usleep_range(10, 15);
	gpiod_set_value(st->start_gpio, 0);
	usleep_range(10, 15);

	return ret;
}


static int ad777x_probe(struct spi_device *spi) {
	struct iio_dev *indio_dev;
	struct ad777x_state *ad777x_st;
	int ret;
	int irq;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ad777x_st));
	if (!indio_dev)
		return -ENOMEM;
	
	ad777x_st = iio_priv(indio_dev);


	ad777x_st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(ad777x_st->vref))
		return PTR_ERR(ad777x_st->vref);

	ret = regulator_enable(ad777x_st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad777x_reg_disable, ad777x_st->vref);
	if (ret)
		return ret;

	ad777x_st->mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(ad777x_st->mclk))
		return PTR_ERR(ad777x_st->mclk);

	ret = clk_prepare_enable(ad777x_st->mclk);
	if (ret < 0)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, ad777x_clk_disable, ad777x_st->mclk);
	if (ret)
		return ret;

	ad777x_st->reset_gpio = devm_gpiod_get(&spi->dev, "resetn",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ad777x_st->reset_gpio))
		return PTR_ERR(ad777x_st->reset_gpio);

	ad777x_st->start_gpio = devm_gpiod_get(&spi->dev, "startn",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ad777x_st->start_gpio))
		return PTR_ERR(ad777x_st->start_gpio);
	
	if (ad777x_st->reset_gpio) {
		gpiod_set_value(ad777x_st->reset_gpio, 1);
		usleep_range(225, 230);
		gpiod_set_value(ad777x_st->reset_gpio, 0);
		usleep_range(1, 2);
	}

	// TODO: check if it relevant to add labels
	// ad777x_st->labels = devm_kzalloc(&ad777x_st->spi->dev,
	// 			   sizeof(*ad777x_st->labels), GFP_KERNEL);

	crc8_populate_msb(ad777x_crc8_table, AD777X_CRC8_POLY);
	ad777x_st->spi = spi;

	ad777x_powerup(indio_dev);

	if (device_property_present(&spi->dev, "interrupts"))
		ret = ad777x_register(ad777x_st, indio_dev);
	else
		ret = ad777x_register_axi(ad777x_st);

	return ret;
}

 static const struct spi_device_id ad777x_id[] = {
	{ "ad7770", ad7770 },
	{ "ad7771", ad7771 },
	{ "ad7779", ad7779 },
	{ }
 };
 MODULE_DEVICE_TABLE(spi, ad777x_id);

static struct spi_driver ad777x_driver = {
	.driver = {
		.name = "ad7771",
	},
	.probe = ad777x_probe,
	.id_table = ad777x_id,
};
module_spi_driver(ad777x_driver);

MODULE_AUTHOR("Ramona Alexandra Nechita <ramona.nechita@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD777X ADC");
MODULE_LICENSE("GPL v2");
