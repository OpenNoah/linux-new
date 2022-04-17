// SPDX-License-Identifier: GPL-2.0
/*
 * ADC driver for the Ingenic JZ4740 SoC
 * Copyright (c) 2019 Artur Rojek <contact@artur-rojek.eu>
 *
 * based on drivers/iio/adc/ingenic-adc.c
 */

#include <dt-bindings/iio/adc/ingenic,adc.h>
#include <linux/clk.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#define JZ_ADC_REG_ENABLE		0x00
#define JZ_ADC_REG_CFG			0x04
#define JZ_ADC_REG_CTRL			0x08
#define JZ_ADC_REG_STATUS		0x0c
#define JZ_ADC_REG_ADSAME		0x10
#define JZ_ADC_REG_ADWAIT		0x14
#define JZ_ADC_REG_ADTCH		0x18
#define JZ_ADC_REG_ADBDAT		0x1c
#define JZ_ADC_REG_ADSDAT		0x20
#define JZ4725B_ADC_REG_ADFLT		0x24
#define JZ4725B_ADC_REG_ADCLK		0x28

#define JZ4725B_ADC_REG_ENA_ENTER_SLP	BIT(6)
#define JZ4725B_ADC_REG_ENA_EXIT_SLP	BIT(5)

#define JZ_ADC_REG_CFG_BAT_MD		BIT(4)
#define JZ4740_ADC_REG_CFG_CLKDIV(n)	((n) << 5)
#define JZ_ADC_REG_CFG_SAMPLE_NUM(n)	((n) << 10)
#define JZ_ADC_REG_CFG_XYZ(n)		((n) << 13)
#define JZ_ADC_REG_CFG_DMA_EN		BIT(15)
#define JZ_ADC_REG_CFG_DNUM(n)		((n) << 16)
#define JZ_ADC_REG_CFG_EX_IN		BIT(30)
#define JZ_ADC_REG_CFG_SPZZ		BIT(31)

#define JZ4740_ADC_REG_CFG_CLKDIV_MASK	GENMASK(9, 5)
#define JZ_ADC_REG_CFG_TOUCH_OPS_MASK	(GENMASK(31, 30) | GENMASK(18, 16) | GENMASK(14, 10))
#define JZ_ADC_REG_CFG_TOUCH_XDYD	(JZ_ADC_REG_CFG_EX_IN | JZ_ADC_REG_CFG_XYZ(0))
#define JZ_ADC_REG_CFG_TOUCH_XSYS	(JZ_ADC_REG_CFG_XYZ(0))
#define JZ_ADC_REG_CFG_TOUCH_XDYDZS	(JZ_ADC_REG_CFG_EX_IN | JZ_ADC_REG_CFG_XYZ(1))
#define JZ_ADC_REG_CFG_TOUCH_XSYSZS	(JZ_ADC_REG_CFG_XYZ(1))
#define JZ_ADC_REG_CFG_TOUCH_XDYDZ1Z2	(JZ_ADC_REG_CFG_EX_IN | JZ_ADC_REG_CFG_XYZ(2) | JZ_ADC_REG_CFG_SPZZ)

#define JZ4725B_ADC_REG_ADCLK_CLKDIV_LSB	0
#define JZ4725B_ADC_REG_ADCLK_CLKDIV10US_LSB	16

#define JZ_ADC_REG_ADTCH_DATA_MASK	(GENMASK(27, 16) | GENMASK(11, 0))

#define JZ_ADC_AUX_VREF				3300
#define JZ_ADC_AUX_VREF_BITS			12
#define JZ_ADC_BATTERY_LOW_VREF			2500
#define JZ_ADC_BATTERY_LOW_VREF_BITS		12
#define JZ4725B_ADC_BATTERY_HIGH_VREF		7500
#define JZ4725B_ADC_BATTERY_HIGH_VREF_BITS	10
#define JZ4740_ADC_BATTERY_HIGH_VREF		(7500 * 0.986)
#define JZ4740_ADC_BATTERY_HIGH_VREF_BITS	12

#define JZ_ADC_IRQ_AUX			BIT(0)
#define JZ_ADC_IRQ_BATTERY		BIT(1)
#define JZ_ADC_IRQ_TOUCH		BIT(2)
#define JZ_ADC_IRQ_PEN_UP		BIT(3)
#define JZ_ADC_IRQ_PEN_DOWN		BIT(4)
#define JZ4725B_ADC_IRQ_SLEEP		BIT(5)

#define SCAN_INDEX_OFFSET		INGENIC_ADC_TOUCH_XYZ_XD
#define SCAN_XD				(INGENIC_ADC_TOUCH_XYZ_XD - SCAN_INDEX_OFFSET)
#define SCAN_YD				(INGENIC_ADC_TOUCH_XYZ_YD - SCAN_INDEX_OFFSET)
#define SCAN_XS				(INGENIC_ADC_TOUCH_XYZ_XS - SCAN_INDEX_OFFSET)
#define SCAN_YS				(INGENIC_ADC_TOUCH_XYZ_YS - SCAN_INDEX_OFFSET)
#define SCAN_Z1				(INGENIC_ADC_TOUCH_XYZ_Z1 - SCAN_INDEX_OFFSET)
#define SCAN_Z2				(INGENIC_ADC_TOUCH_XYZ_Z2 - SCAN_INDEX_OFFSET)
#define SCAN_ZS				(INGENIC_ADC_TOUCH_XYZ_ZS - SCAN_INDEX_OFFSET)
#define SCAN_MASK_XDYD			(BIT(SCAN_XD) | BIT(SCAN_YD))
#define SCAN_MASK_XSYS			(BIT(SCAN_XS) | BIT(SCAN_YS))
#define SCAN_MASK_XDYDZS		(BIT(SCAN_XD) | BIT(SCAN_YD) | BIT(SCAN_ZS))
#define SCAN_MASK_XSYSZS		(BIT(SCAN_XS) | BIT(SCAN_YS) | BIT(SCAN_ZS))
#define SCAN_MASK_XDYDZ1Z2		(BIT(SCAN_XD) | BIT(SCAN_YD) | BIT(SCAN_Z1) | BIT(SCAN_Z2))

struct ingenic_adc;

struct ingenic_adc_soc_data {
	unsigned int battery_high_vref;
	unsigned int battery_high_vref_bits;
	const int *battery_raw_avail;
	size_t battery_raw_avail_size;
	const int *battery_scale_avail;
	size_t battery_scale_avail_size;
	unsigned int battery_vref_mode: 1;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	const unsigned long *available_scan_masks;
	int (*init_clk_div)(struct device *dev, struct ingenic_adc *adc);
};

struct ingenic_adc {
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
	const struct ingenic_adc_soc_data *soc_data;
	u32 xyz_mode;
	bool low_vref_mode;
	bool pen_down;
};

static void ingenic_adc_set_config(struct ingenic_adc *adc,
				   uint32_t mask,
				   uint32_t val)
{
	uint32_t cfg;

	mutex_lock(&adc->lock);

	cfg = readl(adc->base + JZ_ADC_REG_CFG) & ~mask;
	cfg |= val;
	writel(cfg, adc->base + JZ_ADC_REG_CFG);

	mutex_unlock(&adc->lock);
}

static void ingenic_adc_enable_unlocked(struct ingenic_adc *adc,
					int engine,
					bool enabled)
{
	u8 val;

	val = readb(adc->base + JZ_ADC_REG_ENABLE);

	if (enabled)
		val |= BIT(engine);
	else
		val &= ~BIT(engine);

	writeb(val, adc->base + JZ_ADC_REG_ENABLE);
}

static void ingenic_adc_enable(struct ingenic_adc *adc,
			       int engine,
			       bool enabled)
{
	mutex_lock(&adc->lock);
	ingenic_adc_enable_unlocked(adc, engine, enabled);
	mutex_unlock(&adc->lock);
}

static int ingenic_adc_capture(struct ingenic_adc *adc,
			       int engine)
{
	u8 val;
	int ret;

	mutex_lock(&adc->lock);

	ingenic_adc_enable_unlocked(adc, engine, true);
	ret = readb_poll_timeout(adc->base + JZ_ADC_REG_ENABLE, val,
				 !(val & BIT(engine)), 250, 1000);
	if (ret)
		ingenic_adc_enable_unlocked(adc, engine, false);

	mutex_unlock(&adc->lock);

	return ret;
}

static int ingenic_adc_write_raw(struct iio_dev *iio_dev,
				 struct iio_chan_spec const *chan,
				 int val,
				 int val2,
				 long m)
{
	struct ingenic_adc *adc = iio_priv(iio_dev);
	struct device *dev = iio_dev->dev.parent;
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case INGENIC_ADC_BATTERY:
			if (!adc->soc_data->battery_vref_mode)
				return -EINVAL;

			ret = clk_enable(adc->clk);
			if (ret) {
				dev_err(dev, "Failed to enable clock: %d\n",
					ret);
				return ret;
			}

			if (val > JZ_ADC_BATTERY_LOW_VREF) {
				ingenic_adc_set_config(adc,
						       JZ_ADC_REG_CFG_BAT_MD,
						       0);
				adc->low_vref_mode = false;
			} else {
				ingenic_adc_set_config(adc,
						       JZ_ADC_REG_CFG_BAT_MD,
						       JZ_ADC_REG_CFG_BAT_MD);
				adc->low_vref_mode = true;
			}

			clk_disable(adc->clk);

			return 0;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int jz4725b_adc_init_clk_div(struct device *dev, struct ingenic_adc *adc)
{
	struct clk *parent_clk;
	unsigned long parent_rate, rate;
	unsigned int div_main, div_10us;

	parent_clk = clk_get_parent(adc->clk);
	if (!parent_clk) {
		dev_err(dev, "ADC clock has no parent\n");
		return -ENODEV;
	}
	parent_rate = clk_get_rate(parent_clk);

	/*
	 * The JZ4725B ADC works at 500 kHz to 8 MHz.
	 * We pick the highest rate possible.
	 * In practice we typically get 6 MHz, half of the 12 MHz EXT clock.
	 */
	div_main = DIV_ROUND_UP(parent_rate, 8000000);
	div_main = clamp(div_main, 1u, 64u);
	rate = parent_rate / div_main;
	if (rate < 500000 || rate > 8000000) {
		dev_err(dev, "No valid divider for ADC main clock\n");
		return -EINVAL;
	}

	/* We also need a divider that produces a 10us clock. */
	div_10us = DIV_ROUND_UP(rate, 100000);

	writel(((div_10us - 1) << JZ4725B_ADC_REG_ADCLK_CLKDIV10US_LSB) |
	       (div_main - 1) << JZ4725B_ADC_REG_ADCLK_CLKDIV_LSB,
	       adc->base + JZ4725B_ADC_REG_ADCLK);

	return 0;
}

static int jz4740_adc_init_clk_div(struct device *dev, struct ingenic_adc *adc)
{
	struct clk *parent_clk;
	unsigned long parent_rate, rate;
	unsigned int div_main;

	parent_clk = clk_get_parent(adc->clk);
	if (!parent_clk) {
		dev_err(dev, "ADC clock has no parent\n");
		return -ENODEV;
	}
	parent_rate = clk_get_rate(parent_clk);

	/*
	 * The JZ4740 ADC works at 500 kHz to 6 MHz.
	 * We pick the highest rate possible.
	 * In practice we typically get 6 MHz, half of the 12 MHz EXT clock.
	 */
	div_main = DIV_ROUND_UP(parent_rate, 6000000);
	div_main = clamp(div_main, 2u, 24u);
	rate = parent_rate / div_main;
	if (rate < 500000 || rate > 6000000) {
		dev_err(dev, "No valid divider for ADC main clock\n");
		return -EINVAL;
	}

	ingenic_adc_set_config(adc, JZ4740_ADC_REG_CFG_CLKDIV_MASK, JZ4740_ADC_REG_CFG_CLKDIV(div_main - 1));

	return 0;
}

static int ingenic_adc_read_avail(struct iio_dev *iio_dev,
				  struct iio_chan_spec const *chan,
				  const int **vals,
				  int *type,
				  int *length,
				  long m)
{
	struct ingenic_adc *adc = iio_priv(iio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*type = IIO_VAL_INT;
		*length = adc->soc_data->battery_raw_avail_size;
		*vals = adc->soc_data->battery_raw_avail;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_SCALE:
		*type = IIO_VAL_FRACTIONAL_LOG2;
		*length = adc->soc_data->battery_scale_avail_size;
		*vals = adc->soc_data->battery_scale_avail;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ingenic_adc_read_chan_info_raw(struct iio_dev *iio_dev,
					  struct iio_chan_spec const *chan,
					  int *val)
{
	int ret, engine = (chan->channel == INGENIC_ADC_BATTERY);
	struct ingenic_adc *adc = iio_priv(iio_dev);

	ret = clk_enable(adc->clk);
	if (ret) {
		dev_err(iio_dev->dev.parent, "Failed to enable clock: %d\n",
			ret);
		return ret;
	}

	ret = ingenic_adc_capture(adc, engine);
	if (ret)
		goto out;

	switch (chan->channel) {
	case INGENIC_ADC_AUX:
		*val = readw(adc->base + JZ_ADC_REG_ADSDAT);
		break;
	case INGENIC_ADC_BATTERY:
		*val = readw(adc->base + JZ_ADC_REG_ADBDAT);
		break;
	}

	ret = IIO_VAL_INT;
out:
	clk_disable(adc->clk);

	return ret;
}

static int ingenic_adc_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long m)
{
	struct ingenic_adc *adc = iio_priv(iio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		return ingenic_adc_read_chan_info_raw(iio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case INGENIC_ADC_AUX:
			*val = JZ_ADC_AUX_VREF;
			*val2 = JZ_ADC_AUX_VREF_BITS;
			break;
		case INGENIC_ADC_BATTERY:
			if (adc->low_vref_mode) {
				*val = JZ_ADC_BATTERY_LOW_VREF;
				*val2 = JZ_ADC_BATTERY_LOW_VREF_BITS;
			} else {
				*val = adc->soc_data->battery_high_vref;
				*val2 = adc->soc_data->battery_high_vref_bits;
			}
			break;
		}

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ingenic_adc_of_xlate(struct iio_dev *iio_dev,
				const struct of_phandle_args *iiospec)
{
	int i;

	if (!iiospec->args_count)
		return -EINVAL;

	for (i = 0; i < iio_dev->num_channels; ++i)
		if (iio_dev->channels[i].channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static void ingenic_adc_clk_cleanup(void *data)
{
	clk_unprepare(data);
}

static const struct iio_info ingenic_adc_info = {
	.write_raw = ingenic_adc_write_raw,
	.read_raw = ingenic_adc_read_raw,
	.read_avail = ingenic_adc_read_avail,
	.of_xlate = ingenic_adc_of_xlate,
};

static int ingenic_adc_buffer_enable(struct iio_dev *iio_dev)
{
	struct ingenic_adc *adc = iio_priv(iio_dev);
	int ret;

	/* Find a suitable XYZ mode. */
	u32 xyz_mode = 0;
	switch (iio_dev->active_scan_mask[0]) {
	case SCAN_MASK_XDYD:
		xyz_mode = JZ_ADC_REG_CFG_TOUCH_XDYD;
		break;
	case SCAN_MASK_XSYS:
		xyz_mode = JZ_ADC_REG_CFG_TOUCH_XSYS;
		break;
	case SCAN_MASK_XDYDZS:
		xyz_mode = JZ_ADC_REG_CFG_TOUCH_XDYDZS;
		break;
	case SCAN_MASK_XSYSZS:
		xyz_mode = JZ_ADC_REG_CFG_TOUCH_XSYSZS;
		break;
	case SCAN_MASK_XDYDZ1Z2:
		xyz_mode = JZ_ADC_REG_CFG_TOUCH_XDYDZ1Z2;
		break;
	default:
		dev_err(iio_dev->dev.parent, "Invalid scan mask: 0x%lx\n", iio_dev->active_scan_mask[0]);
		return -EINVAL;
	}
	adc->xyz_mode = xyz_mode;
	adc->pen_down = 0;

	ret = clk_enable(adc->clk);
	if (ret) {
		dev_err(iio_dev->dev.parent, "Failed to enable clock: %d\n", ret);
		return ret;
	}

	/* It takes significant time for the touchscreen hw to stabilize. */
	msleep(50);
	ingenic_adc_set_config(adc, JZ_ADC_REG_CFG_TOUCH_OPS_MASK,
			       JZ_ADC_REG_CFG_SAMPLE_NUM(4) |
			       JZ_ADC_REG_CFG_DNUM(4) |
			       xyz_mode);

	writew(80, adc->base + JZ_ADC_REG_ADWAIT);
	writew(2, adc->base + JZ_ADC_REG_ADSAME);
	writel(0, adc->base + JZ_ADC_REG_ADTCH);
	/*
	 * First, enable pen down interrupt.
	 * ADC interrupt must be always enabled,
	 * pen up interrupts won't fire if ADC data and interrupt are not cleared.
	 */
	writeb((u8)~(JZ_ADC_IRQ_TOUCH | JZ_ADC_IRQ_PEN_DOWN), adc->base + JZ_ADC_REG_CTRL);

	ingenic_adc_enable(adc, 2, true);

	return 0;
}

static int ingenic_adc_buffer_disable(struct iio_dev *iio_dev)
{
	struct ingenic_adc *adc = iio_priv(iio_dev);

	adc->pen_down = 0;

	ingenic_adc_enable(adc, 2, false);

	writeb(0xff, adc->base + JZ_ADC_REG_CTRL);
	writeb(0xff, adc->base + JZ_ADC_REG_STATUS);
	ingenic_adc_set_config(adc, JZ_ADC_REG_CFG_TOUCH_OPS_MASK, 0);
	writew(0, adc->base + JZ_ADC_REG_ADSAME);
	writew(0, adc->base + JZ_ADC_REG_ADWAIT);
	clk_disable(adc->clk);

	return 0;
}

static const struct iio_buffer_setup_ops ingenic_buffer_setup_ops = {
	.postenable = &ingenic_adc_buffer_enable,
	.predisable = &ingenic_adc_buffer_disable
};

static irqreturn_t ingenic_adc_irq(int irq, void *data)
{
	struct iio_dev *iio_dev = data;
	struct ingenic_adc *adc = iio_priv(iio_dev);
	unsigned long mask = iio_dev->active_scan_mask[0];
	u32 adc_result[2];
	u8 intflags;
	bool report_pen_up = false, report_adc = false;

	intflags = readb(adc->base + JZ_ADC_REG_STATUS) & \
		   ~readb(adc->base + JZ_ADC_REG_CTRL);
	if (intflags & JZ_ADC_IRQ_TOUCH) {
		u32 v;
		// Read ADC data according to configured scan format
		v = readl(adc->base + JZ_ADC_REG_ADTCH) & JZ_ADC_REG_ADTCH_DATA_MASK;
		adc_result[0] = v;
		if (mask & (SCAN_ZS | SCAN_Z1 | SCAN_Z2)) {
			v = readl(adc->base + JZ_ADC_REG_ADTCH) & JZ_ADC_REG_ADTCH_DATA_MASK;
			adc_result[1] = v;
		}
		report_adc = adc->pen_down;
	}
	if (intflags & JZ_ADC_IRQ_PEN_UP) {
		adc->pen_down = 0;
		report_pen_up = true;
		// Disable pen up, wait for pen down
		writeb((u8)~(JZ_ADC_IRQ_TOUCH | JZ_ADC_IRQ_PEN_DOWN), adc->base + JZ_ADC_REG_CTRL);
	} else if (intflags & JZ_ADC_IRQ_PEN_DOWN) {
		adc->pen_down = 1;
		// Disable pen down, wait for pen up
		writeb((u8)~(JZ_ADC_IRQ_TOUCH | JZ_ADC_IRQ_PEN_UP), adc->base + JZ_ADC_REG_CTRL);
	}
	writeb(intflags, adc->base + JZ_ADC_REG_STATUS);

	if (report_pen_up) {
		// For reporting pen up event, send an ADC result of all zeros
		adc_result[0] = 0;
		adc_result[1] = 0;
	} else if (!report_adc) {
		// No ADC result to report, return
		return IRQ_HANDLED;
	}

	iio_push_to_buffers(iio_dev, adc_result);
	return IRQ_HANDLED;
}

#ifdef CONFIG_INGENIC_JZ4740_ADC_KFIFO_BUF
static void ingenic_adc_iio_buffer_cleanup(void *data)
{
	struct iio_buffer *iio_buffer = data;
	iio_kfifo_free(iio_buffer);
}
#endif

static int ingenic_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *iio_dev;
#ifdef CONFIG_INGENIC_JZ4740_ADC_KFIFO_BUF
	struct iio_buffer *iio_buffer;
#endif
	struct ingenic_adc *adc;
	const struct ingenic_adc_soc_data *soc_data;
	int irq, ret;

	soc_data = device_get_match_data(dev);
	if (!soc_data)
		return -EINVAL;

	iio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!iio_dev)
		return -ENOMEM;

#ifdef CONFIG_INGENIC_JZ4740_ADC_KFIFO_BUF
	iio_buffer = iio_kfifo_allocate();
	if (!iio_buffer)
		return -ENOMEM;
	iio_buffer->direction = IIO_BUFFER_DIRECTION_IN;

	ret = devm_add_action_or_reset(dev, ingenic_adc_iio_buffer_cleanup, iio_buffer);
	if (ret) {
		dev_err(dev, "Unable to add action\n");
		iio_kfifo_free(iio_buffer);
		return ret;
	}
#endif

	adc = iio_priv(iio_dev);
	mutex_init(&adc->lock);
	adc->soc_data = soc_data;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, ingenic_adc_irq, 0,
			       dev_name(dev), iio_dev);
	if (ret < 0) {
		dev_err(dev, "Failed to request irq: %d\n", ret);
		return ret;
	}

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);

	adc->clk = devm_clk_get(dev, "adc");
	if (IS_ERR(adc->clk)) {
		dev_err(dev, "Unable to get clock\n");
		return PTR_ERR(adc->clk);
	}

	ret = clk_prepare_enable(adc->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	/* Set clock dividers. */
	if (soc_data->init_clk_div) {
		ret = soc_data->init_clk_div(dev, adc);
		if (ret) {
			clk_disable_unprepare(adc->clk);
			return ret;
		}
	}

	/* Put hardware in a known passive state. */
	writeb(0x00, adc->base + JZ_ADC_REG_ENABLE);
	writeb(0xff, adc->base + JZ_ADC_REG_CTRL);

	usleep_range(2000, 3000); /* Must wait at least 2ms. */
	clk_disable(adc->clk);

	ret = devm_add_action_or_reset(dev, ingenic_adc_clk_cleanup, adc->clk);
	if (ret) {
		dev_err(dev, "Unable to add action\n");
		clk_unprepare(adc->clk);
		return ret;
	}

	iio_dev->name = "jz-adc";
	iio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	iio_dev->setup_ops = &ingenic_buffer_setup_ops;
	iio_dev->channels = soc_data->channels;
	iio_dev->num_channels = soc_data->num_channels;
	iio_dev->available_scan_masks = soc_data->available_scan_masks;
	iio_dev->info = &ingenic_adc_info;

#ifdef CONFIG_INGENIC_JZ4740_ADC_KFIFO_BUF
	ret = iio_device_attach_buffer(iio_dev, iio_buffer);
	if (ret < 0)
		dev_err(dev, "Unable to attach IIO buffer\n");
#endif

	ret = devm_iio_device_register(dev, iio_dev);
	if (ret)
		dev_err(dev, "Unable to register IIO device\n");

	return ret;
}

static const int jz4725b_adc_battery_raw_avail[] = {
	0, 1, (1 << JZ_ADC_BATTERY_LOW_VREF_BITS) - 1,
};

static const int jz4725b_adc_battery_scale_avail[] = {
	JZ4725B_ADC_BATTERY_HIGH_VREF, JZ4725B_ADC_BATTERY_HIGH_VREF_BITS,
	JZ_ADC_BATTERY_LOW_VREF, JZ_ADC_BATTERY_LOW_VREF_BITS,
};

static const int jz4740_adc_battery_raw_avail[] = {
	0, 1, (1 << JZ_ADC_BATTERY_LOW_VREF_BITS) - 1,
};

static const int jz4740_adc_battery_scale_avail[] = {
	JZ4740_ADC_BATTERY_HIGH_VREF, JZ4740_ADC_BATTERY_HIGH_VREF_BITS,
	JZ_ADC_BATTERY_LOW_VREF, JZ_ADC_BATTERY_LOW_VREF_BITS,
};

static const struct iio_chan_spec jz4740_channels[] = {
	{
		.extend_name = "aux",
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.channel = INGENIC_ADC_AUX,
		.scan_index = -1,
	},
	{
		.extend_name = "battery",
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_RAW) |
						BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.channel = INGENIC_ADC_BATTERY,
		.scan_index = -1,
	},
	{
		.extend_name = "xd",
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = INGENIC_ADC_TOUCH_XYZ_XD,
		.scan_index = SCAN_XD,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
	{
		.extend_name = "yd",
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = INGENIC_ADC_TOUCH_XYZ_YD,
		.scan_index = SCAN_YD,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
	{
		.extend_name = "xs",
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = INGENIC_ADC_TOUCH_XYZ_XS,
		.scan_index = SCAN_XS,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
	{
		.extend_name = "ys",
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = INGENIC_ADC_TOUCH_XYZ_YS,
		.scan_index = SCAN_YS,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
	{
		.extend_name = "z1",
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = INGENIC_ADC_TOUCH_XYZ_Z1,
		.scan_index = SCAN_Z1,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
	{
		.extend_name = "z2",
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = INGENIC_ADC_TOUCH_XYZ_Z2,
		.scan_index = SCAN_Z2,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
	{
		.extend_name = "zs",
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = INGENIC_ADC_TOUCH_XYZ_ZS,
		.scan_index = SCAN_ZS,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
};

static const unsigned long jz4740_adc_scan_masks[] = {
	SCAN_MASK_XDYD,
	SCAN_MASK_XSYS,
	SCAN_MASK_XDYDZS,
	SCAN_MASK_XSYSZS,
	SCAN_MASK_XDYDZ1Z2,
	0
};

static const struct ingenic_adc_soc_data jz4725b_adc_soc_data = {
	.battery_high_vref = JZ4725B_ADC_BATTERY_HIGH_VREF,
	.battery_high_vref_bits = JZ4725B_ADC_BATTERY_HIGH_VREF_BITS,
	.battery_raw_avail = jz4725b_adc_battery_raw_avail,
	.battery_raw_avail_size = ARRAY_SIZE(jz4725b_adc_battery_raw_avail),
	.battery_scale_avail = jz4725b_adc_battery_scale_avail,
	.battery_scale_avail_size = ARRAY_SIZE(jz4725b_adc_battery_scale_avail),
	.battery_vref_mode = true,
	.channels = jz4740_channels,
	.num_channels = ARRAY_SIZE(jz4740_channels),
	.available_scan_masks = jz4740_adc_scan_masks,
	.init_clk_div = jz4725b_adc_init_clk_div,
};

static const struct ingenic_adc_soc_data jz4740_adc_soc_data = {
	.battery_high_vref = JZ4740_ADC_BATTERY_HIGH_VREF,
	.battery_high_vref_bits = JZ4740_ADC_BATTERY_HIGH_VREF_BITS,
	.battery_raw_avail = jz4740_adc_battery_raw_avail,
	.battery_raw_avail_size = ARRAY_SIZE(jz4740_adc_battery_raw_avail),
	.battery_scale_avail = jz4740_adc_battery_scale_avail,
	.battery_scale_avail_size = ARRAY_SIZE(jz4740_adc_battery_scale_avail),
	.battery_vref_mode = true,
	.channels = jz4740_channels,
	.num_channels = ARRAY_SIZE(jz4740_channels),
	.available_scan_masks = jz4740_adc_scan_masks,
	.init_clk_div = jz4740_adc_init_clk_div,
};

static const struct of_device_id ingenic_adc_of_match[] = {
	{ .compatible = "ingenic,jz4725b-adc", .data = &jz4725b_adc_soc_data, },
	{ .compatible = "ingenic,jz4740-adc", .data = &jz4740_adc_soc_data, },
	{ },
};
MODULE_DEVICE_TABLE(of, ingenic_adc_of_match);

static struct platform_driver ingenic_adc_driver = {
	.driver = {
		.name = "jz4740-adc",
		.of_match_table = ingenic_adc_of_match,
	},
	.probe = ingenic_adc_probe,
};
module_platform_driver(ingenic_adc_driver);
MODULE_LICENSE("GPL v2");
