/*
 * Copyright (C) 2017, Yubo Zhi <normanzyb@gmail.com>
 * JZ4740 SoC touchscreen driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <linux/mfd/core.h>
#include <linux/jz4740-adc.h>

#define JZ_REG_ADC_ADSAME	0x00
#define JZ_REG_ADC_ADWAIT	0x04
#define JZ_REG_ADC_ADTCH	0x08

enum {
	JZ_ADC_IRQ_TS = 0,
	JZ_ADC_IRQ_PENUP,
	JZ_ADC_IRQ_PENDOWN,
};

struct jz4740_ts {
	void __iomem *base;

	struct clk *clk;
	const struct mfd_cell *cell;
	struct platform_device *pdev;
	struct input_dev *input;

	int irq[3];
	spinlock_t lock;

	enum {
		JZ_ADC_TS_MODE_XY = 0,
		JZ_ADC_TS_MODE_XYZ,
		JZ_ADC_TS_MODE_XYZ2,
		JZ_ADC_TS_MODE_XYZ4,
	} mode;

	struct ts_val_t {
		union {
			uint32_t xy;
			struct {
				uint16_t x, y;
			};
		};
		union {
			uint32_t z32;
			uint16_t z[2];
		};
	} val;
};

static irqreturn_t jz4740_ts_irq(int irq, void *data)
{
	struct jz4740_ts *ts = data;
	unsigned long flags;
	uint32_t val;

	spin_lock_irqsave(&ts->lock, flags);
	if (irq == ts->irq[JZ_ADC_IRQ_TS]) {
		val = readl(ts->base + JZ_REG_ADC_ADTCH);
		ts->val.xy = val & 0x0fff0fff;
		if (ts->mode != JZ_ADC_TS_MODE_XY) {
			val = readl(ts->base + JZ_REG_ADC_ADTCH);
			ts->val.z32 = val & 0x0fff0fff;
			input_report_abs(ts->input, ABS_PRESSURE, ts->val.z[0]);
		}
		input_report_abs(ts->input, ABS_X, ts->val.x);
		input_report_abs(ts->input, ABS_Y, ts->val.y);
		input_report_key(ts->input, BTN_TOUCH, 1);
	} else if (irq == ts->irq[JZ_ADC_IRQ_PENUP]) {
		enable_irq(ts->irq[JZ_ADC_IRQ_PENDOWN]);
		disable_irq_nosync(ts->irq[JZ_ADC_IRQ_PENUP]);
		input_report_key(ts->input, BTN_TOUCH, 0);
	} else if (irq == ts->irq[JZ_ADC_IRQ_PENDOWN]) {
		enable_irq(ts->irq[JZ_ADC_IRQ_PENUP]);
		disable_irq_nosync(ts->irq[JZ_ADC_IRQ_PENDOWN]);
	}
	spin_unlock_irqrestore(&ts->lock, flags);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

static void jz4740_ts_config(struct jz4740_ts *ts)
{
	struct device *dev = ts->pdev->dev.parent;
	unsigned long rate = clk_get_rate(ts->clk) / 128;
	uint32_t flags = JZ_ADC_CONFIG_EX_IN |
		JZ_ADC_CONFIG_SAMPLE_NUM(2) | JZ_ADC_CONFIG_DNUM(2);

	if (ts->mode != JZ_ADC_TS_MODE_XYZ4)
		flags |= JZ_ADC_CONFIG_XYZ_OFFSET(ts->mode);
	else
		flags |= JZ_ADC_CONFIG_XYZ_OFFSET(JZ_ADC_TS_MODE_XYZ2) |
			JZ_ADC_CONFIG_SPZZ;

	jz4740_adc_set_config(dev, JZ_ADC_CONFIG_SPZZ | JZ_ADC_CONFIG_EX_IN |
		JZ_ADC_CONFIG_DNUM_MASK | JZ_ADC_CONFIG_DMA_ENABLE |
		JZ_ADC_CONFIG_XYZ_MASK | JZ_ADC_CONFIG_SAMPLE_NUM_MASK, flags);

	writew(rate / 100, ts->base + JZ_REG_ADC_ADSAME);
	writew(rate / 100, ts->base + JZ_REG_ADC_ADWAIT);
}

static int jz4740_ts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz4740_ts *ts;
	struct input_dev *input;
	struct resource *mem;
	int i, ret = -EINVAL;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	platform_set_drvdata(pdev, ts);
	ts->cell = mfd_get_cell(pdev);

	ts->clk = clk_get(dev, "adc");
	if (IS_ERR(ts->clk)) {
		dev_err(dev, "cannot get clock source\n");
		ret = PTR_ERR(ts->clk);
		goto err_ts;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ts->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(ts->base)) {
		dev_err(dev, "cannot get base address\n");
		ret = PTR_ERR(ts->base);
		goto err_ts;
	}

	for (i = 0; i != 3; i++) {
		int irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			dev_err(dev, "cannot get irq %d: %d\n", i, irq);
			ret = irq;
			goto err_ts;
		}
		ts->irq[i] = irq;
	}

	ts->pdev = pdev;
	spin_lock_init(&ts->lock);

	for (i = 0; i != 3; i++) {
		ret = devm_request_irq(dev, ts->irq[i], jz4740_ts_irq, 0,
				pdev->name, ts);
		if (ret) {
			dev_err(dev, "cannot request irq %d: %d\n",
				ts->irq[i], ret);
			goto err_irq;
		}
		disable_irq(ts->irq[i]);
	}

	ts->mode = JZ_ADC_TS_MODE_XYZ;

	ts->cell->enable(pdev);
	jz4740_ts_config(ts);

	enable_irq(ts->irq[JZ_ADC_IRQ_TS]);
	enable_irq(ts->irq[JZ_ADC_IRQ_PENDOWN]);

	// Initialise touchscreen interface
	input = input_allocate_device();
	if (!input) {
		dev_err(dev, "cannot allocate input device\n");
		ret = PTR_ERR(input);
		goto err_hwmon;
	}

	ts->input = input;
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, 0x0fff, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 0x0fff, 0, 0);
	if (ts->mode != JZ_ADC_TS_MODE_XY)
		input_set_abs_params(input, ABS_PRESSURE, 0, 0x0fff, 0, 0);

	input->name = "JZ4740 touchscreen";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0xDEAD;
	input->id.product = 0xBEEF;
	input->id.version = 0x0102;

	// Register to the input system
	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(dev, "failed to register input device\n");
		goto err_input;
	}

	return 0;

err_input:
	input_free_device(input);
err_hwmon:
	ts->cell->disable(pdev);
err_irq:
	for (i = 0; i != 3; i++)
		devm_free_irq(dev, ts->irq[i], ts);
err_ts:
	devm_kfree(dev, ts);
	return ret;
}

static int jz4740_ts_remove(struct platform_device *pdev)
{
	struct jz4740_ts *ts = platform_get_drvdata(pdev);

	ts->cell->disable(pdev);
	input_unregister_device(ts->input);
	return 0;
}

static struct platform_driver jz4740_ts_driver = {
	.probe	= jz4740_ts_probe,
	.remove	= jz4740_ts_remove,
	.driver = {
		.name = "jz4740_ts",
	},
};

module_platform_driver(jz4740_ts_driver);

MODULE_DESCRIPTION("JZ4740 SoC touchscreen driver");
MODULE_AUTHOR("Yubo Zhi <normanzyb@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz4740_ts");
