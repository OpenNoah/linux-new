/*
 * spi_atben.c - SPI host look-alike for ATBEN
 *
 * Written 2011 by Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/at86rf230.h>
#include <asm/mach-jz4740/base.h>

#include "at86rf230.h"


enum {
	VDD_OFF	= 1 << 2,	/* VDD disable, PD02 */
	MOSI	= 1 << 8,	/* CMD, PD08 */
	SLP_TR	= 1 << 9,	/* CLK, PD09 */
	MISO	= 1 << 10,	/* DAT0, PD10 */
	SCLK	= 1 << 11,	/* DAT1, PD11 */
	IRQ	= 1 << 12,	/* DAT2, PD12 */
	nSEL    = 1 << 13,	/* DAT3/CD, PD13 */
};

#define	PDPIN	(prv->regs)
#define	PDDATS	(prv->regs+0x14)
#define	PDDATC	(prv->regs+0x18)


struct atben_prv {
	struct device		*dev;
	void __iomem		*regs;
	struct resource		*ioarea;
	struct at86rf230_platform_data
				platform_data;
	/* copy platform_data so that we can adapt .reset_data */
};


/* ----- ATBEN reset ------------------------------------------------------- */


static void atben_reset(void *reset_data)
{
	struct atben_prv *prv = reset_data;
	const int charge = nSEL | MOSI | SLP_TR | SCLK;
	const int discharge = charge | IRQ | MISO;

	dev_info(prv->dev, "atben_reset\n");
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 1 << 2, 1 << 2);
	jz_gpio_port_direction_output(JZ_GPIO_PORTD(0), discharge);
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 0, discharge);
	msleep(100);    /* let power drop */

	/*
	 * Hack: PD12/DAT2/IRQ is an active-high interrupt input, which is
	 * indicated by setting its direction bit to 1. We thus must not
	 * configure it as an "input".
	 */
	jz_gpio_port_direction_input(JZ_GPIO_PORTD(0), MISO);
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), charge, charge);
	msleep(10);     /* precharge caps */

	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 0, VDD_OFF | SLP_TR | SCLK);
	msleep(10);
}


/* ----- SPI transfers ----------------------------------------------------- */


static void rx_only(const struct atben_prv *prv, uint8_t *buf, int len)
{
	uint8_t v;

	while (len--) {
		writel(SCLK, PDDATS);
		v = readl(PDPIN) & MISO ? 0x80 : 0;
		writel(SCLK, PDDATC);

		#define	DO_BIT(m)			\
			writel(SCLK, PDDATS);		\
			if (readl(PDPIN) & MISO)	\
				v |= (m);		\
			writel(SCLK, PDDATC)

		DO_BIT(0x40);
		DO_BIT(0x20);
		DO_BIT(0x10);
		DO_BIT(0x08);
		DO_BIT(0x04);
		DO_BIT(0x02);
		DO_BIT(0x01);

		#undef DO_BIT

		*buf++ = v;
	}
}


static void tx_only(const struct atben_prv *prv, const uint8_t *buf, int len)
{
	uint8_t tv;

	while (len--) {
		tv = *buf++;

		if (tv & 0x80) {
			writel(MOSI, PDDATS);
			goto b6_1;
		} else {
			writel(MOSI, PDDATC);
			goto b6_0;
		}

		#define	DO_BIT(m, this, next)				\
			this##_1:					\
				writel(SCLK, PDDATS);			\
				if (tv & (m)) {				\
					writel(SCLK, PDDATC);		\
					goto next##_1;			\
				} else {				\
					writel(MOSI | SCLK, PDDATC);	\
					goto next##_0;			\
				}					\
			this##_0:					\
				writel(SCLK, PDDATS);			\
				writel(SCLK, PDDATC);			\
				if (tv & (m)) {				\
					writel(MOSI, PDDATS);		\
					goto next##_1;			\
				} else {				\
					goto next##_0;			\
				}

		DO_BIT(0x40, b6, b5);
		DO_BIT(0x20, b5, b4);
		DO_BIT(0x10, b4, b3);
		DO_BIT(0x08, b3, b2);
		DO_BIT(0x04, b2, b1);
		DO_BIT(0x02, b1, b0);
		DO_BIT(0x01, b0, done);

		#undef DO_BIT

done_1:
done_0:
		writel(SCLK, PDDATS);
		writel(SCLK, PDDATC);
		writel(SCLK, PDDATC);	/* delay to meet t5 timing */
	}
}


static void bidir(const struct atben_prv *prv, const uint8_t *tx, uint8_t *rx,
    int len)
{
	uint8_t mask, tv, rv;

	while (len--) {
		tv = *tx++;
		for (mask = 0x80; mask; mask >>= 1) {
			if (tv & mask)
				writel(MOSI, PDDATS);
			else
				writel(MOSI, PDDATC);
			writel(SCLK, PDDATS);
			if (readl(PDPIN) & MISO)
				rv |= mask;
			writel(SCLK, PDDATC);
		}
		*rx++ = rv;
	}
}


static int atben_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct atben_prv *prv = spi_master_get_devdata(spi->master);
	struct spi_transfer *xfer;
	const uint8_t *tx;
	uint8_t *rx;

	if (unlikely(list_empty(&msg->transfers))) {
		dev_err(prv->dev, "transfer is empty\n");
		return -EINVAL;
	}

	msg->actual_length = 0;

	writel(nSEL, PDDATC);
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		tx = xfer->tx_buf;
		rx = xfer->rx_buf;
		msg->actual_length += xfer->len;

		if (!tx)
			rx_only(prv, rx, xfer->len);
		else if (!rx)
			tx_only(prv, tx, xfer->len);
		else
			bidir(prv, tx, rx, xfer->len);
	}
	writel(nSEL, PDDATS);

	msg->status = 0;
	msg->complete(msg->context);

	return 0;
}

static int atben_setup(struct spi_device *spi)
{
	return 0;
}


/* ----- SPI master creation/removal --------------------------------------- */


const static struct at86rf230_platform_data at86rf230_platform_data = {
	.rstn	= -1,
	.slp_tr	= JZ_GPIO_PORTD(9),
	.dig2	= -1,
	.reset	= atben_reset,
	/* set .reset_data later */
};

static int __devinit atben_probe(struct platform_device *pdev)
{
	struct spi_board_info board_info = {
		.modalias	= "at86rf230",
		/* set .irq later */
		.chip_select	= 0,
		.bus_num	= -1,
		.max_speed_hz	= 8 * 1000 * 1000,
	};

	struct spi_master *master;
	struct atben_prv *prv;
	struct resource *regs;
	struct spi_device *spi;
	int err = -ENXIO;

	master = spi_alloc_master(&pdev->dev, sizeof(*prv));
	if (!master)
		return -ENOMEM;

	prv = spi_master_get_devdata(master);
	prv->dev = &pdev->dev;
	platform_set_drvdata(pdev, spi_master_get(master));

	master->mode_bits	= SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bus_num		= pdev->id;
	master->num_chipselect	= 1;
	master->setup		= atben_setup;
	master->transfer	= atben_transfer;

	dev_dbg(prv->dev, "Setting up ATBEN SPI\n");

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(prv->dev, "no IORESOURCE_MEM\n");
		err = -ENOENT;
		goto out_master;
	}
	prv->ioarea = request_mem_region(regs->start, resource_size(regs),
                                        pdev->name);
	if (!prv->ioarea) {
		dev_err(prv->dev, "can't request ioarea\n");
		goto out_master;
	}

	prv->regs = ioremap(regs->start, resource_size(regs));
	if (!prv->regs) {
		dev_err(prv->dev, "can't ioremap\n");
		goto out_ioarea;
	}

	board_info.irq = platform_get_irq(pdev, 0);
	if (board_info.irq < 0) {
		dev_err(prv->dev, "can't get GPIO irq\n");
		err = -ENOENT;
		goto out_regs;
	}

	err = spi_register_master(master);
	if (err) {
		dev_err(prv->dev, "can't register master\n");
		goto out_regs;
	}

	prv->platform_data = at86rf230_platform_data;
	prv->platform_data.reset_data = prv;
	board_info.platform_data = &prv->platform_data;

	spi = spi_new_device(master, &board_info);
	if (!spi) {
		dev_err(&pdev->dev, "can't create new device for %s\n",
		    board_info.modalias);
		err = -ENXIO;
		goto out_registered;
	}

	dev_info(&spi->dev, "ATBEN ready for mischief (IRQ %d)\n",
	    board_info.irq);

	return 0;

out_registered:
	spi_unregister_master(master);

out_regs:
	iounmap(prv->regs);

out_ioarea:
	release_resource(prv->ioarea);
	kfree(prv->ioarea);

out_master:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return err;
}

static int __devexit atben_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct atben_prv *prv = spi_master_get_devdata(master);

// restore GPIOs

	spi_unregister_master(master);

	iounmap(prv->regs);

	release_resource(prv->ioarea);
	kfree(prv->ioarea);

	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return 0;
}

static struct platform_driver atben_driver = {
	.driver = {
		.name	= "spi_atben",
		.owner	= THIS_MODULE,
	},
	.remove		= __devexit_p(atben_remove),
};

static struct resource atben_resources[] = {
	{
		.start  = JZ4740_GPIO_BASE_ADDR+0x300,
		.end    = JZ4740_GPIO_BASE_ADDR+0x3ff,
		.flags  = IORESOURCE_MEM,
	},
	{
		/* set start and end later */
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device atben_device = {
	.name = "spi_atben",
	.id = -1,
	.num_resources = ARRAY_SIZE(atben_resources),
	.resource = atben_resources,
};

/*
 * Registering the platform device just to probe it immediately afterwards
 * seems a little circuitous. Need to see if there's a better way.
 *
 * What we actually should do is this:
 * - in module init, register the device
 * - maybe probe as well, but keep the device also if the probe fails
 *   (due to a conflicting driver already occupying the 8:10 slot)
 * - have a means for user space to kick off driver probing, e.g., when
 *   anything about the 8:10 slot changes
 */

static int __init atben_init(void)
{
	int err;

	err = platform_device_register(&atben_device);
	if (err)
		return err;

	atben_resources[1].start = atben_resources[1].end =
	    gpio_to_irq(JZ_GPIO_PORTD(12));

	return platform_driver_probe(&atben_driver, atben_probe);
}

static void __exit atben_exit(void)
{
	platform_driver_unregister(&atben_driver);
	platform_device_unregister(&atben_device);
}

module_init(atben_init);
module_exit(atben_exit);


MODULE_DESCRIPTION("ATBEN SPI Controller Driver");
MODULE_AUTHOR("Werner Almesberger <werner@almesberger.net>");
MODULE_LICENSE("GPL");
