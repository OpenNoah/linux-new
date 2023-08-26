/*
 * AT86RF230/RF231 driver
 *
 * Copyright (C) 2009 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Written by:
 * Dmitry Eremin-Solenikov <dmitry.baryshkov@siemens.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/spi/at86rf230.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include <net/mac802154.h>
#include <net/wpan-phy.h>

#include "at86rf230.h"


struct at86rf230_local {
	struct spi_device *spi;
	int rstn, slp_tr, dig2;
	void (*reset)(void *reset_data);
	void *reset_data;

	u8 part;
	u8 vers;

	u8 buf[2];
	struct mutex bmux;

	struct work_struct irqwork;
	struct completion tx_complete;

	struct ieee802154_dev *dev;

	volatile unsigned is_tx:1;
};


static int
__at86rf230_write(struct at86rf230_local *lp, u8 addr, u8 data)
{
	u8 *buf = lp->buf;
	int status;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 2,
		.tx_buf		= buf,
	};

	buf[0] = (addr & CMD_REG_MASK) | CMD_REG | CMD_WRITE;
	buf[1] = data;
	dev_vdbg(&lp->spi->dev, "buf[0] = %02x\n", buf[0]);
	dev_vdbg(&lp->spi->dev, "buf[1] = %02x\n", buf[1]);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	status = spi_sync(lp->spi, &msg);
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);
	dev_vdbg(&lp->spi->dev, "buf[0] = %02x\n", buf[0]);
	dev_vdbg(&lp->spi->dev, "buf[1] = %02x\n", buf[1]);

	return status;
}

static int
__at86rf230_read_subreg(struct at86rf230_local *lp,
		u8 addr, u8 mask, int shift, u8 *data)
{
	u8 *buf = lp->buf;
	int status;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 2,
		.tx_buf		= buf,
		.rx_buf		= buf,
	};

	buf[0] = (addr & CMD_REG_MASK) | CMD_REG;
	buf[1] = 0xff;
	dev_vdbg(&lp->spi->dev, "buf[0] = %02x\n", buf[0]);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	status = spi_sync(lp->spi, &msg);
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);
	dev_vdbg(&lp->spi->dev, "buf[0] = %02x\n", buf[0]);
	dev_vdbg(&lp->spi->dev, "buf[1] = %02x\n", buf[1]);

	if (status == 0)
		*data = buf[1];

	return status;
}

static int
at86rf230_read_subreg(struct at86rf230_local *lp,
		u8 addr, u8 mask, int shift, u8 *data)
{
	int status;

	mutex_lock(&lp->bmux);
	status = __at86rf230_read_subreg(lp, addr, mask, shift, data);
	mutex_unlock(&lp->bmux);

	return status;
}

static int
at86rf230_write_subreg(struct at86rf230_local *lp,
		u8 addr, u8 mask, int shift, u8 data)
{
	int status;
	u8 val;

	mutex_lock(&lp->bmux);
	status = __at86rf230_read_subreg(lp, addr, 0xff, 0, &val);
	if (status)
		goto out;

	val &= ~mask;
	val |= (data << shift) & mask;

	status = __at86rf230_write(lp, addr, val);
out:
	mutex_unlock(&lp->bmux);

	return status;
}

static int
at86rf230_write_fbuf(struct at86rf230_local *lp, u8 *data, u8 len)
{
	u8 *buf = lp->buf;
	int status;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len		= 2,
		.tx_buf		= buf,

	};
	struct spi_transfer xfer_buf = {
		.len		= len,
		.tx_buf		= data,
	};

	mutex_lock(&lp->bmux);
	buf[0] = CMD_WRITE | CMD_FB;
	buf[1] = len + 2; /* 2 bytes for CRC that isn't written */

	dev_vdbg(&lp->spi->dev, "buf[0] = %02x\n", buf[0]);
	dev_vdbg(&lp->spi->dev, "buf[1] = %02x\n", buf[1]);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	status = spi_sync(lp->spi, &msg);
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);
	dev_vdbg(&lp->spi->dev, "buf[0] = %02x\n", buf[0]);
	dev_vdbg(&lp->spi->dev, "buf[1] = %02x\n", buf[1]);

	mutex_unlock(&lp->bmux);
	return status;
}

static int
at86rf230_read_fbuf(struct at86rf230_local *lp, u8 *data, u8 *len, u8 *lqi)
{
	u8 *buf = lp->buf;
	int status;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len		= 2,
		.tx_buf		= buf,
		.rx_buf		= buf,
	};
	struct spi_transfer xfer_head1 = {
		.len		= 2,
		.tx_buf		= buf,
		.rx_buf		= buf,
	};
	struct spi_transfer xfer_buf = {
		.len		= 0,
		.rx_buf		= data,
	};

	mutex_lock(&lp->bmux);

	buf[0] = CMD_FB;
	buf[1] = 0x00;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);

	status = spi_sync(lp->spi, &msg);
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);

	if (buf[1] & 0x80) {
		dev_err(&lp->spi->dev, "invalid PHR 0x%02x\n", buf[1]);
		status = -EIO;
		goto fail;
	}
	if (buf[1] >= *len) {
		dev_err(&lp->spi->dev, "PHR 0x%02x >= buffer %d bytes\n",
		    buf[1], *len);
		status = -EMSGSIZE;
		goto fail;
	}
	xfer_buf.len = *(buf + 1) + 1;
	*len = buf[1];

	buf[0] = CMD_FB;
	buf[1] = 0x00;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head1, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	status = spi_sync(lp->spi, &msg);

	if (msg.status)
		status = msg.status;
	dev_vdbg(&lp->spi->dev, "status = %d\n", status);
	dev_vdbg(&lp->spi->dev, "buf[0] = %02x\n", buf[0]);
	dev_vdbg(&lp->spi->dev, "buf[1] = %02x\n", buf[1]);

	if (!status) {
		if (lqi && *len > lp->buf[1])
			*lqi = data[lp->buf[1]];
	}

fail:
	mutex_unlock(&lp->bmux);

	return status;
}

static int
at86rf230_ed(struct ieee802154_dev *dev, u8 *level)
{
	pr_debug("%s\n", __func__);
	might_sleep();
	BUG_ON(!level);
	*level = 0xbe;
	return 0;
}

static int
at86rf230_state(struct ieee802154_dev *dev, int state)
{
	struct at86rf230_local *lp = dev->priv;
	int rc;
	u8 val;
	u8 desired_status;

	pr_debug("%s %d\n", __func__/*, priv->cur_state*/, state);
	might_sleep();

	if (state == STATE_FORCE_TX_ON)
		desired_status = STATE_TX_ON;
	else if (state == STATE_FORCE_TRX_OFF)
		desired_status = STATE_TRX_OFF;
	else
		desired_status = state;

	do {
		rc = at86rf230_read_subreg(lp, SR_TRX_STATUS, &val);
		if (rc)
			goto err;
		pr_debug("%s val1 = %x\n", __func__, val);
	} while (val == STATE_TRANSITION_IN_PROGRESS);

	if (val == desired_status)
		return 0;

	/* state is equal to phy states */
	rc = at86rf230_write_subreg(lp, SR_TRX_CMD, state);
	if (rc)
		goto err;

	do {
		rc = at86rf230_read_subreg(lp, SR_TRX_STATUS, &val);
		if (rc)
			goto err;
		pr_debug("%s val2 = %x\n", __func__, val);
	} while (val == STATE_TRANSITION_IN_PROGRESS);


	if (val == desired_status)
		return 0;
	if (state == STATE_RX_ON && val == STATE_BUSY_RX)
		return 0;

	pr_err("%s unexpected state change: %d, asked for %d\n", __func__,
			val, state);
	return -EBUSY;

err:
	pr_err("%s error: %d\n", __func__, rc);
	return rc;
}

static int
at86rf230_start(struct ieee802154_dev *dev)
{
	struct at86rf230_local *lp = dev->priv;
	u8 rc;

	rc = at86rf230_write_subreg(lp, SR_RX_SAFE_MODE, 1);
	if (rc)
		return rc;
	return at86rf230_state(dev, STATE_RX_ON);
}

static void
at86rf230_stop(struct ieee802154_dev *dev)
{
	at86rf230_state(dev, STATE_FORCE_TRX_OFF);
}

static int
at86rf230_channel(struct ieee802154_dev *dev, int page, int channel)
{
	struct at86rf230_local *lp = dev->priv;
	int rc;

	pr_debug("%s %d\n", __func__, channel);
	might_sleep();

	BUG_ON(page != 0);
	BUG_ON(channel < 11);
	BUG_ON(channel > 26);

	rc = at86rf230_write_subreg(lp, SR_CHANNEL, channel);
	msleep(1); /* Wait for PLL */
	dev->phy->current_channel = channel;

	return 0;
}

/* FIXME:
 * This function currently is a mess. It uses flush_work to guard
 * against concurrent irqwork, etc. One has to use mutexes intead. */
static int
at86rf230_xmit(struct ieee802154_dev *dev, struct sk_buff *skb)
{
	struct at86rf230_local *lp = dev->priv;
	int rc;

	pr_debug("%s\n", __func__);

	might_sleep();

	BUG_ON(lp->is_tx);
	INIT_COMPLETION(lp->tx_complete);

	rc = at86rf230_state(dev, STATE_FORCE_TX_ON);
	if (rc)
		goto err;

	synchronize_irq(lp->spi->irq);
	flush_work(&lp->irqwork);

	lp->is_tx = 1;

	rc = at86rf230_write_fbuf(lp, skb->data, skb->len);
	if (rc)
		goto err_rx;

	if (gpio_is_valid(lp->slp_tr)) {
		gpio_set_value(lp->slp_tr, 1);
		udelay(80); /* > 62.5 */
		gpio_set_value(lp->slp_tr, 0);
	} else {
		rc = at86rf230_write_subreg(lp, SR_TRX_CMD, STATE_BUSY_TX);
		if (rc)
			goto err_rx;
	}

	/* FIXME: the logic is really strange here. Datasheet doesn't
	 * provide us enough info about behaviour in such cases.
	 * Basically either we were interrupted here, or we have lost
	 * the interrupt. Most probably this should be changed to
	 * wait_for_completion_timeout() and handle it's results
	 */
	rc = wait_for_completion_interruptible(&lp->tx_complete);
	if (rc < 0)
		goto err_state;

	lp->is_tx = 0;

	rc = at86rf230_start(dev);
	return rc;

err_state:
	/* try to recover from possibly problematic state */
	at86rf230_state(dev, STATE_FORCE_TX_ON);
	synchronize_irq(lp->spi->irq);
	flush_work(&lp->irqwork);
	lp->is_tx = 0;
err_rx:
	at86rf230_start(dev);
err:
	if (rc)
		pr_err("%s error: %d\n", __func__, rc);

	return rc;
}

static int at86rf230_rx(struct at86rf230_local *lp)
{
	u8 len = 128, lqi = 0;
	int rc, rc2;
	struct sk_buff *skb;

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	/* FIXME: process return status */
	rc = at86rf230_write_subreg(lp, SR_RX_PDT_DIS, 1);
	rc2 = at86rf230_read_fbuf(lp, skb_put(skb, len), &len, &lqi);
	rc = at86rf230_write_subreg(lp, SR_RX_SAFE_MODE, 1);
	rc = at86rf230_write_subreg(lp, SR_RX_PDT_DIS, 0);
	if (rc2 < 0)
		goto err_fbuf;

	if (len < 2)
		goto err;

	skb_trim(skb, len-2); /* We do not put CRC into the frame */


	ieee802154_rx_irqsafe(lp->dev, skb, lqi);

	dev_dbg(&lp->spi->dev, "READ_FBUF: %d %d %x\n", rc, len, lqi);

	return 0;
err:
	pr_debug("%s: received frame is too small\n", __func__);

err_fbuf:
	kfree_skb(skb);
	return -EINVAL;
}

static struct ieee802154_ops at86rf230_ops = {
	.owner = THIS_MODULE,
	.xmit = at86rf230_xmit,
	.ed = at86rf230_ed,
	.set_channel = at86rf230_channel,
	.start = at86rf230_start,
	.stop = at86rf230_stop,
};

static void at86rf230_irqwork(struct work_struct *work)
{
	struct at86rf230_local *lp =
		container_of(work, struct at86rf230_local, irqwork);
	u8 status = 0, val;
	int rc;

	dev_dbg(&lp->spi->dev, "IRQ Worker\n");

	do {
		rc = at86rf230_read_subreg(lp, RG_IRQ_STATUS, 0xff, 0, &val);
		status |= val;
		dev_dbg(&lp->spi->dev, "IRQ Status: %02x\n", status);

		status &= ~IRQ_PLL_LOCK; /* ignore */
		status &= ~IRQ_RX_START; /* ignore */
		status &= ~IRQ_AMI; /* ignore */
		status &= ~IRQ_TRX_UR; /* FIXME: possibly handle ???*/

		if (status & IRQ_TRX_END) {
			status &= ~IRQ_TRX_END;
			if (lp->is_tx)
				complete(&lp->tx_complete);
			else
				at86rf230_rx(lp);
		}

	} while (status != 0);

	enable_irq(lp->spi->irq);
}

static irqreturn_t at86rf230_isr(int irq, void *data)
{
	struct at86rf230_local *lp = data;

	dev_dbg(&lp->spi->dev, "IRQ!\n");

	disable_irq_nosync(irq);
	schedule_work(&lp->irqwork);

	return IRQ_HANDLED;
}


static int at86rf230_hw_init(struct at86rf230_local *lp)
{
	u8 status;
	int rc;

	rc = at86rf230_read_subreg(lp, SR_TRX_STATUS, &status);
	if (rc)
		return rc;

	dev_info(&lp->spi->dev, "Status: %02x\n", status);
	if (status == STATE_P_ON) {
		rc = at86rf230_write_subreg(lp, SR_TRX_CMD, STATE_TRX_OFF);
		if (rc)
			return rc;
		msleep(1);
		rc = at86rf230_read_subreg(lp, SR_TRX_STATUS, &status);
		if (rc)
			return rc;
		dev_info(&lp->spi->dev, "Status: %02x\n", status);
	}

	rc = at86rf230_write_subreg(lp, SR_IRQ_MASK,
			/*IRQ_TRX_UR | IRQ_CCA_ED | IRQ_TRX_END | IRQ_PLL_UNL | IRQ_PLL_LOCK*/ 0xff);
	if (rc)
		return rc;

	/* CLKM changes are applied immediately */
	rc = at86rf230_write_subreg(lp, SR_CLKM_SHA_SEL, 0x00);
	if (rc)
		return rc;

	/* Turn CLKM Off */
	rc = at86rf230_write_subreg(lp, SR_CLKM_CTRL, 0x00);
	if (rc)
		return rc;

	msleep(100);

	rc = at86rf230_write_subreg(lp, SR_TRX_CMD, STATE_TX_ON);
	if (rc)
		return rc;
	msleep(1);

	rc = at86rf230_read_subreg(lp, SR_TRX_STATUS, &status);
	if (rc)
		return rc;
	dev_info(&lp->spi->dev, "Status: %02x\n", status);

	rc = at86rf230_read_subreg(lp, SR_DVDD_OK, &status);
	if (rc)
		return rc;
	if (!status) {
		dev_err(&lp->spi->dev, "DVDD error\n");
		return -EINVAL;
	}

	rc = at86rf230_read_subreg(lp, SR_AVDD_OK, &status);
	if (rc)
		return rc;
	if (!status) {
		dev_err(&lp->spi->dev, "AVDD error\n");
		return -EINVAL;
	}

	return 0;
}

static int at86rf230_suspend(struct spi_device *spi, pm_message_t message)
{
	return 0;
}

static int at86rf230_resume(struct spi_device *spi)
{
	return 0;
}

#ifdef CONFIG_OF
static int at86rf230_fill_data(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct at86rf230_local *lp = spi_get_drvdata(spi);
	struct at86rf230_platform_data *pdata = spi->dev.platform_data;
	enum of_gpio_flags gpio_flags;

	if (pdata) {
		lp->rstn = pdata->rstn;
		lp->slp_tr = pdata->slp_tr;
		lp->dig2 = pdata->dig2;
		lp->reset = pdata->reset;
		lp->reset_data = pdata->reset_data;

		return 0;
	}

	if (!np) {
		dev_err(&spi->dev, "no platform_data and no node data\n");
		return -EINVAL;
	}

	lp->rstn = of_get_gpio_flags(np, 0, &gpio_flags);
	if (!gpio_is_valid(lp->rstn)) {
		dev_err(&spi->dev, "no RSTN GPIO!\n");
		return -EINVAL;
	}

	lp->slp_tr = of_get_gpio_flags(np, 1, &gpio_flags);
	lp->dig2 = of_get_gpio_flags(np, 2, &gpio_flags);

	lp->reset = NULL;

	return 0;
}
#else
static int at86rf230_fill_data(struct spi_device *spi)
{
	struct at86rf230_local *lp = spi_get_drvdata(spi);
	struct at86rf230_platform_data *pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_err(&spi->dev, "no platform_data\n");
		return -EINVAL;
	}

	lp->rstn = pdata->rstn;
	lp->slp_tr = pdata->slp_tr;
	lp->dig2 = pdata->dig2;
	lp->reset = pdata->reset;
	lp->reset_data = pdata->reset_data;

	return 0;
}
#endif

static int __devinit at86rf230_probe(struct spi_device *spi)
{
	struct ieee802154_dev *dev;
	struct at86rf230_local *lp;
	u8 man_id_0, man_id_1;
	int rc;
	const char *chip;
	int supported = 0;

	if (spi->irq < 0) {
		dev_err(&spi->dev, "no IRQ specified\n");
		return -EINVAL;
	}

	dev = ieee802154_alloc_device(sizeof(*lp), &at86rf230_ops);
	if (!dev)
		return -ENOMEM;

	lp = dev->priv;
	lp->dev = dev;

	lp->spi = spi;

	dev->priv = lp;
	dev->parent = &spi->dev;
	dev->extra_tx_headroom = 0;
	/* We do support only 2.4 Ghz */
	dev->phy->channels_supported[0] = 0x7FFF800;
	dev->flags = IEEE802154_HW_OMIT_CKSUM;

	mutex_init(&lp->bmux);
	INIT_WORK(&lp->irqwork, at86rf230_irqwork);
	init_completion(&lp->tx_complete);

	spi_set_drvdata(spi, lp);

	rc = at86rf230_fill_data(spi);
	if (rc)
		goto err_fill;

	if (gpio_is_valid(lp->rstn)) {
		rc = gpio_request(lp->rstn, "rstn");
		if (rc)
			goto err_rstn;
	}

	if (gpio_is_valid(lp->slp_tr)) {
		rc = gpio_request(lp->slp_tr, "slp_tr");
		if (rc)
			goto err_slp_tr;
	}

	if (gpio_is_valid(lp->rstn)) {
		rc = gpio_direction_output(lp->rstn, 1);
		if (rc)
			goto err_gpio_dir;
	}

	if (gpio_is_valid(lp->slp_tr)) {
		rc = gpio_direction_output(lp->slp_tr, 0);
		if (rc)
			goto err_gpio_dir;
	}

	/* Reset */
	if (lp->reset)
		lp->reset(lp->reset_data);
	else {
		msleep(1);
		gpio_set_value(lp->rstn, 0);
		msleep(1);
		gpio_set_value(lp->rstn, 1);
		msleep(1);
	}

	rc = at86rf230_read_subreg(lp, SR_MAN_ID_0, &man_id_0);
	if (rc)
		goto err_gpio_dir;
	rc = at86rf230_read_subreg(lp, SR_MAN_ID_1, &man_id_1);
	if (rc)
		goto err_gpio_dir;

	if (man_id_1 != 0x00 || man_id_0 != 0x1f) {
		dev_err(&spi->dev, "Non-Atmel device found (MAN_ID:"
				"%02x %02x)\n", man_id_1, man_id_0);
		rc = -EINVAL;
		goto err_gpio_dir;
	}

	rc = at86rf230_read_subreg(lp, SR_PART_NUM, &lp->part);
	if (rc)
		goto err_gpio_dir;

	rc = at86rf230_read_subreg(lp, SR_VERSION_NUM, &lp->vers);
	if (rc)
		goto err_gpio_dir;

	switch (lp->part) {
	case 2:
		chip = "at86rf230";
		/* supported = 1;  FIXME: should be easy to support; */
		break;
	case 3:
		chip = "at86rf231";
		supported = 1;
		break;
	default:
		chip = "UNKNOWN";
		break;
	}

	dev_info(&spi->dev, "Detected %s chip version %d\n", chip, lp->vers);
	if (!supported) {
		rc = -ENOTSUPP;
		goto err_gpio_dir;
	}

	rc = at86rf230_hw_init(lp);
	if (rc)
		goto err_gpio_dir;

	rc = request_irq(spi->irq, at86rf230_isr, IRQF_SHARED,
			dev_name(&spi->dev), lp);
	if (rc)
		goto err_gpio_dir;

	dev_dbg(&spi->dev, "registered at86rf230\n");

	rc = ieee802154_register_device(lp->dev);
	if (rc)
		goto err_irq;

	return rc;

err_irq:
	disable_irq(spi->irq);
	flush_work(&lp->irqwork);
	free_irq(spi->irq, lp);
err_gpio_dir:
	if (gpio_is_valid(lp->slp_tr))
		gpio_free(lp->slp_tr);
err_slp_tr:
	if (gpio_is_valid(lp->rstn))
		gpio_free(lp->rstn);
err_rstn:
err_fill:
	spi_set_drvdata(spi, NULL);
	mutex_destroy(&lp->bmux);
	ieee802154_free_device(lp->dev);
	return rc;
}

static int __devexit at86rf230_remove(struct spi_device *spi)
{
	struct at86rf230_local *lp = spi_get_drvdata(spi);

	/*
	 * @@@ this looks wrong - what if a frame arrives before
	 * disable_irq ? -- wa
	 */
	ieee802154_unregister_device(lp->dev);

	disable_irq(spi->irq);
	flush_work(&lp->irqwork);
	free_irq(spi->irq, lp);

	if (gpio_is_valid(lp->slp_tr))
		gpio_free(lp->slp_tr);
	if (gpio_is_valid(lp->rstn))
		gpio_free(lp->rstn);

	spi_set_drvdata(spi, NULL);
	mutex_destroy(&lp->bmux);
	ieee802154_free_device(lp->dev);

	dev_dbg(&spi->dev, "unregistered at86rf230\n");
	return 0;
}

static struct spi_driver at86rf230_driver = {
	.driver = {
		.name	= "at86rf230",
		.owner	= THIS_MODULE,
	},
	.probe      = at86rf230_probe,
	.remove     = __devexit_p(at86rf230_remove),
	.suspend    = at86rf230_suspend,
	.resume     = at86rf230_resume,
};

static int __init at86rf230_init(void)
{
	return spi_register_driver(&at86rf230_driver);
}
module_init(at86rf230_init);

static void __exit at86rf230_exit(void)
{
	spi_unregister_driver(&at86rf230_driver);
}
module_exit(at86rf230_exit);

MODULE_DESCRIPTION("AT86RF230 Transceiver Driver");
MODULE_LICENSE("GPL v2");
