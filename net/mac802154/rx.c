/*
 * Copyright (C) 2007, 2008, 2009 Siemens AG
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
 * Pavel Smolenskiy <pavel.smolenskiy@gmail.com>
 * Maxim Gorbachyov <maxim.gorbachev@siemens.com>
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <linux/crc-ccitt.h>

#include <net/mac802154.h>
#include <net/ieee802154_netdev.h>

#include "mac802154.h"

static void mac802154_subif_rx(struct ieee802154_dev *hw, struct sk_buff *skb)
{
	struct mac802154_priv *priv = mac802154_to_priv(hw);

	BUILD_BUG_ON(sizeof(struct ieee802154_mac_cb) > sizeof(skb->cb));
	pr_debug("%s()\n", __func__);

	if (!(priv->hw.flags & IEEE802154_HW_OMIT_CKSUM)) {
		u16 crc;

		if (skb->len < 2) {
			pr_debug("%s(): Got invalid frame\n", __func__);
			goto out;
		}
		crc = crc_ccitt(0, skb->data, skb->len);
		if (crc) {
			pr_debug("%s(): CRC mismatch\n", __func__);
			goto out;
		}
		skb_trim(skb, skb->len - 2); /* CRC */
	}

	mac802154_monitors_rx(priv, skb);
	mac802154_smacs_rx(priv, skb);
	mac802154_wpans_rx(priv, skb);

out:
	dev_kfree_skb(skb);
	return;
}

static void __mac802154_rx_prepare(struct ieee802154_dev *dev,
		struct sk_buff *skb, u8 lqi)
{
	BUG_ON(!skb);

	mac_cb(skb)->lqi = lqi;

	skb->protocol = htons(ETH_P_IEEE802154);

	skb_reset_mac_header(skb);
}

void mac802154_rx(struct ieee802154_dev *dev, struct sk_buff *skb, u8 lqi)
{
	__mac802154_rx_prepare(dev, skb, lqi);

	mac802154_subif_rx(dev, skb);
}
EXPORT_SYMBOL(mac802154_rx);

struct rx_work {
	struct sk_buff *skb;
	struct work_struct work;
	struct ieee802154_dev *dev;
};

static void mac802154_rx_worker(struct work_struct *work)
{
	struct rx_work *rw = container_of(work, struct rx_work, work);
	struct sk_buff *skb = rw->skb;

	mac802154_subif_rx(rw->dev, skb);
	kfree(rw);
}

void ieee802154_rx_irqsafe(struct ieee802154_dev *dev,
		struct sk_buff *skb, u8 lqi)
{
	struct mac802154_priv *priv = mac802154_to_priv(dev);
	struct rx_work *work = kzalloc(sizeof(struct rx_work), GFP_ATOMIC);

	if (!work)
		return;

	__mac802154_rx_prepare(dev, skb, lqi);

	INIT_WORK(&work->work, mac802154_rx_worker);
	work->skb = skb;
	work->dev = dev;

	queue_work(priv->dev_workqueue, &work->work);
}
EXPORT_SYMBOL(ieee802154_rx_irqsafe);
