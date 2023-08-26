/*
 * Copyright 2007, 2008, 2009 Siemens AG
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
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 * Sergey Lapin <slapin@ossfans.org>
 * Maxim Gorbachyov <maxim.gorbachev@siemens.com>
 */

#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/crc-ccitt.h>

#include <net/mac802154.h>
#include <net/wpan-phy.h>

#include "mac802154.h"

struct xmit_work {
	struct sk_buff *skb;
	struct work_struct work;
	struct mac802154_priv *priv;
	u8 page;
	u8 chan;
};

static void mac802154_xmit_worker(struct work_struct *work)
{
	struct xmit_work *xw = container_of(work, struct xmit_work, work);
	int res;

	BUG_ON(xw->chan == (u8)-1);

	mutex_lock(&xw->priv->phy->pib_lock);
	if (xw->priv->phy->current_channel != xw->chan ||
	    xw->priv->phy->current_page != xw->page) {
		res = xw->priv->ops->set_channel(&xw->priv->hw,
				xw->page,
				xw->chan);
		if (res) {
			pr_debug("set_channel failed\n");
			goto out;
		}
	}

	res = xw->priv->ops->xmit(&xw->priv->hw, xw->skb);

out:
	mutex_unlock(&xw->priv->phy->pib_lock);

	/* FIXME: result processing and/or requeue!!! */
	dev_kfree_skb(xw->skb);

	kfree(xw);
}

netdev_tx_t mac802154_tx(struct mac802154_priv *priv, struct sk_buff *skb,
		u8 page, u8 chan)
{
	struct xmit_work *work;

	if (WARN_ON(!(priv->phy->channels_supported[page] &
					(1 << chan))))
		return NETDEV_TX_OK;

	mac802154_monitors_rx(mac802154_to_priv(&priv->hw), skb);

	if (!(priv->hw.flags & IEEE802154_HW_OMIT_CKSUM)) {
		u16 crc = crc_ccitt(0, skb->data, skb->len);
		u8 *data = skb_put(skb, 2);
		data[0] = crc & 0xff;
		data[1] = crc >> 8;
	}

	if (skb_cow_head(skb, priv->hw.extra_tx_headroom)) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	work = kzalloc(sizeof(struct xmit_work), GFP_ATOMIC);
	if (!work)
		return NETDEV_TX_BUSY;

	INIT_WORK(&work->work, mac802154_xmit_worker);
	work->skb = skb;
	work->priv = priv;
	work->page = page;
	work->chan = chan;

	queue_work(priv->dev_workqueue, &work->work);

	return NETDEV_TX_OK;
}
