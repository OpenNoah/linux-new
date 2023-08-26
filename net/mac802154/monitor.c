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
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/nl802154.h>
#include <linux/crc-ccitt.h>

#include <net/mac802154.h>
#include <net/wpan-phy.h>
#include <linux/hardirq.h>

#include "mac802154.h"

static netdev_tx_t mac802154_monitor_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mac802154_sub_if_data *priv;
	u8 chan, page;

	priv = netdev_priv(dev);

	/* FIXME: locking */
	chan = priv->hw->phy->current_channel;
	page = priv->hw->phy->current_page;

	if (chan == (u8)-1) /* not init */
		return NETDEV_TX_OK;

	BUG_ON(page >= WPAN_NUM_PAGES);
	BUG_ON(chan >= 27);

	skb->skb_iif = dev->ifindex;
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	return mac802154_tx(priv->hw, skb, page, chan);
}


void mac802154_monitors_rx(struct mac802154_priv *priv, struct sk_buff *skb)
{
	struct sk_buff *skb2;
	struct mac802154_sub_if_data *sdata;
	u16 crc = crc_ccitt(0, skb->data, skb->len);
	u8 *data;

	rcu_read_lock();
	list_for_each_entry_rcu(sdata, &priv->slaves, list) {
		if (sdata->type != IEEE802154_DEV_MONITOR)
			continue;

		skb2 = skb_clone(skb, GFP_ATOMIC);
		skb2->dev = sdata->dev;
		skb2->pkt_type = PACKET_HOST;
		data = skb_put(skb2, 2);
		data[0] = crc & 0xff;
		data[1] = crc >> 8;

		if (in_interrupt())
			netif_rx(skb2);
		else
			netif_rx_ni(skb2);
	}
	rcu_read_unlock();
}

static const struct net_device_ops mac802154_monitor_ops = {
	.ndo_open		= mac802154_slave_open,
	.ndo_stop		= mac802154_slave_close,
	.ndo_start_xmit		= mac802154_monitor_xmit,
};

void mac802154_monitor_setup(struct net_device *dev)
{
	struct mac802154_sub_if_data *priv;

	dev->addr_len		= 0;
	dev->features		= NETIF_F_HW_CSUM;
	dev->hard_header_len	= 0;
	dev->needed_tailroom	= 2; /* FCS */
	dev->mtu		= 127;
	dev->tx_queue_len	= 10;
	dev->type		= ARPHRD_IEEE802154_MONITOR;
	dev->flags		= IFF_NOARP | IFF_BROADCAST;
	dev->watchdog_timeo	= 0;

	dev->destructor		= free_netdev;
	dev->netdev_ops		= &mac802154_monitor_ops;
	dev->ml_priv		= &mac802154_mlme_simple;

	priv = netdev_priv(dev);
	priv->type = IEEE802154_DEV_MONITOR;

	priv->chan = -1; /* not initialized */
	priv->page = 0; /* for compat */
}

