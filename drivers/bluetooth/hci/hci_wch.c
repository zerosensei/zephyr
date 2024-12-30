/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/irq.h>
#include <zephyr/random/random.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver_wch);

#define DT_DRV_COMPAT wch_wch_bt_hci

#include <soc.h>
#include "wch_bt_hci.h"

struct bt_wch_data {
	bt_hci_recv_t recv;
};

static K_SEM_DEFINE(hci_send_sem, 1, 1);


static void hci_wch_host_rcv_pkt(struct net_buf *evt)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct bt_wch_data *hci = dev->data;

	LOG_HEXDUMP_DBG(evt->b.data, evt->b.len, "host packet data:");

    LOG_DBG("Calling bt_recv(%p)", evt);

    hci->recv(dev, evt);
}

static void hci_wch_send_ready(void)
{
	k_sem_give(&hci_send_sem);
}

static wch_bt_host_callback_t wch_host_cb = {
    .host_rcv_pkt = hci_wch_host_rcv_pkt,
    .host_send_ready = hci_wch_send_ready,
};

static int bt_wch_open(const struct device *dev, bt_hci_recv_t recv)
{
    int ret;
    struct bt_wch_data *hci = dev->data;

    ret = wch_ble_stack_init();

    if (ret) {
        return ret;
    }

    wch_bt_host_callback_register(&wch_host_cb);

    hci->recv = recv;

	LOG_DBG("WCH BT started");

    return 0;
}

static int bt_wch_close(const struct device *dev)
{
    struct bt_wch_data *hci = dev->data;

    hci->recv = NULL;
    wch_ble_stack_deinit();

	LOG_DBG("WCH BT stopped");

    return 0;
}

static int bt_wch_send(const struct device *dev, struct net_buf *buf)
{
	int err;

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	LOG_HEXDUMP_DBG(buf->data, buf->len, "Final HCI buffer:");

	if (k_sem_take(&hci_send_sem, K_MSEC(2000)) == 0) {
        err = wch_bt_host_send(buf);
    } else {
        LOG_ERR("Send packet timeout error");
		err = -ETIMEDOUT;
    }

	net_buf_unref(buf);
	k_sem_give(&hci_send_sem);

    return err;
}


static const struct bt_hci_driver_api drv = {
    .open = bt_wch_open,
    .send = bt_wch_send,
    .close = bt_wch_close,
};

#define BT_WCH_DEVICE_INIT(inst)    \
	static struct bt_wch_data bt_wch_data_##inst = { \
	}; \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &bt_wch_data_##inst, NULL, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

BT_WCH_DEVICE_INIT(0);