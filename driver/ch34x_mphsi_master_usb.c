/*
 * USB to SPI/I2C/GPIO controller driver for USB converter chip CH347/CH341, etc.
 *
 * Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - add supports for i2c controller, gpio irq function
 * V1.2 - add supports for i2c communication of long packets, use workqueue to implement irq setting operation,
 *      - support more spi clock frequency
 * V1.3 - add supports for gpio level triggered interrupt, add mutex in ch347_spi_transfer_one_message
 * V1.4 - fix the big-endian CPU compatibility issue during SPI configuration
 *      - fix the problem that obtaining GPIO status will change GPIO configuration
 * V1.5 - enable I2C function for CH347F automatically
 *      - fix ch34x_i2c_check_dev function of CH341
 * V1.6 - add support for kernel version beyond 6.8.x
 */

#include "ch34x_mphsi.h"

static DEFINE_IDA(ch34x_devid_ida);

/* Table of devices that work with this driver */
static const struct usb_device_id ch34x_usb_ids[] = {
	{ USB_DEVICE(0x1a86, 0x5512) }, /* CH341A/B/C/F/T/H NON-UART Mode*/
	{ USB_DEVICE_INTERFACE_NUMBER(0x1a86, 0x55de, 0x04) }, /* CH347F */
	{ USB_DEVICE_INTERFACE_NUMBER(
		0x1a86, 0x55db, 0x02) }, /* CH347T Mode1 SPI+IIC+UART */
	{ USB_DEVICE_INTERFACE_NUMBER(0x1a86, 0x55e7, 0x02) }, /* CH339W */
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ch34x_usb_ids);

struct ch341_pin_config ch341_board_config[CH341_CS_NUM] = {
	{ 15, "cs0" },
	{ 16, "cs1" },
	{ 17, "cs2" },
};

struct ch347_pin_config ch347t_board_config[CH347T_MPHSI_GPIOS] = {
	{ 15, NULL, 4, GPIO_MODE_OUT, true },
	{ 2, NULL, 6, GPIO_MODE_IN, true },
	{ 13, NULL, 7, GPIO_MODE_OUT, true },
};

struct ch347_pin_config ch347f_board_config[CH347F_MPHSI_GPIOS] = {
	{ 17, NULL, 0, GPIO_MODE_IN, true },
	{ 18, NULL, 1, GPIO_MODE_OUT, true },
	{ 10, NULL, 2, GPIO_MODE_OUT, true },
	{ 9, NULL, 3, GPIO_MODE_OUT, true },
	{ 23, NULL, 4, GPIO_MODE_OUT, true },
	{ 24, NULL, 5, GPIO_MODE_IN, true },
	{ 25, NULL, 6, GPIO_MODE_OUT, true },
	{ 26, NULL, 7, GPIO_MODE_OUT, true },
};

struct ch347_pin_config ch339w_board_config[CH339W_MPHSI_GPIOS] = {
	{ 59, NULL, 1, GPIO_MODE_OUT, false },
	{ 37, NULL, 2, GPIO_MODE_OUT, false },
	{ 36, NULL, 3, GPIO_MODE_OUT, false },
};

extern int ch34x_mphsi_spi_probe(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_spi_remove(struct ch34x_device *ch34x_dev);
extern int ch34x_spi_probe(struct ch34x_device *ch34x_dev);
extern void ch34x_spi_remove(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_i2c_probe(struct ch34x_device *ch34x_dev);
extern void ch34x_mphsi_i2c_remove(struct ch34x_device *ch34x_dev);
extern int ch347_irq_probe(struct ch34x_device *ch34x_dev);
extern void ch347_irq_remove(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_gpio_probe(struct ch34x_device *ch34x_dev);
extern void ch34x_mphsi_gpio_remove(struct ch34x_device *ch34x_dev);
extern int ch347_irq_check(struct ch34x_device *ch34x_dev, u8 irq);
extern int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len,
			      int in_len);
extern int ch34x_usb_transfer_i2c(struct ch34x_device *ch34x_dev,
				  int out_len, int in_len);
extern bool ch347_get_chipinfo(struct ch34x_device *ch34x_dev);
extern bool ch347_func_switch(struct ch34x_device *ch34x_dev, int index);
extern void ch34x_mphsi_i2c_remove(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_i2c_probe(struct ch34x_device *ch34x_dev);

static int ch34x_cfg_probe(struct ch34x_device *ch34x_dev)
{
	struct ch347_pin_config *ch347cfg;
	int i;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);

	if (ch34x_dev->chiptype == CHIP_CH341) {
		/* Setting out: mosi/out2/sck/cs, in: miso */
		ch34x_dev->gpio_mask = 0x3f;
		ch34x_dev->slave_num = CH341_CS_NUM;
	} else if (ch34x_dev->chiptype == CHIP_CH347F ||
		   ch34x_dev->chiptype == CHIP_CH347T) {
		if ((ch34x_dev->firmver >= 0x0341) ||
		    (ch34x_dev->chiptype == CHIP_CH347F)) {
			ch34x_dev->irq_num = 0;
			ch34x_dev->irq_base = 0;
		}
		ch34x_dev->slave_num = CH347_CS_NUM;

		if (ch34x_dev->chiptype == CHIP_CH347T) {
			for (i = 0; i < CH347T_MPHSI_GPIOS; i++) {
				ch347cfg = ch347t_board_config + i;
				ch34x_dev->gpio_names[ch34x_dev->gpio_num] =
					ch347cfg->name;
				ch34x_dev->gpio_pins[ch34x_dev->gpio_num] =
					ch347cfg;
				if (ch34x_dev->firmver >= 0x0341) {
					ch34x_dev->gpio_irq_map
						[ch34x_dev->gpio_num] =
						ch34x_dev->irq_num;
					ch34x_dev->irq_gpio_map
						[ch34x_dev->irq_num] =
						ch34x_dev->gpio_num;
					DEV_DBG(CH34X_USBDEV,
						"%s gpio%d irq=%d %s",
						ch347cfg->mode ==
								GPIO_MODE_IN ?
							"input " :
							"output",
						ch347cfg->gpioindex,
						ch34x_dev->irq_num,
						ch347cfg->hwirq ?
							"(hwirq)" :
							"");
					ch34x_dev->irq_num++;
				} else
					DEV_DBG(CH34X_USBDEV, "%s gpio%d",
						ch347cfg->mode ==
								GPIO_MODE_IN ?
							"input " :
							"output",
						ch347cfg->gpioindex);

				ch34x_dev->gpio_num++;
			}
		} else {
			for (i = 0; i < CH347F_MPHSI_GPIOS; i++) {
				ch347cfg = ch347f_board_config + i;
				ch34x_dev->gpio_names[ch34x_dev->gpio_num] =
					ch347cfg->name;
				ch34x_dev->gpio_pins[ch34x_dev->gpio_num] =
					ch347cfg;
				ch34x_dev
					->gpio_irq_map[ch34x_dev->gpio_num] =
					ch34x_dev->irq_num;
				ch34x_dev->irq_gpio_map[ch34x_dev->irq_num] =
					ch34x_dev->gpio_num;
				DEV_DBG(CH34X_USBDEV, "%s %d irq=%d %s",
					ch347cfg->mode == GPIO_MODE_IN ?
						"input " :
						"output",
					ch347cfg->gpioindex,
					ch34x_dev->irq_num,
					ch347cfg->hwirq ? "(hwirq)" : "");
				ch34x_dev->irq_num++;
				ch34x_dev->gpio_num++;
			}
		}
	} else if (ch34x_dev->chiptype == CHIP_CH339W) {
		ch34x_dev->slave_num = CH339_CS_NUM;
		if (ch34x_dev->chiptype == CHIP_CH339W) {
			for (i = 0; i < CH339W_MPHSI_GPIOS; i++) {
				ch347cfg = ch339w_board_config + i;
				ch34x_dev->gpio_names[ch34x_dev->gpio_num] =
					ch347cfg->name;
				ch34x_dev->gpio_pins[ch34x_dev->gpio_num] =
					ch347cfg;
				DEV_DBG(CH34X_USBDEV, "%s gpio%d",
					ch347cfg->mode == GPIO_MODE_IN ?
						"input " :
						"output",
					ch347cfg->gpioindex);
				ch34x_dev->gpio_num++;
			}
		}
	}

	return 0;
}

static void ch34x_cfg_remove(struct ch34x_device *ch34x_dev)
{
	CHECK_PARAM(ch34x_dev);

	return;
}

static void ch34x_write_bulk_callback(struct urb *urb)
{
	struct ch34x_device *ch34x_dev;
	struct list_head *pos, *n;
	struct usb_cmd_buf *pusb_cmd;

	ch34x_dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN))
			DEV_ERR(CH34X_USBDEV,
				"%s - Nonzero write bulk status received: %d\n",
				__func__, urb->status);

		spin_lock(&ch34x_dev->err_lock);
		ch34x_dev->errors = urb->status;
		spin_unlock(&ch34x_dev->err_lock);
	}

	if (list_empty(&ch34x_dev->usb_cmd_list_used))
		return;

	list_for_each_safe(pos, n, &ch34x_dev->usb_cmd_list_used) {
		pusb_cmd = list_entry(pos, struct usb_cmd_buf, list);
		if (pusb_cmd != NULL) {
			if (pusb_cmd->urb == urb) {
				list_del(pos);
				list_add_tail(
					&pusb_cmd->list,
					&ch34x_dev->usb_cmd_list_free);
			}
		}
	}
}

static void ch34x_batch_buffer_free(struct ch34x_device *ch34x_dev)
{
	struct usb_cmd_buf *pusb_cmd;
	struct list_head *pos, *n;

	list_for_each_safe(pos, n, &ch34x_dev->usb_cmd_list_used) {
		pusb_cmd = list_entry(pos, struct usb_cmd_buf, list);
		list_del(pos);
		if (pusb_cmd != NULL) {
			list_add_tail(&pusb_cmd->list,
				      &ch34x_dev->usb_cmd_list_free);
		}
	}

	list_for_each_safe(pos, n, &ch34x_dev->usb_cmd_list_free) {
		pusb_cmd = list_entry(pos, struct usb_cmd_buf, list);
		list_del(pos);
		if (pusb_cmd != NULL) {
			if (pusb_cmd->urb) {
				if (pusb_cmd->ibuf)
					usb_free_coherent(
						ch34x_dev->usb_dev,
						MAX_BUFFER_LENGTH * 2,
						pusb_cmd->ibuf,
						pusb_cmd->urb
							->transfer_dma);
				usb_free_urb(pusb_cmd->urb);
			}
			kfree(pusb_cmd);
		}
	}
}

static int ch34x_batch_buffer_alloc(struct ch34x_device *ch34x_dev)
{
	int i;
	int retval;
	struct usb_cmd_buf *pusb_cmd;

	INIT_LIST_HEAD(&ch34x_dev->usb_cmd_list_used);
	INIT_LIST_HEAD(&ch34x_dev->usb_cmd_list_free);

	for (i = 0; i < USB_CMD_BATCH_NR; i++) {
		pusb_cmd = (struct usb_cmd_buf *)kmalloc(
			sizeof(struct usb_cmd_buf), GFP_KERNEL);
		if (pusb_cmd == NULL) {
			retval = -ENOMEM;
			goto exit;
		}
		/* Create a urb, and a buffer for it, and copy the data to the urb */
		pusb_cmd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!pusb_cmd->urb) {
			retval = -ENOMEM;
			goto exit;
		}

		pusb_cmd->ibuf = usb_alloc_coherent(
			ch34x_dev->usb_dev, MAX_BUFFER_LENGTH * 2,
			GFP_KERNEL, &pusb_cmd->urb->transfer_dma);
		if (!pusb_cmd->ibuf) {
			retval = -ENOMEM;
			goto exit;
		}

		pusb_cmd->ilen = MAX_BUFFER_LENGTH * 2;

		list_add_tail(&pusb_cmd->list,
			      &ch34x_dev->usb_cmd_list_free);
	}

	return 0;

exit:
	ch34x_batch_buffer_free(ch34x_dev);

	return retval;
}

int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len,
		       int in_len)
{
	int retval;
	int actual = 0;
	int rlen;
	int size;
	struct usb_cmd_buf *pusb_cmd = NULL;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);

	if (out_len != 0) {
		if (!list_empty(&ch34x_dev->usb_cmd_list_free)) {
			pusb_cmd = list_entry(
				ch34x_dev->usb_cmd_list_free.next,
				struct usb_cmd_buf, list);
			list_del(ch34x_dev->usb_cmd_list_free.next);
			if (pusb_cmd != NULL) {
				pusb_cmd->ilen = out_len;
				memcpy(pusb_cmd->ibuf,
				       ch34x_dev->bulkout_buf, out_len);
				list_add_tail(
					&pusb_cmd->list,
					&ch34x_dev->usb_cmd_list_used);
			} else {
				retval = -ENOMEM;
				goto error;
			}
		} else {
			retval = -ENOMEM;
			goto error;
		}

		/* Initialize the urb properly */
		usb_fill_bulk_urb(
			pusb_cmd->urb, ch34x_dev->usb_dev,
			usb_sndbulkpipe(ch34x_dev->usb_dev,
					ch34x_dev->bulk_out_endpointAddr),
			pusb_cmd->ibuf, pusb_cmd->ilen,
			ch34x_write_bulk_callback, ch34x_dev);
		pusb_cmd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(pusb_cmd->urb, &ch34x_dev->submitted);

		/* Send the data out the bulk port */
		retval = usb_submit_urb(pusb_cmd->urb, GFP_KERNEL);
		if (retval) {
			goto error_unanchor;
		}
	}

	if (in_len == 0) {
		actual = out_len;
		goto exit;
	}

	size = in_len;

	memset(ch34x_dev->bulkin_buf, 0, MAX_BUFFER_LENGTH * 2);

	while (actual < size) {
		retval = usb_bulk_msg(
			ch34x_dev->usb_dev,
			usb_rcvbulkpipe(
				ch34x_dev->usb_dev,
				usb_endpoint_num(ch34x_dev->bulk_in)),
			ch34x_dev->bulkin_buf + actual, size - actual,
			&rlen, 2000);
		if (retval) {
			DEV_ERR(CH34X_USBDEV,
				"%s - Failed in usb_bulk_msg, error %d\n",
				__func__, retval);
			break;
		}
		actual += rlen;
	}

exit:
	return retval < 0 ? retval : actual;

error_unanchor:
	usb_unanchor_urb(pusb_cmd->urb);
error:
	return retval;
}

int ch34x_usb_transfer_i2c(struct ch34x_device *ch34x_dev, int out_len,
			   int in_len)
{
	int retval;
	int actual = 0;
	int rlen;
	int size;
	struct usb_cmd_buf *pusb_cmd = NULL;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);

	if (out_len != 0) {
		if (!list_empty(&ch34x_dev->usb_cmd_list_free)) {
			pusb_cmd = list_entry(
				ch34x_dev->usb_cmd_list_free.next,
				struct usb_cmd_buf, list);
			list_del(ch34x_dev->usb_cmd_list_free.next);
			if (pusb_cmd != NULL) {
				pusb_cmd->ilen = out_len;
				memcpy(pusb_cmd->ibuf,
				       ch34x_dev->bulkout_buf, out_len);
				list_add_tail(
					&pusb_cmd->list,
					&ch34x_dev->usb_cmd_list_used);
			} else {
				retval = -ENOMEM;
				goto error;
			}
		} else {
			retval = -ENOMEM;
			goto error;
		}

		/* Initialize the urb properly */
		usb_fill_bulk_urb(
			pusb_cmd->urb, ch34x_dev->usb_dev,
			usb_sndbulkpipe(ch34x_dev->usb_dev,
					ch34x_dev->bulk_out_endpointAddr),
			pusb_cmd->ibuf, pusb_cmd->ilen,
			ch34x_write_bulk_callback, ch34x_dev);
		pusb_cmd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(pusb_cmd->urb, &ch34x_dev->submitted);

		/* Send the data out the bulk port */
		retval = usb_submit_urb(pusb_cmd->urb, GFP_KERNEL);
		if (retval) {
			goto error_unanchor;
		}
	}

	if (in_len == 0) {
		actual = out_len;
		goto exit;
	}

	size = in_len;
	memset(ch34x_dev->bulkin_buf, 0, MAX_BUFFER_LENGTH * 2);
	while (actual < size) {
		retval = usb_bulk_msg(
			ch34x_dev->usb_dev,
			usb_rcvbulkpipe(
				ch34x_dev->usb_dev,
				usb_endpoint_num(ch34x_dev->bulk_in)),
			ch34x_dev->bulkin_buf + actual, size - actual,
			&rlen, 2000);
		if (retval) {
			DEV_ERR(CH34X_USBDEV,
				"%s - Failed in usb_bulk_msg, error %d\n",
				__func__, retval);
			goto exit;
		}
		actual += rlen;
	}

exit:
	return retval < 0 ? retval : actual;

error_unanchor:
	usb_unanchor_urb(pusb_cmd->urb);
error:
	return retval;
}

bool ch347_get_chipinfo(struct ch34x_device *ch34x_dev)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;
	bool ret = false;

	mutex_lock(&ch34x_dev->io_mutex);
	i = 0;
	io[i++] = USB20_CMD_INFO_RD;
	io[i++] = 1;
	io[i++] = 0;
	io[i++] = 0;
	len = i;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len)
		goto exit;

	len = 4 + USB20_CMD_HEADER;

	if (ch34x_usb_transfer(ch34x_dev, 0, len) != len)
		goto exit;

	if (ch34x_dev->bulkin_buf[0] != USB20_CMD_INFO_RD)
		goto exit;

	ch34x_dev->firmver = (ch34x_dev->bulkin_buf[4] << 8) |
			     ch34x_dev->bulkin_buf[3];
	ret = true;

exit:
	mutex_unlock(&ch34x_dev->io_mutex);
	return ret;
}

bool ch347_func_switch(struct ch34x_device *ch34x_dev, int index)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;
	bool ret = false;

	memset(io, 0x00, USB20_CMD_HEADER + 8);

	mutex_lock(&ch34x_dev->io_mutex);
	i = 0;
	io[i++] = USB20_CMD_FUNC_SWITCH;
	io[i++] = 8;
	io[i++] = 0;
	switch (index) {
	case 0:
		io[USB20_CMD_HEADER] = 0x81;
		break;
	case 1:
		io[USB20_CMD_HEADER + 1] = 0x81;
		break;
	case 2:
		io[USB20_CMD_HEADER + 1] = 0x82;
		break;
	case 3:
		io[USB20_CMD_HEADER + 2] = 0x81;
		io[USB20_CMD_HEADER + 3] = 0x81;
		break;
	case 4:
		io[USB20_CMD_HEADER] = 0x81;
		io[USB20_CMD_HEADER + 2] = 0x81;
		io[USB20_CMD_HEADER + 3] = 0x81;
		break;
	default:
		break;
	}
	len = USB20_CMD_HEADER + 8;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len)
		goto exit;

	len = 4;

	if (ch34x_usb_transfer(ch34x_dev, 0, len) != len)
		goto exit;

	if ((ch34x_dev->bulkin_buf[0] != USB20_CMD_FUNC_SWITCH) ||
	    (ch34x_dev->bulkin_buf[USB20_CMD_HEADER] != 0x00))
		goto exit;

	ret = true;

exit:
	mutex_unlock(&ch34x_dev->io_mutex);
	return ret;
}

static void ch34x_usb_complete_intr_urb(struct urb *urb)
{
	struct ch34x_device *ch34x_dev;
	u8 gpioindex;
	bool triggered;
	int i;
	u16 io_status;
	int gpiocount;

	CHECK_PARAM(urb);
	CHECK_PARAM(ch34x_dev = urb->context);

	if (!urb->status) {
		DEV_DBG(CH34X_USBDEV, "%d", urb->status);

		if (ch34x_dev->chiptype == CHIP_CH347F)
			gpiocount = CH347F_MPHSI_GPIOS;
		else
			gpiocount = CH347T_MPHSI_GPIOS;

		if (ch34x_dev->chiptype != CHIP_CH341) {
			for (i = 0; i < gpiocount; i++) {
				gpioindex =
					ch34x_dev->gpio_pins[i]->gpioindex;
				triggered =
					ch34x_dev->intrin_buf[gpioindex +
							      3] &
					BIT(3);
				DEV_DBG(CH34X_USBDEV,
					"GPIO status: irq_enable[%d]=%d gpioindex=%d triggered=%d",
					i, ch34x_dev->irq_enabled[i],
					gpioindex, triggered);
				if (ch34x_dev->irq_enabled[i] && triggered)
					ch347_irq_check(ch34x_dev, i);
			}
		} else {
			io_status = (ch34x_dev->intrin_buf[1] & 0x0F)
					    << 8 |
				    ch34x_dev->intrin_buf[2];
			/* Bit7~Bit0<==>D7-D0, Bit8<==>ERR#, Bit9<==>PEMP, Bit10<==>INT#, Bit11<==>SLCT */
			DEV_DBG(CH34X_USBDEV, "CH341 io_status: 0x%04x",
				io_status);
		}

		usb_submit_urb(ch34x_dev->intr_urb, GFP_ATOMIC);
	}
}

static void ch34x_usb_free_device(struct ch34x_device *ch34x_dev)
{
	CHECK_PARAM(ch34x_dev)

	if (ch34x_dev->intr_urb)
		usb_free_urb(ch34x_dev->intr_urb);

	usb_set_intfdata(ch34x_dev->intf, NULL);
	usb_kill_anchored_urbs(&ch34x_dev->submitted);
	usb_put_dev(ch34x_dev->usb_dev);
	kfree(ch34x_dev);
}

static int ch34x_usb_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *usb_dev =
		usb_get_dev(interface_to_usbdev(intf));
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *iface_desc;
	struct ch34x_device *ch34x_dev;
	int i;
	int ret = 0;
	int length = 0;

	DEV_DBG(&intf->dev, "Connect device...");

	ch34x_dev = kzalloc(sizeof(struct ch34x_device), GFP_KERNEL);
	if (!ch34x_dev) {
		DEV_ERR(&intf->dev, "Could not allocate device memory");
		return -ENOMEM;
	}

	ch34x_dev->usb_dev = usb_dev;
	ch34x_dev->intf = intf;
	iface_desc = intf->cur_altsetting;

	DEV_DBG(CH34X_USBDEV, "bNumEndpoints=%d",
		iface_desc->desc.bNumEndpoints);

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		DEV_DBG(CH34X_USBDEV,
			"    ->endpoint=%d type=%d dir=%d addr=%0x", i,
			usb_endpoint_type(endpoint),
			usb_endpoint_dir_in(endpoint),
			usb_endpoint_num(endpoint));

		if (usb_endpoint_is_bulk_in(endpoint)) {
			ch34x_dev->bulk_in = endpoint;
			ch34x_dev->bulkin_buf =
				kmalloc(MAX_BUFFER_LENGTH * 2, GFP_KERNEL);
			if (!ch34x_dev->bulkin_buf) {
				DEV_ERR(CH34X_USBDEV,
					"Could not allocate bulkin buffer");
				goto error;
			}
		}

		else if (usb_endpoint_is_bulk_out(endpoint)) {
			ch34x_dev->bulk_out = endpoint;
			ch34x_dev->bulk_out_endpointAddr =
				endpoint->bEndpointAddress;
			ch34x_dev->bulkout_buf =
				kmalloc(MAX_BUFFER_LENGTH * 2, GFP_KERNEL);
			if (!ch34x_dev->bulkout_buf) {
				DEV_ERR(CH34X_USBDEV,
					"Could not allocate bulkout buffer");
				goto error;
			}
		}

		else if (usb_endpoint_xfer_int(endpoint)) {
			ch34x_dev->intr_in = endpoint;
			ch34x_dev->intrin_buf = kmalloc(
				CH347_USB_MAX_INTR_SIZE, GFP_KERNEL);
			if (!ch34x_dev->intrin_buf) {
				DEV_ERR(CH34X_USBDEV,
					"Could not allocate intrin buffer");
				goto error;
			}
		}
	}

	ret = ch34x_batch_buffer_alloc(ch34x_dev);
	if (ret)
		goto error;

	if (id->idProduct == 0x5512) {
		ch34x_dev->chiptype = CHIP_CH341;
		length = CH341_USB_MAX_INTR_SIZE;
	} else if (id->idProduct == 0x55de) {
		ch34x_dev->chiptype = CHIP_CH347F;
		length = CH347_USB_MAX_INTR_SIZE;
	} else if (id->idProduct == 0x55db) {
		ch34x_dev->chiptype = CHIP_CH347T;
		length = CH347_USB_MAX_INTR_SIZE;
	} else if (id->idProduct == 0x55e7) {
		ch34x_dev->chiptype = CHIP_CH339W;
		length = CH347_USB_MAX_INTR_SIZE;
	}

	/* Save the pointer to the new ch34x_device in USB interface device data */
	usb_set_intfdata(intf, ch34x_dev);
	mutex_init(&ch34x_dev->io_mutex);
	spin_lock_init(&ch34x_dev->err_lock);
	init_usb_anchor(&ch34x_dev->submitted);

	if (ch34x_dev->chiptype != CHIP_CH341) {
		if (ch347_get_chipinfo(ch34x_dev) == false) {
			ret = -EPROTO;
			goto error;
		}
		if (ch34x_dev->chiptype == CHIP_CH347F) {
			if (!ch347_func_switch(ch34x_dev, 4)) {
				ret = -EPROTO;
				goto error;
			}
		}
		DEV_DBG(CH34X_USBDEV, "Firmware version: 0x%x.",
			ch34x_dev->firmver);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	ch34x_dev->id = ida_alloc(&ch34x_devid_ida, GFP_KERNEL);
#else
	ch34x_dev->id = ida_simple_get(&ch34x_devid_ida, 0, 0, GFP_KERNEL);
#endif
	if (ch34x_dev->id < 0) {
		ret = ch34x_dev->id;
		goto error;
	}

	ret = ch34x_cfg_probe(ch34x_dev);
	if (ret < 0)
		goto error1;

	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F)) {
		ret = ch347_irq_probe(ch34x_dev);
		if (ret < 0)
			goto error2;
	}

	if (ch34x_dev->chiptype != CHIP_CH341) {
		ret = ch34x_mphsi_gpio_probe(ch34x_dev);
		if (ret < 0)
			goto error3;
	}

	ret = ch34x_mphsi_spi_probe(ch34x_dev);
	if (ret < 0)
		goto error4;

	ret = ch34x_spi_probe(ch34x_dev);
	if (ret < 0)
		goto error5;

	ret = ch34x_mphsi_i2c_probe(ch34x_dev);
	if (ret < 0)
		goto error6;

	if (ch34x_dev->intr_in) {
		/* Xreate URB for handling interrupts */
		if (!(ch34x_dev->intr_urb =
			      usb_alloc_urb(0, GFP_KERNEL))) {
			DEV_ERR(&intf->dev, "failed to alloc URB");
			ret = -ENOMEM;
			goto error7;
		}

		usb_fill_int_urb(
			ch34x_dev->intr_urb, ch34x_dev->usb_dev,
			usb_rcvintpipe(
				ch34x_dev->usb_dev,
				usb_endpoint_num(ch34x_dev->intr_in)),
			ch34x_dev->intrin_buf, length,
			ch34x_usb_complete_intr_urb, ch34x_dev,
			ch34x_dev->intr_in->bInterval);

		usb_submit_urb(ch34x_dev->intr_urb, GFP_ATOMIC);
	}

	DEV_INFO(CH34X_USBDEV,
		 "USB to SPI/I2C/GPIO adapter ch34x now attached.");

	return 0;

error7:
	ch34x_mphsi_i2c_remove(ch34x_dev);
error6:
	ch34x_spi_remove(ch34x_dev);
error5:
	ch34x_mphsi_spi_remove(ch34x_dev);
error4:
	ch34x_mphsi_gpio_remove(ch34x_dev);
error3:
	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F))
		ch347_irq_remove(ch34x_dev);
error2:
	ch34x_cfg_remove(ch34x_dev);
error1:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	ida_free(&ch34x_devid_ida, ch34x_dev->id);
#else
	ida_simple_remove(&ch34x_devid_ida, ch34x_dev->id);
#endif
error:
	ch34x_batch_buffer_free(ch34x_dev);
	if (ch34x_dev->bulkin_buf)
		kfree(ch34x_dev->bulkin_buf);
	if (ch34x_dev->bulkout_buf)
		kfree(ch34x_dev->bulkout_buf);
	if (ch34x_dev->intrin_buf)
		kfree(ch34x_dev->intrin_buf);
	ch34x_usb_free_device(ch34x_dev);

	return ret;
}

static void ch34x_draw_down(struct ch34x_device *ch34x_dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&ch34x_dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&ch34x_dev->submitted);
}

static int ch34x_usb_suspend(struct usb_interface *intf,
			     pm_message_t message)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	if (!ch34x_dev)
		return 0;
	ch34x_draw_down(ch34x_dev);
	return 0;
}

static int ch34x_usb_resume(struct usb_interface *intf)
{
	return 0;
}

static int ch34x_pre_reset(struct usb_interface *intf)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	mutex_lock(&ch34x_dev->io_mutex);
	ch34x_draw_down(ch34x_dev);

	return 0;
}

static int ch34x_post_reset(struct usb_interface *intf)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	ch34x_dev->errors = -EPIPE;
	mutex_unlock(&ch34x_dev->io_mutex);

	return 0;
}

static void ch34x_usb_disconnect(struct usb_interface *intf)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	DEV_INFO(CH34X_USBDEV, "CH34X adapter now disconnected");

	ch34x_batch_buffer_free(ch34x_dev);
	ch34x_mphsi_i2c_remove(ch34x_dev);
	ch34x_spi_remove(ch34x_dev);
	ch34x_mphsi_spi_remove(ch34x_dev);
	ch34x_mphsi_gpio_remove(ch34x_dev);
	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F))
		ch347_irq_remove(ch34x_dev);
	ch34x_cfg_remove(ch34x_dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	ida_free(&ch34x_devid_ida, ch34x_dev->id);
#else
	ida_simple_remove(&ch34x_devid_ida, ch34x_dev->id);
#endif
	if (ch34x_dev->bulkin_buf)
		kfree(ch34x_dev->bulkin_buf);
	if (ch34x_dev->bulkout_buf)
		kfree(ch34x_dev->bulkout_buf);
	if (ch34x_dev->intrin_buf)
		kfree(ch34x_dev->intrin_buf);
	ch34x_usb_free_device(ch34x_dev);
}

static struct usb_driver ch34x_usb_driver = {
	.name = "mphsi-ch34x",
	.id_table = ch34x_usb_ids,
	.probe = ch34x_usb_probe,
	.disconnect = ch34x_usb_disconnect,
	.suspend = ch34x_usb_suspend,
	.resume = ch34x_usb_resume,
	.pre_reset = ch34x_pre_reset,
	.post_reset = ch34x_post_reset,
};

module_usb_driver(ch34x_usb_driver);

MODULE_ALIAS(DRIVER_ALIAS);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
