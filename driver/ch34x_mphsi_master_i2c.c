/*
 * ch347/ch341 MPHSI I2C driver layer
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "ch34x_mphsi.h"

extern int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len, int in_len);
extern bool ch347_func_switch(struct ch34x_device *ch34x_dev, int index);

static int ch34x_i2c_check_dev(struct ch34x_device *ch34x_dev, u8 addr)
{
	int retval;

	ch34x_dev->bulkout_buf[0] = CH341_CMD_I2C_STREAM;
	ch34x_dev->bulkout_buf[1] = CH341_CMD_I2C_STM_STA;
	ch34x_dev->bulkout_buf[2] = CH341_CMD_I2C_STM_OUT; /* NOTE: must be zero length otherwise it
					  messes up the device */
	ch34x_dev->bulkout_buf[3] = (addr << 1) | 0x01;
	ch34x_dev->bulkout_buf[4] = CH341_CMD_I2C_STM_STO;
	ch34x_dev->bulkout_buf[5] = CH341_CMD_I2C_STM_END;

	retval = ch34x_usb_transfer(ch34x_dev, 6, 2);
	if (retval < 0)
		goto exit;

	if (ch34x_dev->chiptype == CHIP_CH341) {
		if (ch34x_dev->bulkin_buf[0] & 0x80) {
			retval = -ETIMEDOUT;
			goto exit;
		}
	} else {
		if (ch34x_dev->bulkin_buf[0] == 0x00) {
			retval = -ETIMEDOUT;
			goto exit;
		}
	}
exit:
	return retval;
}

static int ch341_i2c_stream(struct ch34x_device *ch34x_dev, u32 txlen, void *txbuf, u32 rxlen, void *rxbuf)
{
	int i, j, len;
	int retval = 0;
	u8 *io = ch34x_dev->bulkout_buf;

	i = 0;
	io[i++] = CH341_CMD_I2C_STREAM;
	io[i++] = CH341_CMD_I2C_STM_STA;

	if (txlen) {
		for (j = 0; j < txlen;) {
			len = CH341_USB_MAX_BULK_SIZE - i % CH341_USB_MAX_BULK_SIZE;
			if (len <= 2) {
				while (len--)
					io[i++] = CH341_CMD_I2C_STM_END;
				len = CH341_USB_MAX_BULK_SIZE;
			}
			if (len >= CH341_USB_MAX_BULK_SIZE) {
				io[i++] = CH341_CMD_I2C_STREAM;
				len = CH341_USB_MAX_BULK_SIZE - 1;
			}
			len--;
			len--;
			if (len > txlen - j)
				len = txlen - j;
			io[i++] = (u8)(CH341_CMD_I2C_STM_OUT | len);
			while (len--)
				io[i++] = *((u8 *)txbuf + j++);
		}
	}

	if (rxlen) {
		len = CH341_USB_MAX_BULK_SIZE - i % CH341_USB_MAX_BULK_SIZE;
		if (len <= 3) {
			while (len--)
				io[i++] = CH341_CMD_I2C_STM_END;
			len = CH341_USB_MAX_BULK_SIZE;
		}
		if (len >= CH341_USB_MAX_BULK_SIZE)
			io[i++] = CH341_CMD_I2C_STREAM;
		if (txlen > 1) {
			io[i++] = CH341_CMD_I2C_STM_STA;
			io[i++] = (u8)(CH341_CMD_I2C_STM_OUT | 1);
			io[i++] = *(u8 *)txbuf | 0x01;
		} else if (txlen) {
			i--;
			io[i++] = *(u8 *)txbuf | 0x01;
		}
		for (j = 1; j < rxlen;) {
			len = CH341_USB_MAX_BULK_SIZE - i % CH341_USB_MAX_BULK_SIZE;
			if (len <= 1) {
				if (len)
					io[i++] = CH341_CMD_I2C_STM_END;
				len = CH341_USB_MAX_BULK_SIZE;
			}
			if (len >= CH341_USB_MAX_BULK_SIZE)
				io[i++] = CH341_CMD_I2C_STREAM;
			len = (rxlen - j) >= CH341_USB_MAX_BULK_SIZE ? CH341_USB_MAX_BULK_SIZE : (rxlen - j);
			io[i++] = (u8)(CH341_CMD_I2C_STM_IN | len);
			j += len;
			if (len >= CH341_USB_MAX_BULK_SIZE) {
				io[i] = CH341_CMD_I2C_STM_END;
				i += CH341_USB_MAX_BULK_SIZE - i % CH341_USB_MAX_BULK_SIZE;
			}
		}
		len = CH341_USB_MAX_BULK_SIZE - i % CH341_USB_MAX_BULK_SIZE;
		if (len <= 1) {
			if (len)
				io[i++] = CH341_CMD_I2C_STM_END;
			len = CH341_USB_MAX_BULK_SIZE;
		}
		if (len >= CH341_USB_MAX_BULK_SIZE)
			io[i++] = CH341_CMD_I2C_STREAM;
		io[i++] = CH341_CMD_I2C_STM_IN;
	}
	len = CH341_USB_MAX_BULK_SIZE - i % CH341_USB_MAX_BULK_SIZE;
	if (len <= 1) {
		if (len)
			io[i++] = CH341_CMD_I2C_STM_END;
		len = CH341_USB_MAX_BULK_SIZE;
	}
	if (len >= CH341_USB_MAX_BULK_SIZE)
		io[i++] = CH341_CMD_I2C_STREAM;
	io[i++] = CH341_CMD_I2C_STM_STO;
	io[i++] = CH341_CMD_I2C_STM_END;
	len = 0;

	if (rxlen) {
		len = ch34x_usb_transfer(ch34x_dev, i,
					 (rxlen + (CH341_USB_MAX_BULK_SIZE - 1)) / CH341_USB_MAX_BULK_SIZE *
						 CH341_USB_MAX_BULK_SIZE);
		if (len != rxlen) {
			retval = -EPROTO;
			goto error;
		}
		memcpy(rxbuf, ch34x_dev->bulkin_buf, rxlen);
	} else {
		len = ch34x_usb_transfer(ch34x_dev, i, 0);
		if (len != i) {
			retval = -EPROTO;
			goto error;
		}
	}
error:
	return retval;
}

static int ch347_i2c_stream(struct ch34x_device *ch34x_dev, u32 txlen, void *txbuf, u32 rxlen, void *rxbuf)
{
	int i, j, k, len;
	int retval = 0;
	int ackbits = 0;
	int t_ackbits;
	int packnum = 1;
	int readnum = 0;
	u8 *io = ch34x_dev->bulkout_buf;

	i = 0;
	ackbits = 0;
	io[i++] = CH341_CMD_I2C_STREAM;
	io[i++] = CH341_CMD_I2C_STM_STA;

	if (txlen) {
		for (j = 0; j < txlen;) {
			len = CH347_USB_BULK_EPSIZE - i % CH347_USB_BULK_EPSIZE;
			if (len <= 2) {
				while (len--)
					io[i++] = CH341_CMD_I2C_STM_END;
				len = CH347_USB_BULK_EPSIZE;
			}
			if (len >= CH347_USB_BULK_EPSIZE) {
				io[i++] = CH341_CMD_I2C_STREAM;
				len = CH347_USB_BULK_EPSIZE - 1;
				packnum++;
			}
			len--;
			len--;
			if (len > CH347_CMD_I2C_STM_MAX)
				len = CH347_CMD_I2C_STM_MAX;
			if (len > txlen - j)
				len = txlen - j;
			io[i++] = (u8)(CH341_CMD_I2C_STM_OUT | len);
			ackbits += len;
			while (len--)
				io[i++] = *((u8 *)txbuf + j++);
		}
	}

	if (rxlen) {
		len = CH347_USB_BULK_EPSIZE - i % CH347_USB_BULK_EPSIZE;
		if (len <= 3) {
			while (len--)
				io[i++] = CH341_CMD_I2C_STM_END;
			len = CH347_USB_BULK_EPSIZE;
		}
		if (len >= CH347_USB_BULK_EPSIZE)
			io[i++] = CH341_CMD_I2C_STREAM;
		if (txlen > 1) {
			io[i++] = CH341_CMD_I2C_STM_STA;
			io[i++] = (u8)(CH341_CMD_I2C_STM_OUT | 1);
			io[i++] = *(u8 *)txbuf | 0x01;
			ackbits += 1;
		} else if (txlen) {
			i--;
			io[i++] = *(u8 *)txbuf | 0x01;
		}
		t_ackbits = ackbits;
		for (j = 1; j < rxlen;) {
			len = CH347_USB_BULK_EPSIZE - i % CH347_USB_BULK_EPSIZE;
			if (len <= 1) {
				if (len)
					io[i++] = CH341_CMD_I2C_STM_END;
				len = CH347_USB_BULK_EPSIZE;
			}
			if (len >= CH347_USB_BULK_EPSIZE) {
				io[i++] = CH341_CMD_I2C_STREAM;
				packnum++;
			}
			len = (rxlen - j) >= CH347_CMD_I2C_STM_MAX ? CH347_CMD_I2C_STM_MAX : (rxlen - j);
			if ((readnum + len) >= (CH347_USB_BULK_EPSIZE - t_ackbits)) {
				len = CH347_USB_BULK_EPSIZE - i % CH347_USB_BULK_EPSIZE;
				while (len--)
					io[i++] = CH341_CMD_I2C_STM_END;
				readnum = 0;
				t_ackbits = 0;
				continue;
			}
			io[i++] = (u8)(CH341_CMD_I2C_STM_IN | len);
			j += len;
			readnum += len;
		}
		len = CH347_USB_BULK_EPSIZE - i % CH347_USB_BULK_EPSIZE;
		if (len <= 1) {
			if (len)
				io[i++] = CH341_CMD_I2C_STM_END;
			len = CH347_USB_BULK_EPSIZE;
		}
		if (len >= CH347_USB_BULK_EPSIZE)
			io[i++] = CH341_CMD_I2C_STREAM;
		io[i++] = CH341_CMD_I2C_STM_IN;
	}
	len = CH347_USB_BULK_EPSIZE - i % CH347_USB_BULK_EPSIZE;
	if (len <= 1) {
		if (len)
			io[i++] = CH341_CMD_I2C_STM_END;
		len = CH347_USB_BULK_EPSIZE;
	}
	if (len >= CH347_USB_BULK_EPSIZE)
		io[i++] = CH341_CMD_I2C_STREAM;
	io[i++] = CH341_CMD_I2C_STM_STO;
	io[i++] = CH341_CMD_I2C_STM_END;
	len = 0;

	if (rxlen) {
		len = ch34x_usb_transfer(ch34x_dev, i,
					 (rxlen + (CH347_USB_BULK_EPSIZE - 1)) / CH347_USB_BULK_EPSIZE *
						 (CH347_USB_BULK_EPSIZE + ackbits));
		if (len != (rxlen + ackbits)) {
			retval = -EPROTO;
			goto error;
		}

		for (k = 0; k < ackbits; k++) {
			if (ch34x_dev->bulkin_buf[k] != 0x01) {
				retval = -EPROTO;
				goto error;
			}
		}
		len -= ackbits;
		memmove(ch34x_dev->bulkin_buf, ch34x_dev->bulkin_buf + ackbits, len);
		memcpy(rxbuf, ch34x_dev->bulkin_buf, rxlen);
	} else {
		len = ch34x_usb_transfer(ch34x_dev, i, ackbits);
		if (len != ackbits) {
			retval = -EPROTO;
			goto error;
		}

		for (k = 0; k < ackbits; k++) {
			if (ch34x_dev->bulkin_buf[k] != 0x01) {
				retval = -EPROTO;
				goto error;
			}
		}
	}
error:
	return retval;
}

static int ch34x_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct ch34x_device *ch34x_dev = (struct ch34x_device *)adapter->algo_data;
	int retval;
	u8 *txbuf;

	txbuf = kmalloc(MAX_BUFFER_LENGTH * 2, GFP_KERNEL);
	if (!txbuf)
		return -ENOMEM;

	mutex_lock(&ch34x_dev->io_mutex);

	if (ch34x_dev->chiptype == CHIP_CH341) {
		retval = ch34x_i2c_check_dev(ch34x_dev, msgs[0].addr);
		if (retval < 0)
			goto exit;
	}

	if (num == 1) {
		/* size larger than endpoint max transfer size */
		if ((msgs[0].len + 5) > MAX_BUFFER_LENGTH) {
			retval = -EIO;
			goto exit;
		}

		if (msgs[0].flags & I2C_M_RD) {
			txbuf[0] = (msgs[0].addr << 1) | 0x01;
			if (ch34x_dev->chiptype == CHIP_CH341)
				retval = ch341_i2c_stream(ch34x_dev, 0x01, txbuf, msgs[0].len, msgs[0].buf);
			else
				retval = ch347_i2c_stream(ch34x_dev, 0x01, txbuf, msgs[0].len, msgs[0].buf);
			if (retval < 0)
				goto exit;
		} else {
			txbuf[0] = msgs[0].addr << 1;
			memcpy(&txbuf[1], msgs[0].buf, msgs[0].len);
			if (ch34x_dev->chiptype == CHIP_CH341)
				retval = ch341_i2c_stream(ch34x_dev, msgs[0].len + 1, txbuf, 0, NULL);
			else
				retval = ch347_i2c_stream(ch34x_dev, msgs[0].len + 1, txbuf, 0, NULL);
			if (retval < 0)
				goto exit;
		}
	} else if (num == 2) {
		if (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD)) {
			/* size larger than endpoint max transfer size */
			if (((msgs[0].len + 3) > MAX_BUFFER_LENGTH) || ((msgs[1].len + 5) > MAX_BUFFER_LENGTH)) {
				retval = -EIO;
				goto exit;
			}
			txbuf[0] = msgs[0].addr << 1;
			memcpy(&txbuf[1], msgs[0].buf, msgs[0].len);
			if (ch34x_dev->chiptype == CHIP_CH341)
				retval = ch341_i2c_stream(ch34x_dev, msgs[0].len + 1, txbuf, msgs[1].len, msgs[1].buf);
			else
				retval = ch347_i2c_stream(ch34x_dev, msgs[0].len + 1, txbuf, msgs[1].len, msgs[1].buf);
			if (retval < 0)
				goto exit;
		} else {
			retval = -EIO;
			goto exit;
		}
	} else {
		dev_err(&adapter->dev, "This case(num > 2) has not been support now\n");
		retval = -EIO;
		goto exit;
	}
	mutex_unlock(&ch34x_dev->io_mutex);
	kfree(txbuf);
	return num;

exit:
	mutex_unlock(&ch34x_dev->io_mutex);
	kfree(txbuf);
	return retval;
}

static u32 ch34x_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ch34x_i2c_algorithm = {
	.master_xfer = ch34x_i2c_xfer,
	.functionality = ch34x_i2c_func,
};

static int ch34x_mphsi_i2c_init(struct ch34x_device *ch34x_dev, u8 speed)
{
	int retval;

	if ((ch34x_dev->firmver < 0x0341) && (ch34x_dev->chiptype != CHIP_CH347F)) {
		if (speed > 3)
			return -EINVAL;
	}

	mutex_lock(&ch34x_dev->io_mutex);

	/* set ch34x i2c speed */
	ch34x_dev->bulkout_buf[0] = CH341_CMD_I2C_STREAM;
	ch34x_dev->bulkout_buf[1] = CH341_CMD_I2C_STM_SET | speed;
	ch34x_dev->bulkout_buf[2] = CH341_CMD_I2C_STM_END;
	retval = ch34x_usb_transfer(ch34x_dev, 3, 0);
	if (retval < 0)
		retval = -EIO;
	else
		retval = 0;

	mutex_unlock(&ch34x_dev->io_mutex);
	return retval;
}

static int ch34x_mphsi_i2c_setstretch(struct ch34x_device *ch34x_dev, bool enable)
{
	int retval;

	if ((ch34x_dev->firmver < 0x0341) && (ch34x_dev->chiptype != CHIP_CH347F))
		return 0;

	mutex_lock(&ch34x_dev->io_mutex);

	/* set ch34x i2c speed */
	ch34x_dev->bulkout_buf[0] = CH341_CMD_I2C_STREAM;
	if (enable)
		ch34x_dev->bulkout_buf[1] = CH347_CMD_I2C_STRETCH_Y;
	else
		ch34x_dev->bulkout_buf[1] = CH347_CMD_I2C_STRETCH_N;
	ch34x_dev->bulkout_buf[2] = CH341_CMD_I2C_STM_END;
	retval = ch34x_usb_transfer(ch34x_dev, 3, 0);
	if (retval < 0)
		retval = -EIO;
	else
		retval = 0;

	mutex_unlock(&ch34x_dev->io_mutex);
	return retval;
}

void ch34x_mphsi_i2c_remove(struct ch34x_device *ch34x_dev)
{
	CHECK_PARAM(ch34x_dev);

	if (ch34x_dev->i2c_init)
		i2c_del_adapter(&ch34x_dev->adapter);

	return;
}

int ch34x_mphsi_i2c_probe(struct ch34x_device *ch34x_dev)
{
	int retval;

	/* setup i2c adapter description */
	ch34x_dev->adapter.owner = THIS_MODULE;
	ch34x_dev->adapter.class = I2C_CLASS_HWMON;
	ch34x_dev->adapter.algo = &ch34x_i2c_algorithm;
	ch34x_dev->adapter.algo_data = ch34x_dev;
	ch34x_dev->adapter.dev.parent = &ch34x_dev->intf->dev;
	snprintf(ch34x_dev->adapter.name, sizeof(ch34x_dev->adapter.name), "ch34x-mphsi-i2c at bus %03d device %03d",
		 ch34x_dev->usb_dev->bus->busnum, ch34x_dev->usb_dev->devnum);

	/* and finally attach to i2c layer */
	retval = i2c_add_adapter(&ch34x_dev->adapter);
	if (retval < 0) {
		dev_err(&ch34x_dev->adapter.dev, "register i2c master failed\n");
		return -ENODEV;
	}

	if (ch34x_dev->chiptype == CHIP_CH347F) {
		retval = ch347_func_switch(ch34x_dev, 3);
		if (retval < 0) {
			dev_err(&ch34x_dev->adapter.dev, "ch347f switch i2c failed\n");
			goto error;
		}
	}

	/* set ch34x i2c speed */
	retval = ch34x_mphsi_i2c_init(ch34x_dev, I2C_SPEED_100K);
	if (retval < 0) {
		dev_err(&ch34x_dev->adapter.dev, "init i2c speed failed\n");
		goto error;
	}
	ch34x_dev->i2c_init = true;

	if ((ch34x_dev->firmver >= 0x0341) || (ch34x_dev->chiptype == CHIP_CH347F)) {
		retval = ch34x_mphsi_i2c_setstretch(ch34x_dev, false);
		if (retval < 0) {
			dev_err(&ch34x_dev->adapter.dev, "set i2c stretch failed\n");
			goto error;
		}
	}

	DEV_INFO(CH34X_USBDEV, "I2C master connected to I2C bus %d", ch34x_dev->adapter.nr);

	return 0;

error:
	ch34x_mphsi_i2c_remove(ch34x_dev);
	return retval;
}
