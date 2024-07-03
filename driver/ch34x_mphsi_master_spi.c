/*
 * ch347/ch341 MPHSI SPI driver layer
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

#define SPIDEV
#undef SPIDEV

#define SPI_DMA_XFER 0

#define ch34x_spi_maser_to_dev(m) *((struct ch34x_device **)spi_master_get_devdata(m))

static int param_bus_num = -1;
module_param_named(spi_bus_num, param_bus_num, int, 0600);
MODULE_PARM_DESC(spi_bus_num, "SPI master bus number (if negative, dynamic allocation)");

struct spi_board_info ch34x_spi_devices[CH341_SPI_MAX_NUM_DEVICES];

extern int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len, int in_len);
extern bool ch347_func_switch(struct ch34x_device *ch34x_dev, int index);

struct spi_board_info ch341_spi_device_template = {
	.modalias = "spidev",
	.max_speed_hz = CH341_SPI_MAX_FREQ,
	.bus_num = 0,
	.chip_select = 0,
	.mode = SPI_MODE_0,
};

struct spi_board_info ch347_spi_device_template = {
	.modalias = "spidev",
	.max_speed_hz = CH347_SPI_MAX_FREQ,
	.bus_num = 0,
	.chip_select = 0,
	.mode = SPI_MODE_0,
};

int ch34x_mphsi_spi_probe(struct ch34x_device *ch34x_dev)
{
	struct device *parent = &ch34x_dev->intf->dev;
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc("ch34x-mphsi-spi", 0);
	if (!pdev)
		return -ENOMEM;

	pdev->dev.parent = parent;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	pdev->dev.fwnode = NULL;
#endif
	pdev->id = ch34x_dev->id;

	ret = platform_device_add_data(pdev, NULL, 0);
	if (ret < 0)
		goto err;

	ret = platform_device_add(pdev);
	if (ret < 0)
		goto err;

	dev_dbg(&pdev->dev, "%s done\n", __func__);
	ch34x_dev->spi_pdev = pdev;
	return 0;

err:
	dev_err(parent, "%s: Can't create MPHSI SPI device %d\n", __func__, ret);
	platform_device_put(pdev);
	return ret;
}

int ch34x_mphsi_spi_remove(struct ch34x_device *ch34x_dev)
{
	platform_device_unregister(ch34x_dev->spi_pdev);
	return 0;
}

static u8 ch341_spi_swap_byte(const u8 byte)
{
	u8 orig = byte;
	u8 swap = 0;
	int i;

	for (i = 0; i < 8; ++i) {
		swap = swap << 1;
		swap |= (orig & 1);
		orig = orig >> 1;
	}
	return swap;
}

static void ch341_spi_update_io_data(struct ch34x_device *ch34x_dev)
{
	ch34x_dev->bulkout_buf[0] = CH341_CMD_UIO_STREAM;
	ch34x_dev->bulkout_buf[1] = CH341_CMD_UIO_STM_DIR | ch34x_dev->gpio_mask;
	ch34x_dev->bulkout_buf[2] = CH341_CMD_UIO_STM_OUT | (ch34x_dev->gpio_io_data & ch34x_dev->gpio_mask);
	ch34x_dev->bulkout_buf[3] = CH341_CMD_UIO_STM_END;

	ch34x_usb_transfer(ch34x_dev, 4, 0);
}

static void ch341_spi_set_cs(struct spi_device *spi, bool active)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(spi->master);

	if (spi->mode & SPI_NO_CS)
		return;

	if (spi->chip_select > CH341_SPI_MAX_NUM_DEVICES) {
		DEV_ERR(CH34X_USBDEV, "invalid CS value %d, 0~%d are available", spi->chip_select,
			CH341_SPI_MAX_NUM_DEVICES - 1);
	}

	if (active)
		ch34x_dev->gpio_io_data &= ~(1 << spi->chip_select);
	else
		ch34x_dev->gpio_io_data |= (1 << spi->chip_select);

	ch341_spi_update_io_data(ch34x_dev);
}

/**
 * ch347spi_get_hwcfg - get spi setting from hardware
 */
bool ch347spi_get_hwcfg(struct ch34x_device *ch34x_dev, stream_hw_cfgs *streamcfg)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;

	i = 0;
	io[i++] = USB20_CMD_INFO_RD;
	io[i++] = 1;
	io[i++] = 0;
	io[i++] = 1;
	len = i;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len)
		goto exit;

	len = 26 + 3;

	if (ch34x_usb_transfer(ch34x_dev, 0, len) != len)
		goto exit;

	if (streamcfg)
		memcpy(streamcfg, ch34x_dev->bulkin_buf + 3, len - 3);

	return true;

exit:
	return false;
}

/**
 * ch347spi_clockinit - system clock initialization
 */
bool ch347spi_clockinit(struct ch34x_device *ch34x_dev, u8 index)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;

	i = 0;
	io[i++] = USB20_CMD_SPI_CLK_INIT;
	io[i++] = 1;
	io[i++] = 0;
	io[i++] = index;
	len = i;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len)
		goto exit;

	if (ch34x_usb_transfer(ch34x_dev, 0, len) != len)
		goto exit;

	return true;

exit:
	return false;
}

void hwcfg_cpu_to_le(stream_hw_cfgs *hwcfg)
{
	hwcfg->spi_rw_interval = __cpu_to_le16(hwcfg->spi_rw_interval);
	hwcfg->spi_initcfg.spi_direction = __cpu_to_le16(hwcfg->spi_initcfg.spi_direction);
	hwcfg->spi_initcfg.spi_mode = __cpu_to_le16(hwcfg->spi_initcfg.spi_mode);
	hwcfg->spi_initcfg.spi_datasize = __cpu_to_le16(hwcfg->spi_initcfg.spi_datasize);
	hwcfg->spi_initcfg.s_spi_cpol = __cpu_to_le16(hwcfg->spi_initcfg.s_spi_cpol);
	hwcfg->spi_initcfg.s_spi_cpha = __cpu_to_le16(hwcfg->spi_initcfg.s_spi_cpha);
	hwcfg->spi_initcfg.spi_nss = __cpu_to_le16(hwcfg->spi_initcfg.spi_nss);
	hwcfg->spi_initcfg.spi_baudrate_scale = __cpu_to_le16(hwcfg->spi_initcfg.spi_baudrate_scale);
	hwcfg->spi_initcfg.spi_firstbit = __cpu_to_le16(hwcfg->spi_initcfg.spi_firstbit);
	hwcfg->spi_initcfg.spi_crc_poly = __cpu_to_le16(hwcfg->spi_initcfg.spi_crc_poly);
}

/**
 * spicfg_to_hwcfg - update spi setting to hardware
 * @spicfg: pointer to spi configuration
 * @hwcfg: pointer to hardware configuration
 *
 * The function return true if successful, false if fail.
 */
bool spicfg_to_hwcfg(mspi_cfgs *spicfg, stream_hw_cfgs *hwcfg)
{
	if (spicfg->imode > 3)
		return false;

	if (spicfg->iclock > 7)
		return false;

	spicfg->ibyteorder &= 0x01;
	hwcfg->spi_initcfg.spi_firstbit = spicfg->ibyteorder ? 0x00 : 0x80;

	switch (spicfg->imode) {
	case 0:
		hwcfg->spi_initcfg.s_spi_cpha = SPI_CPHA_1Edge;
		hwcfg->spi_initcfg.s_spi_cpol = SPI_CPOL_Low;
		break;
	case 1:
		hwcfg->spi_initcfg.s_spi_cpha = SPI_CPHA_2Edge;
		hwcfg->spi_initcfg.s_spi_cpol = SPI_CPOL_Low;
		break;
	case 2:
		hwcfg->spi_initcfg.s_spi_cpha = SPI_CPHA_1Edge;
		hwcfg->spi_initcfg.s_spi_cpol = SPI_CPOL_High;
		break;
	case 3:
		hwcfg->spi_initcfg.s_spi_cpha = SPI_CPHA_2Edge;
		hwcfg->spi_initcfg.s_spi_cpol = SPI_CPOL_High;
		break;
	default:
		hwcfg->spi_initcfg.s_spi_cpha = SPI_CPHA_2Edge;
		hwcfg->spi_initcfg.s_spi_cpol = SPI_CPOL_High;
		break;
	}

	hwcfg->spi_initcfg.spi_baudrate_scale = spicfg->iclock * 8;
	hwcfg->spi_outdef = spicfg->ispi_out_def;
	hwcfg->spi_rw_interval = spicfg->ispi_rw_interval;

	if (spicfg->cs0_polar)
		hwcfg->misc_cfg |= 0x80;
	else
		hwcfg->misc_cfg &= ~0x80;

	if (spicfg->cs1_polar)
		hwcfg->misc_cfg |= 0x40;
	else
		hwcfg->misc_cfg &= ~0x40;

	hwcfg_cpu_to_le(hwcfg);

	return true;
}

/**
 * ch347spi_init - SPI interface initialization
 */
bool ch347spi_init(struct ch34x_device *ch34x_dev, mspi_cfgs *spicfg)
{
	u8 *io = ch34x_dev->bulkout_buf;
	stream_hw_cfgs hwcfg = { { 0 }, 0 };
	int len, i;

	if (!ch347spi_get_hwcfg(ch34x_dev, &hwcfg))
		return false;

	if (!spicfg_to_hwcfg(spicfg, &hwcfg))
		return false;

	i = 0;
	io[i++] = USB20_CMD_SPI_INIT;
	len = sizeof(stream_hw_cfgs);
	io[i++] = (u8)(len & 0xFF);
	io[i++] = (u8)((len >> 8) & 0xFF);
	memcpy(&io[i], &hwcfg, len);
	len += i;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) == len) {
		len = USB20_CMD_HEADER + 1;
		if (ch34x_usb_transfer(ch34x_dev, 0, len) == len) {
			if (ch34x_dev->bulkin_buf[0] != USB20_CMD_SPI_INIT)
				return false;
			if (ch34x_dev->bulkin_buf[USB20_CMD_HEADER] == 0) {
				memcpy(&ch34x_dev->spicfg, spicfg, sizeof(mspi_cfgs));
				memcpy(&ch34x_dev->hwcfg, &hwcfg, sizeof(stream_hw_cfgs));
				return true;
			}
		}
	}

	return false;
}

/**
 * SPI chip selection initialization
 * @ch34x_dev: device pointer
 * @ienable_cs: low 8 bits: CS0, high 8 bits: CS1, byte value -> 1: set CS, 0: ignore CS setting
 * @ics: low 8 bits: CS0, high 8 bits: CS1, CS output, byte value -> 1: set CS, 0: cancel CS
 * @iauto_de_cs: low 16 bits: CS0, high 16 bits: CS1, automatically undo the CS after operation completed
 * @iactive_delay: low 16 bits: CS0, high 16 bits: CS1, delay time of read and write operation after setting CS, unit: us
 * @ideactive_delay: low 16 bits: CS0, high 16 bits: CS1,, delay time of read and write operation after canceling CS, unit: us
 *
 */
bool ch347spi_set_cs(struct ch34x_device *ch34x_dev, u16 ienable_cs, u16 ics, int iauto_de_cs, int iactive_delay,
		     int ideactive_delay)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;

	/* illegal parameter */
	if ((ienable_cs & 0x0101) == 0)
		return false;

	i = 0;
	io[i++] = USB20_CMD_SPI_CONTROL;
	i += 2; /* reserved for length */

	/* CS0 Setting */
	if ((ienable_cs & 0xFF))
		io[i] |= 0x80; /* CS0 enable */
	else
		io[i] &= 0x7F; /* CS0 disable */

	if ((ics & 0x01) == SET_CS) {
		io[i] &= 0xBF; /* set CS0 */
	} else
		io[i] |= 0x40; /* cancel CS0 */

	if ((iauto_de_cs & 0xFF))
		io[i] |= 0x20; /* undo CS0 after operation */
	else
		io[i] &= 0xDF; /* keep CS0 */
	i++;
	io[i++] = (u8)(iactive_delay & 0xFF);
	io[i++] = (u8)((iactive_delay >> 8) & 0xFF);
	io[i++] = (u8)(ideactive_delay & 0xFF);
	io[i++] = (u8)((ideactive_delay >> 8) & 0xFF);

	/* CS1 Setting */
	if ((ienable_cs & 0xFF00))
		io[i] |= 0x80; /* CS1 enable */
	else
		io[i] &= 0x7F; /* CS1 disable */

	if ((ics & 0x0100) == SET_CS) {
		io[i] &= 0xBF; /* set CS1 */
	} else
		io[i] |= 0x40; /* cancel CS1 */

	if ((iauto_de_cs & 0xFF00))
		io[i] |= 0x20; /* undo CS1 after operation */
	else
		io[i] &= 0xDF; /* keep CS1 */
	i++;
	io[i++] = (u8)((iactive_delay >> 16) & 0xFF);
	io[i++] = (u8)((iactive_delay >> 24) & 0xFF);
	io[i++] = (u8)((ideactive_delay >> 16) & 0xFF);
	io[i++] = (u8)((ideactive_delay >> 24) & 0xFF);

	io[1] = (u8)((i - 3) & 0xFF);
	io[2] = (u8)(((i - 3) >> 8) & 0xFF);

	len = i;
	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len)
		return false;

	return true;
}

/**
 * SPI CS setting, must call ch347spi_init first 
 */
bool ch347spi_change_cs(struct ch34x_device *ch34x_dev, u8 istatus)
{
	u16 ienable_cs;	 /* low 8 bits: CS0, high 8 bits: CS1, byte value -> 1: set CS, 0: ignore CS setting */
	u16 ics;	 /* low 8 bits: CS0, high 8 bits: CS1, CS output, byte value -> 1: set CS, 0: cancel CS */
	int iauto_de_cs; /* low 16 bits: CS0, high 16 bits: CS1, automatically undo the CS after operation completed */
	int iactive_delay; /* low 16 bits: CS0, high 16 bits: CS1, delay time of read and write operation after setting CS, unit: us */
	int ideactive_delay; /* low 16 bits: CS0, high 16 bits: CS1,, delay time of read and write operation after canceling CS, unit: us */

	iauto_de_cs = ch34x_dev->spicfg.iauto_de_cs;
	iactive_delay = ch34x_dev->spicfg.iactive_delay;
	ideactive_delay = ch34x_dev->spicfg.ideactive_delay;
	ics = istatus;

	/* CS1 selected */
	if (ch34x_dev->spicfg.ics & 0x8000) {
		ienable_cs = 0x0100;
		ics = (ics << 8) & 0xFF00;
		iauto_de_cs = (iauto_de_cs << 8) & 0xFF00;
		iactive_delay = (iactive_delay << 16) & 0xFFFF0000;
		ideactive_delay = (ideactive_delay << 16) & 0xFFFF0000;
	} else
		ienable_cs = 0x01;

	return ch347spi_set_cs(ch34x_dev, ienable_cs, ics, iauto_de_cs, iactive_delay, ideactive_delay);
}

static bool ch347_spi_set_cs(struct spi_device *spi, bool active)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(spi->master);

	if (spi->mode & SPI_NO_CS)
		return true;

	if (spi->chip_select > CH347_SPI_MAX_NUM_DEVICES) {
		DEV_ERR(CH34X_USBDEV, "invalid CS value %d, 0~%d are available", spi->chip_select,
			CH341_SPI_MAX_NUM_DEVICES - 1);
	}

	if (spi->chip_select == 0)
		ch34x_dev->spicfg.ics = 0x80;
	else
		ch34x_dev->spicfg.ics = 0x80 << 8;

	return ch347spi_change_cs(ch34x_dev, active ? 0x00 : 0x01);
}

/* Implementation of bit banging protocol uses following IOs to be compatible
 * with the hardware SPI interface
 *
 *   D7     D6     D5     D4     D3     D2     D1     D0
 *   MISO   IN2    MOSI   OUT2   SCK    CS1    CS0    CS0
 * 
 * CPOL=0, CPHA=0   data must be stable while clock is high, can be changed while clock is low
 * mode0            data sampled on raising clock edge
 *
 * CPOL=0, CPHA=1   data must be stable while clock is low, can be changed while clock is high
 * mode1            data sampled on falling clock edge
 *
 * CPOL=1, CPHA=0   data must be stable while clock is low, can be changed while clock is high
 * mode2            data sampled on falling clock edge
 *
 * CPOL=1, CPHA=1   data must be stable while clock is high, can be changed while clock is low
 * mode3            data sampled on raising clock edge
*/
static int ch341_spi_bitbang(struct ch34x_device *ch34x_dev, struct spi_device *spi, const u8 *tx, u8 *rx, int len)
{
	u8 byte, bit;
	u8 *io = ch34x_dev->bulkout_buf;
	int result = 0;
	int k = 0;
	int i, b;
	u8 SCK_H = SCK_BIT;
	u8 SCK_L = 0;
	u8 CPOL = (spi->mode & SPI_CPOL) ? SCK_BIT : 0;
	u8 CS_MASK = ch34x_dev->gpio_io_data & ((1 << ch34x_dev->slave_num) - 1);
	u8 DATA = ch34x_dev->gpio_io_data & ch34x_dev->gpio_mask;

	u8 mode = spi->mode & SPI_MODE_3;
	bool lsb = spi->mode & SPI_LSB_FIRST;

	DEV_DBG(CH34X_USBDEV, "start");

	/* mask SPI GPIO data */
	DATA &= ~MOSI_BIT & ~SCK_BIT & ~CS_MASK;

	for (b = 0; b < len; b++) {
		k = 0;
		io[k++] = CH341_CMD_UIO_STREAM;

		byte = lsb ? ch341_spi_swap_byte(tx[b]) : tx[b];
		for (i = 0; i < 8; i++) {
			bit = byte & 0x80 ? MOSI_BIT : 0; /* lsb first */
			byte <<= 1;

			if (mode == SPI_MODE_0 || mode == SPI_MODE_3) {
				/* set SCK=LOW, set MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_MASK | SCK_L | bit;
				/* set SCK=HIGH, keep MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_MASK | SCK_H | bit;
			} else {
				/* set SCK=HIGH, set MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_MASK | SCK_H | bit;
				/* set SCK=LOW, keep MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_MASK | SCK_L | bit;
			}
			/* read MISO */
			io[k++] = CH341_CMD_UIO_STM_IN;
		}
		/* SCK=CPOL, MOSI=LOW */
		io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_MASK | CPOL;
		io[k++] = CH341_CMD_UIO_STM_END;
		if ((result = ch34x_usb_transfer(ch34x_dev, k, 8)) < 0)
			return result;

		byte = 0;
		for (i = 0; i < 8; i++) {
			byte = byte << 1;
			byte = byte | ((ch34x_dev->bulkin_buf[i] & MISO_BIT) ? 1 : 0);
		}
		rx[b] = lsb ? ch341_spi_swap_byte(byte) : byte;
	}

	DEV_DBG(CH34X_USBDEV, "done");

	return 0;
}

static int ch341_spi_native(struct ch34x_device *ch34x_dev, struct spi_device *spi, const u8 *tx, u8 *rx, int len)
{
	bool lsb = spi->mode & SPI_LSB_FIRST;
	int bytes_to_copy;
	int result = 0;
	int i;

	while (len) {
		bytes_to_copy = min(len, CH341_USB_MAX_BULK_SIZE - 1);

		/* fill output buffer with command and output data, master expects lsb first */
		ch34x_dev->bulkout_buf[0] = CH341_CMD_SPI_STREAM;
		if (lsb) {
			memcpy(ch34x_dev->bulkout_buf + 1, tx, bytes_to_copy);
		} else {
			for (i = 0; i < bytes_to_copy; i++)
				ch34x_dev->bulkout_buf[i + 1] = ch341_spi_swap_byte(tx[i]);
		}
		tx += bytes_to_copy;

		result = ch34x_usb_transfer(ch34x_dev, bytes_to_copy + 1, bytes_to_copy);
		if (result < 0)
			break;

		if (result != bytes_to_copy) {
			result = -EIO;
			break;
		}

		if (rx) {
			/* fill input data with input buffer, master delivers lsb first */
			if (lsb) {
				memcpy(rx, ch34x_dev->bulkin_buf, bytes_to_copy);
			} else {
				for (i = 0; i < bytes_to_copy; i++)
					rx[i] = ch341_spi_swap_byte(ch34x_dev->bulkin_buf[i]);
			}
			rx += bytes_to_copy;
		}

		len -= bytes_to_copy;
		result = 0;
	}
	return result;
}

static int ch347_spi_native(struct ch34x_device *ch34x_dev, struct spi_device *spi, const u8 *tx, u8 *rx, int len)
{
	int bytes_to_copy;
	int result = 0;

	if (!tx && !rx) {
		dev_err(&spi->dev, "No buffer for transfer\n");
		return -EINVAL;
	}

	/* spi output only */
	if (tx && rx) {
		while (len) {
			bytes_to_copy = min(len, CH347_USB_MAX_BULK_SIZE - USB20_CMD_HEADER);

			/* fill output buffer with command and output data */
			ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_RD_WR;
			ch34x_dev->bulkout_buf[1] = (u8)bytes_to_copy;
			ch34x_dev->bulkout_buf[2] = (u8)(bytes_to_copy >> 8);
			memcpy(ch34x_dev->bulkout_buf + USB20_CMD_HEADER, tx, bytes_to_copy);
			tx += bytes_to_copy;

			result = ch34x_usb_transfer(ch34x_dev, bytes_to_copy + USB20_CMD_HEADER, 0);
			if (result != (bytes_to_copy + USB20_CMD_HEADER)) {
				result = -EIO;
				goto exit;
			}

			result = ch34x_usb_transfer(ch34x_dev, 0, bytes_to_copy + USB20_CMD_HEADER);
			if ((result != (bytes_to_copy + USB20_CMD_HEADER)) ||
			    (ch34x_dev->bulkin_buf[0] != USB20_CMD_SPI_RD_WR)) {
				result = -EIO;
				goto exit;
			}
			/* fill input data with input buffer */
			memcpy(rx, ch34x_dev->bulkin_buf + USB20_CMD_HEADER, bytes_to_copy);
			rx += bytes_to_copy;
			len -= bytes_to_copy;
			result = 0;
		}
	} else if (tx) {
		while (len) {
			bytes_to_copy = min(len, CH347_USB_MAX_BULK_SIZE - USB20_CMD_HEADER);

			/* fill output buffer with command and output data */
			ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_BLCK_WR;
			ch34x_dev->bulkout_buf[1] = (u8)bytes_to_copy;
			ch34x_dev->bulkout_buf[2] = (u8)(bytes_to_copy >> 8);
			memcpy(ch34x_dev->bulkout_buf + USB20_CMD_HEADER, tx, bytes_to_copy);
			tx += bytes_to_copy;

			result = ch34x_usb_transfer(ch34x_dev, bytes_to_copy + USB20_CMD_HEADER, 0);
			if (result != (bytes_to_copy + USB20_CMD_HEADER)) {
				result = -EIO;
				goto exit;
			}

			result = ch34x_usb_transfer(ch34x_dev, 0, USB20_CMD_HEADER + 1);
			if ((result != (USB20_CMD_HEADER + 1)) || (ch34x_dev->bulkin_buf[USB20_CMD_HEADER] != 0x00)) {
				result = -EIO;
				goto exit;
			}

			len -= bytes_to_copy;
			result = 0;
		}
	} else if (rx) {
		bytes_to_copy = 0x04;

		/* fill output buffer with command and output data */
		ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_BLCK_RD;
		ch34x_dev->bulkout_buf[1] = 0x04;
		ch34x_dev->bulkout_buf[2] = 0x00;
		ch34x_dev->bulkout_buf[3] = (u8)len;
		ch34x_dev->bulkout_buf[4] = (u8)(len >> 8);
		ch34x_dev->bulkout_buf[5] = (u8)(len >> 16);
		ch34x_dev->bulkout_buf[6] = (u8)(len >> 24);

		result = ch34x_usb_transfer(ch34x_dev, bytes_to_copy + USB20_CMD_HEADER, 0);
		if (result != (bytes_to_copy + USB20_CMD_HEADER)) {
			result = -EIO;
			goto exit;
		}

		while (len) {
			bytes_to_copy = min(len, CH347_USB_MAX_BULK_SIZE - USB20_CMD_HEADER);

			result = ch34x_usb_transfer(ch34x_dev, 0, bytes_to_copy + USB20_CMD_HEADER);
			if ((result != (bytes_to_copy + USB20_CMD_HEADER)) ||
			    (ch34x_dev->bulkin_buf[0] != USB20_CMD_SPI_BLCK_RD)) {
				result = -EIO;
				goto exit;
			}
			/* fill input data with input buffer */
			memcpy(rx, ch34x_dev->bulkin_buf + USB20_CMD_HEADER, bytes_to_copy);
			rx += bytes_to_copy;

			len -= bytes_to_copy;
			result = 0;
		}
	}

exit:
	return result;
}

static int ch34x_spi_transfer_one(struct ch34x_device *ch34x_dev, struct spi_device *spi, struct spi_transfer *t)
{
	int result;

	CHECK_PARAM_RET(ch34x_dev, EIO);
	CHECK_PARAM_RET(spi, EIO)
	CHECK_PARAM_RET(t, EIO);

	if (ch34x_dev->chiptype == CHIP_CH341) {
		if (spi->mode & SPI_MODE_3) {
			/* use slow bitbang implementation for SPI_MODE_1, SPI_MODE_2 and SPI_MODE_3 */
			result = ch341_spi_bitbang(ch34x_dev, spi, t->tx_buf, t->rx_buf, t->len);
		} else {
			/* otherwise the faster hardware implementation */
			result = ch341_spi_native(ch34x_dev, spi, t->tx_buf, t->rx_buf, t->len);
		}
	} else {
		result = ch347_spi_native(ch34x_dev, spi, t->tx_buf, t->rx_buf, t->len);
	}

	return result;
}

static int ch341_spi_transfer_one_message(struct spi_master *ctlr, struct spi_message *msg)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(ctlr);
	struct spi_transfer *xfer;
	bool keep_cs = false;
	bool cpol = msg->spi->mode & SPI_CPOL;
	int ret = 0;

	mutex_lock(&ch34x_dev->io_mutex);

	if (ch34x_dev->last_cpol != cpol) {
		ch34x_dev->last_cpol = cpol;
		ch34x_dev->gpio_io_data = (ch34x_dev->gpio_io_data & ~SCK_BIT) | (cpol ? SCK_BIT : 0);
		ch341_spi_update_io_data(ch34x_dev);
	}

	ch341_spi_set_cs(msg->spi, true);

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if ((xfer->tx_buf || xfer->rx_buf) && xfer->len) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
			reinit_completion(&ctlr->xfer_completion);
#endif
			ret = ch34x_spi_transfer_one(ch34x_dev, msg->spi, xfer);
			if (ret < 0) {
				dev_err(&msg->spi->dev, "SPI transfer failed: %d\n", ret);
				goto out;
			}
		} else {
			if (xfer->len)
				dev_err(&msg->spi->dev, "Bufferless transfer has length %u\n", xfer->len);
		}

		if (xfer->cs_change) {
			if (list_is_last(&xfer->transfer_list, &msg->transfers)) {
				keep_cs = true;
			} else {
				ch341_spi_set_cs(msg->spi, false);
				udelay(10);
				ch341_spi_set_cs(msg->spi, true);
			}
		}

		msg->actual_length += xfer->len;
	}

out:
	if (ret != 0 || !keep_cs)
		ch341_spi_set_cs(msg->spi, false);

	mutex_unlock(&ch34x_dev->io_mutex);

	if (msg->status == -EINPROGRESS)
		msg->status = ret;

	spi_finalize_current_message(ctlr);

	return ret;
}

static int ch347_spi_build_packet(struct ch34x_device *ch34x_dev, struct spi_device *spi, struct spi_transfer *t)
{
	const u8 *tx = t->tx_buf;
	int len = t->len;

	/* spi output only */
	if (tx) {
		/* fill output buffer with command and output data */
		memcpy(ch34x_dev->bulkout_buf + ch34x_dev->windex, tx, len);
		ch34x_dev->windex += len;
	} else {
		dev_err(&spi->dev, "error transfer has null tx\n");
		return -EIO;
	}

	return len;
}

static int ch347_spi_transfer_one_message(struct spi_master *ctlr, struct spi_message *msg)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(ctlr);
	struct spi_transfer *xfer;
	bool keep_cs = false;
	int ret = 0;

	int bytes_to_xfer = 0;
	int bytes_to_copy;
	u8 *rbuf = NULL;

	mutex_lock(&ch34x_dev->io_mutex);

	if (((ch34x_dev->firmver >= 0x0341) || (ch34x_dev->chiptype == CHIP_CH347F)) && SPI_DMA_XFER) {
		rbuf = kmalloc(MAX_BUFFER_LENGTH * 2, GFP_KERNEL);
		if (!rbuf) {
			return -ENOMEM;
		}

		ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_RD_WR;
		bytes_to_xfer += USB20_CMD_HEADER;
		ch34x_dev->windex = USB20_CMD_HEADER;

		list_for_each_entry(xfer, &msg->transfers, transfer_list) {
			if ((xfer->tx_buf || xfer->rx_buf) && xfer->len) {
				if (xfer->len > (MAX_BUFFER_LENGTH - USB20_CMD_HEADER)) {
					dev_err(&msg->spi->dev, "%s xfer->len: %d too long.\n", __func__, xfer->len);
					ret = -EPROTO;
					goto out;
				}
				bytes_to_copy = ch347_spi_build_packet(ch34x_dev, msg->spi, xfer);
				if (bytes_to_copy < 0) {
					ret = -EIO;
					goto out;
				}
				bytes_to_xfer += bytes_to_copy;
			} else {
				if (xfer->len)
					dev_err(&msg->spi->dev, "Bufferless transfer has length %u\n", xfer->len);
			}
			if (msg->status != -EINPROGRESS) {
				goto out;
			}
		}
		if (msg->spi->chip_select == 0) {
			ch34x_dev->bulkout_buf[1] = (u8)(bytes_to_xfer - USB20_CMD_HEADER);
			ch34x_dev->bulkout_buf[2] = (u8)((bytes_to_xfer - USB20_CMD_HEADER) >> 8) | BIT(7) | BIT(6);
		} else {
			ch34x_dev->bulkout_buf[1] = (u8)(bytes_to_xfer - USB20_CMD_HEADER);
			ch34x_dev->bulkout_buf[2] = (u8)((bytes_to_xfer - USB20_CMD_HEADER) >> 8) | BIT(5) | BIT(4);
		}

		/* usb transfer */
		ret = ch34x_usb_transfer(ch34x_dev, bytes_to_xfer, 0);
		if (ret != bytes_to_xfer) {
			ret = -EIO;
			goto out;
		}

		ret = ch34x_usb_transfer(ch34x_dev, 0, bytes_to_xfer);
		if (ret != bytes_to_xfer) {
			ret = -EIO;
			goto out;
		}

		/* fill input data with input buffer */
		if (ch34x_dev->bulkin_buf[0] != USB20_CMD_SPI_RD_WR) {
			ret = -EIO;
			goto out;
		}
		/* fill input data with input buffer */
		memcpy(rbuf, ch34x_dev->bulkin_buf + USB20_CMD_HEADER, bytes_to_xfer - USB20_CMD_HEADER);

		list_for_each_entry(xfer, &msg->transfers, transfer_list) {
			if ((xfer->tx_buf || xfer->rx_buf) && xfer->len) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
				reinit_completion(&ctlr->xfer_completion);
#endif
				memcpy(xfer->rx_buf, rbuf + msg->actual_length, xfer->len);
			} else {
				if (xfer->len)
					dev_err(&msg->spi->dev, "Bufferless transfer has length %u\n", xfer->len);
			}

			if (msg->status != -EINPROGRESS) {
				goto out;
			}

			msg->actual_length += xfer->len;
		}

		ret = 0;
	} else {
		ch347_spi_set_cs(msg->spi, true);

		list_for_each_entry(xfer, &msg->transfers, transfer_list) {
			if ((xfer->tx_buf || xfer->rx_buf) && xfer->len) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
				reinit_completion(&ctlr->xfer_completion);
#endif
				ret = ch34x_spi_transfer_one(ch34x_dev, msg->spi, xfer);
				if (ret < 0) {
					dev_err(&msg->spi->dev, "SPI transfer failed: %d\n", ret);
					goto out1;
				}
			} else {
				if (xfer->len)
					dev_err(&msg->spi->dev, "Bufferless transfer has length %u\n", xfer->len);
			}

			if (msg->status != -EINPROGRESS)
				goto out1;

			if (xfer->cs_change) {
				if (list_is_last(&xfer->transfer_list, &msg->transfers)) {
					keep_cs = true;
				} else {
					ch347_spi_set_cs(msg->spi, false);
					udelay(10);
					ch347_spi_set_cs(msg->spi, true);
				}
			}
			msg->actual_length += xfer->len;
		}
out1:
		if (ret != 0 || !keep_cs)
			ch347_spi_set_cs(msg->spi, false);
	}

out:
	mutex_unlock(&ch34x_dev->io_mutex);

	if (msg->status == -EINPROGRESS)
		msg->status = ret;

	spi_finalize_current_message(ctlr);

	if (rbuf)
		kfree(rbuf);

	return ret;
}

static int ch341_spi_setup(struct spi_device *spi)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(spi->master);
	u8 cs_mask = (1 << spi->chip_select);

	mutex_lock(&ch34x_dev->io_mutex);

	if (!(ch34x_dev->gpio_io_data & cs_mask)) {
		ch34x_dev->gpio_io_data |= cs_mask;
		ch341_spi_update_io_data(ch34x_dev);
	}

	mutex_unlock(&ch34x_dev->io_mutex);

	return 0;
}

static int ch347_spi_setup(struct spi_device *spi)
{
	struct spi_master *master = spi->master;
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(spi->master);
	mspi_cfgs spicfg = { 0 };
	u8 scale;
	int ret;
	int i, j;
	u8 clock_index;
	int clk_table0[] = { 60e6,   48e6,   36e6,  30e6,  28e6,   24e6,    18e6,   15e6,  14e6,    12e6,   9e6,
			     75e5,   7e6,    6e6,   45e5,  375e4,  35e5,    3e6,    225e4, 1875e3,  175e4,  15e5,
			     1125e3, 9375e2, 875e3, 750e3, 5625e2, 46875e1, 4375e2, 375e3, 28125e1, 21875e1 };
	int clk_table1[] = {
		28e6,  14e6,  7e6,   35e5,   175e4,  875e3, 4375e2, 21875e1, 72e6,   36e6,    18e6,
		9e6,   45e5,  225e4, 1125e3, 5625e2, 48e6,  24e6,   12e6,    6e6,    3e6,     15e5,
		750e3, 375e3, 60e6,  30e6,   15e6,   75e5,  375e4,  1875e3,  9375e2, 46875e1,
	};
	int clk_table2[] = { 60e6, 30e6, 15e6, 75e5, 375e4, 1875e3, 9375e2, 46875e1 };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	if (spi->max_speed_hz < master->min_speed_hz || spi->max_speed_hz > master->max_speed_hz)
		return -EINVAL;
#else
	if (spi->max_speed_hz < CH347_SPI_MIN_FREQ || spi->max_speed_hz > CH347_SPI_MAX_FREQ)
		return -EINVAL;
#endif

	if (spi->chip_select == 1) {
		if (ch34x_dev->chiptype == CHIP_CH347F) {
			if (!ch347_func_switch(ch34x_dev, 0)) {
				DEV_ERR(CH34X_USBDEV, "Failed to init SPI1 CS1 of CH347F.");
				return -EPROTO;
			}
		}
	}
	mutex_lock(&ch34x_dev->io_mutex);
	if (spi->chip_select == 0)
		spicfg.ics = 0x80;
	else if (spi->chip_select == 1)
		spicfg.ics = 0x80 << 8;
	else {
		ret = -EINVAL;
		goto exit;
	}

	switch (spi->mode) {
	case SPI_MODE_0:
		spicfg.imode = 0;
		break;
	case SPI_MODE_1:
		spicfg.imode = 1;
		break;
	case SPI_MODE_2:
		spicfg.imode = 2;
		break;
	case SPI_MODE_3:
		spicfg.imode = 3;
		break;
	default:
		break;
	}

	if ((ch34x_dev->firmver >= 0x0341) || (ch34x_dev->chiptype == CHIP_CH347F)) {
		for (i = 0; i < sizeof(clk_table0) / sizeof(int); i++) {
			if (clk_table0[i] <= spi->max_speed_hz) {
				for (j = 0; j < sizeof(clk_table1) / sizeof(int); j++) {
					if (clk_table0[i] == clk_table1[j])
						break;
				}
				clock_index = j / 8 + 1;
				scale = clk_table1[j] / clk_table1[clock_index * 8 - 1];
				if (clock_index == 2)
					clock_index = 5;
				spicfg.iclock = 7;
				while (scale / 2) {
					spicfg.iclock--;
					scale /= 2;
				}
				break;
			}
		}
	} else {
		for (i = 0; i < sizeof(clk_table2) / sizeof(int); i++) {
			if (clk_table2[i] <= spi->max_speed_hz) {
				scale = clk_table2[i] / clk_table2[7];
				spicfg.iclock = 7;
				while (scale / 2) {
					spicfg.iclock--;
					scale /= 2;
				}
				break;
			}
		}
	}

	DEV_DBG(CH34X_USBDEV, "spimode:%d, max_speed_hz: %d, scale: %d, iclock: %d\n", spicfg.imode, spi->max_speed_hz,
		scale, spicfg.iclock);

	if (spi->mode & SPI_LSB_FIRST)
		spicfg.ibyteorder = 0;
	else
		spicfg.ibyteorder = 1;

	/* BIT0：CS0/1 polar control, 0：low active, 1：high active */
	if (spi->mode & SPI_CS_HIGH) {
		if (spi->chip_select == 0)
			spicfg.cs0_polar = 1;
		else
			spicfg.cs1_polar = 1;
	} else {
		if (spi->chip_select == 0)
			spicfg.cs0_polar = 0;
		else
			spicfg.cs1_polar = 0;
	}

	/* spi output [0xFF] by default */
	spicfg.ispi_out_def = 0xFF;
	/* SPI read and write interval, unit: us */
	spicfg.ispi_rw_interval = 0;
	/* automatically undo the CS after operation completed */
	spicfg.iauto_de_cs = 0;
	/* delay time of read and write operation after setting CS, unit: us */
	spicfg.iactive_delay = 0;
	/* delay time of read and write operation after canceling CS, unit: us */
	spicfg.ideactive_delay = 0;

	if ((ch34x_dev->firmver >= 0x0341) || (ch34x_dev->chiptype == CHIP_CH347F)) {
		/* init spi interface */
		if (ch347spi_clockinit(ch34x_dev, clock_index) == false) {
			DEV_ERR(CH34X_USBDEV, "Failed to init SPI clock.");
			ret = -EPROTO;
			goto exit;
		} else
			ret = 0;
	}
	/* init spi interface */
	if (ch347spi_init(ch34x_dev, &spicfg) == false) {
		DEV_ERR(CH34X_USBDEV, "Failed to init SPI interface.");
		ret = -1;
	} else
		ret = 0;

exit:
	mutex_unlock(&ch34x_dev->io_mutex);
	return ret;
}

int ch34x_spi_probe(struct ch34x_device *ch34x_dev)
{
	int result;
#ifdef SPIDEV
	int i;
#endif

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);

	DEV_DBG(CH34X_USBDEV, "start");

	/* allocate a new SPI master with a pointer to ch34x_device as device data */
	ch34x_dev->master = spi_alloc_master(CH34X_USBDEV, sizeof(struct ch34x_device *));
	if (!ch34x_dev->master) {
		DEV_ERR(CH34X_USBDEV, "SPI master allocation failed");
		return -ENOMEM;
	}

	platform_set_drvdata(ch34x_dev->spi_pdev, ch34x_dev->master);

	/* save the pointer to ch34x_dev in the SPI master device data field */
	ch34x_spi_maser_to_dev(ch34x_dev->master) = ch34x_dev;

	/* set SPI master configuration */
	ch34x_dev->master->bus_num = (param_bus_num >= 0) ? param_bus_num : -1;
	if (ch34x_dev->chiptype == CHIP_CH341)
		ch34x_dev->master->num_chipselect = CH341_SPI_MAX_NUM_DEVICES;
	else
		ch34x_dev->master->num_chipselect = CH347_SPI_MAX_NUM_DEVICES;
	ch34x_dev->master->mode_bits = SPI_MODE_3 | SPI_LSB_FIRST | SPI_CS_HIGH;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
	ch34x_dev->master->flags = SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
	ch34x_dev->master->bits_per_word_mask = SPI_BPW_MASK(8);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	ch34x_dev->master->bits_per_word_mask = SPI_BIT_MASK(8);
#endif

	if (ch34x_dev->chiptype == CHIP_CH341) {
		ch34x_dev->master->transfer_one_message = ch341_spi_transfer_one_message;
		ch34x_dev->master->setup = ch341_spi_setup;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
		ch34x_dev->master->max_speed_hz = CH341_SPI_MAX_FREQ;
		ch34x_dev->master->min_speed_hz = CH341_SPI_MIN_FREQ;
#endif
	} else {
		ch34x_dev->master->transfer_one_message = ch347_spi_transfer_one_message;
		ch34x_dev->master->setup = ch347_spi_setup;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
		ch34x_dev->master->max_speed_hz = CH347_SPI_MAX_FREQ;
		ch34x_dev->master->min_speed_hz = CH347_SPI_MIN_FREQ;
#endif
	}

	/* register spi master */
	if ((result = spi_register_master(ch34x_dev->master))) {
		DEV_ERR(CH34X_USBDEV, "could not register SPI master");
		spi_master_put(ch34x_dev->master);
		ch34x_dev->master = 0;
		return result;
	}

	DEV_INFO(CH34X_USBDEV, "SPI master connected to SPI bus %d", ch34x_dev->master->bus_num);

	if (ch34x_dev->chiptype == CHIP_CH341) {
		mutex_lock(&ch34x_dev->io_mutex);
		ch34x_dev->gpio_io_data |= (1 << ch34x_dev->slave_num) - 1;
		ch341_spi_update_io_data(ch34x_dev);
		mutex_unlock(&ch34x_dev->io_mutex);
	}

#ifdef SPIDEV
	/* create SPI slaves */
	for (i = 0; i < ch34x_dev->slave_num; i++) {
		ch34x_spi_devices[i].bus_num = ch34x_dev->master->bus_num;
		if ((ch34x_dev->slaves[i] = spi_new_device(ch34x_dev->master, &ch34x_spi_devices[i]))) {
			DEV_INFO(CH34X_USBDEV, "SPI device /dev/spidev%d.%d created", ch34x_dev->master->bus_num,
				 ch34x_spi_devices[i].chip_select);
		}
	}
#endif

	DEV_DBG(CH34X_USBDEV, "done");

	return 0;
}

void ch34x_spi_remove(struct ch34x_device *ch34x_dev)
{
#ifdef SPIDEV
	int i;
#endif

	CHECK_PARAM(ch34x_dev);

#ifdef SPIDEV
	for (i = 0; i < ch34x_dev->slave_num; i++)
		if (ch34x_dev->slaves[i])
			spi_unregister_device(ch34x_dev->slaves[i]);
#endif

	if (ch34x_dev->master)
		spi_unregister_master(ch34x_dev->master);

	return;
}
