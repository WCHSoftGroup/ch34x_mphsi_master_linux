/*
 * CH347/CH341 MPHSI SPI driver layer
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
 */

#include "ch34x_mphsi.h"

#define SPI_AUTOPROBE
#undef SPI_AUTOPROBE

/*
 * SPI_DMA_XFER: SPI high-efficiency transmission switch, but with a 4k packet length limit and xfer->cs_change no longer works when enabled.
 * SPI_DMA_XFER_NOMERGE: Used with SPI_DMA_XFER = 1, it addresses the 4K transmission limitations,
 * but multiple SPIs will not be automatically merged for transmission.
 * 
 * Quick Speed Mode: SPI_DMA_XFER = 1 & SPI_DMA_XFER_NOMERGE = 0, each SPI transmission of all xfers must not exceed 4k in length.
 * Medium Speed Mode: SPI_DMA_XFER = 1 & SPI_DMA_XFER_NOMERGE = 1, an xfer of each SPI transmission must not exceed 4k in length.
 * Normal Speed Mode: SPI_DMA_XFER = 0, an xfer of each SPI transmission must not exceed 4k in length,
 * flexible chip selection control and good compatibility.
 */

#define SPI_DMA_XFER 1
#define SPI_DMA_XFER_NOMERGE 1

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
#define ch34x_spi_controller_to_dev(m) \
	*((struct ch34x_device **)spi_controller_get_devdata(m))
#else
#define ch34x_spi_maser_to_dev(m) \
	*((struct ch34x_device **)spi_master_get_devdata(m))
#endif

static int param_bus_num = -1;
module_param_named(spi_bus_num, param_bus_num, int, 0600);
MODULE_PARM_DESC(
	spi_bus_num,
	"SPI controller bus number (if negative, dynamic allocation)");

struct spi_board_info ch34x_spi_devices[CH341_SPI_MAX_NUM_DEVICES];

extern int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len,
			      int in_len);
extern bool ch347_func_switch(struct ch34x_device *ch34x_dev, int index);
extern int ch34x_mphsi_spi_probe(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_spi_remove(struct ch34x_device *ch34x_dev);
static bool ch347spi_get_hwcfg(struct ch34x_device *ch34x_dev,
			       stream_hw_cfgs *streamcfg);
static bool ch347spi_clockinit(struct ch34x_device *ch34x_dev, u8 index);
static void hwcfg_cpu_to_le(stream_hw_cfgs *hwcfg);
static bool spicfg_to_hwcfg(mspi_cfgs *spicfg, stream_hw_cfgs *hwcfg);
static bool ch347spi_init(struct ch34x_device *ch34x_dev,
			  mspi_cfgs *spicfg);
static bool ch347spi_change_cs(struct ch34x_device *ch34x_dev, u8 istatus);
extern int ch34x_spi_probe(struct ch34x_device *ch34x_dev);
extern void ch34x_spi_remove(struct ch34x_device *ch34x_dev);
static bool ch347spi_set_cs(struct ch34x_device *ch34x_dev, u16 ienable_cs,
			    u16 ics, int iauto_de_cs, int iactive_delay,
			    int ideactive_delay);

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
	dev_err(parent, "%s: Can't create MPHSI SPI device %d\n", __func__,
		ret);
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
	ch34x_dev->bulkout_buf[1] = CH341_CMD_UIO_STM_DIR |
				    ch34x_dev->gpio_mask;
	ch34x_dev->bulkout_buf[2] =
		CH341_CMD_UIO_STM_OUT |
		(ch34x_dev->gpio_io_data & ch34x_dev->gpio_mask);
	ch34x_dev->bulkout_buf[3] = CH341_CMD_UIO_STM_END;

	ch34x_usb_transfer(ch34x_dev, 4, 0);
}

static void ch341_spi_set_cs(struct spi_device *spi, bool active)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	struct ch34x_device *ch34x_dev =
		ch34x_spi_controller_to_dev(spi->controller);
#else
	struct ch34x_device *ch34x_dev =
		ch34x_spi_maser_to_dev(spi->master);
#endif

	if (spi->mode & SPI_NO_CS)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (spi_get_chipselect(spi, 0) > CH341_SPI_MAX_NUM_DEVICES) {
		DEV_ERR(CH34X_USBDEV,
			"Invalid CS value %d, 0~%d are available",
			spi_get_chipselect(spi, 0),
			CH341_SPI_MAX_NUM_DEVICES - 1);
	}
	if (active)
		ch34x_dev->gpio_io_data &=
			~(1 << spi_get_chipselect(spi, 0));
	else
		ch34x_dev->gpio_io_data |=
			(1 << spi_get_chipselect(spi, 0));
#else
	if (spi->chip_select > CH341_SPI_MAX_NUM_DEVICES) {
		DEV_ERR(CH34X_USBDEV,
			"Invalid CS value %d, 0~%d are available",
			spi->chip_select, CH341_SPI_MAX_NUM_DEVICES - 1);
	}
	if (active)
		ch34x_dev->gpio_io_data &= ~(1 << spi->chip_select);
	else
		ch34x_dev->gpio_io_data |= (1 << spi->chip_select);
#endif

	ch341_spi_update_io_data(ch34x_dev);
}

bool ch347spi_get_hwcfg(struct ch34x_device *ch34x_dev,
			stream_hw_cfgs *streamcfg)
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
	hwcfg->spi_initcfg.spi_direction =
		__cpu_to_le16(hwcfg->spi_initcfg.spi_direction);
	hwcfg->spi_initcfg.spi_mode =
		__cpu_to_le16(hwcfg->spi_initcfg.spi_mode);
	hwcfg->spi_initcfg.spi_datasize =
		__cpu_to_le16(hwcfg->spi_initcfg.spi_datasize);
	hwcfg->spi_initcfg.s_spi_cpol =
		__cpu_to_le16(hwcfg->spi_initcfg.s_spi_cpol);
	hwcfg->spi_initcfg.s_spi_cpha =
		__cpu_to_le16(hwcfg->spi_initcfg.s_spi_cpha);
	hwcfg->spi_initcfg.spi_nss =
		__cpu_to_le16(hwcfg->spi_initcfg.spi_nss);
	hwcfg->spi_initcfg.spi_baudrate_scale =
		__cpu_to_le16(hwcfg->spi_initcfg.spi_baudrate_scale);
	hwcfg->spi_initcfg.spi_firstbit =
		__cpu_to_le16(hwcfg->spi_initcfg.spi_firstbit);
	hwcfg->spi_initcfg.spi_crc_poly =
		__cpu_to_le16(hwcfg->spi_initcfg.spi_crc_poly);
}

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
				memcpy(&ch34x_dev->spicfg, spicfg,
				       sizeof(mspi_cfgs));
				memcpy(&ch34x_dev->hwcfg, &hwcfg,
				       sizeof(stream_hw_cfgs));
				return true;
			}
		}
	}

	return false;
}

bool ch347spi_set_cs(struct ch34x_device *ch34x_dev, u16 ienable_cs,
		     u16 ics, int iauto_de_cs, int iactive_delay,
		     int ideactive_delay)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;

	/* Illegal parameter */
	if ((ienable_cs & 0x0101) == 0)
		return false;

	i = 0;
	io[i++] = USB20_CMD_SPI_CONTROL;
	i += 2; /* Reserved for length */

	/* CS0 Setting */
	if ((ienable_cs & 0xFF))
		io[i] |= 0x80; /* CS0 enable */
	else
		io[i] &= 0x7F; /* CS0 disable */

	if ((ics & 0x01) == SET_CS) {
		io[i] &= 0xBF; /* Set CS0 */
	} else
		io[i] |= 0x40; /* Cancel CS0 */

	if ((iauto_de_cs & 0xFF))
		io[i] |= 0x20; /* Undo CS0 after operation */
	else
		io[i] &= 0xDF; /* Keep CS0 */
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
		io[i] &= 0xBF; /* Set CS1 */
	} else
		io[i] |= 0x40; /* Cancel CS1 */

	if ((iauto_de_cs & 0xFF00))
		io[i] |= 0x20; /* Undo CS1 after operation */
	else
		io[i] &= 0xDF; /* Keep CS1 */
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

bool ch347spi_change_cs(struct ch34x_device *ch34x_dev, u8 istatus)
{
	u16 ienable_cs;
	u16 ics;
	int iauto_de_cs;
	int iactive_delay;
	int ideactive_delay;

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

	return ch347spi_set_cs(ch34x_dev, ienable_cs, ics, iauto_de_cs,
			       iactive_delay, ideactive_delay);
}

static bool ch347_spi_set_cs(struct spi_device *spi, bool active)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	struct ch34x_device *ch34x_dev =
		ch34x_spi_controller_to_dev(spi->controller);
#else
	struct ch34x_device *ch34x_dev =
		ch34x_spi_maser_to_dev(spi->master);
#endif

	if (spi->mode & SPI_NO_CS)
		return true;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (spi_get_chipselect(spi, 0) > CH347_SPI_MAX_NUM_DEVICES) {
		DEV_ERR(CH34X_USBDEV,
			"Invalid CS value %d, 0~%d are available",
			spi_get_chipselect(spi, 0),
			CH341_SPI_MAX_NUM_DEVICES - 1);
	}

	if (spi_get_chipselect(spi, 0) == 0)
		ch34x_dev->spicfg.ics = 0x80;
	else
		ch34x_dev->spicfg.ics = 0x80 << 8;
#else
	if (spi->chip_select > CH347_SPI_MAX_NUM_DEVICES) {
		DEV_ERR(CH34X_USBDEV,
			"Invalid CS value %d, 0~%d are available",
			spi->chip_select, CH341_SPI_MAX_NUM_DEVICES - 1);
	}

	if (spi->chip_select == 0)
		ch34x_dev->spicfg.ics = 0x80;
	else
		ch34x_dev->spicfg.ics = 0x80 << 8;
#endif

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
static int ch341_spi_bitbang(struct ch34x_device *ch34x_dev,
			     struct spi_device *spi, const u8 *tx, u8 *rx,
			     int len)
{
	u8 byte, bit;
	u8 *io = ch34x_dev->bulkout_buf;
	int result = 0;
	int k = 0;
	int i, b;
	u8 SCK_H = SCK_BIT;
	u8 SCK_L = 0;
	u8 CPOL = (spi->mode & SPI_CPOL) ? SCK_BIT : 0;
	u8 CS_MASK = ch34x_dev->gpio_io_data &
		     ((1 << ch34x_dev->slave_num) - 1);
	u8 DATA = ch34x_dev->gpio_io_data & ch34x_dev->gpio_mask;

	u8 mode = spi->mode & SPI_MODE_3;
	bool lsb = spi->mode & SPI_LSB_FIRST;

	DEV_DBG(CH34X_USBDEV, "start");

	/* Mask SPI GPIO data */
	DATA &= ~MOSI_BIT & ~SCK_BIT & ~CS_MASK;

	for (b = 0; b < len; b++) {
		k = 0;
		io[k++] = CH341_CMD_UIO_STREAM;

		byte = lsb ? ch341_spi_swap_byte(tx[b]) : tx[b];
		for (i = 0; i < 8; i++) {
			bit = byte & 0x80 ? MOSI_BIT : 0; /* LSB first */
			byte <<= 1;

			if (mode == SPI_MODE_0 || mode == SPI_MODE_3) {
				/* Set SCK=LOW, set MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA |
					  CS_MASK | SCK_L | bit;
				/* Set SCK=HIGH, keep MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA |
					  CS_MASK | SCK_H | bit;
			} else {
				/* Set SCK=HIGH, set MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA |
					  CS_MASK | SCK_H | bit;
				/* Set SCK=LOW, keep MOSI */
				io[k++] = CH341_CMD_UIO_STM_OUT | DATA |
					  CS_MASK | SCK_L | bit;
			}
			/* Read MISO */
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
			byte = byte |
			       ((ch34x_dev->bulkin_buf[i] & MISO_BIT) ? 1 :
									0);
		}
		rx[b] = lsb ? ch341_spi_swap_byte(byte) : byte;
	}

	DEV_DBG(CH34X_USBDEV, "done");

	return 0;
}

static int ch341_spi_native(struct ch34x_device *ch34x_dev,
			    struct spi_device *spi, const u8 *tx, u8 *rx,
			    int len)
{
	bool lsb = spi->mode & SPI_LSB_FIRST;
	int bytes_to_copy;
	int result = 0;
	int i;

	while (len) {
		bytes_to_copy = min(len, CH341_USB_MAX_BULK_SIZE - 1);

		/* Fill output buffer with command and output data, controller expects lsb first */
		ch34x_dev->bulkout_buf[0] = CH341_CMD_SPI_STREAM;
		if (lsb) {
			memcpy(ch34x_dev->bulkout_buf + 1, tx,
			       bytes_to_copy);
		} else {
			for (i = 0; i < bytes_to_copy; i++)
				ch34x_dev->bulkout_buf[i + 1] =
					ch341_spi_swap_byte(tx[i]);
		}
		tx += bytes_to_copy;

		result = ch34x_usb_transfer(ch34x_dev, bytes_to_copy + 1,
					    bytes_to_copy);
		if (result < 0)
			break;

		if (result != bytes_to_copy) {
			result = -EIO;
			break;
		}

		if (rx) {
			/* Fill input data with input buffer, controller delivers lsb first */
			if (lsb) {
				memcpy(rx, ch34x_dev->bulkin_buf,
				       bytes_to_copy);
			} else {
				for (i = 0; i < bytes_to_copy; i++)
					rx[i] = ch341_spi_swap_byte(
						ch34x_dev->bulkin_buf[i]);
			}
			rx += bytes_to_copy;
		}

		len -= bytes_to_copy;
		result = 0;
	}
	return result;
}

static int ch347_spi_native(struct ch34x_device *ch34x_dev,
			    struct spi_device *spi, const u8 *tx, u8 *rx,
			    int len)
{
	int bytes_to_copy;
	int result = 0;

	if (!tx && !rx) {
		dev_err(&spi->dev, "No buffer for transfer\n");
		return -EINVAL;
	}

	/* SPI output only */
	if (tx && rx) {
		while (len) {
			bytes_to_copy = min(len, CH347_USB_MAX_BULK_SIZE -
							 USB20_CMD_HEADER);

			/* Fill output buffer with command and output data */
			ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_RD_WR;
			ch34x_dev->bulkout_buf[1] = (u8)bytes_to_copy;
			ch34x_dev->bulkout_buf[2] =
				(u8)(bytes_to_copy >> 8);
			memcpy(ch34x_dev->bulkout_buf + USB20_CMD_HEADER,
			       tx, bytes_to_copy);
			tx += bytes_to_copy;

			result = ch34x_usb_transfer(
				ch34x_dev,
				bytes_to_copy + USB20_CMD_HEADER, 0);
			if (result != (bytes_to_copy + USB20_CMD_HEADER)) {
				result = -EIO;
				goto exit;
			}

			result = ch34x_usb_transfer(
				ch34x_dev, 0,
				bytes_to_copy + USB20_CMD_HEADER);
			if ((result !=
			     (bytes_to_copy + USB20_CMD_HEADER)) ||
			    (ch34x_dev->bulkin_buf[0] !=
			     USB20_CMD_SPI_RD_WR)) {
				result = -EIO;
				goto exit;
			}
			/* Fill input data with input buffer */
			memcpy(rx,
			       ch34x_dev->bulkin_buf + USB20_CMD_HEADER,
			       bytes_to_copy);
			rx += bytes_to_copy;
			len -= bytes_to_copy;
			result = 0;
		}
	} else if (tx) {
		while (len) {
			bytes_to_copy = min(len, CH347_USB_MAX_BULK_SIZE -
							 USB20_CMD_HEADER);

			/* Fill output buffer with command and output data */
			ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_BLCK_WR;
			ch34x_dev->bulkout_buf[1] = (u8)bytes_to_copy;
			ch34x_dev->bulkout_buf[2] =
				(u8)(bytes_to_copy >> 8);
			memcpy(ch34x_dev->bulkout_buf + USB20_CMD_HEADER,
			       tx, bytes_to_copy);
			tx += bytes_to_copy;

			result = ch34x_usb_transfer(
				ch34x_dev,
				bytes_to_copy + USB20_CMD_HEADER, 0);
			if (result != (bytes_to_copy + USB20_CMD_HEADER)) {
				result = -EIO;
				goto exit;
			}

			result = ch34x_usb_transfer(ch34x_dev, 0,
						    USB20_CMD_HEADER + 1);
			if ((result != (USB20_CMD_HEADER + 1)) ||
			    (ch34x_dev->bulkin_buf[USB20_CMD_HEADER] !=
			     0x00)) {
				result = -EIO;
				goto exit;
			}

			len -= bytes_to_copy;
			result = 0;
		}
	} else if (rx) {
		bytes_to_copy = 0x04;

		/* Fill output buffer with command and output data */
		ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_BLCK_RD;
		ch34x_dev->bulkout_buf[1] = 0x04;
		ch34x_dev->bulkout_buf[2] = 0x00;
		ch34x_dev->bulkout_buf[3] = (u8)len;
		ch34x_dev->bulkout_buf[4] = (u8)(len >> 8);
		ch34x_dev->bulkout_buf[5] = (u8)(len >> 16);
		ch34x_dev->bulkout_buf[6] = (u8)(len >> 24);

		result = ch34x_usb_transfer(
			ch34x_dev, bytes_to_copy + USB20_CMD_HEADER, 0);
		if (result != (bytes_to_copy + USB20_CMD_HEADER)) {
			result = -EIO;
			goto exit;
		}

		while (len) {
			bytes_to_copy = min(len, CH347_USB_MAX_BULK_SIZE -
							 USB20_CMD_HEADER);

			result = ch34x_usb_transfer(
				ch34x_dev, 0,
				bytes_to_copy + USB20_CMD_HEADER);
			if ((result !=
			     (bytes_to_copy + USB20_CMD_HEADER)) ||
			    (ch34x_dev->bulkin_buf[0] !=
			     USB20_CMD_SPI_BLCK_RD)) {
				result = -EIO;
				goto exit;
			}
			/* Fill input data with input buffer */
			memcpy(rx,
			       ch34x_dev->bulkin_buf + USB20_CMD_HEADER,
			       bytes_to_copy);
			rx += bytes_to_copy;

			len -= bytes_to_copy;
			result = 0;
		}
	}

exit:
	return result;
}

static int ch347_spi_native_dma(struct ch34x_device *ch34x_dev,
				struct spi_device *spi, bool cs_start,
				bool cs_stop, const u8 *tx, u8 *rx,
				int len)
{
	int bytes_to_copy;
	int result = 0;

	if (!tx) {
		dev_err(&spi->dev, "No buffer for transfer\n");
		return -EINVAL;
	}

	if (!tx && !rx) {
		dev_err(&spi->dev, "No buffer for transfer\n");
		return -EINVAL;
	}

	while (len) {
		bytes_to_copy =
			min(len, MAX_BUFFER_LENGTH - USB20_CMD_HEADER);

		/* Fill output buffer with command and output data */
		ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_RD_WR;
		ch34x_dev->bulkout_buf[1] = (u8)bytes_to_copy;
		ch34x_dev->bulkout_buf[2] = (u8)(bytes_to_copy >> 8);
		memcpy(ch34x_dev->bulkout_buf + USB20_CMD_HEADER, tx,
		       bytes_to_copy);
		tx += bytes_to_copy;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
		if (spi_get_chipselect(spi, 0) == 0) {
#else
		if (spi->chip_select == 0) {
#endif
			if (cs_start)
				ch34x_dev->bulkout_buf[2] |= BIT(7);
			if (cs_stop)
				ch34x_dev->bulkout_buf[2] |= BIT(6);
		} else {
			if (cs_start)
				ch34x_dev->bulkout_buf[2] |= BIT(5);
			if (cs_stop)
				ch34x_dev->bulkout_buf[2] |= BIT(4);
		}

		result = ch34x_usb_transfer(
			ch34x_dev, bytes_to_copy + USB20_CMD_HEADER, 0);
		if (result != (bytes_to_copy + USB20_CMD_HEADER)) {
			result = -EIO;
			goto exit;
		}

		result = ch34x_usb_transfer(
			ch34x_dev, 0, bytes_to_copy + USB20_CMD_HEADER);
		if ((result != (bytes_to_copy + USB20_CMD_HEADER)) ||
		    (ch34x_dev->bulkin_buf[0] != USB20_CMD_SPI_RD_WR)) {
			result = -EIO;
			goto exit;
		}
		if (rx) {
			/* Fill input data with input buffer */
			memcpy(rx,
			       ch34x_dev->bulkin_buf + USB20_CMD_HEADER,
			       bytes_to_copy);
			rx += bytes_to_copy;
		}
		len -= bytes_to_copy;
		result = 0;
	}

	result = 0;

exit:
	return result;
}

static int ch34x_spi_transfer_one(struct ch34x_device *ch34x_dev,
				  struct spi_device *spi,
				  struct spi_transfer *t)
{
	int result;

	CHECK_PARAM_RET(ch34x_dev, EIO);
	CHECK_PARAM_RET(spi, EIO)
	CHECK_PARAM_RET(t, EIO);

	if (ch34x_dev->chiptype == CHIP_CH341) {
		if (spi->mode & SPI_MODE_3) {
			/* Use slow bitbang implementation for SPI_MODE_1, SPI_MODE_2 and SPI_MODE_3 */
			result = ch341_spi_bitbang(ch34x_dev, spi,
						   t->tx_buf, t->rx_buf,
						   t->len);
		} else {
			/* Otherwise the faster hardware implementation */
			result = ch341_spi_native(ch34x_dev, spi,
						  t->tx_buf, t->rx_buf,
						  t->len);
		}
	} else {
		result = ch347_spi_native(ch34x_dev, spi, t->tx_buf,
					  t->rx_buf, t->len);
	}

	return result;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
static int ch341_spi_transfer_one_message(struct spi_controller *ctrl,
					  struct spi_message *msg)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_controller_to_dev(ctrl);
#else
static int ch341_spi_transfer_one_message(struct spi_master *ctrl,
					  struct spi_message *msg)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(ctrl);
#endif
	struct spi_transfer *xfer;
	bool keep_cs = false;
	bool cpol = msg->spi->mode & SPI_CPOL;
	int ret = 0;

	mutex_lock(&ch34x_dev->io_mutex);

	if (ch34x_dev->last_cpol != cpol) {
		ch34x_dev->last_cpol = cpol;
		ch34x_dev->gpio_io_data =
			(ch34x_dev->gpio_io_data & ~SCK_BIT) |
			(cpol ? SCK_BIT : 0);
		ch341_spi_update_io_data(ch34x_dev);
	}

	ch341_spi_set_cs(msg->spi, true);

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if ((xfer->tx_buf || xfer->rx_buf) && xfer->len) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
			reinit_completion(&ctrl->xfer_completion);
#endif
			ret = ch34x_spi_transfer_one(ch34x_dev, msg->spi,
						     xfer);
			if (ret < 0) {
				dev_err(&msg->spi->dev,
					"SPI transfer failed: %d\n", ret);
				goto out;
			}
		} else {
			if (xfer->len)
				dev_err(&msg->spi->dev,
					"Bufferless transfer has length %u\n",
					xfer->len);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
		spi_transfer_delay_exec(xfer);
#else
		if (xfer->delay_usecs)
			udelay(xfer->delay_usecs);
#endif

		if (xfer->cs_change) {
			if (list_is_last(&xfer->transfer_list,
					 &msg->transfers)) {
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

	spi_finalize_current_message(ctrl);

	return ret;
}

static int ch347_spi_build_packet(struct ch34x_device *ch34x_dev,
				  struct spi_device *spi,
				  struct spi_transfer *t)
{
	const u8 *tx = t->tx_buf;
	int len = t->len;

	/* SPI output only */
	if (tx) {
		/* Fill output buffer with command and output data */
		memcpy(ch34x_dev->bulkout_buf + ch34x_dev->windex, tx,
		       len);
		ch34x_dev->windex += len;
	} else {
		dev_err(&spi->dev, "Error transfer has null tx\n");
		return -EIO;
	}

	return len;
}

static int ch347_spi_transfer_setup(struct ch34x_device *ch34x_dev,
				    struct spi_transfer *first_xfer)
{
	u8 clock_index;
	int i, j, ret = 0;
	u8 scale;
	int clk_table0[] = { 60e6,    48e6,   36e6,    30e6,   28e6,
			     24e6,    18e6,   15e6,    14e6,   12e6,
			     9e6,     75e5,   7e6,     6e6,    45e5,
			     375e4,   35e5,   3e6,     225e4,  1875e3,
			     175e4,   15e5,   1125e3,  9375e2, 875e3,
			     750e3,   5625e2, 46875e1, 4375e2, 375e3,
			     28125e1, 21875e1 };
	int clk_table1[] = {
		28e6, 14e6, 7e6,  35e5, 175e4, 875e3,  4375e2, 21875e1,
		72e6, 36e6, 18e6, 9e6,	45e5,  225e4,  1125e3, 5625e2,
		48e6, 24e6, 12e6, 6e6,	3e6,   15e5,   750e3,  375e3,
		60e6, 30e6, 15e6, 75e5, 375e4, 1875e3, 9375e2, 46875e1,
	};

	if ((!first_xfer->tx_buf && !first_xfer->rx_buf) ||
	    (first_xfer->len <= 0) ||
	    (first_xfer->len > (MAX_BUFFER_LENGTH - USB20_CMD_HEADER))) {
		dev_err(&ch34x_dev->master->dev,
			"%s Invalid xfer. len:%d\n", __func__,
			first_xfer->len);
		ret = -EPROTO;
		goto out;
	}

	if ((first_xfer->speed_hz > 0) &&
	    (first_xfer->speed_hz != ch34x_dev->cur_speed_hz)) {
		ch34x_dev->cur_speed_hz = first_xfer->speed_hz;

		for (i = 0; i < sizeof(clk_table0) / sizeof(int); i++) {
			if (clk_table0[i] <= ch34x_dev->cur_speed_hz) {
				for (j = 0;
				     j < sizeof(clk_table1) / sizeof(int);
				     j++) {
					if (clk_table0[i] == clk_table1[j])
						break;
				}
				clock_index = j / 8 + 1;
				scale = clk_table1[j] /
					clk_table1[clock_index * 8 - 1];
				if (clock_index == 2)
					clock_index = 5;
				ch34x_dev->spicfg.iclock = 7;
				while (scale / 2) {
					ch34x_dev->spicfg.iclock--;
					scale /= 2;
				}
				break;
			}
		}

		if ((ch34x_dev->firmver >= 0x0341) ||
		    (ch34x_dev->chiptype == CHIP_CH347F)) {
			/* Init SPI interface */
			if (ch347spi_clockinit(ch34x_dev, clock_index) ==
			    false) {
				DEV_ERR(CH34X_USBDEV,
					"Failed to init SPI clock.");
				ret = -EPROTO;
				goto out;
			} else
				ret = 0;
		}

		/* Init SPI interface */
		if (ch347spi_init(ch34x_dev, &ch34x_dev->spicfg) ==
		    false) {
			DEV_ERR(CH34X_USBDEV,
				"Failed to init SPI interface.");
			ret = -1;
		} else
			ret = 0;
	}
out:
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
static int ch347_spi_transfer_one_message(struct spi_controller *ctrl,
					  struct spi_message *msg)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_controller_to_dev(ctrl);
#else
static int ch347_spi_transfer_one_message(struct spi_master *ctrl,
					  struct spi_message *msg)
{
	struct ch34x_device *ch34x_dev = ch34x_spi_maser_to_dev(ctrl);
#endif
	struct spi_transfer *xfer;
	struct spi_transfer *first_xfer;
	bool keep_cs = false;
	int ret = 0;

	int bytes_to_xfer = 0;
	int bytes_to_copy;
	u8 *rbuf = NULL;

	bool cs_start = true;
	bool cs_stop = false;

	mutex_lock(&ch34x_dev->io_mutex);

	if (((ch34x_dev->firmver >= 0x0341) ||
	     (ch34x_dev->chiptype == CHIP_CH347F) ||
	     (ch34x_dev->chiptype == CHIP_CH339W)) &&
	    SPI_DMA_XFER) {
		rbuf = kmalloc(MAX_BUFFER_LENGTH * 2, GFP_KERNEL);
		if (!rbuf) {
			return -ENOMEM;
		}

		if (SPI_DMA_XFER_NOMERGE == 0) {
			ch34x_dev->bulkout_buf[0] = USB20_CMD_SPI_RD_WR;
			bytes_to_xfer += USB20_CMD_HEADER;
			ch34x_dev->windex = USB20_CMD_HEADER;

			list_for_each_entry(xfer, &msg->transfers,
					    transfer_list) {
				if ((xfer->tx_buf || xfer->rx_buf) &&
				    xfer->len) {
					if (xfer->len >
					    (MAX_BUFFER_LENGTH -
					     USB20_CMD_HEADER)) {
						dev_err(&msg->spi->dev,
							"%s xfer->len: %d too long.\n",
							__func__,
							xfer->len);
						ret = -EPROTO;
						goto out;
					}
					bytes_to_copy =
						ch347_spi_build_packet(
							ch34x_dev,
							msg->spi, xfer);
					if (bytes_to_copy < 0) {
						ret = -EIO;
						goto out;
					}
					bytes_to_xfer += bytes_to_copy;
				} else {
					if (xfer->len)
						dev_err(&msg->spi->dev,
							"Bufferless transfer has length %u\n",
							xfer->len);
				}
				if (msg->status != -EINPROGRESS) {
					goto out;
				}
			}

			if (bytes_to_copy >
			    (MAX_BUFFER_LENGTH - USB20_CMD_HEADER)) {
				dev_err(&msg->spi->dev,
					"%s xfer total length: %d too long.\n",
					__func__, bytes_to_copy);
				ret = -EPROTO;
				goto out;
			}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
			if (spi_get_chipselect(msg->spi, 0) == 0) {
#else
			if (msg->spi->chip_select == 0) {

#endif
				ch34x_dev->bulkout_buf[1] =
					(u8)(bytes_to_xfer -
					     USB20_CMD_HEADER);
				ch34x_dev->bulkout_buf[2] =
					(u8)((bytes_to_xfer -
					      USB20_CMD_HEADER) >>
					     8) |
					BIT(7) | BIT(6);
			} else {
				ch34x_dev->bulkout_buf[1] =
					(u8)(bytes_to_xfer -
					     USB20_CMD_HEADER);
				ch34x_dev->bulkout_buf[2] =
					(u8)((bytes_to_xfer -
					      USB20_CMD_HEADER) >>
					     8) |
					BIT(5) | BIT(4);
			}

			ret = ch34x_usb_transfer(ch34x_dev, bytes_to_xfer,
						 0);
			if (ret != bytes_to_xfer) {
				ret = -EIO;
				goto out;
			}

			ret = ch34x_usb_transfer(ch34x_dev, 0,
						 bytes_to_xfer);
			if (ret != bytes_to_xfer) {
				ret = -EIO;
				goto out;
			}

			/* Fill input data with input buffer */
			if (ch34x_dev->bulkin_buf[0] !=
			    USB20_CMD_SPI_RD_WR) {
				ret = -EIO;
				goto out;
			}
			/* Fill input data with input buffer */
			memcpy(rbuf,
			       ch34x_dev->bulkin_buf + USB20_CMD_HEADER,
			       bytes_to_xfer - USB20_CMD_HEADER);

			list_for_each_entry(xfer, &msg->transfers,
					    transfer_list) {
				if ((xfer->tx_buf || xfer->rx_buf) &&
				    xfer->len) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
					reinit_completion(
						&ctrl->xfer_completion);
#endif
					memcpy(xfer->rx_buf,
					       rbuf + msg->actual_length,
					       xfer->len);
				} else {
					if (xfer->len)
						dev_err(&msg->spi->dev,
							"Bufferless transfer has length %u\n",
							xfer->len);
				}

				if (msg->status != -EINPROGRESS) {
					goto out;
				}

				msg->actual_length += xfer->len;

				ret = 0;
			}
		} else {
			first_xfer = list_first_entry(&msg->transfers,
						      struct spi_transfer,
						      transfer_list);
			ret = ch347_spi_transfer_setup(ch34x_dev,
						       first_xfer);
			if (ret < 0)
				goto out;

			list_for_each_entry(xfer, &msg->transfers,
					    transfer_list) {
				if ((xfer->tx_buf || xfer->rx_buf) &&
				    xfer->len) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
					reinit_completion(
						&ctrl->xfer_completion);
#endif
					if (list_is_last(
						    &xfer->transfer_list,
						    &msg->transfers)) {
						if (xfer->cs_change)
							cs_stop =
								false; /* Keep cs active when this xfer ends */
						else
							cs_stop =
								true; /* Deactive cs when this xfer ends */
					} else {
						if (xfer->cs_change)
							cs_stop =
								true; /* Deactive cs when this xfer ends */
						else
							cs_stop =
								false; /* Keep cs active when this xfer ends */
					}

					ret = ch347_spi_native_dma(
						ch34x_dev, msg->spi,
						cs_start, cs_stop,
						xfer->tx_buf, xfer->rx_buf,
						xfer->len);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
					spi_transfer_delay_exec(xfer);
#else
					if (xfer->delay_usecs)
						udelay(xfer->delay_usecs);
#endif

					if (list_is_last(
						    &xfer->transfer_list,
						    &msg->transfers)) {
					} else {
						if (xfer->cs_change)
							cs_start =
								true; /* Active cs when next xfer starts */
						else
							cs_start =
								false; /* Keep cs when next xfer starts */
					}

					if (ret < 0) {
						dev_err(&msg->spi->dev,
							"SPI transfer failed: %d\n",
							ret);
						goto out;
					}
				} else {
					if (xfer->len)
						dev_err(&msg->spi->dev,
							"Bufferless transfer has length %u\n",
							xfer->len);
				}

				if (msg->status != -EINPROGRESS)
					goto out;
				msg->actual_length += xfer->len;
			}
		}
	} else {
		first_xfer = list_first_entry(&msg->transfers,
					      struct spi_transfer,
					      transfer_list);
		ch347_spi_transfer_setup(ch34x_dev, first_xfer);
		if (ret < 0)
			goto out;

		ch347_spi_set_cs(msg->spi, true);

		list_for_each_entry(xfer, &msg->transfers, transfer_list) {
			if ((xfer->tx_buf || xfer->rx_buf) && xfer->len) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
				reinit_completion(&ctrl->xfer_completion);
#endif
				ret = ch34x_spi_transfer_one(
					ch34x_dev, msg->spi, xfer);
				if (ret < 0) {
					dev_err(&msg->spi->dev,
						"SPI transfer failed: %d\n",
						ret);
					goto out1;
				}
			} else {
				if (xfer->len)
					dev_err(&msg->spi->dev,
						"Bufferless transfer has length %u\n",
						xfer->len);
			}

			if (msg->status != -EINPROGRESS)
				goto out1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
			spi_transfer_delay_exec(xfer);
#else
			if (xfer->delay_usecs)
				udelay(xfer->delay_usecs);
#endif

			if (xfer->cs_change) {
				if (list_is_last(&xfer->transfer_list,
						 &msg->transfers)) {
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

	spi_finalize_current_message(ctrl);

	if (rbuf)
		kfree(rbuf);

	return ret;
}

static int ch341_spi_setup(struct spi_device *spi)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	struct ch34x_device *ch34x_dev =
		ch34x_spi_controller_to_dev(spi->controller);
#else
	struct ch34x_device *ch34x_dev =
		ch34x_spi_maser_to_dev(spi->master);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	u8 cs_mask = (1 << spi_get_chipselect(spi, 0));
#else
	u8 cs_mask = (1 << spi->chip_select);
#endif

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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	struct spi_controller *controller = spi->controller;
	struct ch34x_device *ch34x_dev =
		ch34x_spi_controller_to_dev(spi->controller);
#else
	struct spi_master *master = spi->master;
	struct ch34x_device *ch34x_dev =
		ch34x_spi_maser_to_dev(spi->master);
#endif

	mspi_cfgs spicfg = { 0 };
	u8 scale = 1;
	int ret;
	int i, j;
	u8 clock_index = 0;
	int clk_table0[] = { 60e6,    48e6,   36e6,    30e6,   28e6,
			     24e6,    18e6,   15e6,    14e6,   12e6,
			     9e6,     75e5,   7e6,     6e6,    45e5,
			     375e4,   35e5,   3e6,     225e4,  1875e3,
			     175e4,   15e5,   1125e3,  9375e2, 875e3,
			     750e3,   5625e2, 46875e1, 4375e2, 375e3,
			     28125e1, 21875e1 };
	int clk_table1[] = {
		28e6, 14e6, 7e6,  35e5, 175e4, 875e3,  4375e2, 21875e1,
		72e6, 36e6, 18e6, 9e6,	45e5,  225e4,  1125e3, 5625e2,
		48e6, 24e6, 12e6, 6e6,	3e6,   15e5,   750e3,  375e3,
		60e6, 30e6, 15e6, 75e5, 375e4, 1875e3, 9375e2, 46875e1,
	};
	int clk_table2[] = { 60e6,  30e6,   15e6,   75e5,
			     375e4, 1875e3, 9375e2, 46875e1 };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (spi->max_speed_hz < controller->min_speed_hz ||
	    spi->max_speed_hz > controller->max_speed_hz)
		return -EINVAL;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	if (spi->max_speed_hz < master->min_speed_hz ||
	    spi->max_speed_hz > master->max_speed_hz)
		return -EINVAL;
#else
	if (spi->max_speed_hz < CH347_SPI_MIN_FREQ ||
	    spi->max_speed_hz > CH347_SPI_MAX_FREQ)
		return -EINVAL;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (spi_get_chipselect(spi, 0) == 1) {
#else
	if (spi->chip_select == 1) {
#endif
		if (ch34x_dev->chiptype == CHIP_CH347F) {
			if (!ch347_func_switch(ch34x_dev, 0)) {
				DEV_ERR(CH34X_USBDEV,
					"Failed to init SPI1 CS1 of CH347F.");
				return -EPROTO;
			}
		}
	}
	mutex_lock(&ch34x_dev->io_mutex);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (spi_get_chipselect(spi, 0) == 0)
		spicfg.ics = 0x80;
	else if (spi_get_chipselect(spi, 0) == 1)
		spicfg.ics = 0x80 << 8;
	else {
		ret = -EINVAL;
		goto exit;
	}
#else
	if (spi->chip_select == 0)
		spicfg.ics = 0x80;
	else if (spi->chip_select == 1)
		spicfg.ics = 0x80 << 8;
	else {
		ret = -EINVAL;
		goto exit;
	}
#endif

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

	ch34x_dev->cur_speed_hz = spi->max_speed_hz;
	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F)) {
		for (i = 0; i < sizeof(clk_table0) / sizeof(int); i++) {
			if (clk_table0[i] <= ch34x_dev->cur_speed_hz) {
				for (j = 0;
				     j < sizeof(clk_table1) / sizeof(int);
				     j++) {
					if (clk_table0[i] == clk_table1[j])
						break;
				}
				clock_index = j / 8 + 1;
				scale = clk_table1[j] /
					clk_table1[clock_index * 8 - 1];
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
			if (clk_table2[i] <= ch34x_dev->cur_speed_hz) {
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

	DEV_DBG(CH34X_USBDEV,
		"SPI configuration, mode:%d, max_speed_hz: %d, scale: %d, iclock: %d",
		spicfg.imode, spi->max_speed_hz, scale, spicfg.iclock);

	if (spi->mode & SPI_LSB_FIRST)
		spicfg.ibyteorder = 0;
	else
		spicfg.ibyteorder = 1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (spi->mode & SPI_CS_HIGH) {
		if (spi_get_chipselect(spi, 0) == 0)
			spicfg.cs0_polar = 1;
		else
			spicfg.cs1_polar = 1;
	} else {
		if (spi_get_chipselect(spi, 0) == 0)
			spicfg.cs0_polar = 0;
		else
			spicfg.cs1_polar = 0;
	}
#else
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
#endif

	/* SPI output [0xFF] by default */
	spicfg.ispi_out_def = 0xFF;
	/* SPI read and write interval, unit: us */
	spicfg.ispi_rw_interval = 0;
	/* Automatically undo the CS after operation completed */
	spicfg.iauto_de_cs = 0;
	/* Delay time of read and write operation after setting CS, unit: us */
	spicfg.iactive_delay = 0;
	/* Delay time of read and write operation after canceling CS, unit: us */
	spicfg.ideactive_delay = 0;

	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F)) {
		/* Init SPI interface */
		if (ch347spi_clockinit(ch34x_dev, clock_index) == false) {
			DEV_ERR(CH34X_USBDEV, "Failed to init SPI clock.");
			ret = -EPROTO;
			goto exit;
		} else
			ret = 0;
	}
	/* Init SPI interface */
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
#ifdef SPI_AUTOPROBE
	int gpio_index;
	struct spi_board_info ch34x_spi_slaves[3] = { 0 };
#endif

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);

	DEV_DBG(CH34X_USBDEV, "start");

	/* Allocate a new SPI controller with a pointer to ch34x_device as device data */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	ch34x_dev->master = spi_alloc_host(CH34X_USBDEV,
					   sizeof(struct ch34x_device *));
#else
	ch34x_dev->master = spi_alloc_master(
		CH34X_USBDEV, sizeof(struct ch34x_device *));
#endif
	if (!ch34x_dev->master) {
		DEV_ERR(CH34X_USBDEV, "SPI controller allocation failed");
		return -ENOMEM;
	}

	platform_set_drvdata(ch34x_dev->spi_pdev, ch34x_dev->master);

	/* Save the pointer to ch34x_dev in the SPI controller device data field */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	ch34x_spi_controller_to_dev(ch34x_dev->master) = ch34x_dev;
#else
	ch34x_spi_maser_to_dev(ch34x_dev->master) = ch34x_dev;
#endif

	/* Set SPI controller configuration */
	ch34x_dev->master->bus_num = (param_bus_num >= 0) ? param_bus_num :
							    -1;
	if (ch34x_dev->chiptype == CHIP_CH341)
		ch34x_dev->master->num_chipselect =
			CH341_SPI_MAX_NUM_DEVICES;
	else if (ch34x_dev->chiptype == CHIP_CH347F ||
		 ch34x_dev->chiptype == CHIP_CH347T)
		ch34x_dev->master->num_chipselect =
			CH347_SPI_MAX_NUM_DEVICES;
	else
		ch34x_dev->master->num_chipselect =
			CH339_SPI_MAX_NUM_DEVICES;
	ch34x_dev->master->mode_bits = SPI_MODE_3 | SPI_LSB_FIRST |
				       SPI_CS_HIGH;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	ch34x_dev->master->flags = SPI_CONTROLLER_MUST_RX |
				   SPI_CONTROLLER_MUST_TX;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
	ch34x_dev->master->flags = SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
	ch34x_dev->master->bits_per_word_mask = SPI_BPW_MASK(8);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	ch34x_dev->master->bits_per_word_mask = SPI_BIT_MASK(8);
#endif

	if (ch34x_dev->chiptype == CHIP_CH341) {
		ch34x_dev->master->transfer_one_message =
			ch341_spi_transfer_one_message;
		ch34x_dev->master->setup = ch341_spi_setup;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
		ch34x_dev->master->max_speed_hz = CH341_SPI_MAX_FREQ;
		ch34x_dev->master->min_speed_hz = CH341_SPI_MIN_FREQ;
#endif
	} else {
		ch34x_dev->master->transfer_one_message =
			ch347_spi_transfer_one_message;
		ch34x_dev->master->setup = ch347_spi_setup;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
		ch34x_dev->master->max_speed_hz = CH347_SPI_MAX_FREQ;
		ch34x_dev->master->min_speed_hz = CH347_SPI_MIN_FREQ;
#endif
	}

	/* Register SPI controller */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if ((result = spi_register_controller(ch34x_dev->master))) {
		DEV_ERR(CH34X_USBDEV, "Could not register SPI controller");
		spi_controller_put(ch34x_dev->master);
		ch34x_dev->master = 0;
		return result;
	}
#else
	if ((result = spi_register_master(ch34x_dev->master))) {
		DEV_ERR(CH34X_USBDEV, "Could not register SPI controller");
		spi_master_put(ch34x_dev->master);
		ch34x_dev->master = 0;
		return result;
	}
#endif

	DEV_INFO(CH34X_USBDEV, "SPI controller connected to SPI bus %d",
		 ch34x_dev->master->bus_num);

	if (ch34x_dev->chiptype == CHIP_CH341) {
		mutex_lock(&ch34x_dev->io_mutex);
		ch34x_dev->gpio_io_data |= (1 << ch34x_dev->slave_num) - 1;
		ch341_spi_update_io_data(ch34x_dev);
		mutex_unlock(&ch34x_dev->io_mutex);
	}

	/* Create SPI slaves */
#ifdef SPI_AUTOPROBE
	/************************** Create SPI device #1 **************************/
	ch34x_spi_slaves[0].bus_num = ch34x_dev->master->bus_num;
	/*
	 * The modalias parameter should match with the driver name of SPI device driver, such as:
	 *
	 * static struct spi_driver spidev_spi_driver = {
	 *     .driver = {
	 *         .name = "spidev",
	 *         ...
	 *     },
	 *     ...
	 * }
	 */
	strcpy(ch34x_spi_slaves[0].modalias, "spidev");

	ch34x_spi_slaves[0].chip_select =
		0; /* Chip select 0/1 corresponds to SCS0 or SCS1 of CH347 */
	ch34x_spi_slaves[0].max_speed_hz =
		3000000; /* SPI clock frequency */
	ch34x_spi_slaves[0].mode = SPI_MODE_0; /* SPI mode */

	/**
	 * If the SPI device driver uses interrupt I/O, there are two cases:
	 * Case 1: Using extended GPIO of CH347 as the interrupt I/O
	 *      The gpio_index variable indicates the hardware GPIO pin index of CH347
	 *      (CH347F: 0-7, corresponding to GPIO0~7; CH347T: 0-2, corresponding to GPIO4/6/7)
	 * Case 2: Using the SoC's GPIO as the interrupt I/O
	 *      The gpio_index variable indicates the corresponding GPIO number.
	 *      Generally, you can view the gpiochip/base+index value in the system's /sys/class/gpio directory
	 * 
	 * Example 1: Using GPIO1 of CH347F as the interrupt I/O
	 * gpio_index = 2;
	 * ch34x_spi_slaves[0].irq = gpio_to_irq(ch34x_dev->gpio.base + gpio_index);
	 * 
	 * Example 2: Using the SoC's GPIO as the interrupt I/O
	 * gpio_index = 505;
	 * ch34x_spi_slaves[0].irq = gpio_to_irq(gpio_index);
	 * 
	 * Note: If the I2C device driver does not require interrupt I/O, ignore the above instructions!
	 */
	gpio_index = 1;
	ch34x_spi_slaves[0].irq =
		gpio_to_irq(ch34x_dev->gpio.base + gpio_index);

	/* Create and register a new SPI device */
	ch34x_dev->slaves[0] =
		spi_new_device(ch34x_dev->master, &ch34x_spi_slaves[0]);
	if (ch34x_dev->slaves[0]) {
		DEV_INFO(CH34X_USBDEV, "SPI device spi%d.%d created",
			 ch34x_spi_slaves[0].bus_num,
			 ch34x_spi_slaves[0].chip_select);
	}

	/************************** Create SPI device #2 **************************/
	ch34x_spi_slaves[1].bus_num = ch34x_dev->master->bus_num;
	strcpy(ch34x_spi_slaves[1].modalias, "spidev");

	ch34x_spi_slaves[1].chip_select =
		1; /* Chip select 0/1 corresponds to SCS0 or SCS1 of CH347 */
	ch34x_spi_slaves[1].max_speed_hz =
		3000000; /* SPI clock frequency */
	ch34x_spi_slaves[1].mode = SPI_MODE_0; /* SPI mode */

	gpio_index = 2;
	ch34x_spi_slaves[1].irq =
		gpio_to_irq(ch34x_dev->gpio.base + gpio_index);

	/* Create and register a new SPI device */
	ch34x_dev->slaves[1] =
		spi_new_device(ch34x_dev->master, &ch34x_spi_slaves[1]);
	if (ch34x_dev->slaves[1]) {
		DEV_INFO(CH34X_USBDEV, "SPI device spi%d.%d created",
			 ch34x_spi_slaves[1].bus_num,
			 ch34x_spi_slaves[1].chip_select);
	}

	/**
	* You can continue creating SPI devices with similar code and configuration as above.
	*/

#endif

	DEV_DBG(CH34X_USBDEV, "done");

	return 0;
}

void ch34x_spi_remove(struct ch34x_device *ch34x_dev)
{
	int i;

	CHECK_PARAM(ch34x_dev);

	for (i = 0; i < ch34x_dev->slave_num; i++)
		if (ch34x_dev->slaves[i])
			spi_unregister_device(ch34x_dev->slaves[i]);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (ch34x_dev->master)
		spi_unregister_controller(ch34x_dev->master);
#else
	if (ch34x_dev->master)
		spi_unregister_master(ch34x_dev->master);
#endif

	return;
}
