/*
 * ch347/ch341 MPHSI GPIO and IRQ driver layer
 *
 * Copyright (C) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
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

#define SYSFS_GPIO
#undef SYSFS_GPIO

/* parameters */
static int param_gpio_base = -1;
module_param_named(gpio_base_num, param_gpio_base, int, 0600);
MODULE_PARM_DESC(gpio_base_num, "GPIO master base number (if negative, dynamic allocation)");

static int ch347gpio_control(struct ch34x_device *ch34x_dev, u8 *cfg_data, u8 *status_data);
extern int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len, int in_len);
static void irq_set_work(struct work_struct *work);

static bool ch347_irq_control(struct ch34x_device *ch34x_dev, u8 gpioindex, bool enable, unsigned int type)
{
	u8 cfg_data[CH347_GPIO_CNT] = { 0 };

	cfg_data[gpioindex] |= BIT(7);
	cfg_data[gpioindex] |= BIT(6);
	cfg_data[gpioindex] &= ~BIT(5);
	if (enable) {
		cfg_data[gpioindex] |= BIT(2);
		switch (type) {
		case IRQ_TYPE_EDGE_FALLING:
			cfg_data[gpioindex] &= ~BIT(1);
			cfg_data[gpioindex] &= ~BIT(0);
			break;
		case IRQ_TYPE_EDGE_RISING:
			cfg_data[gpioindex] &= ~BIT(1);
			cfg_data[gpioindex] |= BIT(0);
			break;
		case IRQ_TYPE_EDGE_BOTH:
			cfg_data[gpioindex] |= BIT(1);
			cfg_data[gpioindex] &= ~BIT(0);
			break;
		default:
			return false;
		}
	} else
		cfg_data[gpioindex] &= ~BIT(2);

	if (ch347gpio_control(ch34x_dev, cfg_data, NULL))
		return false;

	return true;
}

static void ch347_irq_enable_control(struct irq_data *data, bool enable)
{
	struct ch34x_device *ch34x_dev;
	int irq;
	u8 gpioindex;

	CHECK_PARAM(data && (ch34x_dev = irq_data_get_irq_chip_data(data)));

	/* calculate local IRQ */
	irq = data->irq - ch34x_dev->irq_base;
	if (irq < 0 || irq >= ch34x_dev->irq_num)
		return;

	gpioindex = ch34x_dev->gpio_pins[irq]->gpioindex;

	schedule_delayed_work(&ch34x_dev->work, msecs_to_jiffies(5));

	/* enable local IRQ */
	ch34x_dev->irq_enabled[irq] = enable;

	DEV_DBG(CH34X_USBDEV, "irq=%d enabled=%d", data->irq, ch34x_dev->irq_enabled[irq] ? 1 : 0);
}

static void ch347_irq_enable(struct irq_data *data)
{
	ch347_irq_enable_control(data, true);
}

static void ch347_irq_disable(struct irq_data *data)
{
	ch347_irq_enable_control(data, false);
}

static int ch347_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct ch34x_device *ch34x_dev;
	int irq;
	u8 gpioindex;

	CHECK_PARAM_RET(data && (ch34x_dev = irq_data_get_irq_chip_data(data)), -EINVAL);

	/* calculate local IRQ */
	irq = data->irq - ch34x_dev->irq_base;
	if (irq < 0 || irq >= ch34x_dev->irq_num)
		return -EINVAL;

	gpioindex = ch34x_dev->gpio_pins[irq]->gpioindex;

	schedule_delayed_work(&ch34x_dev->work, msecs_to_jiffies(5));

	ch34x_dev->irq_types[irq] = type;

	DEV_DBG(CH34X_USBDEV, "irq=%d flow_type=%d", data->irq, type);

	return 0;
}

int ch347_irq_check(struct ch34x_device *ch34x_dev, u8 irq)
{
	int type;
	unsigned long flags;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);
	CHECK_PARAM_RET(irq < ch34x_dev->irq_num, -EINVAL);

	if (irq < 0 || irq >= ch34x_dev->irq_num)
		return -EINVAL;

	/* if IRQ is disabled, just return with success */
	if (!ch34x_dev->irq_enabled[irq])
		return 0;

	type = ch34x_dev->irq_types[irq];

	DEV_DBG(CH34X_USBDEV, "hardware irq=%d %d", irq, type);

	spin_lock_irqsave(&ch34x_dev->irq_lock, flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)
	handle_simple_irq(irq_data_to_desc(irq_get_irq_data(ch34x_dev->irq_base + irq)));
#else
	handle_simple_irq(ch34x_dev->irq_base + irq, irq_data_to_desc(irq_get_irq_data(ch34x_dev->irq_base + irq)));
#endif
	spin_unlock_irqrestore(&ch34x_dev->irq_lock, flags);

	return 0;
}

int ch347_irq_probe(struct ch34x_device *ch34x_dev)
{
	int i;
	int result;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);

	DEV_DBG(CH34X_USBDEV, "start");

	ch34x_dev->irq.name = "ch34x";
	ch34x_dev->irq.irq_enable = ch347_irq_enable;
	ch34x_dev->irq.irq_disable = ch347_irq_disable;
	ch34x_dev->irq.irq_set_type = ch347_irq_set_type;

	if (!ch34x_dev->irq_num)
		return 0;

	if ((result = irq_alloc_descs(-1, 0, ch34x_dev->irq_num, 0)) < 0) {
		DEV_ERR(CH34X_USBDEV, "failed to allocate IRQ descriptors");
		return result;
	}

	ch34x_dev->irq_base = result;

	DEV_DBG(CH34X_USBDEV, "irq_base=%d", ch34x_dev->irq_base);

	for (i = 0; i < ch34x_dev->irq_num; i++) {
		ch34x_dev->irq_enabled[i] = false;

		irq_set_chip(ch34x_dev->irq_base + i, &ch34x_dev->irq);
		irq_set_chip_data(ch34x_dev->irq_base + i, ch34x_dev);
		irq_clear_status_flags(ch34x_dev->irq_base + i, IRQ_NOREQUEST | IRQ_NOPROBE);
	}

	spin_lock_init(&ch34x_dev->irq_lock);

	INIT_DELAYED_WORK(&ch34x_dev->work, irq_set_work);

	DEV_DBG(CH34X_USBDEV, "done");

	return 0;
}

void ch347_irq_remove(struct ch34x_device *ch34x_dev)
{
	CHECK_PARAM(ch34x_dev);

	if (ch34x_dev->irq_base)
		irq_free_descs(ch34x_dev->irq_base, ch34x_dev->irq_num);

	cancel_delayed_work(&ch34x_dev->work);

	return;
}

/**
 * ch347gpio_control - gpio setting
 * @fd: file descriptor of device
 * @cfg_data: pointer to gpio configuraion array
 * @status_data: pointer to gpio status array
 *
 * The function return true if success, others if fail.
 */
static int ch347gpio_control(struct ch34x_device *ch34x_dev, u8 *cfg_data, u8 *status_data)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int i, len;
	int ret = 0;

	mutex_lock(&ch34x_dev->io_mutex);

	i = 0;
	io[i++] = USB20_CMD_GPIO_CMD;
	io[i++] = CH347_GPIO_CNT;
	io[i++] = 0;
	memcpy(&io[i], cfg_data, CH347_GPIO_CNT);
	len = i + CH347_GPIO_CNT;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len) {
		ret = -EPROTO;
		goto exit;
	}

	if (ch34x_usb_transfer(ch34x_dev, 0, len) != len) {
		ret = -EPROTO;
		goto exit;
	}

	if (status_data)
		memcpy(status_data, ch34x_dev->bulkin_buf + 3, len - USB20_CMD_HEADER);

exit:
	mutex_unlock(&ch34x_dev->io_mutex);
	return ret;
}

/**
 * ch347gpio_get - get gpio status
 * @fd: file descriptor of device
 * @idir: gpio direction bits, bits0-7 on gpio0-7, 1 on ouput, 0 on input
 * @idata: gpio level bits, bits0-7 on gpio0-7, 1 on high, 0 on low
 *
 * The function return true if success, others if fail.
 */
static int ch347gpio_get(struct ch34x_device *ch34x_dev, u8 *cfg_data, u8 *idir, u8 *idata)
{
	int j;
	u8 status_data[CH347_GPIO_CNT] = { 0 };

	if (ch347gpio_control(ch34x_dev, cfg_data, status_data))
		return -EPROTO;

	*idir = *idata = 0;
	for (j = 0; j < CH347_GPIO_CNT; j++) {
		/* get direction bit */
		if (status_data[j] & BIT(7))
			*idir |= (1 << j);
		/* get value bit */
		if (status_data[j] & BIT(6))
			*idata |= (1 << j);
	}
	return 0;
}

/**
 * ch347gpio_set - gpio setting
 * @fd: file descriptor of device
 * @ienable: gpio function enable bits, bits0-7 on gpio0-7, 1 on enable
 * @idirout: gpio direction bits, bits0-7 on gpio0-7, 1 on ouput, 0 on input
 * @idataout: gpio output bits, bits0-7 on gpio0-7, if gpio direction is output, 1 on high, 0 on low
 *
 * The function return true if success, others if fail.
 */
static int ch347gpio_set(struct ch34x_device *ch34x_dev, u8 ienable, u8 idirout, u8 idataout)
{
	u8 cfg_data[CH347_GPIO_CNT] = { 0 };
	int i;

	for (i = 0; i < CH347_GPIO_CNT; i++) {
		if (ienable & (1 << i)) {
			cfg_data[i] |= BIT(7);
			cfg_data[i] |= BIT(6);
			if (idirout & (1 << i))
				cfg_data[i] |= BIT(5);
			else
				cfg_data[i] &= ~BIT(5);
			if (idirout & (1 << i)) {
				cfg_data[i] |= BIT(4);
				if (idataout & (1 << i))
					cfg_data[i] |= BIT(3);
				else
					cfg_data[i] &= ~BIT(3);
			}
		} else {
			cfg_data[i] &= ~BIT(7);
		}
	}
	if (ch347gpio_control(ch34x_dev, cfg_data, NULL))
		return -EPROTO;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int ch34x_mphsi_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	struct ch34x_device *ch34x_dev = (struct ch34x_device *)gpiochip_get_data(chip);
#else
	struct ch34x_device *ch34x_dev = container_of(chip, struct ch34x_device, gpio);
#endif
	int mode;
	u8 gpioindex = ch34x_dev->gpio_pins[offset]->gpioindex;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);
	CHECK_PARAM_RET(offset < ch34x_dev->gpio_num, -EINVAL);

	mode = (ch34x_dev->gpio_pins[offset]->mode == GPIO_MODE_IN) ? 1 : 0;

	DEV_DBG(CH34X_USBDEV, "gpio=%d dir=%d", gpioindex, mode);

	return mode;
}
#endif

static int ch34x_mphsi_gpio_set_direction(struct gpio_chip *chip, unsigned offset, bool input, int value)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	struct ch34x_device *ch34x_dev = (struct ch34x_device *)gpiochip_get_data(chip);
#else
	struct ch34x_device *ch34x_dev = container_of(chip, struct ch34x_device, gpio);
#endif
	u8 ienable = 0x00, idirout = 0x00, idataout = 0x00;
	u8 gpioindex = ch34x_dev->gpio_pins[offset]->gpioindex;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);
	CHECK_PARAM_RET(offset < ch34x_dev->gpio_num, -EINVAL);

	DEV_DBG(CH34X_USBDEV, "gpio=%d direction=%s", gpioindex, input ? "input" : "output");

	ch34x_dev->gpio_pins[offset]->mode = input ? GPIO_MODE_IN : GPIO_MODE_OUT;

	ienable = 1 << gpioindex;
	if (!input) {
		idirout = 1 << gpioindex;
		if (value)
			idataout = 1 << gpioindex;
	}

	return ch347gpio_set(ch34x_dev, ienable, idirout, idataout);
}

static int ch34x_mphsi_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return ch34x_mphsi_gpio_set_direction(chip, offset, true, 1);
}

static int ch34x_mphsi_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	return ch34x_mphsi_gpio_set_direction(chip, offset, false, value);
}

static int ch34x_mphsi_gpio_get(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	struct ch34x_device *ch34x_dev = (struct ch34x_device *)gpiochip_get_data(chip);
#else
	struct ch34x_device *ch34x_dev = container_of(chip, struct ch34x_device, gpio);
#endif
	int value;
	u8 cfg_data[CH347_GPIO_CNT] = { 0 };
	u8 idir, idata;
	u8 gpioindex = ch34x_dev->gpio_pins[offset]->gpioindex;
	int i;
	int retval;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);
	CHECK_PARAM_RET(offset < ch34x_dev->gpio_num, -EINVAL);

	for (i = 0; i < CH347_GPIO_CNT; i++)
		cfg_data[i] = 0x00;

	cfg_data[gpioindex] = BIT(7);

	retval = ch347gpio_get(ch34x_dev, cfg_data, &idir, &idata);
	if (retval)
		return retval;

	value = (idata & BIT(gpioindex)) ? 1 : 0;

	DEV_DBG(CH34X_USBDEV, "offset=%u, gpio%d, value=%d", offset, gpioindex, value);

	return value;
}

static void ch34x_mphsi_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	struct ch34x_device *ch34x_dev = (struct ch34x_device *)gpiochip_get_data(chip);
#else
	struct ch34x_device *ch34x_dev = container_of(chip, struct ch34x_device, gpio);
#endif

	u8 ienable = 0x00, idirout = 0x00, idataout = 0x00;
	u8 gpioindex = ch34x_dev->gpio_pins[offset]->gpioindex;

	CHECK_PARAM(ch34x_dev);
	CHECK_PARAM(offset < ch34x_dev->gpio_num);

	DEV_DBG(CH34X_USBDEV, "offset=%u value=%d", offset, value);

	ienable = 1 << gpioindex;
	idirout = 1 << gpioindex;
	if (value)
		idataout = 1 << gpioindex;

	ch347gpio_set(ch34x_dev, ienable, idirout, idataout);
}

static int ch34x_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	struct ch34x_device *ch34x_dev = (struct ch34x_device *)gpiochip_get_data(chip);
#else
	struct ch34x_device *ch34x_dev = container_of(chip, struct ch34x_device, gpio);
#endif
	int irq;

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);
	CHECK_PARAM_RET(offset < ch34x_dev->gpio_num, -EINVAL);

	irq = ch34x_dev->gpio_irq_map[offset];
	irq = (irq >= 0 ? ch34x_dev->irq_base + irq : 0);

	DEV_DBG(CH34X_USBDEV, "gpio=%d irq=%d", offset, irq);

	return irq;
}

static void irq_set_work(struct work_struct *work)
{
	struct ch34x_device *ch34x_dev = container_of(to_delayed_work(work), struct ch34x_device, work);

	int irq = 0;
	int gpioindex;

	for (irq = 0; irq < ch34x_dev->irq_num; irq++) {
		gpioindex = ch34x_dev->gpio_pins[irq]->gpioindex;
		ch347_irq_control(ch34x_dev, gpioindex, ch34x_dev->irq_enabled[irq], ch34x_dev->irq_types[irq]);
	}
}

int ch34x_mphsi_gpio_probe(struct ch34x_device *ch34x_dev)
{
	struct gpio_chip *gpio = &ch34x_dev->gpio;
	int result;
#ifdef SYSFS_GPIO
	int i, j = 0;
#endif

	CHECK_PARAM_RET(ch34x_dev, -EINVAL);

	DEV_DBG(CH34X_USBDEV, "start");

	gpio->label = "ch34x-mphsi-gpio";
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	gpio->parent = &ch34x_dev->usb_dev->dev;
#else
	gpio->dev = &ch34x_dev->usb_dev->dev;
#endif
	gpio->owner = THIS_MODULE;
	gpio->request = NULL;
	gpio->free = NULL;
	gpio->base = (param_gpio_base >= 0) ? param_gpio_base : -1;
	gpio->ngpio = ch34x_dev->gpio_num;
	gpio->can_sleep = 1;
	gpio->names = (void *)ch34x_dev->gpio_names;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	gpio->get_direction = ch34x_mphsi_gpio_get_direction;
#endif
	gpio->direction_input = ch34x_mphsi_gpio_direction_input;
	gpio->direction_output = ch34x_mphsi_gpio_direction_output;
	gpio->get = ch34x_mphsi_gpio_get;
	gpio->set = ch34x_mphsi_gpio_set;
	gpio->to_irq = ch34x_gpio_to_irq;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	if ((result = gpiochip_add_data(gpio, ch34x_dev)))
#else
	if ((result = gpiochip_add(gpio)))
#endif
	{
		DEV_ERR(CH34X_USBDEV, "failed to register GPIOs: %d", result);
		gpio->base = -1;
		return result;
	}

	DEV_INFO(CH34X_USBDEV, "registered GPIOs from %d to %d", gpio->base, gpio->base + gpio->ngpio - 1);

#ifdef SYSFS_GPIO
	for (i = 0; i < ch34x_dev->gpio_num; i++) {
		result = gpio_request(gpio->base + i, "ch34x gpio expander");
		if (result) {
			DEV_ERR(CH34X_USBDEV, "Failed to allocate pin %d\n", gpio->base + j);
			return result;
		}
		result = gpio_export(gpio->base + i, true);
		if (result) {
			DEV_ERR(CH34X_USBDEV, "failed to export gpio pin %d", gpio->base + i);
			/* reduce number of GPIOs to avoid crashes during free in case of error */
			ch34x_dev->gpio_num = i ? i - 1 : 0;
			return result;
		}
	}
#endif

	DEV_DBG(CH34X_USBDEV, "done");

	return 0;
}

void ch34x_mphsi_gpio_remove(struct ch34x_device *ch34x_dev)
{
#ifdef SYSFS_GPIO
	int i;
#endif

	CHECK_PARAM(ch34x_dev);

	if (ch34x_dev->gpio.base > 0) {
#ifdef SYSFS_GPIO
		for (i = 0; i < ch34x_dev->gpio_num; ++i)
			gpio_free(ch34x_dev->gpio.base + i);
#endif
		gpiochip_remove(&ch34x_dev->gpio);
	}

	return;
}
