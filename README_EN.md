# CH347/CH341 linux USB to SPI/I2C/GPIO Master Driver

## Description

The Master driver supports the use of USB to UART/JTAG/SPI/I2C/GPIO conversion chip CH347F/T and USB to UART/SPI/I2C/GPIO conversion chip CH341A/B/C/F/H/T on Linux and Android hosts.

This driver can only be used with SPI/I2C/GPIO interfaces. This document mainly introduces the relevant features of CH347F/T.

### CH347F SPI Interface

| Pin  | SPI Function | GPIO name |
| :--: | :----------: | :-------: |
|  13  |     SCS0     |     -     |
|  7   |     SCS1     |     -     |
|  14  |     SCK      |     -     |
|  16  |     MOSI     |     -     |
|  15  |     MISO     |     -     |

### CH347T SPI Interface

| Pin  | SPI Function | GPIO name |
| :--: | :----------: | :-------: |
|  5   |     SCS0     |   gpio2   |
|  9   |     SCS1     |   gpio5   |
|  6   |     SCK      |   gpio0   |
|  8   |     MOSI     |     -     |
|  7   |     MISO     |   gpio1   |

The SPI hardware interface supports:

- SPI Mode 0/1/2/3
- Clock frequency 60MHz~218.75KHz
- MSB/LSB transfer
- 8/16 bits per word(16bits transfer to be continued...)
- 2 slaves at maximum
- Chip selection high/low active

### CH347F I2C Interface

| Pin  | I2C Function | GPIO name |
| :--: | :----------: | :-------: |
|  11  |     SCL      |     -     |
|  12  |     SDA      |     -     |

### CH347T I2C Interface

| Pin  | I2C Function | GPIO name |
| :--: | :----------: | :-------: |
|  11  |     SCL      |   gpio3   |
|  12  |     SDA      |     -     |

CH347F/T supports I2C clock: 20kHz, 100kHz, 400kHz and 750kHz, etc. The driver inits I2C clock to 100kHz by default, it is possible to change it with the ch34x_mphsi_i2c_init function.

Adding support for a device supported by Linux is easy. For instance:

```
modprobe bmi160_i2c
echo "bmi160 0x68" > /sys/bus/i2c/devices/i2c-$DEV/new_device
```

or:

```
modprobe tcs3472
echo "tcs3472 0x29" > /sys/bus/i2c/devices/i2c-$DEV/new_device
```

Files from these drivers will be created somewhere in /sys/bus/i2c/devices/i2c-$DEV/

### CH347F GPIO Interface

| Pin  | GPIO name |
| :--: | :-------: |
|  17  |   gpio0   |
|  18  |   gpio1   |
|  10  |   gpio2   |
|  9   |   gpio3   |
|  23  |   gpio4   |
|  24  |   gpio5   |
|  25  |   gpio6   |
|  26  |   gpio7   |

The hardware interface of CH347F supports GPIO0~GPIO7, and the pins are not multiplexed with SPI and I2C. This driver supports all GPIOs.

### CH347T GPIO Interface

| Pin  | GPIO name |
| :--: | :-------: |
|  15  |   gpio4   |
|  2   |   gpio6   |
|  13  |   gpio7   |

The GPIO hardware interface supports GPIO0~GPIO7 actually, but this driver only supports GPIO4, GPIO6 and GPIO7 cause the other GPIOs are multiplexed.

### Driver Operating Overview

- Compile the driver using "make" or other method according to your environment, you will see the module "ch34x_mphsi_master.ko" if successful

- Type "sudo make load" or "sudo insmod ch34x_mphsi_master.ko" to load the driver dynamically, in this way the spi bus number and gpio base number will be allocated dynamically, also they can be specified by parameters.

  exp: "sudo insmod ch34x_mphsi_master.ko spi_bus_num=3 gpio_base_num=60".

- Type "sudo make unload" or "sudo rmmod ch34x_mphsi_master.ko" to unload the driver

- Type "sudo make install" to make the driver work permanently

- Type "sudo make uninstall" to remove the driver

Before the driver works, you should make sure that the USB device has been plugged in and is working properly, you can use shell command "lsusb" or "dmesg" to confirm that, VID of USB device is [1A86].

If USB device works well, you can type "ls /sys/class/master" and "ls /sys/class/gpio" to confirm the master node.

## Usage from user space

### Using SPI slaves

Once the driver is loaded successfully, it provides up to 2 SPI slave devices on next available SPI bus, CH347F/T e.g.,

```
/dev/spidev0.0
/dev/spidev0.1
```

according to the naming scheme ```/dev/spidev<bus>.<cs>```. ```<bus>``` is the bus number selected automatically by the driver and ```<cs>``` is the chip select signal of the according pin.

Since linux-5.15 binding to spidev driver is required to make slave devices available via /dev/, e.g. for slave 1 on bus 0:

```
# echo spidev > /sys/class/spi_master/spi0/spi0.1/driver_override
# echo spi0.1 > /sys/bus/spi/drivers/spidev/bind
```

For all devices handled by ch34x_mphsi_master driver:

```
# for i in /sys/bus/usb/drivers/mphsi-ch34x/*/spi_master/spi*/spi*.*; do echo spidev > $i/driver_override; echo $(basename $i) > /sys/bus/spi/drivers/spidev/bind; done
```

Standard I/O functions like ```open```, ```ioctl``` and ```close``` can be used to communicate with one of the slaves connected to the SPI.

To open an SPI device simply use:

```
int spi = open("/dev/spidev0.0", O_RDWR));
```

Once the device is opened successfully, you can modify SPI configurations and transfer data using ```ioctl``` function.

```
uint8_t mode = SPI_MODE_0;
uint8_t lsb = SPI_LSB_FIRST;
...
ioctl(spi, SPI_IOC_WR_MODE, &mode);
ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb);
```

Function ```ioctl``` is also used to transfer data:

```
uint8_t *mosi; // output data
uint8_t *miso; // input data
...
// fill mosi with output data
...
struct spi_ioc_transfer spi_trans;
memset(&spi_trans, 0, sizeof(spi_trans));

spi_trans.tx_buf = (unsigned long) mosi;
spi_trans.rx_buf = (unsigned long) miso;
spi_trans.len = len;

int status = ioctl (spi, SPI_IOC_MESSAGE(1), &spi_trans);

// use input data in miso
```

### Attaching SPI NOR flash as MTD

E.g. flash IC is attached to bus 0 chip 0 (spi0.0):

```
# echo spi0.0 > /sys/bus/spi/drivers/spidev/unbind
# echo spi-nor > /sys/bus/spi/devices/spi0.0/driver_override
# echo spi0.0 > /sys/bus/spi/drivers/spi-nor/bind
```

**Please note:** this driver will create spidev devices by default, you can unbind the device using the above command, or undefine the macro "SPIDEV" in file ch34x_mphsi_master_spi.c. 

### Using GPIOs

To access GPIOs from user space, ```sysfs``` can be used . For each configured GPIO, a directory 

```
/sys/class/gpio/<gpio>/
```

is created by the system, where ```<gpio>``` is the name of the GPIO as defined in the driver variable ```ch347*_board_config```. These directories contain

- the file ```value``` that can be used to read from and write to GPIOs
- the file ```edge``` that can be used to control whether and what type of interrupt is enabled
- the file ```direction``` that can be used to change the direction of the GPIO if possible

**Please note:** For read and write operations from and/or to these files, the user requires read and/or write permissions, respectively.

Possible interrupt types are 

- ```rising``` for interrupts on rising signal edges,
- ```falling``` for interrupts on falling signal edges, and
- ```both``` for interrupts on rising as well as falling signal edges.

#### Open a GPIO

Before a GPIO can be used, file ```value``` has to be opened

```
int  fd;

if ((fd = open("/sys/class/gpio/<gpio>/value", O_RDWR)) == -1) 
{
    perror("open");
    return -1;
}
```

where ```<gpio>``` is again the name of the GPIO.

#### Change the GPIO direction

To change the direction of a GPIO pin configured as input or output, simply write as ```root``` keyword ```in``` or keyword ```out``` to the file ```direction```, e.g.

```
echo out > /sys/class/gpio/gpio4/direction
```

#### Write GPIO output

Once the file ```value``` is opened, you can use standard I/O functions to read and write. To write a GPIO value, simply use function ```write``` as following. The value is written to the GPIO out immediately.

```
if (write(fd, value ? "1" : "0", 1) == -1) 
{
    perror ("write");
	return -1;
}
```

#### Read GPIO input

To read values from GPIOs immediately, you can simply use function ```read``` as following:

```
char buf;

if (read(fd, &buf, 1) == -1) 
{
    perror("read");
    return -1;
}

value = (buf == '0') ? 0 : 1;
```

After each read operation, file position has to be rewound to first character before the next value can be read.

```
if (lseek(fd, 0, SEEK_SET) == -1) {
    perror("lseek");
    return -1;
}
```

#### Reacting on GPIO input interrupt

Complete gpio driver example to use GPIO interrupt function.

```
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/interrupt.h>

#define GPIO_NUMBER 509 /* modify with actual gpio number */

static irqreturn_t gpio_interrupt(int irq, void *dev_id)
{
	printk("gpio_interrupt callback.\n");

	return IRQ_HANDLED;
}

static int __init ch34x_gpio_init(void)
{
	unsigned long flags = IRQF_TRIGGER_FALLING;
	int ret;
	int irq;

	irq = gpio_to_irq(GPIO_NUMBER);
	printk("irq: %d\n", irq);
	ret = gpio_request(GPIO_NUMBER, "gpioint");
	if (ret) {
		printk("gpio_request failed.\n");
		goto exit;
	}
	ret = gpio_direction_input(GPIO_NUMBER);
	if (ret) {
		printk("gpio_direction_input failed.\n");
		gpio_free(GPIO_NUMBER);
		goto exit;
	}
	irq_set_irq_type(irq, flags);
	ret = request_irq(irq, gpio_interrupt, 0, "gpio_handler", NULL);
	printk("%s - request_irq = %d result = %d\n", __func__, irq, ret);

exit:
	return ret;
}

static void __exit ch34x_gpio_exit(void)
{
	int irq;

	irq = gpio_to_irq(GPIO_NUMBER);
	free_irq(irq, NULL);
	gpio_free(GPIO_NUMBER);
	printk("gpio driver exit.\n");
}

module_init(ch34x_gpio_init);
module_exit(ch34x_gpio_exit);

MODULE_LICENSE("GPL");

```

**Please note:** this driver will create gpio devices by default,  users should undefine the macro "SYSFS_GPIO" in file ch34x_mphsi_master_gpio.c before using gpio interruption in kernel.

## Note

​	**CH341 supports 2 modes:**

​	mode0: [UART]

​	mode1: [SPI+ I2C + GPIO]

​	mode2: [LPT]

​	**CH347F supports 1 mode:**

​	mode0: [UART + SPI + I2C + JTAG + SWD]

​	**CH347T supports 4 modes:**

​	mode0: [UART * 2] in VCP/CDC driver mode

​	mode1: [SPI + I2C + UART * 1] in VCP driver mode

​	mode2: [SPI + I2C + UART * 1] in HID driver mode

​	mode3: [JTAG + UART * 1] in VCP driver mode

​	This driver supports **CH347F**, **CH341 mode1**, **CH347T mode1**.

​	Any question, you can send feedback to mail: tech@wch.cn
