# CH347/CH341/CH339/CH9341 Linux USB to SPI/I2C/GPIO Controller Driver

## 1. Overview

On Android and Linux hosts, native CPU SPI, I2C, and GPIO controller resources are often insufficient or fail to meet application performance requirements. Using the CH347 USB 2.0 high-speed interface chip together with the vendor-provided USB-to-MPHSI (Multi-Protocol High-Speed Serial Interface) controller bus driver (`CH34X-MPHSI-Master`), systems can easily expand SPI/I2C buses, GPIO expanders, and interrupt lines.

Once the driver is operational, it creates new SPI and I2C controllers in the system with independent bus numbers. Native SPI and I2C client device drivers can attach directly to these buses without code modification. The driver also creates corresponding GPIO resources, accessible via sysfs or user-space applications. Other kernel device drivers can also request access to these GPIOs, acquire corresponding IRQ numbers, and register Interrupt Service Routines (ISRs).

### Supported Chips

- **CH347F/T**: High-Speed USB to UART / JTAG / SPI / I2C / GPIO converter
- **CH341A/B/C/F/H/T**: Full-Speed USB to UART / SPI / I2C / GPIO converter
- **CH339W**: High-Speed USB to UART / SPI / I2C / Ethernet / Hub converter
- **CH9341C**: Full-Speed USB to SPI / I2C / PWM / GPIO converter

*Note: This driver specifically supports SPI, I2C, and GPIO interfaces.

### Driver Features

1. Supports CH347, CH341, CH339, and CH9341 bus converter chips.
2. Supports expansion of SPI, I2C, GPIO, IRQ, and related functions.
3. Supports both dynamic allocation and static assignment of SPI/I2C bus numbers, GPIO base numbers, and IRQ numbers.
4. Supports automatic binding of SPI device drivers.

### 1.1 CH347F SPI Interface

| **Pin** | **SPI Function** | **Multiplexed GPIO** |
| :-----: | :--------------: | :------------------: |
|   13    |       SCS0       |          -           |
|    7    |       SCS1       |          -           |
|   14    |       SCK        |          -           |
|   16    |       MOSI       |          -           |
|   15    |       MISO       |          -           |

### 1.2 CH347T SPI Interface

| **Pin** | **SPI Function** | **Multiplexed GPIO** |
| :-----: | :--------------: | :------------------: |
|    5    |       SCS0       |        GPIO2         |
|    9    |       SCS1       |        GPIO5         |
|    6    |       SCK        |        GPIO0         |
|    8    |       MOSI       |          -           |
|    7    |       MISO       |        GPIO1         |

**SPI Features:**

- SPI Modes: 0 / 1 / 2 / 3
- Clock Frequency: 218.75 kHz – 60 MHz
- Bit Ordering: MSB / LSB first
- Transfer Width: 8-bit / 16-bit
- Chip Select: 2 channels (Active High or Active Low)

### 1.3 CH9341C SPI Interface

| **Pin** | **SPI Function** | **Multiplexed GPIO** |
| :-----: | :--------------: | :------------------: |
|   24    |       SCS0       |          -           |
|   17    |       SCS1       |        GPIO4         |
|   16    |       SCS2       |        GPIO5         |
|   15    |       SCS3       |        GPIO6         |
|   25    |       SCK        |          -           |
|   26    |       MOSI       |          -           |
|    1    |       MISO       |          -           |

**SPI Features:**

- SPI Modes: 0 / 3
- Clock Frequency: 750 kHz – 24 MHz
- Bit Ordering: MSB / LSB first
- Transfer Width: 8-bit
- Chip Select: 4 channels (Active High or Active Low)

### 1.4 CH347F I2C Interface

| **Pin** | **I2C Function** | **Multiplexed GPIO** |
| :-----: | :--------------: | :------------------: |
|   11    |       SCL        |          -           |
|   12    |       SDA        |          -           |

### 1.5 CH347T I2C Interface

| **Pin** | **I2C Function** | **Multiplexed GPIO** |
| :-----: | :--------------: | :------------------: |
|   11    |       SCL        |        GPIO3         |
|   12    |       SDA        |          -           |

**I2C Clock Rates (CH347F/T):** 20 kHz, 100 kHz, 400 kHz, 750 kHz, and 1 MHz etc. Default initialization is 100 kHz (configurable in `ch34x_mphsi_i2c_init`).

### 1.6 CH9341C I2C Interface

| **Pin** | **I2C Function** | **Multiplexed GPIO** |
| :-----: | :--------------: | :------------------: |
|   22    |       SCL        |          -           |
|   23    |       SDA        |          -           |

**I2C Clock Rates (CH9341C):** 20 kHz, 100 kHz, 400 kHz, and 750 kHz. Default initialization is 100 kHz (configurable in `ch34x_mphsi_i2c_init`).

### 1.7 CH347F GPIO Interface

| **Pin** | **Multiplexed GPIO** | **GPIO Interrupt**                                           |
| :-----: | :------------------: | ------------------------------------------------------------ |
|   17    |        GPIO0         | Supported *(Note: GPIO0 and GPIO1 cannot be used for interrupts simultaneously)* |
|   18    |        GPIO1         | Supported *(Note: GPIO0 and GPIO1 cannot be used for interrupts simultaneously)* |
|   10    |        GPIO2         | Supported                                                    |
|    9    |        GPIO3         | Supported                                                    |
|   23    |        GPIO4         | Supported                                                    |
|   24    |        GPIO5         | Supported                                                    |
|   25    |        GPIO6         | Supported                                                    |
|   26    |        GPIO7         | Supported                                                    |

*Hardware supports GPIO0–GPIO7 without pin multiplexing conflicts against SPI/I2C; all GPIOs are exposed by this driver.*

### 1.8 CH347T GPIO Interface

| **Pin** | **Multiplexed GPIO** | **GPIO Interrupt** |
| :-----: | :------------------: | :----------------: |
|   15    |        GPIO4         |     Supported      |
|    2    |        GPIO6         |     Supported      |
|   13    |        GPIO7         |     Supported      |

*Due to pin sharing with SPI and I2C, the driver only exposes GPIO4, GPIO6, and GPIO7.*

### 1.9 CH9341C GPIO Interface

| **Pin** | **Multiplexed GPIO** | **GPIO Interrupt** |
| ------- | -------------------- | ------------------ |
| 21      | GPIO0                | Supported          |
| 20      | GPIO1                | Supported          |
| 19      | GPIO2                | Supported          |
| 18      | GPIO3                | -                  |
| 17      | GPIO4                | -                  |
| 16      | GPIO5                | -                  |
| 15      | GPIO6                | -                  |
| 13      | GPIO7                | Supported          |
| 12      | GPIO8                | Supported          |
| 11      | GPIO9                | Supported          |

## 2. Driver Build & Operations

- **Build Driver**: Run `make`. Upon successful dynamic compilation, `ch34x_mphsi_master.ko` is generated.

- **Load Driver**: Run `sudo make load` or `sudo insmod ch34x_mphsi_master.ko`. SPI bus numbers and GPIO base numbers are automatically assigned by default. Specific values can be passed via parameters:

  Bash

  ```
  sudo insmod ch34x_mphsi_master.ko spi_bus_num=3 gpio_base_num=60
  ```

- **Unload Driver**: Run `sudo make unload` or `sudo rmmod ch34x_mphsi_master.ko`.

- **Install (Auto-load on boot)**: Run `sudo make install`.

- **Uninstall**: Run `sudo make uninstall`.

**Hardware Verification:**

Ensure the USB device is plugged in and recognized using `lsusb` (Vendor VID is `0x1A86`) or `dmesg`. Once loaded properly, check device node paths via `ls /sys/class/spi_master` and `ls /sys/class/gpio`.

## 3. Interface Applications

### 3.1 Matching SPI Device Drivers

Because this extended SPI master controller is not an internal SoC hardware block, its resources cannot be declared directly in the Device Tree Source (DTS). Instead, the driver supports auto-matching via hardcoded device definitions, achieving the same result as DTS-based probing.

Enable auto-probing in `ch34x_mphsi_master_spi.c`:

```
#define SPI_AUTOPROBE
//#undef SPI_AUTOPROBE
```

#### 3.1.1 Adding an SPI Slave Device

Locate the following code in `ch34x_mphsi_master_spi.c`:

```
int ch34x_spi_probe(struct ch34x_device *ch34x_dev)
{
    ...
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
	ch34x_spi_slaves[0].chip_select = 0;
	ch34x_spi_slaves[0].max_speed_hz =
		3000000; /* SPI clock frequency */
	ch34x_spi_slaves[0].mode = SPI_MODE_0; /* SPI mode */

	/**
	 * If the SPI device driver uses interrupt I/O, there are two cases:
	 * Case 1: Using extended GPIO of chip as the interrupt I/O
	 *      The gpio_index variable indicates the hardware GPIO pin index of chip
	 *      (CH347F: 0-7, corresponding to GPIO0~7; CH347T: 0-2, corresponding to GPIO4/6/7;
	 *       CH9341C: 0/1/2/7/8/9, corresponding to GPIO0/1/2/7/8/9)
	 * Case 2: Using the SoC's GPIO as the interrupt I/O
	 *      The gpio_index variable indicates the corresponding GPIO number.
	 *      Generally, you can view the gpiochip/base+index value in the system's /sys/class/gpio directory
	 * 
	 * Example 1: Using GPIO1 of CH347F as the interrupt I/O
	 * gpio_index = 1;
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

	/**
	* You can continue creating SPI devices with similar code and configuration as above.
	*/

#endif
    ...
}
```

### 3.2 Accessing SPI from User Space

#### 3.2.1 Adaptation Method

User-space applications can communicate via the standard `spidev` driver:

- `drivers/spi/spidev.c`: Kernel user-space SPI driver providing character devices (e.g., `/dev/spidev0.0`) and standard `ioctl` APIs.
- `tools/spi/spidev_test.c`: Kernel sample utility to test and verify SPI bus connectivity.
- `Documentation/spi/spidev.rst`: Documentation for `spidev`.

#### 3.2.2 Verification & Operations

After loading the driver, nodes will appear under `/dev/`:

```
root@user:/dev# ls /dev/spidev*
/dev/spidev0.0
```

Naming format: `/dev/spidev<bus>.<cs>`, where `<bus>` is the dynamically assigned SPI bus number and `<cs>` is the chip select index.

*Starting from Linux 5.15, explicit binding is required to expose devices under `/dev/` (e.g., slave 1 on bus 0):*

```
echo spidev > /sys/class/spi_master/spi0/spi0.1/driver_override
echo spi0.1 > /sys/bus/spi/drivers/spidev/bind
```

To bind all devices managed by `ch34x_mphsi_master` at once:

```
for i in /sys/bus/usb/drivers/mphsi-ch34x/*/spi_master/spi*/spi*.*; do
    echo spidev > $i/driver_override
    echo $(basename $i) > /sys/bus/spi/drivers/spidev/bind
done
```

**C Application Example:**

```
int spi = open("/dev/spidev0.0", O_RDWR);

uint8_t mode = SPI_MODE_0;
uint8_t lsb = SPI_LSB_FIRST;

ioctl(spi, SPI_IOC_WR_MODE, &mode);
ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb);

// Transferring data:
struct spi_ioc_transfer spi_trans;
memset(&spi_trans, 0, sizeof(spi_trans));

spi_trans.tx_buf = (unsigned long)mosi;
spi_trans.rx_buf = (unsigned long)miso;
spi_trans.len = len;

int status = ioctl(spi, SPI_IOC_MESSAGE(1), &spi_trans);
```

### 3.3 Matching I2C Device Drivers

Similar to SPI, the extended I2C adapter can match drivers without DTS. Enable auto-probing in `ch34x_mphsi_master_i2c.c`:

```
#define I2C_AUTOPROBE
//#undef I2C_AUTOPROBE
```

If the client driver provides an `id_table`, configure the matching entry inside `ch34x_mphsi_i2c_probe`:

```
int ch34x_mphsi_i2c_probe(struct ch34x_device *ch34x_dev)
{
    ...
#ifdef I2C_AUTOPROBE
	/*
	 * The modalias parameter should match with the driver name of I2C device driver, such as:
	 * 
	 * static const struct i2c_device_id at24_ids[] = {
	 *     { "at24", 0 },
	 * };
	 * 
	 */
	strcpy(ch34x_i2c_slaves[0].type, "at24");
	/* I2C device address, generally specified by the device side, can be obtained using i2c-tools */
	ch34x_i2c_slaves[0].addr = 0x50;

	/**
	 * If the SPI device driver uses interrupt I/O, there are two cases:
	 * Case 1: Using extended GPIO of chip as the interrupt I/O
	 *      The gpio_index variable indicates the hardware GPIO pin index of chip
	 *      (CH347F: 0-7, corresponding to GPIO0~7; CH347T: 0-2, corresponding to GPIO4/6/7;
	 *       CH9341C: 0/1/2/7/8/9, corresponding to GPIO0/1/2/7/8/9)
	 * Case 2: Using the SoC's GPIO as the interrupt I/O
	 *      The gpio_index variable indicates the corresponding GPIO number.
	 *      Generally, you can view the gpiochip/base+index value in the system's /sys/class/gpio directory
	 * 
	 * Example 1: Using GPIO2 of CH347F as the interrupt I/O
	 * gpio_index = 2;
	 * ch34x_spi_slaves[0].irq = gpio_to_irq(ch34x_dev->gpio.base + gpio_index);
	 * 
	 * Example 2: Using the SoC's GPIO as the interrupt I/O
	 * gpio_index = 505;
	 * ch34x_spi_slaves[0].irq = gpio_to_irq(gpio_index);
	 * 
	 * Note: If the I2C device driver does not require interrupt I/O, ignore the above instructions!
	 */
	gpio_index = 2;
	ch34x_i2c_slaves[0].irq =
		gpio_to_irq(ch34x_dev->gpio.base + gpio_index);

	/* Create and register a new I2C device */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	ch34x_dev->client[0] = i2c_new_client_device(&ch34x_dev->adapter,
						     &ch34x_i2c_slaves[0]);
#else
	ch34x_dev->client[0] =
		i2c_new_device(&ch34x_dev->adapter, &ch34x_i2c_slaves[0]);
#endif
	if (!ch34x_dev->client[0]) {
		dev_err(&ch34x_dev->adapter.dev,
			"Add new I2C client failed\n");
		return -ENODEV;
	}

	/**
	* If using a 1-master-to-multiple-slave configuration, 
	* you can continue adding the above code to create and register more I2C devices.
	*/
#endif
    ...
}
```

If the target driver lacks an `id_table`, add one to the driver:

```
static const struct i2c_device_id xxx_i2c_id[] = {
    { "ch934x", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, xxx_i2c_id);

static struct i2c_driver xxx_i2c_driver = {
    ...
    .id_table = xxx_i2c_id, /* Add this line */
    .probe = xxx_i2c_probe,
    .remove = xxx_i2c_remove,
};
```

### 3.4 Using `i2c-tools`

Once the I2C controller is registered, standard utilities such as `i2cdetect`, `i2cdump`, `i2cget`, and `i2cset` can be used to scan and debug devices on the bus.

### 3.5 Accessing GPIO from User Space

To export GPIO controllers directly to sysfs, uncomment the following in `ch34x_mphsi_master_gpio.c`:

```
#define SYSFS_GPIO
// #undef SYSFS_GPIO
```

Sysfs paths are created under `/sys/class/gpio/<gpio>/` with the following control files:

- `value`: Reads or writes the logic level (0 or 1).
- `edge`: Configures interrupt trigger types (`rising`, `falling`, `both`).
- `direction`: Configures pin direction (`in` or `out`).

#### 3.5.1 Opening GPIO

```
int fd = open("/sys/class/gpio/<gpio>/value", O_RDWR);
if (fd == -1) {
    perror("open");
    return -1;
}
```

#### 3.5.2 Setting Direction

```
echo out > /sys/class/gpio/gpio4/direction
```

#### 3.5.3 Writing Output

```
if (write(fd, value ? "1" : "0", 1) == -1) {
    perror("write");
    return -1;
}
```

#### 3.5.4 Reading Input

```
char buf;
if (read(fd, &buf, 1) == -1) {
    perror("read");
    return -1;
}
value = (buf == '0') ? 0 : 1;

// Reset offset for subsequent reads:
lseek(fd, 0, SEEK_SET);
```

#### 3.5.5 Kernel GPIO Interrupt Example

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

#define GPIO_NUMBER 509 /* Replace with target system GPIO number */

static irqreturn_t gpio_interrupt(int irq, void *dev_id)
{
    printk("gpio_interrupt callback.\n");
    return IRQ_HANDLED;
}

static int __init ch34x_gpio_init(void)
{
    unsigned long flags = IRQF_TRIGGER_FALLING;
    int ret, irq;

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
    int irq = gpio_to_irq(GPIO_NUMBER);
    free_irq(irq, NULL);
    gpio_free(GPIO_NUMBER);
    printk("gpio driver exit.\n");
}

module_init(ch34x_gpio_init);
module_exit(ch34x_gpio_exit);

MODULE_LICENSE("GPL");
```

### 3.6 Notes on Chip Operating Modes

- **CH341**: Supports 3 operating modes
  - Mode 0: [UART]
  - Mode 1: [SPI + I2C + GPIO]
  - Mode 2: [Printer Port]
- **CH347F**: Supports 1 operating mode
  - Mode 0: [UART + SPI + I2C + JTAG + SWD]
- **CH347T**: Supports 4 operating modes
  - Mode 0: [UART * 2] (VCP / CDC mode)
  - Mode 1: [SPI + I2C + UART * 1] (VCP mode)
  - Mode 2: [SPI + I2C + UART * 1] (HID mode)
  - Mode 3: [JTAG + UART * 1] (VCP mode)
- **CH339W**: Supports 1 operating mode
  - Mode 0: [UART + SPI + I2C + JTAG + ...]
- **CH9341C**: Supports 1 operating mode
  - Mode 0: [SPI + I2C + PWM + GPIO]

**Driver Compatibility Summary:**

This driver supports **CH347F**, **CH339W**, **CH9341C**, **CH341 (Mode 1)**, and **CH347T (Mode 1)**.



For technical support, contact: **tech@wch.cn**