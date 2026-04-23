# CH347/CH341 Linux USB to SPI/I2C/GPIO Controller Driver

# 1 概述

在安卓/Linux主机上经常会遇到CPU原生SPI/I2C/GPIO Controller资源通道不够或者功性能不满足实际产品需求的情况，基于USB2.0高速USB转接芯片CH347，配合厂商提供的USB转MPHSI（Multi Protocol High-Speed Serial Interface）控制器总线驱动（CH34X-MPHSI-Master）可轻松实现为系统扩展SPI和I2C总线、GPIO Expander、中断信号等。

该驱动软件正常工作后，会在系统下创建新的SPI和I2C Controller控制器，拥有独立的bus num，原SPI和I2C器件的设备驱动可直接挂载到该总线上，无需修改。驱动会同时创建GPIO相关资源，各GPIO可通过sysfs文件系统或应用层软件直接访问，也可以由其他设备驱动申请该GPIO的访问权以及申请GPIO对应中断号并注册中断服务程序。

该驱动支持：高速USB转串口/JTAG/SPI/I2C/GPIO转换芯片CH347F/T，全速USB转串口/SPI/I2C/GPIO转换芯片CH341A/B/C/F/H/T，高速USB转串口/SPI/I2C/Ethernet/Hub转换芯片CH339W。

驱动仅支持SPI/I2C/GPIO接口，该文档主要介绍CH347F/T芯片的相关特性。

驱动功能：
1、支持CH347与CH341总线转接芯片；
2、支持SPI、I2C、GPIO、IRQ等接口和功能扩展；
3、支持SPI、I2C的bus总线号、GPIO编号、IRQ中断号的动态分配以及静态指定；
4、支持自动绑定SPI设备驱动程序；

## 1.1 CH347F SPI接口

| PIN脚 | SPI功能脚 | GPIO复用脚 |
| :---: | :-------: | :--------: |
|  13   |   SCS0    |     -      |
|   7   |   SCS1    |     -      |
|  14   |    SCK    |     -      |
|  16   |   MOSI    |     -      |
|  15   |   MISO    |     -      |

## 1.2 CH347T SPI接口

| PIN脚 | SPI功能脚 | GPIO复用脚 |
| :---: | :-------: | :--------: |
|   5   |   SCS0    |   gpio2    |
|   9   |   SCS1    |   gpio5    |
|   6   |    SCK    |   gpio0    |
|   8   |   MOSI    |     -      |
|   7   |   MISO    |   gpio1    |

SPI接口特性：

- SPI模式0/1/2/3
- SPI时钟频率60MHz~218.75KHz
- MSB/LSB传输
- 8位/16位传输
- 2路片选
- 片选高/低有效

## 1.3 CH347F I2C接口

| PIN脚 | I2C功能脚 | GPIO复用脚 |
| :---: | :-------: | :--------: |
|  11   |    SCL    |     -      |
|  12   |    SDA    |     -      |

## 1.4 CH347T I2C接口

| PIN脚 | I2C功能脚 | GPIO复用脚 |
| :---: | :-------: | :--------: |
|  11   |    SCL    |   gpio3    |
|  12   |    SDA    |     -      |

CH347F/T支持I2C时钟：20kHz，100kHz，400kHz，750kHz和1MHz等。驱动会默认将I2C的时钟初始化为100KHz，可以在ch34x_mphsi_i2c_init函数中修改。

## 1.5 CH347F GPIO接口

| PIN脚 | GPIO复用脚 | GPIO中断                                     |
| :---: | :--------: | -------------------------------------------- |
|  17   |   gpio0    | 支持，注：gpio0和gpio1的中断功能无法同时使用 |
|  18   |   gpio1    | 支持，注：gpio0和gpio1的中断功能无法同时使用 |
|  10   |   gpio2    | 支持                                         |
|   9   |   gpio3    | 支持                                         |
|  23   |   gpio4    | 支持                                         |
|  24   |   gpio5    | 支持                                         |
|  25   |   gpio6    | 支持                                         |
|  26   |   gpio7    | 支持                                         |

CH347F的硬件接口支持GPIO0~GPIO7，引脚未与SPI和I2C复用，此驱动开放支持了所有GPIO。

## 1.6 CH347T GPIO接口

| PIN脚 | GPIO复用脚 | GPIO中断 |
| :---: | :--------: | -------- |
|  15   |   gpio4    | 支持     |
|   2   |   gpio6    | 支持     |
|  13   |   gpio7    | 支持     |

CH347T的硬件接口支持GPIO0~GPIO7，考虑到部分引脚被SPI和I2C接口占用，驱动仅开放支持GPIO4，GPIO6和GPIO7。

# 2 驱动操作说明

- 使用“make”或者其他方式编译此驱动，如果动态编译成功会生成“ch34x_mphsi_master.ko”驱动模块
- 使用“sudo make load”或“sudo insmod ch34x_mphsi_master.ko”动态加载驱动，使用此方式加载SPI总线号和GPIO起始序号会自动分配，也可以通过增加参数进行指定。
- 如：“sudo insmod ch34x_mphsi_master.ko spi_bus_num=3 gpio_base_num=60”
- 使用“sudo make unload”或“sudo rmmod ch34x_mphsi_master.ko”卸载驱动
- 使用“sudo make install”将驱动开机自动工作
- 使用“sudo make uninstall”卸载该驱动

使用此驱动，需要确认USB设备已经插入主机并且工作正常，可以使用“lsusb”或“dmesg”指令来确定，设备的厂商VID是0x1A86。

如果芯片工作正常，可以使用“ls /sys/class/master”，“ls /sys/class/gpio”指令确认设备节点路径。

# 3 接口应用

## 3.1 匹配SPI设备驱动程序

通过CH347芯片配合该厂商驱动程序，可在Linux系统扩展出一路新的SPI主机(SPI Controller)接口，由于该SPI主机不属于SOC内部资源，所以不能在DTS文件中描述该SPI主机的资源信息，本节介绍一种自动匹配设备驱动程序的方法，底层原理等同于使用DTS方式。

用户需要在驱动程序文件 ch34x_mphsi_master_spi.c 中注释如下代码：

```c
#define SPI_AUTOPROBE
//#undef SPI_AUTOPROBE
```

#### 3.1.1 添加从机
在ch34x_mphsi_master_spi.c文件中，找到如下代码

```c
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
	ch34x_spi_slaves[0].max_speed_hz = 3000000; /* SPI clock frequency */
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
	gpio_index = 2;
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
```

## 3.2 用户态访问SPI接口

### 3.2.1 适配方法

在用户空间可以使用spidev模块访问SPI接口，默认驱动程序匹配此驱动模块，Linux内核源码和应用文档：

- drivers\spi\spidev.c：是Linux 内核的SPI用户空间驱动。可以在 /dev目录下创建字符设备（如 /dev/spidev0.0），为应用程序提供标准的ioctl接口，使其能够直接配置SPI参数（模式、频率等）并进行数据传输。它是进行SPI硬件原型验证和调试的底层基础设施。
- tools\spi\spidev_test.c：是一个配套的 用户空间测试程序（源码位于 tools/spi/）。它并非内核模块，而是一个使用 spidev驱动接口的示例工具。其作用是演示如何正确调用驱动 API，并方便开发者快速验证 SPI 总线的功能和连通性。

- Documentation\spi\spidev.rst：spidev模块应用说明文档。

### 3.2.2 功能验证

驱动程序加载成功后，在/dev目录会中出现spidevX.X设备节点，示例

```
root@user:/dev# ls /dev/spidev*
/dev/spidev0.0
```

根据设备名称规则 ```/dev/spidev<bus>.<cs>```，```<bus>```是驱动自动选择的总线号，```<cs>``` 是芯片指定引脚的片选信号

自linux内核5.15开始绑定到spidev驱动需要主动bind使/dev目录下设备可用，如bus 0下slave 1：

```
# echo spidev > /sys/class/spi_master/spi0/spi0.1/driver_override
# echo spi0.1 > /sys/bus/spi/drivers/spidev/bind
```

对所有ch34x_mphsi_master驱动管理的设备，可将以下内容保存成shell脚本执行：

```
# for i in /sys/bus/usb/drivers/mphsi-ch34x/*/spi_master/spi*/spi*.*; do echo spidev > $i/driver_override; echo $(basename $i) > /sys/bus/spi/drivers/spidev/bind; done
```

标准I/O函数如 ```open```, ```ioctl``` 和```close``` 可以直接和该spi slave进行通讯，打开SPI设备：

```
int spi = open("/dev/spidev0.0", O_RDWR));
```

设备打开成功后，可以使用 ```ioctl```函数修改SPI配置和传输数据等。

```
uint8_t mode = SPI_MODE_0;
uint8_t lsb = SPI_LSB_FIRST;
...
ioctl(spi, SPI_IOC_WR_MODE, &mode);
ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb);
```

函数 ```ioctl```传输数据示例：

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

## 3.3 匹配I2C设备驱动程序

通过CH347芯片配合该厂商驱动程序，可在Linux系统扩展出一路新的I2C主机(I2C adapter)接口，由于该I2C主机不属于SOC内部资源，所以不能在DTS文件中描述该SPI主机的资源信息，本节介绍一种匹配设备驱动程序的方法，底层原理等同于使用DTS方式。

用户需要在驱动程序文件 ch34x_mphsi_master_i2c.c 中注释如下代码：

```c
#define I2C_AUTOPROBE
//#undef I2C_AUTOPROBE
```

I2C设备驱动中如包含 id_table 成员，则直接修改ch34x_mphsi_master_i2c.c文件，在如下代码中进行匹配调整：

```c
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
	 * If the I2C device driver uses interrupt I/O, there are two cases:
	 * Case 1: Using extended GPIO of CH347 as the interrupt I/O
	 *      The gpio_index variable indicates the hardware GPIO pin index of CH347
	 *      (CH347F: 0-7, corresponding to GPIO0~7; CH347T: 0-2, corresponding to GPIO4/6/7)
	 * Case 2: Using the SoC's GPIO as the interrupt I/O
	 *      The gpio_index variable indicates the corresponding GPIO number.
	 *      Generally, you can view the gpiochip/base+index value in the system's /sys/class/gpio directory
	 * 
	 * Example 1: Using GPIO1 of CH347F as the interrupt I/O
	 * gpio_index = 1;
	 * ch34x_i2c_slaves[0].irq = gpio_to_irq(ch34x_dev->gpio.base + gpio_index);
	 * 
	 * Example 2: Using the SoC's GPIO as the interrupt I/O
	 * gpio_index = 504;
	 * ch34x_i2c_slaves[0].irq = gpio_to_irq(gpio_index);
	 * 
	 * Note: If the I2C device driver does not require interrupt I/O, ignore the above instructions!
	 */
	gpio_index = 1;
	ch34x_i2c_slaves[0].irq = gpio_to_irq(ch34x_dev->gpio.base + gpio_index);

	/* Create and register a new I2C device */
	ch34x_dev->client[0] = i2c_new_device(&ch34x_dev->adapter, &ch34x_i2c_slaves[0]);
	if (!ch34x_dev->client[0]) {
	    dev_err(&ch34x_dev->adapter.dev, "add new i2c client failed\n");
	    return -ENODEV;
	}

    /**
	* If using a 1-master-to-multiple-slave configuration, 
	* you can continue adding the above code to create and register more I2C devices.
	*/
#endif
}
```

如I2C设备驱动程序，不包含 id_table 成员，则需要主动添加匹配参数，并调整 ch34x_mphsi_i2c_probe 的相关名称。

```c
static const struct i2c_device_id xxx_i2c_id[] = {
	{ "ch934x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, xxx_i2c_id);

static struct i2c_driver xxx_i2c_driver = {
    ...
    .id_table = xxx_i2c_id, /* 新增该项 */
    .probe = xxx_i2c_probe,
    .remove = xxx_i2c_remove,
};
```

## 3.4 使用i2c-tools工具

驱动创建I2C Controller完成后，可以使用i2c-tools工具进行总线与设备的调试。

可以参考Blog：https://blog.csdn.net/WCH_TechGroup/article/details/131538476

## 3.5 用户态访问GPIO接口

如果需要在用户空间完成GPIO操作，可以将驱动创建的GPIO Controller默认导出至sysfs用户空间，用户需要在驱动程序文件“ch34x_mphsi_master_gpio.c”中注释如下代码：

```c
#define SYSFS_GPIO
// #undef SYSFS_GPIO
```

用户空间访问GPIO，可以使用```sysfs```，对驱动支持的GPIO，可在如下系统目录下查看。

```bash
/sys/class/gpio/<gpio>/
```

```<gpio>```是定义在驱动变量 ```cH347*_board_config```中的GPIO名称 ，目录包含

- ```value``` 文件用于配置或读取GPIO电平
- ```edge```文件用于配置GPIO中断使能以及中断类型
- ```direction```文件用于改变支持双向GPIO的引脚方向

**注：**对文件的读写操作，用户需要指定的读写权限。

当前支持的中断类型包括：

- ```rising``` 上升沿中断
- ```falling``` 下降沿中断
- ```both``` 双边沿中断

### 3.5.1 打开GPIO

使用GPIO前，需要先打开```value```文件

```c
int  fd;

if ((fd = open("/sys/class/gpio/<gpio>/value", O_RDWR)) == -1) 
{
    perror("open");
    return -1;
}
```

 ```<gpio>```是GPIO的名称

### 3.5.2 设置GPIO方向

配置GPIO方向为input或output，可在root权限下简单地写入```in```或```out```字符串到```direction```文件。

```bash
echo out > /sys/class/gpio/gpio4/direction
```

### 3.5.3 设置GPIO输出

文件```value```打开后，可使用标准I/O函数进行读写，配置GPIO输出电平，可简单使用```write```函数，写入后GPIO会立刻输出指定电平。

```c
if (write(fd, value ? "1" : "0", 1) == -1) 
{
    perror ("write");
	return -1;
}
```

### 3.5.4 读取GPIO电平

读取GPIO电平，可简单使用```read```函数：

```c
char buf;

if (read(fd, &buf, 1) == -1) 
{
    perror("read");
    return -1;
}

value = (buf == '0') ? 0 : 1;
```

每一次读操作后，需要将文件位置指针需要重新定位到首字节。

```c
if (lseek(fd, 0, SEEK_SET) == -1) {
    perror("lseek");
    return -1;
}
```

### 3.5.5 使用GPIO中断

完整的使用GPIO中断功能的驱动例程：

```c
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

## 3.6 注意事项

**CH341支持3种工作模式**

- 模式0: [串口]
- 模式1: [SPI+ I2C + GPIO]
- 模式2: [打印口]

**CH347F支持1种工作模式**

- 模式0: [串口 + SPI + I2C + JTAG +SWD]	

**CH347T支持4种模式**

- 模式0: [串口* 2] VCP/CDC 驱动模式
- 模式1: [SPI + I2C + 串口* 1]  VCP 驱动模式
- 模式2: [SPI + I2C + 串口* 1] HID 驱动模式
- 模式3: [JTAG + 串口* 1]  VCP 驱动模式

**CH347W支持1种工作模式**

- 模式0: [串口 + SPI + I2C + JTAG + ...]	

**该驱动支持 CH347F，CH339W，CH341 的模式1 ，CH347T 的 模式1**



有技术问题，可以发邮件至技术邮箱: tech@wch.cn