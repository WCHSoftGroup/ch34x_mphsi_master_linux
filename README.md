# ch347/ch341 linux USB to SPI/I2C/GPIO Master Driver

##  概述

该Master驱动支持在Linux和安卓主机上，使用USB转串口/JTAG/SPI/I2C/GPIO转换芯片CH347F/T，和USB转串口/SPI/I2C/GPIO转换芯片CH341A/B/C/F/H/T。

驱动仅支持SPI/I2C/GPIO接口，该文档主要介绍CH347F/T芯片的相关特性。

### CH347F SPI接口

| PIN脚 | SPI功能脚 | GPIO复用脚 |
| :---: | :-------: | :--------: |
|  13   |   SCS0    |     -      |
|   7   |   SCS1    |     -      |
|  14   |    SCK    |     -      |
|  16   |   MOSI    |     -      |
|  15   |   MISO    |     -      |

### CH347T SPI接口

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

### CH347F I2C接口

| PIN脚 | I2C功能脚 | GPIO复用脚 |
| :---: | :-------: | :--------: |
|  11   |    SCL    |     -      |
|  12   |    SDA    |     -      |

### CH347T I2C接口

| PIN脚 | I2C功能脚 | GPIO复用脚 |
| :---: | :-------: | :--------: |
|  11   |    SCL    |   gpio3    |
|  12   |    SDA    |     -      |

CH347F/T支持I2C时钟：20kHz，100kHz，400kHz和750kHz等。驱动会默认将I2C的时钟初始化为100KHz，可以在ch34x_mphsi_i2c_init函数中修改。

在Linux上增加对器件的驱动支持十分方便，只需要将该器件的设备驱动绑定到此Master驱动生成的总线下即可。举例：

```
modprobe bmi160_i2c
echo "bmi160 0x68" > /sys/bus/i2c/devices/i2c-$DEV/new_device
```

或

```
modprobe tcs3472
echo "tcs3472 0x29" > /sys/bus/i2c/devices/i2c-$DEV/new_device
```

驱动创建的I2C设备文件在/sys/bus/i2c/devices/i2c-$DEV/ 目录下

###  CH347F GPIO接口

| PIN脚 | GPIO复用脚 |
| :---: | :--------: |
|  17   |   gpio0    |
|  18   |   gpio1    |
|  10   |   gpio2    |
|   9   |   gpio3    |
|  23   |   gpio4    |
|  24   |   gpio5    |
|  25   |   gpio6    |
|  26   |   gpio7    |

CH347F的硬件接口支持GPIO0~GPIO7，引脚未与SPI和I2C复用，此驱动开放支持了所有GPIO。

###  CH347T GPIO接口

| PIN脚 | GPIO复用脚 |
| :---: | :--------: |
|  15   |   gpio4    |
|   2   |   gpio6    |
|  13   |   gpio7    |

CH347T的硬件接口支持GPIO0~GPIO7，考虑到部分引脚被SPI和I2C的接口占用了，此驱动仅开放支持了GPIO4，GPIO6和GPIO7。

### 驱动操作说明

- 使用“make”或者其他方式编译此驱动，如果动态编译成功会生成“ch34x_mphsi_master.ko”驱动模块
- 使用“sudo make load”或“sudo insmod ch34x_mphsi_master.ko”动态加载驱动，使用此方式加载SPI总线号和GPIO起始序号会自动分配，也可以通过增加参数进行指定。
- 如：“sudo insmod ch34x_mphsi_master.ko spi_bus_num=3 gpio_base_num=60”
- 使用“sudo make unload”或“sudo rmmod ch34x_mphsi_master.ko”卸载驱动
- 使用“sudo make install”将驱动开机自动工作
- 使用“sudo make uninstall”卸载该驱动

使用此驱动，需要确认USB设备已经插入主机并且工作正常，可以使用“lsusb”或“dmesg”指令来确定，设备的厂商VID是0x1A86。

如果芯片工作正常，可以使用“ls /sys/class/master”，“ls /sys/class/gpio”指令确认设备节点路径。

## 用户空间访问

### 使用SPI接口

一旦驱动加载成功，默认会提供2个关联到这个新的SPI Bus的SPI Slave设备，以CH347F/T为例：

```
/dev/spidev0.0
/dev/spidev0.1
```

根据设备名称规则 ```/dev/spidev<bus>.<cs>```，```<bus>```是驱动自动选择的总线号，```<cs>``` 是芯片指定引脚的片选信号。

自linux内核5.15开始绑定到spidev驱动需要主动bind使/dev目录下设备可用，如bus 0下slave 1：

```
# echo spidev > /sys/class/spi_master/spi0/spi0.1/driver_override
# echo spi0.1 > /sys/bus/spi/drivers/spidev/bind
```

对所有ch34x_mphsi_master驱动管理的设备：

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

### 挂载SPI NOR FLASH作为MTD存储设备

举例：flash器件挂载到bus 0 chip 0（spi0.0）

```
# echo spi0.0 > /sys/bus/spi/drivers/spidev/unbind
# echo spi-nor > /sys/bus/spi/devices/spi0.0/driver_override
# echo spi0.0 > /sys/bus/spi/drivers/spi-nor/bind
```

**注：**为方便用户使用，该驱动默认会创建spidev设备，用户可以使用上面的命令主动解绑与spidev的绑定，或者undefine在ch34x_mphsi_master_spi.c文件中的“SPIDEV”宏定义。

### 使用GPIO接口

用户空间访问GPIO，可以使用```sysfs```，对驱动支持的GPIO，可在如下系统目录下查看。

```
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

#### 打开GPIO

使用GPIO前，需要先打开```value```文件

```
int  fd;

if ((fd = open("/sys/class/gpio/<gpio>/value", O_RDWR)) == -1) 
{
    perror("open");
    return -1;
}
```

 ```<gpio>```是GPIO的名称

#### 设置GPIO方向

配置GPIO方向为input或output，可在root权限下简单地写入```in```或```out```字符串到```direction```文件。

```
echo out > /sys/class/gpio/gpio4/direction
```

#### 设置GPIO输出

文件```value```打开后，可使用标准I/O函数进行读写，配置GPIO输出电平，可简单使用```write```函数，写入后GPIO会立刻输出指定电平。

```
if (write(fd, value ? "1" : "0", 1) == -1) 
{
    perror ("write");
	return -1;
}
```

#### 读取GPIO电平

读取GPIO电平，可简单使用```read```函数：

```
char buf;

if (read(fd, &buf, 1) == -1) 
{
    perror("read");
    return -1;
}

value = (buf == '0') ? 0 : 1;
```

每一次读操作后，需要将文件位置指针需要重新定位到首字节。

```
if (lseek(fd, 0, SEEK_SET) == -1) {
    perror("lseek");
    return -1;
}
```

#### 使用GPIO中断

完整的使用GPIO中断功能的驱动例程：

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

**注：**该驱动默认会创建gpio设备，若需要在内核中使用中断功能，需要undefine在ch34x_mphsi_master_gpio.c中定义的“SYSFS_GPIO”宏。

## 注

​	**CH341支持3种工作模式**

​	模式0: [串口]

​	模式1: [SPI+ I2C + GPIO]

​	模式2: [打印口]

​	**CH347F支持1种工作模式**

​	模式0: [串口 + SPI + I2C + JTAG +SWD]	

​	**CH347T支持4种模式**

​	模式0: [串口* 2] VCP/CDC 驱动模式

​	模式1: [SPI + I2C + 串口* 1]  VCP 驱动模式

​	模式2: [SPI + I2C + 串口* 1] HID 驱动模式

​	模式3: [JTAG + 串口* 1]  VCP 驱动模式

​	该驱动支持 **CH347F**，**CH341 的模式1** ，**CH347T 的 模式1**

​	有技术问题，可以发邮件至技术邮箱: tech@wch.cn
