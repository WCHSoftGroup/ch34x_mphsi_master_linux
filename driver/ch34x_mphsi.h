#ifndef _CH34X_MPHSI_H
#define _CH34X_MPHSI_H

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/idr.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/list.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
#include <linux/gpio/machine.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#endif

#define CH34X_USBDEV (&(ch34x_dev->intf->dev))
#define DEV_ERR(d, f, ...) \
	dev_err(d, "%s: " f "\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_DBG(d, f, ...) \
	dev_dbg(d, "%s: " f "\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_INFO(d, f, ...) \
	dev_info(d, "%s: " f "\n", __FUNCTION__, ##__VA_ARGS__)

#define DRIVER_AUTHOR "WCH"
#define DRIVER_ALIAS "SPI/I2C/GPIO: CH347/CH341/CH339/CH9341"
#define DRIVER_DESC \
	"USB to SPI/I2C/GPIO controller driver for CH347/CH341/CH339/CH9341, etc."
#define VERSION_DESC "V1.7 On 2026.09"

/* Check for condition and return with or without err code if it fails */
#define CHECK_PARAM_RET(cond, err) \
	if (!(cond))               \
		return err;
#define CHECK_PARAM(cond) \
	if (!(cond))      \
		return;

#define MAX_BUFFER_LENGTH (64 * 1024)

#define CH34X_SPI_MAX_NUM_DEVICES 8
#define CH34X_I2C_MAX_NUM_DEVICES 16
#define CH34X_MAX_GPIOS 64
#define CH34X_SPI_MAX_XFER_SIZE (64 * 1024 - 16 * USB20_CMD_HEADER)

#define CH341_CS_NUM 3

#define CH341_USB_MAX_BULK_SIZE 32
#define CH341_USB_MAX_INTR_SIZE 8

#define CH341_CMD_SPI_STREAM 0xA8 /* CH341 SPI command */
#define CH341_CMD_UIO_STREAM 0xAB /* UIO command */

#define CH341_SPI_MAX_NUM_DEVICES 3
#define CH341_SPI_BUS_NUM 0
#define CH341_SPI_MODE SPI_MODE_0
#define CH341_SPI_MIN_FREQ 400
#define CH341_SPI_MAX_FREQ 1e6
#define CH341_SPI_MIN_BITS_PER_WORD 4
#define CH341_SPI_MAX_BITS_PER_WORD 32

#define CH341_CMD_SPI_STREAM 0xA8
#define CH341_CMD_UIO_STREAM 0xAB
#define CH341_CMD_UIO_STM_IN 0x00
#define CH341_CMD_UIO_STM_OUT 0x80
#define CH341_CMD_UIO_STM_DIR 0x40
#define CH341_CMD_UIO_STM_END 0x20
#define CH341_CMD_UIO_STM_US 0xC0

#define CH341_CMD_I2C_STREAM 0xAA
#define CH341_CMD_I2C_STM_STA 0x74
#define CH341_CMD_I2C_STM_STO 0x75
#define CH341_CMD_I2C_STM_OUT 0x80
#define CH341_CMD_I2C_STM_IN 0xC0
#define CH341_CMD_I2C_STM_SET 0x60
#define CH341_CMD_I2C_STM_END 0x00

#define CH347_CS_NUM 2
#define CH347_SPI_MAX_BULK_SIZE (4096 - USB20_CMD_HEADER)
#define CH347_USB_BULK_EPSIZE 512
#define CH347_USB_MAX_INTR_SIZE 512

#define CH347_MAX_GPIOS 8
#define CH347F_MPHSI_GPIOS 8
#define CH347T_MPHSI_GPIOS 3

#define CH347_SPI_MAX_NUM_DEVICES 2
#define CH347_SPI_MIN_FREQ 218750
#define CH347_SPI_MAX_FREQ 60e6

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define CH347_GPIO_CNT 8

#define SCK_BIT (1 << 3)
#define MOSI_BIT (1 << 5)
#define MISO_BIT (1 << 7)

#define SET_CS 0
#define CLR_CS 1
#define SPI_CS_ACTIVE 0x00
#define SPI_CS_DEACTIVE 0x01

#define SPI_CPOL_Low ((u16)0x0000)
#define SPI_CPOL_High ((u16)0x0002)

#define SPI_CPHA_1Edge ((u16)0x0000)
#define SPI_CPHA_2Edge ((u16)0x0001)

#define USB20_CMD_HEADER 3
#define USB20_CMD_SPI_INIT 0xC0
#define USB20_CMD_SPI_CONTROL 0xC1
#define USB20_CMD_SPI_RD_WR 0xC2
#define USB20_CMD_SPI_BLCK_RD 0xC3
#define USB20_CMD_SPI_BLCK_WR 0xC4
#define USB20_CMD_INFO_RD 0xCA
#define USB20_CMD_SPI_CLK_INIT 0xE1
#define USB20_CMD_FUNC_SWITCH 0xE2

#define CH347_CMD_I2C_STM_MAX 0x3F
#define CH347_CMD_I2C_STRETCH_Y 0x15
#define CH347_CMD_I2C_STRETCH_N 0x16

#define USB20_CMD_GPIO_CMD 0xCC

#define CH339_CS_NUM 1
#define CH339W_MPHSI_GPIOS 3
#define CH339_SPI_MAX_NUM_DEVICES 1

#define CH9341C_MPHSI_GPIOS 10
#define CH9341_SPI_MAX_LENGTH 0x1000
#define CH9341C_MAX_GPIOS 6

#define CH9341_CS_NUM 4
#define CH9341_SPI_MAX_NUM_DEVICES 4
#define CH9341_SPI_MIN_FREQ 750e3
#define CH9341_SPI_MAX_FREQ 24e6

#define CH9341_USB_MAX_BULK_SIZE 64
#define CH9341_USB_MAX_INTR_SIZE 12

#define CH9341_CMD_HEADER 4

#define I2C_SPEED_20K 0
#define I2C_SPEED_50K 4
#define I2C_SPEED_100K 1
#define I2C_SPEED_200K 5
#define I2C_SPEED_400K 2
#define I2C_SPEED_750K 3
#define I2C_SPEED_1M 6
#define I2C_SPEED_2M 7

#define CH9341_CMD_GPIO_FUN_SET 0xCE
#define CH9341_GPIO_CNT 10

#define CH9341_CMD_I2C_INIT 0xDB
#define CH9341_CMD_I2C_STREAM 0xD9

#define CH9341_I2C_MAX_LENGTH 0x0400

#define CH9341_CMD_I2C_NEW_STM_US 0x01
#define CH9341_CMD_I2C_NEW_STM_MS 0x02
#define CH9341_CMD_I2C_NEW_STM_START 0x03
#define CH9341_CMD_I2C_NEW_STM_STOP 0x04
#define CH9341_CMD_I2C_NEW_STM_OUT 0x05
#define CH9341_CMD_I2C_NEW_STM_IN_NACK 0x06
#define CH9341_CMD_I2C_NEW_STM_IN_ACK 0x07
#define CH9341_CMD_I2C_NEW_STM_ADDR 0x08

#define DEFAULT_TIMEOUT 1000
#define CH34X_NW 4

#define MAXCH 3

#ifndef USB_DEVICE_INTERFACE_NUMBER
#define USB_DEVICE_INTERFACE_NUMBER(vend, prod, num)   \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE |    \
		       USB_DEVICE_ID_MATCH_INT_NUMBER, \
	.idVendor = (vend), .idProduct = (prod),       \
	.bInterfaceNumber = (num)
#endif

enum _CHANNEL_TYPE {
	CH_ALL = 0,
	CH_SPI = 0,
	CH_I2C = 1,
	CH_DEF = 2,
};

typedef enum _CHIP_TYPE {
	CHIP_CH341 = 0,
	CHIP_CH347F = 1,
	CHIP_CH347T = 2,
	CHIP_CH339W = 3,
	CHIP_CH9341C,
} CHIP_TYPE;

struct ch341_pin_config {
	u8 pin; /* pin number of CH341 */
	char *name; /* pin name */
};

struct ch34x_pin_config {
	u8 pin; /* pin number of CH347 */
	char *name; /* pin name */
	u8 gpioindex;
	u8 mode;
	bool hwirq;
};

#pragma pack(1)

/* SPI setting structure */
typedef struct _spi_config {
	u8 imode; /* 0-3: SPI Mode0/1/2/3 */
	u8 iclock; /* SPI clock divider value, caculated by driver automatically */
	u8 ibyteorder; /* 0: LSB, 1: MSB */
	u16 ispi_rw_interval; /* SPI read and write interval, unit: us */
	u8 ispi_out_def; /* SPI output data by default while read */
	u16 ics; /* SPI chip select, valid while BIT7 is 1, low byte: CS0, high byte: CS1 */
	u8 cs0_polar; /* BIT0：CS0 polar control, 0：low active, 1：high active */
	u8 cs1_polar; /* BIT0：CS1 polar control, 0：low active, 1：high active */
	u16 iauto_de_cs; /* automatically undo the CS after operation completed */
	u16 iactive_delay; /* delay time of read and write operation after setting CS, unit: us */
	u32 ideactive_delay; /* delay time of read and write operation after canceling CS, unit: us */
} mspi_cfgs, *mpspi_cfgs;

/* SPI Init structure definition */
typedef struct _spi_init_typedef {
	u16 spi_direction; /* Specifies the SPI unidirectional or bidirectional data mode.
                            This parameter can be a value of @ref SPI_data_direction */
	u16 spi_mode; /* Specifies the SPI operating mode.
                            This parameter can be a value of @ref SPI_mode */
	u16 spi_datasize; /* Specifies the SPI data size.
                            This parameter can be a value of @ref SPI_data_size */
	u16 s_spi_cpol; /* Specifies the serial clock steady state.
                            This parameter can be a value of @ref SPI_Clock_Polarity */
	u16 s_spi_cpha; /* Specifies the clock active edge for the bit capture.
                            This parameter can be a value of @ref SPI_Clock_Phase */
	u16 spi_nss; /* Specifies whether the NSS signal is managed by
                            hardware (NSS pin) or by software using the SSI bit.
                            This parameter can be a value of @ref SPI_Slave_Select_management */
	u16 spi_baudrate_scale; /* Specifies the Baud Rate prescaler value which will be
                                used to configure the transmit and receive SCK clock.
                                This parameter can be a value of @ref SPI_BaudRate_Prescaler.
                                @note The communication clock is derived from the master
                                clock. The slave clock does not need to be set. */
	u16 spi_firstbit; /* Specifies whether data transfers start from MSB or LSB bit.
                            This parameter can be a value of @ref SPI_MSB_LSB_transmission */
	u16 spi_crc_poly; /* Specifies the polynomial used for the CRC calculation. */
} spi_init_typedef;

typedef struct _stream_usbcfg {
	spi_init_typedef spi_initcfg;
	u16 spi_rw_interval; /* SPI read and write interval, unit: us */
	u8 spi_outdef; /* SPI output data by default while read */
	u8 misc_cfg; /* misc option
                            BIT7: CS0 polar control, 0：low active, 1：high active
                            BIT6：CS2 polar control, 0：low active, 1：high active
                            BIT5：I2C clock stretch control, 0：disable 1: enable
                            BIT4：generates NACK or not when read the last byte for I2C operation
                            BIT3-0：reserved
                         */
	u8 reserved[4]; /* reserved */
} stream_hw_cfgs, *pstream_hw_cfgs;

typedef struct {
	uint8_t mode; /* SPI interface operating mode setting */
	uint8_t fre_div; /* SPI interface master mode clock divider setting */
	uint8_t fill_data; /* SPI master default transmit bytes */
	uint8_t control; /* Other SPI interface configuration byte */
	uint8_t scs_polar; /* SPI interface chip select pin polarity control */
	uint16_t data_delay; /* SPI communication delay between each two bytes, unit: mircosecond */
	uint16_t cs_data_delay; /* SPI communication delay between chip select activate with data start, unit: mircosecond */
	uint16_t data_cs_delay; /* SPI communication delay between data transmit over with chip select deactivate, unit: mircosecond */
	uint8_t timeout; /* SPI interface slave mode receive timeout setting, unit: 100uS */
	uint8_t reserved[4]; /* Bytes reserved */
} mspi_cfg_ch9341;

typedef struct {
	uint16_t gpio_cfg[10]; /* GPIO function configuration */
	uint16_t reserved[6]; /* Bytes reserved */
	uint8_t filter; /* GPIO delay filter time, in us. This setting applies to all input GPIOs simultaneously. */
	uint8_t reserved2[3]; /* Bytes reserved */
} mgpio_cfg_ch9341;

typedef struct __attribute__((packed)) {
	uint16_t gpio_cfg[10]; /* GPIO pin function configuration */
} mgpio_cfg_recv_ch9341;

#pragma pack()

struct ch34x_wb {
	struct ch34x_device *instance;
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb *urb;
};

/* Device specific structure */
struct ch34x_device {
	struct mutex io_mutex;
	struct mutex ops_mutex;
	struct usb_device *usb_dev; /* usb device */
	struct usb_interface *intf; /* usb interface */

	struct usb_endpoint_descriptor
		*bulk_in[MAXCH]; /* usb endpoint bulk in */
	struct usb_endpoint_descriptor
		*bulk_out[MAXCH]; /* usb endpoint bulk out */
	struct usb_endpoint_descriptor
		*intr_in; /* usb endpoint interrupt in */
	u8 bulk_out_endpointAddr[MAXCH]; /*bulk output endpoint*/

	u8 bulkin_num;
	u8 bulkout_num;

	u8 *bulkin_buf; /* usb bulk in buffer */
	u8 *bulkout_buf; /* usb bulk out buffer */
	u8 *intrin_buf; /* usb interrupt in buffer */
	wait_queue_head_t wq_send; /* for usb write */
	int writesize;
	struct ch34x_wb wb[CH34X_NW];
	int write_used; /* number of non-empty write buffers */
	int transmitting;
	unsigned int susp_count; /* number of suspended interfaces */
	bool disconnecting; /* device is being disconnected */
	spinlock_t write_lock;

	int errors;
	spinlock_t err_lock;

	int windex;

	struct urb *intr_urb;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	struct spi_controller *master;
#else
	struct spi_master *master;
#endif
	struct spi_device *slaves[CH34X_SPI_MAX_NUM_DEVICES];
	int slave_num;
	bool last_cpol; /* last message CPOXL */
	int cur_speed_hz;

	u8 gpio_mask; /* configuratoin mask defines IN/OUT pins */
	u8 gpio_io_data; /* current value of ch341 I/O register */

	CHIP_TYPE chiptype;
	u16 firmver;
	stream_hw_cfgs hwcfg;
	mspi_cfgs spicfg;

	mspi_cfg_ch9341 spicfg_ch9341;

	int id;
	struct platform_device *spi_pdev;

	struct gpio_chip gpio;
	u8 gpio_num;

	struct ch34x_pin_config *gpio_pins[CH34X_MAX_GPIOS];
	char *gpio_names[CH34X_MAX_GPIOS]; /* pin names (gpio_num elements) */
	int gpio_irq_map
		[CH34X_MAX_GPIOS]; /* GPIO to IRQ map (gpio_num elements) */

	/* IRQ device description */
	struct irq_chip irq; /* chip descriptor for IRQs */
	u8 irq_num; /* number of pins with IRQs */
	int irq_base; /* base IRQ allocated */
	int irq_types[CH34X_MAX_GPIOS]; /* IRQ types (irq_num elements) */
	bool irq_enabled
		[CH34X_MAX_GPIOS]; /* IRQ enabled flag (irq_num elements) */
	int irq_gpio_map
		[CH34X_MAX_GPIOS]; /* IRQ to GPIO pin map (irq_num elements) */
	u8 irq_index;
	raw_spinlock_t irq_lock;

	struct delayed_work work;

	/* I2C controller */
	struct i2c_adapter adapter;
	struct i2c_client *client[CH34X_I2C_MAX_NUM_DEVICES];
	bool i2c_init;
};

#endif
