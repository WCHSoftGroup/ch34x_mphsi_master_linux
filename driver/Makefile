CONFIG_MODULE_SIG=n

DRIVERNAME := ch34x-usb
all : modules

ifneq ($(KERNELRELEASE),)
obj-m += $(DRIVERNAME).o
$(DRIVERNAME)-y := ch34x_mphsi_usb.o ch34x_mphsi_spi.o ch34x_mphsi_i2c.o ch34x_mphsi_gpio.o
else
KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	rm -rf *.mk .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd Module.markers modules.order *.a *.mod
load:
	insmod $(DRIVERNAME).ko
unload:
	rmmod $(DRIVERNAME)
install : modules
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/usb/misc
	cp -f ./$(DRIVERNAME).ko /lib/modules/$(shell uname -r)/kernel/drivers/usb/misc/
	depmod -a
uninstall:
	rm -f /lib/modules/$(shell uname -r)/kernel/drivers/usb/misc/$(DRIVERNAME).ko
	depmod -a
endif
