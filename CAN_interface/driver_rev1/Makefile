# Makefile for compiling Kernel
# modules on the fly.

obj-m += rpi_can.o
rpi_can-objs := can_main.o can_fifo.o can_gpio.o can_parse.o can_sja1000.o can_filter.o

ccflags-y := -I$(src)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/canbus
	sudo cp rpi_can.ko /lib/modules/$(shell uname -r)/kernel/drivers/canbus/rpi_can_rev1.ko

uninstall:
	@-sudo rm /lib/modules/$(shell uname -r)/kernel/drivers/canbus/rpi_can_rev1.ko

