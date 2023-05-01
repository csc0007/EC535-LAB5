obj-m += door_v5d.o

KDIR := /home/yyguo/EC535/lab4/stock-linux-4.19.82-ti-rt-r33/
CROSS_COMPILE := arm-linux-gnueabihf-
ARCH := arm

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

