Please use the following steps to integrate H3LIS100DL driver into your kernel:

- copy the driver source code in the kernel:
	cp h3lis100dl.c <KERNEL_SRC_ROOT>/drivers/input/misc/
	cp h3lis100dl.h <KERNEL_SRC_ROOT>/include/linux/input/

- add following entry to <KERNEL_SRC_ROOT>/drivers/input/misc/Kconfig:

config INPUT_LSM9DS1
	tristate "STMicroelectronics LSM9DS1"
	depends on INPUT
	help
	Say Y here to enable STMicroelectronics H3LIS100DL inertial module.

- edit the <KERNEL_SRC_ROOT>/drivers/input/misc/Makefile to add object file:
	obj-$(CONFIG_INPUT_H3LIS100DL) += h3lis100dl.o

To enable this driver you have to make the needed changes either in the board file
or into device tree.  The i2c binding for device tree is shown below:

	h3lis100dl@19 {
		compatible = "st,h3lis100dl";
		reg = <0x19>;
		poll-interval = <100>;
	};

