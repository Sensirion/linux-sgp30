export KERNELDIR ?= linux
export ARCH ?= $(shell uname -m)
# export ARCH = arm
# export CROSS_COMPILE ?= ~/opt/toolchain/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
export CONFIG_SENSIRION_SGPXX="m"

MODNAME = sgpxx
CONFIG_CRC8 ?= $(CONFIG_SENSIRION_SGPXX)
CONFIG_IIO ?= $(CONFIG_SENSIRION_SGPXX)
CONFIG_IIO_BUFFER ?= "y"
CONFIG_IIO_TRIGGERED_BUFFER ?= $(CONFIG_SENSIRION_SGPXX)

MODSRC = $(MODNAME)_src
MODOBJ = $(MODNAME)_obj
$(MODSRC) = $(MODNAME)/$(MODNAME).c
$(MODOBJ) = $(MODNAME)/$(MODNAME).ko

.PHONY: check prepare reload

all: modules

sensirion-sgp/Makefile:
	touch $@

prepare:
	cd $(KERNELDIR) && \
	echo "CONFIG_CRC8=$(CONFIG_CRC8)" >> .config && \
	echo "CONFIG_IIO=$(CONFIG_IIO)" >> .config && \
	echo "CONFIG_IIO_BUFFER=$(CONFIG_IIO_BUFFER)" >> .config && \
	echo "CONFIG_IIO_TRIGGERED_BUFFER=$(CONFIG_IIO_TRIGGERED_BUFFER)" >> .config && \
	make modules_prepare; \
	cd -

clean:
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) $@; rm -f module.order Module.symvers

$(MODSRC): $($(MODSRC))
$(MODOBJ): $(MODSRC)

modules: $(MODOBJ)
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) src=$(PWD)/$(MODNAME) ARCH=$(ARCH) CROSS_COMPILE="$(CROSS_COMPILE)" $@

check:
	# Remove lines with checkpatch warnings: datasheet links and LINUX_VERSION
	sed -e '/https/d;/LINUX_VERSION/d' $($(MODSRC)) | $(KERNELDIR)/scripts/checkpatch.pl --no-tree -f -

reload:
	lsmod | grep $(MODNAME) && sudo rmmod $(MODNAME); sudo insmod $(MODNAME).ko
