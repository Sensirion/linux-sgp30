export KERNELDIR ?= linux
export ARCH ?= $(shell uname -m)
# export ARCH = arm
# export CROSS_COMPILE ?= ~/opt/toolchain/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
export CONFIG_IIO_SENSIRION_SGP="m"

MODNAME = sgpxx
CONFIG_CRC8 ?= $(CONFIG_IIO_SENSIRION_SGP)

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
	make modules_prepare; \
	cd -

clean:
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) $@; rm -f module.order Module.symvers

$(MODSRC): $($(MODSRC))
$(MODOBJ): $(MODSRC)

modules: $(MODOBJ)
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) src=$(PWD)/$(MODNAME) ARCH=$(ARCH) CROSS_COMPILE="$(CROSS_COMPILE)" $@

check:
	$(KERNELDIR)/scripts/checkpatch.pl --no-tree -f $($(MODSRC))

reload:
	lsmod | grep $(MODNAME) && sudo rmmod $(MODNAME); sudo insmod $(MODNAME).ko
