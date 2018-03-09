# Sensirion SGP30/SGPC3 Linux Kernel Driver

The sgp30 driver for the Sensirion SGP30 and SGPC3 drivers is based on the IIO
subsystem

## Requirements
Kernel with the following config options set, either as module or compiled in.
The config options must be listed in `.config` (e.g. selected with
`make menuconfig`)

Minimal driver requirements:

* `CONFIG_IIO`

## Directory Structure
/:      The root directory contains the Makefile needed for out-of-tree
        module builds.

/sgp30: The sgp30 directory contains the driver source and Kconfig as needed
        for upstream merging with the Linux sources.

## Building

### Local Machine

Compiling for your local machine is useful to test if the driver compiles
correctly. Prerequisites are the kernel headers and build tools.

```bash
# Install dependencies
sudo apt install build-essential linux-headers-$(uname -r)
# configure
export KERNELDIR=/lib/modules/$(uname -r)/build
# build
make -j
```
To check for style issues we can use the `check` target

```bash
make check
```

### Cross Compiling

To cross compile, one needs to install a cross compilation tool chain and set
the `ARCH` and `CROSS_COMPILE` environment variables accordingly:

```bash
# Example to cross compile for raspbian
export ARCH=arm
export CROSS_COMPILE=~/opt/toolchain/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
```

## Usage
The driver is meant for direct use with sysfs or libiio.
Errors are printed to the kernel log (dmesg)

### Loading the Kernel Module
Load the dependencies of the sgp30.ko kernel module:

```bash
sudo modprobe industrialio
sudo modprobe crc8
```

Load the kernel module with the appropriate command:

* Out-of-tree build

      ```bash
      sudo insmod sgp30.ko
      ```

* In-kernel build

      ```bash
      sudo modprobe sgp30
      ```

### Instantiation
Instantiate the driver on the correct i2c bus with

```bash
echo sgp30 0x58 | sudo tee /sys/class/i2c-adapter/i2c-1/new_device
```

Only `sgp30` and `sgpc3` are permissible names. Use of the name of the wrong
chip will result in an error during probing.

### Operation
Query device files by reading from the iio subsytem's device:

```bash
cat /sys/bus/iio/devices/iio\:device0/in_selftest
OK
```

or alternatively on the i2c bus: `/sys/bus/i2c/devices/1-0058/iio:device0/`

### Unloading
Unload the driver by removing the device instance and then unloading the module.

```bash
echo 0x58 | sudo tee /sys/class/i2c-adapter/i2c-1/delete_device
sudo rmmod sgp30
```

