# Sensirion SGP30/SGPC3 Linux Kernel Driver

The sgp30 driver for the Sensirion SGP30 and SGPC3 drivers is based on the IIO
subsystem

## Requirements
Kernel with the following config options set, either as module or compiled in.
The config options must be listed in `.config` (e.g. selected with
`make menuconfig`)

Minimal driver requirements:

* `CONFIG_IIO`

With support for triggered buffers:

* `CONFIG_IIO_TRIGGERED_BUFFER`,
* `CONFIG_IIO_SW_TRIGGER`,
* `CONFIG_IIO_HRTIMER_TRIGGER`

## Directory Structure
/:      The root directory contains the Makefile needed for out-of-tree
        module builds.

/sgp30: The sgp30 directory contains the driver source and Kconfig as needed
        for upstream merging with the Linux sources.

## Building

### Local Machine

Compiling for your local machine is useful to test if the driver compiles
correctly. For that we need the kernel headers and build tools.

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

Additionally, when compiled with `CONFIG_IIO_TRIGGERED_BUFFER`:

```bash
sudo modprobe industrialio-triggered-buffer
```

For kernel-triggered timed readings (see Automatic Polling section), also load
the following modules:

```bash
sudo modprobe iio-trig-hrtimer
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

#### Automatic Polling
Automatic polling requires the kernel options `CONFIG_IIO_TRIGGERED_BUFFER`,
`CONFIG_IIO_SW_TRIGGER`, `CONFIG_IIO_HRTIMER_TRIGGER` to be enabled and the
modules to be loaded:

```bash
sudo modprobe industrialio-triggered-buffer
sudo modprobe iio-trig-hrtimer
```

For easier reading, we make use of the following variables:

```bash
sgp_path=/sys/bus/i2c/devices/1-0058/iio:device0
timer_name=sgp-timer
timer_path=/config/iio/triggers/hrtimer/$timer_name
trigger_path=/sys/bus/iio/devices/trigger0
```

##### Attribute Configuration
Attributes that are to be read periodically into the buffer must first be
explicitly enabled. For this we must set the "enable files" by writing to the
corresponding attribute's file with the `_en` suffix.

```bash
echo 1 | sudo tee $sgp_path/scan_elements/in_timestamp_en
echo 1 | sudo tee $sgp_path/scan_elements/in_concentration_voc_en
```

##### Create a New hrtimer Trigger
1. Mount configfs. This step is required to create the software timer.

       ```bash
       sudo mkdir /config
       sudo mount -t configfs none /config
       ```

2. Create the timer, this will automatically create the trigger path.

       ```bash
       sudo mkdir $timer_path
       ```

3. Set the sampling frequency to 1Hz (SGP30), 0.5Hz (SGPC3)

       ```bash
       echo 1 | sudo tee $trigger_path/sampling_frequency
       ```

4. Link the created trigger with the sgp driver

       ```bash
       echo $timer_name | sudo tee $sgp_path/trigger/current_trigger
       ```

5. Set the buffer size

       ```bash
       echo 256 | sudo tee $sgp_path/buffer/length
       ```

6. Enable the buffer

       ```bash
       echo 1 | sudo tee $sgp_path/buffer/enable
       ```

##### Using the iio Util
We can also use the iio tools to link the trigger, set the buffer size and read
out the buffer:

```bash
sudo iio_readdev -t sgp-timer -b 256 iio:device0
```

As an alternative to the iio utils, we can also read out (binary) data manually
from the kernel buffer:

```bash
sudo cat /dev/iio:device0
```

### Unloading
Unload the driver by removing the device instance and then unloading the module.

```bash
echo 0x58 | sudo tee /sys/class/i2c-adapter/i2c-1/delete_device
sudo rmmod sgp30
```

