User Space Driver SGP30
=======================

Supported chips:
    * Sensirion SGP30 (https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Sensirion_Gas_Sensors_SGP30_Datasheet_EN.pdf)

Author:
  Pascal Sachs <pascal.sachs@sensirion.com>


Description
-----------
This driver is a sample implementation of an SGP30 user space driver that
performs Measure_test and Init_air_quality on initialization.

Afterwards the sgp30_read_values can be exectued once per second to read out
tvoc and co2 equivalent from the chip.


Interface
---------
The driver API is defined and documented in sgp30_userspace.h. To instantiate
the driver, you need to know the sensor address and to which bus it is
connected.

An example of how to use the driver can be found in sgp30_test.c.

You have to provide the i2c adapter number when executing sgp30_test.
The adapter number is e.g. 0 if the SGP30 is on /dev/i2c-0

