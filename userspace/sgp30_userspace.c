/*
 * Copyright (c) 2017, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "sgp30_userspace.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* commands */
static const unsigned char SGP30_CMD_IAQ_INIT[]     = { 0x20, 0x03 };
static const unsigned char SGP30_CMD_IAQ_MEASURE[]  = { 0x20, 0x08 };
static const unsigned char SGP30_CMD_SELFTEST[] = { 0x20, 0x32 };
static const unsigned char SGP30_SELFTEST_OK[] = { 0x00, 0xd4 }
static const size_t SGP30_COMMAND_LENGTH = 2;

static const size_t SGP30_FILENAME_LENGTH = 20;
static const size_t SGP30_REGISTER_LENGTH = 3;
static const size_t SGP30_RESPONSE_LENGTH = 6;
static const useconds_t SGP30_MEASURE_WAIT_TIME = 12000;
static const useconds_t SGP30_SELFTEST_WAIT_TIME = 220000;

int sgp30_init(int adapterNumber, int address) {
    int handle, status;
    char fileName[SGP30_FILENAME_LENGTH];
    uint8_t statusRegister[SGP30_REGISTER_LENGTH];

    snprintf(fileName, SGP30_FILENAME_LENGTH - 1, "/dev/i2c-%d", adapterNumber);
    handle = open(fileName, O_RDWR);
    if (handle > 0) {
        status = ioctl(handle, I2C_SLAVE, address);
        if (status < 0) {
            perror("could not open i2c device");
            return status;
        }
    }
    // probe sensor by reading the status register
    if (write(handle, SGP30_CMD_SELFTEST, SGP30_COMMAND_LENGTH) !=
        (int)SGP30_COMMAND_LENGTH) {
        perror("selftest write failed");
        return -EIO;
    }

    // wait for selftest to finish
    usleep(SGP30_SELFTEST_WAIT_TIME);

    // read status register
    if (read(handle, statusRegister, SGP30_REGISTER_LENGTH) !=
        (int)SGP30_REGISTER_LENGTH) {
        perror("selftest read failed");
        return -EIO;
    }

    // check if selftest worked (result is big endian)
    if (statusRegister[0] != SGP30_SELFTEST_OK[1] ||
        statusRegister[1] != SGP30_SELFTEST_OK[0]) {
        perror("register failed");
        return -EIO;
    }

    // Init IAQ measurements
    if (write(handle, SGP30_CMD_IAQ_INIT, SGP30_COMMAND_LENGTH) !=
        (int)SGP30_COMMAND_LENGTH) {
        return -EIO;
    }

    // wait for IAQ init to finish
    usleep(SGP30_MEASURE_WAIT_TIME);

    return handle;
}

void sgp30_close(int handle) {
    close(handle);
}

int sgp30_read_values(int handle, float* tvoc, float* co2_eq) {
    uint8_t buffer[SGP30_RESPONSE_LENGTH];
    if (write(handle, SGP30_CMD_IAQ_MEASURE, SGP30_COMMAND_LENGTH) !=
        (int)SGP30_COMMAND_LENGTH) {
        return -EIO;
    }

    usleep(SGP30_MEASURE_WAIT_TIME);

    if (read(handle, buffer, SGP30_RESPONSE_LENGTH) !=
        (int)SGP30_RESPONSE_LENGTH) {
        return -EIO;
    }

    if (co2_eq != NULL) {
        *co2_eq = (float)((buffer[0] << 8) | buffer[1]);
    }

    if (tvoc != NULL) {
        *tvoc = (float)((buffer[3] << 8) | buffer[4]);
    }

    return 0;
}
