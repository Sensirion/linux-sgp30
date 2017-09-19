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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static const int SGP30_ADDRESS = 0x58;

/**
 * Example program using the userspace SGP30 driver.
 *
 * Performs the following steps:
 * - Initialize the driver, handling any errors
 * - Perform one measurement
 * - Print out the results
 * - Close the driver
 */
int main(int argc, const char* argv[]) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s <adapterNumber>\n", argv[0]);
        exit(1);
    }
    int adapterNumber = atoi(argv[1]);
    printf("using adapter %d\n", adapterNumber);

    int handle = sgp30_init(adapterNumber, SGP30_ADDRESS);
    if (handle < 0) {
        perror("could not open i2c device");
        exit(2);
    }

    int ix, status;
    float tvoc, co2_eq;

    for (ix = 0; ix < 100; ix++) {
        status = sgp30_read_values(handle, &tvoc, &co2_eq);
        if (status == 0) {
            printf("tvoc: %f, co2_eq: %f\n", tvoc, co2_eq);
        } else {
            perror("could not measure");
            exit(3);
        }

        // sleep for 1 second
        usleep(1000000);
    }
    sgp30_close(handle);
    return 0;
}
