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

#ifndef SGP30_USERSPACE_H_
#define SGP30_USERSPACE_H_

/**
 * Initialize the SGP30 userspace driver and checks if a sensor is connected.
 * Must be called before any other sgp30_* function.
 *
 * If an error occurs, errno will be set when this function returns.
 * You can use perror() to print the error message.
 *
 * @param adapterNumber the number of the i2c adapter to which the SGP30
 *                      is connected. Run 'i2cdetect -l' to list all available
 *                      adapters.
 * @param address       the i2c address of the SGP30 0X58.
 * @return On success, a (positive) handle, which must be passed to the other
 *         sgp30_* functions. On failure, a negative error code.
 */
int sgp30_init(int adapterNumber, int address);

/**
 * Close the SGP30 userspace driver.
 *
 * @param handle Handle that was returned by sgp30_init()
 */
void sgp30_close(int handle);

/**
 * Start a measurement and read out the measured values.
 *
 * If an error occurs, errno will be set when this function returns.
 * You can use perror() to print the error message.
 *
 * @param handle handle that was returned by sgp30_init()
 * @param tvoc pointer to the variable to hold the measured tvoc
 * @param co2_eq pointer to the variable to hold the measured co2 equivalent
 * @return 0 on success, negative error code otherwise
 */
int sgp30_read_values(int handle, float* tvoc, float* co2_eq);

#endif
