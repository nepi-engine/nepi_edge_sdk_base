/*
 * NEPI Dual-Use License
 * Project: nepi_edge_sdk_base
 *
 * This license applies to any user of NEPI Engine software
 *
 * Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
 * see https://github.com/numurus-nepi/nepi_edge_sdk_base
 *
 * This software is dual-licensed under the terms of either a NEPI software developer license
 * or a NEPI software commercial license.
 *
 * The terms of both the NEPI software developer and commercial licenses
 * can be found at: www.numurus.com/licensing-nepi-engine
 *
 * Redistributions in source code must retain this top-level comment block.
 * Plagiarizing this software to sidestep the license obligations is illegal.
 *
 * Contact Information:
 * ====================
 * - https://www.numurus.com/licensing-nepi-engine
 * - mailto:nepi@numurus.com
 *
 */
#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>

typedef struct
{
  int fd = 0;
} i2c_struct_t;

int i2c_init( i2c_struct_t *i2c, uint32_t bus_id );
int i2c_write( const i2c_struct_t* const i2c, uint8_t addr, uint8_t *data, int len );
int i2c_read( const i2c_struct_t* const i2c, uint8_t addr, uint8_t *data, int len );

#endif
