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
#ifndef __LTC1669_DAC_H
#define __LTC1669_DAC_H

#include <stdint.h>
#include <drivers/i2c.h>

typedef struct
{
  i2c_struct_t i2c;
  uint8_t addr_7_bit = 0x0;
  uint16_t curr_dac_val = 0x0;
  uint8_t initialized = 0;
} ltc1669_dac_struct_t;

int ltc1669_dac_init(ltc1669_dac_struct_t *dac, uint32_t i2c_bus_id, uint8_t addr_7_bit);
int ltc1669_dac_write(ltc1669_dac_struct_t* dac, uint16_t val);
inline uint16_t ltc1669_dac_get_current_val(const ltc1669_dac_struct_t* const dac){return dac->curr_dac_val;}

#endif
