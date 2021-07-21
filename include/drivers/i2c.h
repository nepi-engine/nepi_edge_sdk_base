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
