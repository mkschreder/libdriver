/*
 * This file is part of libdriver
 *
 * Copyright (c) 2019 Martin Schröder
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <libfirmware/driver.h>
#include <libfirmware/spi.h>
#include <libfirmware/gpio.h>
#include <libfirmware/console.h>
#include <libfirmware/mutex.h>
#include <libfirmware/i2c.h>

#include <libfdt/libfdt.h>

#define MCP4XXX_REG_WIPER0 0x00
#define MCP4XXX_REG_WIPER1 0x10
#define MCP4XXX_REG_NV_WIPER0 0x20
#define MCP4XXX_REG_NV_WIPER1 0x30
#define MCP4XXX_REG_TCON0 0x40
#define MCP4XXX_REG_STATUS 0x50
#define MCP4XXX_REG_WIPER2 0x60
#define MCP4XXX_REG_WIPER3 0x70
#define MCP4XXX_REG_NV_WIPER2 0x80
#define MCP4XXX_REG_NV_WIPER3 0x90
#define MCP4XXX_REG_TCON1 0xa0

#define MCP4XXX_OP_READ 0x0C
#define MCP4XXX_OP_WRITE 0x00

static int _mcp4461_probe(void *fdt, int fdt_node){
	i2c_device_t i2c = i2c_find_by_ref(fdt, fdt_node, "i2c");
	uint8_t addr = (uint8_t)fdt_get_int_or_default(fdt, fdt_node, "reg", 0x2d);
	uint8_t data[2];

	if(!i2c){
		printk("mcp4461: invalid i2c\n");
		return -1;
	}

	i2c_read_buf(i2c, addr, MCP4XXX_REG_STATUS | MCP4XXX_OP_READ, data, 2);

	if(data[0] != 0x01 || data[1] != 0x82){
		printk("mcp4461: invalid status\n");
		return -1;
	}

	printk("mcp4461: ready (addr %02x)\n", addr);

	data[0] = 0xff;
	i2c_write_buf(i2c, addr, MCP4XXX_REG_WIPER2 | MCP4XXX_OP_WRITE, data, 1);
	i2c_write_buf(i2c, addr, MCP4XXX_REG_WIPER3 | MCP4XXX_OP_WRITE, data, 1);

	return 0;
}

static int _mcp4461_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(mcp4461, "fw,mcp4461", _mcp4461_probe, _mcp4461_remove)

