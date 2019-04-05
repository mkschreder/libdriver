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
#include <libfirmware/analog.h>
#include <libfirmware/math.h>

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

struct mcp4461 {
	i2c_device_t i2c;
	gpio_device_t gpio;
	uint8_t addr;
	uint32_t reset_pin, wp_pin;
	struct analog_device dev;
};

static int _mcp4461_analog_write(analog_device_t dev, unsigned int chan, float value){
	struct mcp4461 *self = container_of(dev, struct mcp4461, dev.ops);

	value = constrain_float(value, 0, 1.0f);
	uint16_t val = (uint16_t)((float)((uint16_t)0x3ff)*value) & 0x3ff;

	uint8_t reg = 0;
	switch(chan){
		case 0: reg = MCP4XXX_REG_WIPER0; break;
		case 1: reg = MCP4XXX_REG_WIPER1; break;
		case 2: reg = MCP4XXX_REG_WIPER2; break;
		case 3: reg = MCP4XXX_REG_WIPER3; break;
		case 4: reg = MCP4XXX_REG_NV_WIPER0; break;
		case 5: reg = MCP4XXX_REG_NV_WIPER1; break;
		case 6: reg = MCP4XXX_REG_NV_WIPER2; break;
		case 7: reg = MCP4XXX_REG_NV_WIPER3; break;
		default: return -EINVAL;
	}

	int ret = 0;
	if((ret = i2c_write8_reg8(self->i2c, self->addr, reg | MCP4XXX_OP_WRITE | (uint8_t)(val >> 8), (uint8_t)(val & 0xff))) < 0) return ret;

	return 0;
}

static int _mcp4461_analog_read(analog_device_t dev, unsigned int chan, float *value){
	return -1;
}

static struct analog_device_ops _analog_ops = {
	.write = _mcp4461_analog_write,
	.read = _mcp4461_analog_read
};

static int _mcp4461_probe(void *fdt, int fdt_node){
	i2c_device_t i2c = i2c_find_by_ref(fdt, fdt_node, "i2c");
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	uint8_t addr = (uint8_t)fdt_get_int_or_default(fdt, fdt_node, "reg", 0x2d);
	int reset_pin = fdt_get_int_or_default(fdt, fdt_node, "reset_pin", -1);
	int wp_pin = fdt_get_int_or_default(fdt, fdt_node, "wp_pin", -1);
	uint8_t data[2];

	if(!i2c){
		printk("mcp4461: invalid i2c\n");
		return -EINVAL;
	}

	if(gpio && (reset_pin < 0 || wp_pin < 0)){
		printk("mcp4461: gpio requires wp_pin and reset_pin fields\n");
		return -EINVAL;
	}

	struct mcp4461 *self = kzmalloc(sizeof(struct mcp4461));
	self->i2c = i2c;
	self->gpio = gpio;
	self->addr = addr;
	
	if(self->gpio){
		self->reset_pin = (uint32_t)reset_pin;
		self->wp_pin = (uint32_t)wp_pin;

		gpio_set(self->gpio, self->wp_pin);
		gpio_set(self->gpio, self->reset_pin);
		thread_sleep_ms(1);
		gpio_reset(self->gpio, self->reset_pin);
		thread_sleep_ms(1);
		gpio_set(self->gpio, self->reset_pin);
		thread_sleep_ms(1);
	}

	int ret = 0;
	if((ret = i2c_read8_buf(i2c, addr, MCP4XXX_REG_STATUS | MCP4XXX_OP_READ, data, 2)) < 0){
		printk("mcp4461: i2c error\n");
		return -EIO;
	}

	if(data[0] != 0x01 || data[1] != 0x82){
		printk("mcp4461: invalid status\n");
		return -1;
	}

	analog_device_init(&self->dev, fdt, fdt_node, &_analog_ops);
	analog_device_register(&self->dev);

	printk("mcp4461: ready (addr %02x)\n", addr);

	return 0;
}

static int _mcp4461_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(mcp4461, "fw,mcp4461", _mcp4461_probe, _mcp4461_remove)

