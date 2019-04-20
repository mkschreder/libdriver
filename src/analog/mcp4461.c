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

#define MCP4461_TIMEOUT 1000

struct mcp4461 {
	i2c_device_t i2c;
	gpio_device_t gpio;
	uint8_t addr;
	uint32_t reset_pin, wp_pin;
	struct analog_device dev;
};

static int _mcp4461_write_reg(struct mcp4461 *self, uint8_t reg, uint8_t val){
	int ret = 0;
	uint8_t data[] = {reg | MCP4XXX_OP_WRITE | (uint8_t)(val >> 8), (uint8_t)(val & 0xff)};
	if((ret = i2c_transfer(self->i2c, self->addr, data, 2, NULL, 0, MCP4461_TIMEOUT)) < 0) return ret;
	return 0;
}

static int _mcp4461_read_reg(struct mcp4461 *self, uint8_t reg, uint16_t *val){
	int ret = 0;
	reg |= MCP4XXX_OP_READ;

	if((ret = i2c_transfer(self->i2c, self->addr, &reg, 1, val, 2, MCP4461_TIMEOUT)) < 0){
		return ret;
	}

	return 0;
}

static int _mcp4461_analog_write(analog_device_t dev, unsigned int chan, float value){
	struct mcp4461 *self = container_of(dev, struct mcp4461, dev.ops);

	if(chan > 7) return -EINVAL;

	value = constrain_float(value, 0, 1.0f);
	uint16_t val = (uint16_t)((float)((uint16_t)0x3ff)*value) & 0x3ff;

	static const uint8_t regs[8] = {
		MCP4XXX_REG_WIPER0,
		MCP4XXX_REG_WIPER1,
		MCP4XXX_REG_WIPER2,
		MCP4XXX_REG_WIPER3,
		MCP4XXX_REG_NV_WIPER0,
		MCP4XXX_REG_NV_WIPER1,
		MCP4XXX_REG_NV_WIPER2,
		MCP4XXX_REG_NV_WIPER3
	};

	return _mcp4461_write_reg(self, regs[chan & 7] | (uint8_t)(val >> 8), (uint8_t)(val & 0xff));
}

static int _mcp4461_analog_read(analog_device_t dev, unsigned int chan, float *value){
	return -1;
}

static struct analog_device_ops _mcp4461_analog_ops = {
	.write = _mcp4461_analog_write,
	.read = _mcp4461_analog_read
};

static int _mcp4461_probe(void *fdt, int fdt_node){
	i2c_device_t i2c = i2c_find_by_ref(fdt, fdt_node, "i2c");
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	uint8_t addr = (uint8_t)fdt_get_int_or_default(fdt, fdt_node, "reg", 0x2d);
	int reset_pin = fdt_get_int_or_default(fdt, fdt_node, "reset_pin", -1);
	int wp_pin = fdt_get_int_or_default(fdt, fdt_node, "wp_pin", -1);

	if(!i2c){
		printk("mcp4461: invalid i2c\n");
		return -EINVAL;
	}

	struct mcp4461 *self = kzmalloc(sizeof(struct mcp4461));
	self->i2c = i2c;
	self->gpio = gpio;
	self->addr = addr;
	
	if(self->gpio && wp_pin >= 0){
		self->wp_pin = (uint32_t)wp_pin;
		gpio_set(self->gpio, self->wp_pin);
	}

	if(self->gpio && reset_pin >= 0){
		self->reset_pin = (uint32_t)reset_pin;

		gpio_set(self->gpio, self->reset_pin);
		thread_sleep_ms(1);
		gpio_reset(self->gpio, self->reset_pin);
		thread_sleep_ms(1);
		gpio_set(self->gpio, self->reset_pin);
		thread_sleep_ms(1);
	}

	uint16_t status = 0;
	if(_mcp4461_read_reg(self, MCP4XXX_REG_STATUS, &status) != 0){
		printk(PRINT_ERROR "mcp4461: could not read status\n");
		return -EIO;
	}
	
	if(((status >> 8) & 0xff) != 0x82 || (status & 0xff) != 0x01){
		printk("mcp4461: invalid status (%02x)\n", status);
		return -1;
	}

	analog_device_init(&self->dev, fdt, fdt_node, &_mcp4461_analog_ops);
	analog_device_register(&self->dev);

	printk("mcp4461: ready (addr %02x)\n", addr);

	return 0;
}

static int _mcp4461_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(mcp4461, "fw,mcp4461", _mcp4461_probe, _mcp4461_remove)

