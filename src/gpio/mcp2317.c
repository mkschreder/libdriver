/*
 * This file contains a lot of code from Ninjaflight.
 *
 * Authors: Martin Schröder & Cleanflight project
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
#include <libfirmware/thread/mutex.h>

#include <libfdt/libfdt.h>

#define MCP2317_TIMEOUT 100

#define MCP2317_REG_IODIRA 0x00
#define MCP2317_REG_IODIRB 0x01
#define MCP2317_REG_IPOLA 0x02
#define MCP2317_REG_IPOLB 0x03
#define MCP2317_REG_GPINTENA 0x04
#define MCP2317_REG_GPINTENB 0x05
#define MCP2317_REG_DEFVALA 0x06
#define MCP2317_REG_DEFVALB 0x07
#define MCP2317_REG_INTCONA 0x08
#define MCP2317_REG_INTCONB 0x09
#define MCP2317_REG_IOCONA 0x0a
#define MCP2317_REG_IOCONB 0x0b
#define MCP2317_REG_GPPUA 0x0c
#define MCP2317_REG_GPPUB 0x0d
#define MCP2317_REG_INTFA 0x0e
#define MCP2317_REG_INTFB 0x0f
#define MCP2317_REG_INTCAPA 0x10
#define MCP2317_REG_INTCAPB 0x11
#define MCP2317_REG_GPIOA 0x12
#define MCP2317_REG_GPIOB 0x13
#define MCP2317_REG_OLATA 0x14
#define MCP2317_REG_OLATB 0x15

struct mcp2317 {
	spi_device_t spi;
	gpio_device_t gpio;
	uint32_t cs_pin, reset_pin;
	struct gpio_device dev;
	struct mutex lock;
};

static uint8_t _mcp2317_read_reg(struct mcp2317 *self, uint8_t reg){
	uint8_t tx[3] = { 0x41, reg, 0};
	uint8_t rx[3] = {0, 0, 0};

	spi_transfer(self->spi, self->gpio, self->cs_pin, tx, rx, 3, MCP2317_TIMEOUT);

	return rx[2];
}

static void _mcp2317_write_reg(struct mcp2317 *self, uint8_t reg, uint8_t value){
	uint8_t tx[3] = { 0x40, reg, (uint8_t)value };
	uint8_t rx[3] = {0, 0, 0};

	spi_transfer(self->spi, self->gpio, self->cs_pin, tx, rx, 3, MCP2317_TIMEOUT);
}


static void _mcp2317_write_reg_bits(struct mcp2317 *self, uint8_t reg,
		uint8_t bits, uint8_t values){
	thread_mutex_lock(&self->lock);
	uint8_t oval = _mcp2317_read_reg(self, reg);
	uint8_t val = (uint8_t)((oval & (uint8_t)~bits) | values);
	_mcp2317_write_reg(self, reg, val);
	thread_mutex_unlock(&self->lock);
}

static int _mcp2317_write_pin(gpio_device_t dev, uint32_t pin, bool value){
	struct mcp2317 *self = container_of(dev, struct mcp2317, dev.ops);
	if(pin < 8){
		thread_mutex_lock(&self->lock);
		_mcp2317_write_reg(self, MCP2317_REG_GPIOA,
			(uint8_t)((_mcp2317_read_reg(self, MCP2317_REG_GPIOA) & (uint32_t)~(1 << pin)) | ((uint32_t)value << pin))
		);
		thread_mutex_unlock(&self->lock);
	} else {
		pin &= 0x07;
		thread_mutex_lock(&self->lock);
		_mcp2317_write_reg(self, MCP2317_REG_GPIOB,
			(uint8_t)((_mcp2317_read_reg(self, MCP2317_REG_GPIOA) & (uint32_t)~(1 << pin)) | ((uint32_t)value << pin))
		);
		thread_mutex_unlock(&self->lock);
	}
	return 0;
}

static int _mcp2317_read_pin(gpio_device_t dev, uint32_t pin, bool *value){
	struct mcp2317 *self = container_of(dev, struct mcp2317, dev.ops);
	if(pin < 8){
		thread_mutex_lock(&self->lock);
		*value = ((_mcp2317_read_reg(self, MCP2317_REG_GPIOA) >> pin) & 1)?1:0;
		thread_mutex_unlock(&self->lock);
	} else {
		pin &= 0x07;
		thread_mutex_lock(&self->lock);
		*value = ((_mcp2317_read_reg(self, MCP2317_REG_GPIOB) >> pin) & 1)?1:0;
		thread_mutex_unlock(&self->lock);
	}
	return 0;
}

static struct gpio_device_ops _mcp2317_ops = {
	.write_pin = _mcp2317_write_pin,
	.read_pin = _mcp2317_read_pin
};

static int _mcp2317_cmd(console_device_t con, void *userptr, int argc, char **argv){
	struct mcp2317 *self = (struct mcp2317*)userptr;
	if(argc == 2 && strcmp(argv[1], "dumpregs") == 0){
		for(uint8_t reg = 0; reg < 0x16; reg++){
			uint8_t value = _mcp2317_read_reg(self, reg);
			console_printf(con, "%02x: %02x\n", reg, value);
		}
	} else {
		if(argc > 1){
			console_printf(con, "Unknown action %s\n", argv[2]);
		}
	}
	return 0;
}

static int _mcp2317_probe(void *fdt, int fdt_node){
	spi_device_t spi = spi_find_by_ref(fdt, fdt_node, "spi");
	if(!spi){
		printk("mcp2317: spi error\n");
		return -1;
	}
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	if(!gpio){
		printk("mcp2317: gpio error\n");
		return -1;
	}
	int cs_pin = fdt_get_int_or_default(fdt, fdt_node, "cs_pin", -1);
	int reset_pin = (int)fdt_get_int_or_default(fdt, fdt_node, "reset_pin", -1);
	if(cs_pin < 0 || reset_pin < 0){
		printk("mcp2317: cs_pin and reset_pin must be specified\n");
		return -1;
	}
	struct mcp2317 *self = kzmalloc(sizeof(struct mcp2317));
	self->reset_pin = (uint32_t)reset_pin;
	self->cs_pin = (uint32_t)cs_pin;
	self->gpio = gpio;
	self->spi = spi;
	thread_mutex_init(&self->lock);

	console_device_t console = console_find_by_ref(fdt, fdt_node, "console");
	if(console){
		console_add_command(console, self, fdt_get_name(fdt, fdt_node, NULL), "mcp2317 device control", "", _mcp2317_cmd);
	}

	gpio_set(self->gpio, self->cs_pin);

	gpio_set(self->gpio, self->reset_pin);
	gpio_reset(self->gpio, self->reset_pin);
	gpio_set(self->gpio, self->reset_pin);

	// expect registers to have default values after reset
	if(_mcp2317_read_reg(self, 0x00) != 0xff ||
		_mcp2317_read_reg(self, 0x01) != 0xff){
		printk(PRINT_ERROR "Register values are garbage\n");
		//return -1;
	}

	int len = 0;
	const fdt32_t *val = (const fdt32_t*)fdt_getprop(fdt, fdt_node, "pinctrl", &len);
    if(val && len > 0){
		if((len * 4) % 3 != 0){
			printk("mcp2317: pinctrl format not multiple of 3\n");
		} else {
    		int pin_count = (uint8_t)(len / 4 / 3);
			for(int c = 0; c < pin_count; c++){
				const fdt32_t *base = val + (3 * c);
				uint32_t idx = (uint32_t)fdt32_to_cpu(*(base)) & 0xf;
				uint32_t options = (uint32_t)fdt32_to_cpu(*(base + 1));
				uint32_t def_value = (uint32_t)fdt32_to_cpu(*(base + 2));

				uint8_t reg_iodir = MCP2317_REG_IODIRA;
				uint8_t reg_gppu = MCP2317_REG_GPPUA;
				uint8_t reg_gpio = MCP2317_REG_GPIOA;

				if(idx >= 8){
					reg_iodir = MCP2317_REG_IODIRB;
					reg_gppu = MCP2317_REG_GPPUB;
					reg_gpio = MCP2317_REG_GPIOB;
					idx &= 0x07;
				}

				_mcp2317_write_reg_bits(self, reg_iodir,
						(uint8_t)(1 << idx),
						(uint8_t)((options & 1) << idx)
				);
				_mcp2317_write_reg_bits(self, reg_gppu,
						(uint8_t)(1 << idx),
						(uint8_t)(((options >> 1) & 1) << idx)
				);
				_mcp2317_write_reg_bits(self, reg_gpio,
						(uint8_t)(1 << idx),
						(uint8_t)((def_value & 1) << idx));
			}
		}
	}

	gpio_device_init(&self->dev, fdt, fdt_node, &_mcp2317_ops);
	gpio_device_register(&self->dev);

	printk("mcp2317: ready (cs_pin: %d, reset_pin: %d)\n", self->cs_pin, self->reset_pin);
	return 0;
}

static int _mcp2317_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(mcp2317, "fw,mcp2317", _mcp2317_probe, _mcp2317_remove)

