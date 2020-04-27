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
#include <libfirmware/thread/mutex.h>
#include <libfirmware/analog.h>
#include <libfirmware/math.h>
#include <libfirmware/adc.h>
#include <libfirmware/memory.h>

#include "hbridge/drv8302.h"

#include <libfdt/libfdt.h>

#define PIN_EN_GATE 0
#define PIN_GAIN 1
#define PIN_MOC 2
#define PIN_DC_CAL 3
#define PIN_OCW 4
#define PIN_FAULT 5

struct drv8302 {
	gpio_device_t gpio;
	analog_device_t pwm;
	adc_device_t adc;

	struct memory_device mem;
};

static int _drv8302_cmd(console_device_t con, void *userptr, int argc, char **argv){
	struct drv8302 *self = (struct drv8302*)userptr;
	if(argc == 5 && strcmp(argv[1], "out") == 0){
		float va = constrain_float((float)atof(argv[2]), 0.0f, 1.0f);
		float vb = constrain_float((float)atof(argv[3]), 0.0f, 1.0f);
		float vc = constrain_float((float)atof(argv[4]), 0.0f, 1.0f);

		analog_write(self->pwm, 0, va);
		analog_write(self->pwm, 1, vb);
		analog_write(self->pwm, 2, vc);
	} else if(argc == 2 && strcmp(argv[1], "reset") == 0) {
		gpio_reset(self->gpio, PIN_EN_GATE);
		gpio_set(self->gpio, PIN_EN_GATE);
	} else if(argc >= 2 && strcmp(argv[1], "dccal") == 0){
		if(argc != 3){
			console_printf(con, PRINT_ERROR "usage: dccal <on|off>\n");
			return -EINVAL;
		}
		if(strcmp(argv[2], "on") == 0){
			gpio_set(self->gpio, PIN_DC_CAL);
		} else if(strcmp(argv[2], "off") == 0){
			gpio_reset(self->gpio, PIN_DC_CAL);
		} else {
			console_printf(con, PRINT_ERROR "invalid argument\n");
			return -EINVAL;
		}
	} else if(argc >= 2 && strcmp(argv[1], "dcgain") == 0) {
		if(argc != 3){
			console_printf(con, PRINT_ERROR "Specify gain 10 or 40\n");
			return -EINVAL;
		}
		if(strcmp(argv[2], "40") == 0){
			gpio_set(self->gpio, PIN_GAIN);
		} else if(strcmp(argv[1], "10") == 0){
			gpio_reset(self->gpio, PIN_GAIN);
		} else {
			console_printf(con, PRINT_ERROR "Only 10 or 40 are valid options!\n");
		}
	} else {
		console_printf(con, "Unknown action\n");
	}
	return 0;
}

enum {
	DRV8302_OFS_DC_CAL,
	DRV8302_OFS_OC_MODE,
	DRV8302_OFS_FAULT,
	DRV8302_OFS_OCW,
	DRV8302_OFS_GAIN,
	DRV8302_OFS_ENABLE
};

void drv8302_enable_calibration(drv8302_t dev, bool en){
	uint8_t r = (uint8_t)en;
	memory_write((memory_device_t)dev, DRV8302_OFS_DC_CAL, &r, 1);
}

void drv8302_set_oc_mode(drv8302_t dev, drv8302_oc_mode_t mode){
	uint8_t r = (uint8_t)mode;
	memory_write((memory_device_t)dev, DRV8302_OFS_OC_MODE, &r, 1);
}

void drv8302_enable(drv8302_t dev, bool en){
	uint8_t r = (uint8_t)en;
	memory_write((memory_device_t)dev, DRV8302_OFS_ENABLE, &r, 1);
}

void drv8302_set_gain(drv8302_t dev, drv8302_gain_t gain){
	uint8_t r = (uint8_t)gain;
	memory_write((memory_device_t)dev, DRV8302_OFS_GAIN, &r, 1);
}

bool drv8302_is_in_error(drv8302_t dev){
	uint8_t r = 0;
	memory_read((memory_device_t)dev, DRV8302_OFS_FAULT, &r, 1);
	return !(bool)r;
}

bool drv8302_is_in_overcurrent(drv8302_t dev){
	uint8_t r = 0;
	memory_read((memory_device_t)dev, DRV8302_OFS_OCW, &r, 1);
	return !(bool)r;
}

int drv8302_get_gain(drv8302_t dev){
	uint8_t r = 0;
	memory_read((memory_device_t)dev, DRV8302_OFS_GAIN, &r, 1);
	if(r == DRV8302_GAIN_10V) return 10;
	return 40;
}

static int _drv8302_read(memory_device_t dev, size_t offs, void *data, size_t size){
	struct drv8302 *self = container_of(dev, struct drv8302, mem.ops);

	if(size != 1) return -EINVAL;
	switch(offs){
		case DRV8302_OFS_FAULT: *(uint8_t*)data = (uint8_t)gpio_read(self->gpio, PIN_FAULT); break;
		case DRV8302_OFS_OCW: *(uint8_t*)data = (uint8_t)gpio_read(self->gpio, PIN_OCW); break;
		case DRV8302_OFS_GAIN: *(uint8_t*)data = (uint8_t)gpio_read(self->gpio, PIN_GAIN); break;
	}
	return 1;
}

static int _drv8302_write(memory_device_t dev, size_t offs, const void *data, size_t size){
	struct drv8302 *self = container_of(dev, struct drv8302, mem.ops);

	if(size != 1) return -EINVAL;
	switch(offs){
		case DRV8302_OFS_DC_CAL: {
			if(*(uint8_t*)data){
				gpio_set(self->gpio, PIN_DC_CAL);
			} else {
				gpio_reset(self->gpio, PIN_DC_CAL);
			}
		} break;
		case DRV8302_OFS_OC_MODE: {
			drv8302_oc_mode_t mode = *(uint8_t*)data;
			if(mode == DRV8302_OC_MODE_CYCLE_BY_CYCLE){
				gpio_reset(self->gpio, PIN_MOC);
			} else if(mode == DRV8302_OC_MODE_SHUTDOWN){
				gpio_set(self->gpio, PIN_MOC);
			}
		} break;
		case DRV8302_OFS_ENABLE: {
			if(*(uint8_t*)data){
				gpio_set(self->gpio, PIN_EN_GATE);
			} else {
				gpio_reset(self->gpio, PIN_EN_GATE);
			}
		} break;
		case DRV8302_OFS_GAIN: {
			drv8302_gain_t gain = *(uint8_t*)data;
			if(gain == DRV8302_GAIN_10V){
				gpio_reset(self->gpio, PIN_GAIN);
			} else if(gain == DRV8302_GAIN_40V){
				gpio_set(self->gpio, PIN_GAIN);
			}
		} break;
	}
	return 1;
}

static struct memory_device_ops _memory_ops = {
	.read = _drv8302_read,
	.write = _drv8302_write
};

int _drv8302_probe(void *fdt, int fdt_node){
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	analog_device_t pwm = analog_find_by_ref(fdt, fdt_node, "pwm");
	adc_device_t adc = adc_find_by_ref(fdt, fdt_node, "adc");

	if(!gpio){
		printk("drv8302: gpio invalid\n");
		return -1;
	}

	if(!pwm){
		printk("drv8302: pwm invalid\n");
		return -1;
	}

	if(!adc){
		printk("drv8302: adc invalid\n");
		return -1;
	}

	struct drv8302 *self = kzmalloc(sizeof(struct drv8302));
	self->gpio = gpio;
	self->pwm = pwm;
	self->adc = adc;

	console_device_t console = console_find_by_ref(fdt, fdt_node, "console");
	if(console){
		char name[32];
		static int instances = 0;
		snprintf(name, sizeof(name), "drv8302_%d", instances++);
		console_add_command(console, self, name, "drv8302 utilities", "", _drv8302_cmd);
	}

	gpio_set(self->gpio, PIN_EN_GATE);
	gpio_reset(self->gpio, PIN_DC_CAL);
	gpio_reset(self->gpio, PIN_GAIN); // 0 = 10V, 1 = 40V

	memory_device_init(&self->mem, fdt, fdt_node, &_memory_ops);
	memory_device_register(&self->mem);

	printk("drv8302: ready\n");

	return 0;
}

int _drv8302_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(drv8302, "fw,drv8302", _drv8302_probe, _drv8302_remove)
