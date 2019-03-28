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
#include <libfirmware/analog.h>
#include <libfirmware/math.h>

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
};

static int _drv8302_cmd(console_t con, void *userptr, int argc, char **argv){
	struct drv8302 *self = (struct drv8302*)userptr;
	if(argc == 5 && strcmp(argv[1], "out") == 0){
		float va = constrain_float((float)atof(argv[2]), 0.0f, 1.0f);
		float vb = constrain_float((float)atof(argv[3]), 0.0f, 1.0f);
		float vc = constrain_float((float)atof(argv[4]), 0.0f, 1.0f);

		analog_write(self->pwm, 0, va);
		analog_write(self->pwm, 1, vb);
		analog_write(self->pwm, 2, vc);
	} else {
		if(argc > 1){
			console_printf(con, "Unknown action %s\n", argv[2]);
		}
	}
	return 0;
}

int _drv8302_probe(void *fdt, int fdt_node){
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	analog_device_t pwm = analog_find_by_ref(fdt, fdt_node, "pwm");

	if(!gpio){
		printk("drv8302: gpio invalid\n");
		return -1;
	}

	if(!pwm){
		printk("drv8302: pwm invalid\n");
		return -1;
	}

	struct drv8302 *self = kzmalloc(sizeof(struct drv8302));
	self->gpio = gpio;
	self->pwm = pwm;

	console_t console = console_find_by_ref(fdt, fdt_node, "console");
	if(console){
		char name[32];
		static int instances = 0;
		snprintf(name, sizeof(name), "drv8302_%d", instances++);
		console_add_command(console, self, name, "drv8302 utilities", "", _drv8302_cmd);
	}

	gpio_set(self->gpio, PIN_EN_GATE);

	printk("drv8302: ready\n");

	return 0;
}

int _drv8302_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(drv8302, "fw,drv8302", _drv8302_probe, _drv8302_remove)
