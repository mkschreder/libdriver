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

#include <libfdt/libfdt.h>

#define PIN_EN_GATE 0
#define PIN_GAIN 1
#define PIN_MOC 2
#define PIN_DC_CAL 3
#define PIN_OCW 4
#define PIN_FAULT 5

struct drv8302 {
	gpio_device_t gpio;
};

int _drv8302_probe(void *fdt, int fdt_node){
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");

	if(!gpio){
		printk("drv8302: gpio invalid\n");
		return -1;
	}

	struct drv8302 *self = kzmalloc(sizeof(struct drv8302));
	self->gpio = gpio;

	gpio_set(self->gpio, PIN_EN_GATE);

	printk("drv8302: ready\n");

	return 0;
}

int _drv8302_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(drv8302, "fw,drv8302", _drv8302_probe, _drv8302_remove)
