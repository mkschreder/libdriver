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
#include <libfirmware/encoder.h>

#include <libfirmware/thread/mutex.h>

#include <libfdt/libfdt.h>

#define MAX14890_CMD_WRITE_CONFIG (5 << 13)
#define MAX14890_CMD_FAULT_STATUS (2 << 13)

#define MAX14890_HITH_LOW		(0)
#define MAX14890_HITH_HIGH		((1 << 12))

#define MAX14890_MODEA_DHTL		(0)
#define MAX14890_MODEA_RS422	((1 << 10))
#define MAX14890_MODEA_SEHTL	((2 << 10))
#define MAX14890_MODEA_TTL		((3 << 10))

#define MAX14890_MODEB_DHTL		(0)
#define MAX14890_MODEB_RS422	((1 << 8))
#define MAX14890_MODEB_SEHTL	((2 << 8))
#define MAX14890_MODEB_TTL		((3 << 8))

#define MAX14890_MODEZ_DHTL		((0))
#define MAX14890_MODEZ_RS422	((4 << 5))
#define MAX14890_MODEZ_SEHTL	((2 << 5))
#define MAX14890_MODEZ_TTL		((6 << 5))
#define MAX14890_MODEZ_DI		((1 << 5))

#define MAX14890_MODEY_RS422 	(0)
#define MAX14890_MODEY_TTL		((3 << 3))
#define MAX14890_MODEY_DI		((1 << 3))

#define MAX14890_MODED2_DI		((1 << 2))
#define MAX14890_MODED2_TTL		0
#define MAX14890_MODED3_DI		((1 << 1))
#define MAX14890_MODED3_TTL		0

#define MAX14890_FILTER_OFF		0
#define MAX14890_FILTER_ON		((1 << 0))

#define MAX14890_TIMEOUT		10

struct max14890 {
	gpio_device_t gpio;
	spi_device_t spi;
	uint32_t cs_pin;
};

static int _max14890_cmd(struct max14890 *self, uint32_t txdata, uint16_t *rxdata){
	uint8_t tx[2] = {(uint8_t)(txdata >> 8), (uint8_t)txdata};
	uint8_t rx[2] = {0};

	thread_sleep_us(50);

	spi_transfer(self->spi, self->gpio, self->cs_pin, tx, rx, sizeof(tx), MAX14890_TIMEOUT);

	thread_sleep_us(50);

	// read status
	spi_transfer(self->spi, self->gpio, self->cs_pin, tx, rx, sizeof(tx), MAX14890_TIMEOUT);

	*rxdata = (uint16_t)(((int)rx[0] << 8) | rx[1]);

	return 0;
}

static int _max14890_probe(void *fdt, int fdt_node){
	spi_device_t spi = spi_find_by_ref(fdt, fdt_node, "spi");
	if(!spi){
		printk("max14890: spi error\n");
		return -1;
	}
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	if(!gpio){
		printk("max14890: gpio error\n");
		return -1;
	}
	uint32_t cs_pin = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "cs_pin", 0);

	struct max14890 *self = kzmalloc(sizeof(struct max14890));
	self->gpio = gpio;
	self->cs_pin = cs_pin;
	self->spi = spi;

	gpio_set(self->gpio, self->cs_pin);

	// configure mode for pins
	uint16_t status = 0;
	_max14890_cmd(self,
		MAX14890_CMD_WRITE_CONFIG |
		MAX14890_MODEA_TTL |
		MAX14890_MODEB_TTL |
		MAX14890_MODEZ_TTL |
		MAX14890_MODEY_TTL |
		MAX14890_MODED2_TTL |
		MAX14890_MODED3_TTL
	, &status);

	_max14890_cmd(self,
		MAX14890_CMD_FAULT_STATUS, &status);

	if((status & 0xe000) != 0x4000){
		printk(PRINT_ERROR "max14890: fault status (%04x) invalid. Chip unreachable!\n", status);
	} else {
		printk(PRINT_SUCCESS "max14890: ready (%04x)\n", status);
	}

	return 0;
}

static int _max14890_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(max14890, "fw,max14890", _max14890_probe, _max14890_remove)

