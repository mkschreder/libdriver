#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <libfirmware/driver.h>
#include <libfirmware/spi.h>
#include <libfirmware/gpio.h>
#include <libfirmware/console.h>
#include <libfirmware/memory.h>
#include <libfirmware/mutex.h>

#include <libfdt/libfdt.h>

#include "ice40-fw.c"

struct ice40 {
	memory_device_t flash;
	gpio_device_t gpio;
	uint32_t creset_pin, cdone_pin, cs_pin;
};

int _ice40_probe(void *fdt, int fdt_node){
	memory_device_t flash = memory_find_by_ref(fdt, fdt_node, "flash");
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	int creset_pin = fdt_get_int_or_default(fdt, fdt_node, "creset_pin", -1);
	int cdone_pin = fdt_get_int_or_default(fdt, fdt_node, "cdone_pin", -1);
	int cs_pin = fdt_get_int_or_default(fdt, fdt_node, "cs_pin", -1);

	if(!flash){
		printk("ice40: missing flash device!\n");
		return -1;
	}

	if(!gpio){
		printk("ice40: missing gpio device!\n");
		return -1;
	}

	if(creset_pin < 0){
		printk("ice40: missing creset_pin!\n");
		return -1;
	}

	if(cdone_pin < 0){
		printk("ice40: missing cdone_pin!\n");
		return -1;
	}

	if(cs_pin < 0){
		printk("ice40: missing cs_pin!\n");
		return -1;
	}

	struct ice40 *self = kzmalloc(sizeof(struct ice40));
	self->flash = flash;
	self->gpio = gpio;
	self->creset_pin = (uint32_t)creset_pin;
	self->cdone_pin = (uint32_t)cdone_pin;
	self->cs_pin = (uint32_t)cs_pin;

	gpio_reset(self->gpio, self->cs_pin);
	gpio_reset(self->gpio, self->creset_pin);

	int r = memory_write(self->flash, 0, example_bin, example_bin_len);
	if(r < 0){
		printk("ice40: failed to write flash!\n");
	} else {
		printk("ice40: written %d bytes\n", r);
	}

	gpio_set(self->gpio, self->creset_pin);

	return 0;
}

int _ice40_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(ice40, "fw,ice40", _ice40_probe, _ice40_remove)
