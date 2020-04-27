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
#include <libfirmware/thread/mutex.h>

#include <libfdt/libfdt.h>

#include "ice40-fw.c"

struct ice40 {
	memory_device_t flash;
	gpio_device_t gpio;
	spi_device_t spi;
	uint32_t creset_pin, cdone_pin, cs_pin;
};

int _ice40_probe(void *fdt, int fdt_node){
	memory_device_t flash = memory_find_by_ref(fdt, fdt_node, "flash");
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	spi_device_t spi = spi_find_by_ref(fdt, fdt_node, "spi");
	int creset_pin = fdt_get_int_or_default(fdt, fdt_node, "creset_pin", -1);
	int cdone_pin = fdt_get_int_or_default(fdt, fdt_node, "cdone_pin", -1);
	int cs_pin = fdt_get_int_or_default(fdt, fdt_node, "cs_pin", -1);
	int pwr_pin = fdt_get_int_or_default(fdt, fdt_node, "pwr_pin", -1);

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
	self->spi = spi;
	self->creset_pin = (uint32_t)creset_pin;
	self->cdone_pin = (uint32_t)cdone_pin;
	self->cs_pin = (uint32_t)cs_pin;

	// to enter spi slave mode we set the SS low and then toggle power
	if(spi){
		int r = 0;
		uint8_t dummy[8] = {0, 0, 0, 0, 0, 0, 0, 0};

		// reset the chip into slave mode
		gpio_reset(self->gpio, self->cs_pin);
		gpio_reset(self->gpio, self->creset_pin);
		thread_sleep_ms(1);

		// power cycle
		if(pwr_pin >= 0){
			gpio_reset(self->gpio, (uint32_t)pwr_pin);
			thread_sleep_ms(50);
			gpio_set(self->gpio, (uint32_t)pwr_pin);
			thread_sleep_ms(50);
		}

		gpio_set(self->gpio, self->creset_pin);
		// chip needs 1200us to boot into slave mode
		thread_sleep_ms(2);

		// pull cs high and output a byte
		gpio_set(self->gpio, self->cs_pin);
		spi_transfer(self->spi, NULL, 0, dummy, NULL, 1, 1000);
		thread_sleep_ms(1);
	
		// now pull it low and send the whole image!
		gpio_reset(self->gpio, self->cs_pin);
		r = spi_transfer(self->spi, NULL, 0, example_bin, NULL, example_bin_len, 1000);
		gpio_set(self->gpio, self->cs_pin);

		if(r < 0){
			printk("ice40: failed to write configuration image!\n");
		} else {
			printk("ice40: written %d bytes to FPGA!\n", r);
		}
		// we are not done yet so CDONE should be low!
		if(gpio_read(self->gpio, self->cdone_pin)){
			printk("ice40: WARNING: CDONE is high when it shouldn't!\n");
		}
		spi_transfer(self->spi, NULL, 0, dummy, NULL, sizeof(dummy), 1000);
		if(gpio_read(self->gpio, self->cdone_pin)){
			printk("ice40: FPGA configured!\n", r);
		} else {
			printk("ice40: FAILED to start FPGA! (timeout waiting for CDONE)\n", r);
		}
		spi_transfer(self->spi, NULL, 0, dummy, NULL, sizeof(dummy), 1000);
	} else {
		// chip is going to be spi master so we write the firmware here to flash while chip is in reset
		gpio_reset(self->gpio, self->creset_pin);
		gpio_set(self->gpio, self->cs_pin);
		thread_sleep_ms(1);

		// power cycle
		if(pwr_pin >= 0){
			gpio_reset(self->gpio, (uint32_t)pwr_pin);
			thread_sleep_ms(50);
			gpio_set(self->gpio, (uint32_t)pwr_pin);
			thread_sleep_ms(50);
		}

		int r = memory_write(self->flash, 0, example_bin, example_bin_len);
		if(r < 0){
			printk("ice40: failed to write flash!\n");
		} else {
			printk("ice40: written %d bytes\n", r);
		}

		gpio_set(self->gpio, self->creset_pin);
	}

	return 0;
}

int _ice40_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(ice40, "fw,ice40", _ice40_probe, _ice40_remove)
