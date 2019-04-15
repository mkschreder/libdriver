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
#include <libfirmware/analog.h>
#include <libfirmware/gpio.h>
#include <libfirmware/console.h>
#include <libfirmware/mutex.h>
#include <libfirmware/i2c.h>
#include <libfirmware/math.h>

#include <libfdt/libfdt.h>

enum {
	LP55231_REG_CTRL1 = 0x00,
	LP55231_REG_CTRL1_BIT_EN = 0x40,
	LP55231_REG_CTRL2 = 0x01,
	LP55231_REG_OUT_RATH = 0x02,
	LP55231_REG_OUT_RATL = 0x03,
	LP55231_REG_OUT_ENH = 0x04,
	LP55231_REG_OUT_ENL = 0x05,
	LP55231_REG_D1_PWM = 0x16,
	LP55231_REG_D2_PWM = 0x17,
	LP55231_REG_D3_PWM = 0x18,
	LP55231_REG_D4_PWM = 0x19,
	LP55231_REG_D5_PWM = 0x1a,
	LP55231_REG_D6_PWM = 0x1b,
	LP55231_REG_D7_PWM = 0x1c,
	LP55231_REG_D8_PWM = 0x1d,
	LP55231_REG_D9_PWM = 0x1e,
	LP55231_REG_MISC = 0x36,
	LP55231_REG_STATUS = 0x3a,
	LP55231_REG_RESET = 0x3C,
	LP55231_REG_LED_TEST = 0x41,
	LP55231_REG_LED_TEST_ADC = 0x42
};

#define LP55231_REG_RESET_DO_RESET 0xff

#define LP55231_REG_MISC_CLOCK_INTERNAL 0x01
#define LP55231_REG_MISC_CP_FORCED_BYPASS 0x08

#define LP55231_REG_STATUS_LED_TEST_DONE_BIT (1 << 7)
#define LP55231_REG_STATUS_MASK_BUSY_BIT (1 << 6)
#define LP55231_REG_STATUS_STARTUP_BUSY_BIT (1 << 5)
#define LP55231_REG_STATUS_ENGINE_BUSY_BIT (1 << 4)

#define LP55231_REG_LED_TEST_START_CONVERSION_BIT (1 << 7)

#define LP55231_LED_COUNT 9

#define LP55231_TIMEOUT 1000

struct lp55231 {
	i2c_device_t i2c;
	gpio_device_t gpio;
	uint32_t en_pin;
	uint8_t addr;
	struct analog_device analog_dev;
	struct mutex lock;
};

static int _lp55231_write_reg(struct lp55231 *self, uint8_t reg, uint8_t value){
	uint8_t data[] = {reg, value};
	return i2c_transfer(self->i2c, self->addr, data, 2, NULL, 0, LP55231_TIMEOUT);
}

static int _lp55231_read_reg(struct lp55231 *self, uint8_t reg, uint8_t *val){
	return i2c_transfer(self->i2c, self->addr, &reg, 1, val, 1, LP55231_TIMEOUT);
}

static int _lp55231_read_led_voltage(struct lp55231 *self, unsigned led, float *voltage){
	if(led > 9) return -EINVAL;

	// start a conversion
	_lp55231_write_reg(self, LP55231_REG_LED_TEST,
			(uint8_t)(LP55231_REG_LED_TEST_START_CONVERSION_BIT | (led & 0x1f)));

	// the measurement takes 2.7ms according to datasheet
	thread_sleep_ms(4);

	uint8_t status = 0;
	if(_lp55231_read_reg(self, LP55231_REG_STATUS, &status) == 0 && (status & LP55231_REG_STATUS_LED_TEST_DONE_BIT)){
		uint8_t res = 0;
		if(_lp55231_read_reg(self, LP55231_REG_LED_TEST_ADC, &res) == 0){
			*voltage = (float)res * 0.03f - 1.478f;
			return 0;
		}
	}
	return -1;
}

static int _lp55231_analog_write(analog_device_t dev, unsigned int chan, float value){
	struct lp55231 *self = container_of(dev, struct lp55231, analog_dev.ops);

	thread_mutex_lock(&self->lock);

	chan = (chan > 9)?9:chan;
	value = constrain_float(value, 0.f, 1.f);
	int ret = _lp55231_write_reg(self, (uint8_t)(LP55231_REG_D1_PWM + chan), (uint8_t)(127.f * value));

	thread_mutex_unlock(&self->lock);

	return ret;
}

static int _lp55231_analog_read(analog_device_t dev, unsigned int chan, float *data){
	struct lp55231 *self = container_of(dev, struct lp55231, analog_dev.ops);

	thread_mutex_lock(&self->lock);
	int r = _lp55231_read_led_voltage(self, chan, data);
	thread_mutex_unlock(&self->lock);

	return r;
}

static struct analog_device_ops _lp55231_analog_ops = {
	.write = _lp55231_analog_write,
	.read = _lp55231_analog_read
};

static int _lp55231_probe(void *fdt, int fdt_node){
	i2c_device_t i2c = i2c_find_by_ref(fdt, fdt_node, "i2c");
	uint8_t addr = (uint8_t)fdt_get_int_or_default(fdt, fdt_node, "reg", 0x32);
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	uint32_t en_pin = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "en_pin", 0);

	if(!i2c) { printk(PRINT_ERROR "lp55231: no i2c\n"); return -EINVAL; }
	if(!gpio) { printk(PRINT_ERROR "lp55231: no gpio\n"); return -EINVAL; }

	struct lp55231 *self = kzmalloc(sizeof(struct lp55231));
	if(!self){
		printk(PRINT_ERROR "lp55231: nomem\n");
		return -1;
	}

	self->addr = addr;
	self->i2c = i2c;
	self->gpio = gpio;
	self->en_pin = en_pin;
	thread_mutex_init(&self->lock);

	// toggle hardware enable pin
	gpio_set(self->gpio, self->en_pin);

	// run reset sequence
	_lp55231_write_reg(self, LP55231_REG_RESET, LP55231_REG_RESET_DO_RESET);

	// toggle enable in the register
	_lp55231_write_reg(self, LP55231_REG_CTRL1, LP55231_REG_CTRL1_BIT_EN);

	// the chip goes through startup sequence 500us according to datasheet
	thread_sleep_ms(1);

	// select clock and charge pump mode
	_lp55231_write_reg(self, LP55231_REG_MISC, LP55231_REG_MISC_CLOCK_INTERNAL | LP55231_REG_MISC_CP_FORCED_BYPASS);

	printk("lp55231: checking LED voltages...\n");
	for(unsigned c = 0; c < LP55231_LED_COUNT; c++){
		float volts = 0;
		if(_lp55231_read_led_voltage(self, c, &volts)  == 0) {
			printk("lp55231: LED %d: voltage %dmV\n", c, (int32_t)(volts * 1000));
		} else {
			printk(PRINT_ERROR "lp55231: could not read LED %d voltage\n", c);
		}
	}

	// initialize all leds to half pwm
	for(int c = 0; c < 8; c++){
		_lp55231_write_reg(self, (uint8_t)(LP55231_REG_D1_PWM + c), 0x80);
	}


	analog_device_init(&self->analog_dev, fdt, fdt_node, &_lp55231_analog_ops);
	analog_device_register(&self->analog_dev);

	printk("lp55231: ready (addr %02x)\n", addr);

	return 0;
}

static int _lp55231_remove(void *fdt, int fdt_mode){
	return 0;
}

DEVICE_DRIVER(lp55231, "fw,lp55231", _lp55231_probe, _lp55231_remove)
