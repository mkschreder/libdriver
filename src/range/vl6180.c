#include <libfirmware/driver.h>
#include <libfirmware/analog.h>
#include <libfirmware/i2c.h>
#include <libfirmware/gpio.h>
#include <libfirmware/mutex.h>

#define VL6180_DEFAULT_ADDRESS 0x29

#define VL6180_REG_ID_MODEL					0x000
#define VL6180_REG_ID_MODEL_REV_MAJOR		0x001
#define VL6180_REG_ID_MODEL_REV_MINOR		0x002
#define VL6180_REG_ID_MODULE_REV_MAJOR		0x003
#define VL6180_REG_ID_MODULE_REV_MINOR		0x004
#define VL6180_REG_ID_DATE_HI				0x006
#define VL6180_REG_ID_DATE_LO				0x007
#define VL6180_REG_ID_TIME_HI				0x008
#define VL6180_REG_ID_TIME_LO				0x009
#define VL6180_REG_FRESH_OUT_OF_RESET		0x016

#define VL6180_MODEL_ID		0xb4

#define VL6180_I2C_TIMEOUT 100

struct vl6180 {
	i2c_device_t i2c;
	gpio_device_t gpio;
	uint32_t ce_pin;
	uint8_t address;
	struct mutex lock;
};

static int _vl6180_read_reg(struct vl6180 *self, uint16_t reg, uint8_t *value){
	uint8_t tx[2] = { (uint8_t)(reg >> 8) & 0xff, (uint8_t)(reg & 0xff) };
	return i2c_transfer(self->i2c, self->address, tx, 2, value, 1, VL6180_I2C_TIMEOUT);
}

static void _vl6180_begin(struct vl6180 *self){
	thread_sleep_ms(10);
	gpio_set(self->gpio, self->ce_pin);
	thread_sleep_ms(10);
}

static void _vl6180_end(struct vl6180 *self){
	thread_sleep_ms(10);
	gpio_reset(self->gpio, self->ce_pin);
	thread_sleep_ms(10);
}

static int _vl6180_probe(void *fdt, int fdt_node){
	struct vl6180 *self = kzmalloc(sizeof(struct vl6180));
	
	DEVICE_REF("vl6180", i2c, i2c);
	DEVICE_REF("vl6180", gpio, gpio);
	self->ce_pin = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "ce_pin", 0);
	self->address = VL6180_DEFAULT_ADDRESS;

	thread_sleep_ms(3);

	_vl6180_begin(self);

	uint8_t model = 0, rev_major = 0, rev_minor = 0, mod_major = 0, mod_minor = 0, date_hi = 0, date_lo = 0;

#define CHECK(x) do { if((x) < 0) { printk(PRINT_ERROR "vl6180: i2c communication error!\n"); /* return -EIO;*/ } } while(0)
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODEL, &model));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODEL_REV_MAJOR, &rev_major));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODEL_REV_MINOR, &rev_minor));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODULE_REV_MAJOR, &mod_major));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODULE_REV_MINOR, &mod_minor));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_DATE_HI, &date_hi));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_DATE_LO, &date_lo));

	_vl6180_end(self);

	switch(model){
		case VL6180_MODEL_ID: {
			printk(PRINT_DEFAULT "vl6180: model rev: %d.%d, module rev: %d.%d, date: %d.%d\n",
				rev_major,
				rev_minor,
				mod_major,
				mod_minor,
				date_hi,
				date_lo);
		} break;
		default: {
			printk(PRINT_ERROR "vl6180: unknown device (%02x)\n", model);
		} break;
	}
	return 0;
}

static int _vl6180_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(vl6180, "fw,vl6180", _vl6180_probe, _vl6180_remove)
