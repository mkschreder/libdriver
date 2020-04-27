#include <libfirmware/driver.h>
#include <libfirmware/analog.h>
#include <libfirmware/i2c.h>
#include <libfirmware/gpio.h>
#include <libfirmware/thread/mutex.h>

#define vl6180_debug printk

#define VL6180_DEFAULT_ADDRESS 0x29

#define VL6180_REG_ID_MODEL_ID              0x0000
#define VL6180_REG_ID_MODEL_REV_MAJOR       0x0001
#define VL6180_REG_ID_MODEL_REV_MINOR       0x0002
#define VL6180_REG_ID_MODULE_REV_MAJOR      0x0003
#define VL6180_REG_ID_MODULE_REV_MINOR      0x0004
#define VL6180_REG_ID_DATE                  0x0006 //16bit value
#define VL6180_REG_ID_TIME                  0x0008 //16bit value

#define VL6180_REG_SYS_MODE_GPIO0                    0x0010
#define VL6180_REG_SYS_MODE_GPIO1                    0x0011
#define VL6180_REG_SYS_HISTORY_CTRL                  0x0012
#define VL6180_REG_SYS_INTERRUPT_CONFIG_GPIO         0x0014
#define VL6180_REG_SYS_INTERRUPT_CLEAR               0x0015
#define VL6180_REG_SYS_FRESH_OUT_OF_RESET            0x0016
#define VL6180_REG_SYS_GROUPED_PARAMETER_HOLD        0x0017

#define VL6180_REG_SYSRANGE_START                       0x0018
#define VL6180_REG_SYSRANGE_THRESH_HIGH                 0x0019
#define VL6180_REG_SYSRANGE_THRESH_LOW                  0x001A
#define VL6180_REG_SYSRANGE_INTERMEASUREMENT_PERIOD     0x001B
#define VL6180_REG_SYSRANGE_MAX_CONVERGENCE_TIME        0x001C
#define VL6180_REG_SYSRANGE_CROSSTALK_COMPENSATION_RATE 0x001E
#define VL6180_REG_SYSRANGE_CROSSTALK_VALID_HEIGHT      0x0021
#define VL6180_REG_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  0x0022
#define VL6180_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET   0x0024
#define VL6180_REG_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   0x0025
#define VL6180_REG_SYSRANGE_RANGE_IGNORE_THRESHOLD      0x0026
#define VL6180_REG_SYSRANGE_MAX_AMBIENT_LEVEL_MULT      0x002C
#define VL6180_REG_SYSRANGE_RANGE_CHECK_ENABLES         0x002D
#define VL6180_REG_SYSRANGE_VHV_RECALIBRATE             0x002E
#define VL6180_REG_SYSRANGE_VHV_REPEAT_RATE             0x0031

#define VL6180_REG_SYSALS_START                         0x0038
#define VL6180_REG_SYSALS_THRESH_HIGH                   0x003A
#define VL6180_REG_SYSALS_THRESH_LOW                    0x003C
#define VL6180_REG_SYSALS_INTERMEASUREMENT_PERIOD       0x003E
#define VL6180_REG_SYSALS_ANALOGUE_GAIN                 0x003F
#define VL6180_REG_SYSALS_INTEGRATION_PERIOD            0x0040

#define VL6180_REG_RESULT_RANGE_STATUS                  0x004D
#define VL6180_REG_RESULT_ALS_STATUS                    0x004E
#define VL6180_REG_RESULT_INTERRUPT_STATUS_GPIO         0x004F
#define VL6180_REG_RESULT_ALS_VAL                       0x0050
#define VL6180_REG_RESULT_HISTORY_BUFFER                0x0052 
#define VL6180_REG_RESULT_RANGE_VAL                     0x0062
#define VL6180_REG_RESULT_RANGE_RAW                     0x0064
#define VL6180_REG_RESULT_RANGE_RETURN_RATE             0x0066
#define VL6180_REG_RESULT_RANGE_REFERENCE_RATE          0x0068
#define VL6180_REG_RESULT_RANGE_RETURN_SIGNAL_COUNT     0x006C
#define VL6180_REG_RESULT_RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define VL6180_REG_RESULT_RANGE_RETURN_AMB_COUNT        0x0074
#define VL6180_REG_RESULT_RANGE_REFERENCE_AMB_COUNT     0x0078
#define VL6180_REG_RESULT_RANGE_RETURN_CONV_TIME        0x007C
#define VL6180_REG_RESULT_RANGE_REFERENCE_CONV_TIME     0x0080

#define VL6180_REG_READOUT_AVERAGING_SAMPLE_PERIOD      0x010A
#define VL6180_REG_FIRMWARE_BOOTUP                      0x0119
#define VL6180_REG_FIRMWARE_RESULT_SCALER               0x0120
#define VL6180_REG_I2C_SLAVE_DEVICE_ADDRESS             0x0212
#define VL6180_REG_INTERLEAVED_MODE_ENABLE              0x02A3

#define VL6180_MODEL_ID		0xb4

#define VL6180_I2C_TIMEOUT 100

struct vl6180 {
	i2c_device_t i2c;
	gpio_device_t gpio;
	struct analog_device dev;
	uint32_t ce_pin;
	uint8_t address;
	uint8_t convergence_time;
	struct mutex lock;
};

static int _vl6180_read_reg(struct vl6180 *self, uint16_t reg, uint8_t *value){
	uint8_t tx[2] = { (uint8_t)(reg >> 8) & 0xff, (uint8_t)(reg & 0xff) };
	return i2c_transfer(self->i2c, self->address, tx, 2, value, 1, VL6180_I2C_TIMEOUT);
}

static int _vl6180_read_reg16(struct vl6180 *self, uint16_t reg, uint16_t *value){
	uint8_t tx[2] = { (uint8_t)(reg >> 8) & 0xff, (uint8_t)(reg & 0xff) };
	uint8_t rx[2] = {0, 0};
	int r = i2c_transfer(self->i2c, self->address, tx, 2, rx, 2, VL6180_I2C_TIMEOUT);
	if(r >= 0){
		*value = (uint16_t)(((int)rx[0] << 8) | rx[1]);
		return 1;
	}
	return r;
}

static int _vl6180_write_reg(struct vl6180 *self, uint16_t reg, uint8_t value){
	uint8_t tx[3] = { (uint8_t)(reg >> 8) & 0xff, (uint8_t)(reg & 0xff), value };
	return i2c_transfer(self->i2c, self->address, tx, 3, NULL, 0, VL6180_I2C_TIMEOUT);
}
#if 0
static int _vl6180_write_reg16(struct vl6180 *self, uint16_t reg, uint16_t value){
	uint8_t tx[4] = { (uint8_t)(reg >> 8) & 0xff, (uint8_t)(reg & 0xff), (uint8_t)(value >> 8), (uint8_t)(value) };
	return i2c_transfer(self->i2c, self->address, tx, 4, NULL, 0, VL6180_I2C_TIMEOUT);
}
#endif
#if 0
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
#endif 

#define CHECK(x) do { if((x) < 0) { printk(PRINT_ERROR "vl6180: i2c communication error!\n"); return -EIO; } } while(0)
static int _vl6180_configure_module(struct vl6180 *self){
	uint8_t is_unconfigured = 0;
	CHECK(_vl6180_read_reg(self, VL6180_REG_SYS_FRESH_OUT_OF_RESET, &is_unconfigured));
	if(!is_unconfigured) return 0;

	// Default initialization procedure according to app note below.
	//http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
	CHECK(_vl6180_write_reg(self, 0x0207, 0x01));
	CHECK(_vl6180_write_reg(self, 0x0208, 0x01));
	CHECK(_vl6180_write_reg(self, 0x0096, 0x00));
	CHECK(_vl6180_write_reg(self, 0x0097, 0xfd));
	CHECK(_vl6180_write_reg(self, 0x00e3, 0x00));
	CHECK(_vl6180_write_reg(self, 0x00e4, 0x04));
	CHECK(_vl6180_write_reg(self, 0x00e5, 0x02));
	CHECK(_vl6180_write_reg(self, 0x00e6, 0x01));
	CHECK(_vl6180_write_reg(self, 0x00e7, 0x03));
	CHECK(_vl6180_write_reg(self, 0x00f5, 0x02));
	CHECK(_vl6180_write_reg(self, 0x00d9, 0x05));
	CHECK(_vl6180_write_reg(self, 0x00db, 0xce));
	CHECK(_vl6180_write_reg(self, 0x00dc, 0x03));
	CHECK(_vl6180_write_reg(self, 0x00dd, 0xf8));
	CHECK(_vl6180_write_reg(self, 0x009f, 0x00));
	CHECK(_vl6180_write_reg(self, 0x00a3, 0x3c));
	CHECK(_vl6180_write_reg(self, 0x00b7, 0x00));
	CHECK(_vl6180_write_reg(self, 0x00bb, 0x3c));
	CHECK(_vl6180_write_reg(self, 0x00b2, 0x09));
	CHECK(_vl6180_write_reg(self, 0x00ca, 0x09));
	CHECK(_vl6180_write_reg(self, 0x0198, 0x01));
	CHECK(_vl6180_write_reg(self, 0x01b0, 0x17));
	CHECK(_vl6180_write_reg(self, 0x01ad, 0x00));
	CHECK(_vl6180_write_reg(self, 0x00ff, 0x05));
	CHECK(_vl6180_write_reg(self, 0x0100, 0x05));
	CHECK(_vl6180_write_reg(self, 0x0199, 0x05));
	CHECK(_vl6180_write_reg(self, 0x01a6, 0x1b));
	CHECK(_vl6180_write_reg(self, 0x01ac, 0x3e));
	CHECK(_vl6180_write_reg(self, 0x01a7, 0x1f));
	CHECK(_vl6180_write_reg(self, 0x0030, 0x00));

	//CHECK(_vl6180_write_reg(self, VL6180_REG_READOUT_AVERAGING_SAMPLE_PERIOD, 255));
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSALS_ANALOGUE_GAIN, 0x06));
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSRANGE_VHV_REPEAT_RATE, 0xFF));
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSALS_INTEGRATION_PERIOD, 0x63)); // Set ALS integration time to 100ms
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSRANGE_VHV_RECALIBRATE, 0x01)); // perform a single temperature calibration
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A)); // Set default ALS inter-measurement period to 100ms
	//Optional settings from datasheet
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09)); // Set default ranging inter-measurement period to 100ms
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32));
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01));
	//CHECK(_vl6180_write_reg16(self, VL6180_REG_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B ));
	//CHECK(_vl6180_write_reg16(self, VL6180_REG_SYSALS_INTEGRATION_PERIOD, 0x64));

	//CHECK(_vl6180_write_reg(self, VL6180_REG_READOUT_AVERAGING_SAMPLE_PERIOD,0x30));
	//CHECK(_vl6180_write_reg(self, VL6180_REG_SYSALS_ANALOGUE_GAIN,0x40));
	//CHECK(_vl6180_write_reg(self, VL6180_REG_FIRMWARE_RESULT_SCALER,0x01));

	CHECK(_vl6180_write_reg(self, VL6180_REG_SYS_FRESH_OUT_OF_RESET, 0));

	return 0;
}

#if 0
static const char *_vl6180_strerror(uint8_t status){
	static const char *str[16] = {
		"Success",
		"VCSEL continuity test",
		"VCSEL watchdog test",
		"VCSEL watchdog",
		"PLL1 Lock",
		"PLL2 Lock",
		"Early convergence",
		"Max convergence",
		"No target ignore",
		"Unknown",
		"Unknown",
		"Max SNR",
		"Raw range underflow",
		"Raw range overflow",
		"Range underflow",
		"Range overflow"
	};
	return str[(status >> 4) & 0xf];
}
#endif
static int _vl6180_analog_read(analog_device_t dev, unsigned int channel, float *value){
	struct vl6180 *self = container_of(dev, struct vl6180, dev.ops);
	if(channel == 0){
		thread_mutex_lock(&self->lock);
		_vl6180_write_reg(self, VL6180_REG_SYSRANGE_START, 0x01); //Start Single shot mode
		thread_sleep_ms((uint32_t)(4 + self->convergence_time + 5));
		uint8_t status;
		_vl6180_read_reg(self, VL6180_REG_RESULT_RANGE_STATUS, &status);
		_vl6180_write_reg(self, VL6180_REG_SYS_INTERRUPT_CLEAR, 0x07);
		if((status >> 4) != 0){
			//vl6180_debug(PRINT_ERROR "vl6180 (%s): ranging error %s\n", DEVICE_NAME(&self->dev), _vl6180_strerror(status));
			thread_mutex_unlock(&self->lock);
			return -EBUSY;
		}
		uint8_t val = 0;
		int r = _vl6180_read_reg(self, VL6180_REG_RESULT_RANGE_VAL, &val);
		thread_mutex_unlock(&self->lock);
		*value = (float)val;
		return r;
	} else {
		return -EINVAL;
	}
}

static int _vl6180_analog_write(analog_device_t dev, unsigned int channel, float value){
	return -1;
}

static const struct analog_device_ops _vl6180_analog_ops = {
	.read = _vl6180_analog_read,
	.write = _vl6180_analog_write
};

static int _vl6180_probe(void *fdt, int fdt_node){
	struct vl6180 *self = kzmalloc(sizeof(struct vl6180));
	
	DEVICE_REF("vl6180", i2c, i2c);
	DEVICE_REF("vl6180", gpio, gpio);
	self->ce_pin = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "ce_pin", 0);
	uint8_t new_addr = (uint8_t)fdt_get_int_or_default(fdt, fdt_node, "reg", VL6180_DEFAULT_ADDRESS);
	self->convergence_time = 31;

	thread_mutex_init(&self->lock);

	// address set to default for initialization
	self->address = VL6180_DEFAULT_ADDRESS;

	// boot the sensor
	gpio_set(self->gpio, self->ce_pin);
	thread_sleep_ms(10);

	uint8_t model = 0, rev_major = 0, rev_minor = 0, mod_major = 0, mod_minor = 0;
	uint16_t date = 0, time = 0;

	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODEL_ID, &model));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODEL_REV_MAJOR, &rev_major));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODEL_REV_MINOR, &rev_minor));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODULE_REV_MAJOR, &mod_major));
	CHECK(_vl6180_read_reg(self, VL6180_REG_ID_MODULE_REV_MINOR, &mod_minor));
	CHECK(_vl6180_read_reg16(self, VL6180_REG_ID_DATE, &date));
	CHECK(_vl6180_read_reg16(self, VL6180_REG_ID_TIME, &time));

	switch(model){
		case VL6180_MODEL_ID: {
			printk(PRINT_DEFAULT "vl6180: model rev: %d.%d, module rev: %d.%d, date: %d, time %d\n",
				rev_major,
				rev_minor,
				mod_major,
				mod_minor,
				date,
				time);
		} break;
		default: {
			printk(PRINT_ERROR "vl6180: unknown device (%02x)\n", model);
			return -ENODEV;
		} break;
	}

	CHECK(_vl6180_write_reg(self, VL6180_REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr));
	self->address = new_addr;

	printk("vl6180: using address %02x\n", self->address);

	CHECK(_vl6180_configure_module(self));

	analog_device_init(&self->dev, fdt, fdt_node, &_vl6180_analog_ops);
	analog_device_register(&self->dev);

	return 0;
}

static int _vl6180_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(vl6180, "fw,vl6180", _vl6180_probe, _vl6180_remove)
