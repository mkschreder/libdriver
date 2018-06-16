#include <errno.h>

#include <libfirmware/i2c.h>
#include <libfirmware/driver.h>
#include <libfirmware/baro.h>
//#include <libfirmware/barometer.h>

#include <libfdt/libfdt.h>

#define baro_debug dbg_printk

#define BMP280_DEFAULT_I2C_ADDRESS           (0x76)
#define BMP280_DEFAULT_CHIP_ID               (0x58)

#define BMP280_CHIP_ID_REG                   (0xD0)  /* Chip ID Register */
#define BMP280_RST_REG                       (0xE0)  /* Softreset Register */
#define BMP280_STAT_REG                      (0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG                    (0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
#define BMP280_FORCED_MODE                   (0x01)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE               (6)

#define BMP280_OVERSAMP_SKIPPED          (0x00)
#define BMP280_OVERSAMP_1X               (0x01)
#define BMP280_OVERSAMP_2X               (0x02)
#define BMP280_OVERSAMP_4X               (0x03)
#define BMP280_OVERSAMP_8X               (0x04)
#define BMP280_OVERSAMP_16X              (0x05)

// configure pressure and temperature oversampling, forced sampling mode
#define BMP280_PRESSURE_OSR              (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR           (BMP280_OVERSAMP_1X)
#define BMP280_MODE                      (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_FORCED_MODE)

#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms

struct bmp280_calib_data {
    uint16_t dig_T1; /* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
    int32_t t_fine; /* calibration t_fine data */
};

struct bmp280 {
    i2c_device_t i2c;
    uint8_t address;
    struct bmp280_calib_data cal_data;
	struct baro_device dev;
};

int fdt_find_node_by_ref(void *fdt, int fdt_node, const char *prop_ref){
	/* Find the node referenced by pins label and then parse out the pins of that node for gpio references */
	int len = 0;
	const fdt32_t *val = (const fdt32_t*)fdt_getprop(fdt, fdt_node, prop_ref, &len);
	if(len != 4) return -1;

	uint32_t pins_handle = (uint32_t)fdt32_to_cpu(*val);

	int node = fdt_node_offset_by_phandle(fdt, pins_handle);
	if(node < 0) return -1;
	return node;
}
/*
static int _bmp280_read_pressure(baro_device_t dev, float *pressure){
    // start measurement
    uint32_t delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16);
    // set oversampling + power mode (forced), and start sampling
    if(i2c_write_reg(self->i2c, self->address, BMP280_CTRL_MEAS_REG, BMP280_MODE) < 0) return -EIO;
    thread_sleep_ms(delay);

    uint8_t data[BMP280_DATA_FRAME_SIZE];

    // read data from sensor
    i2c_read_buf(self->i2c, self->address, BMP280_PRESSURE_MSB_REG, data, BMP280_DATA_FRAME_SIZE);
    int32_t up = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    int32_t ut = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
}
*/

int _bmp280_read(baro_device_t dev, struct baro_reading *out){
	struct bmp280 *self = container_of(dev, struct bmp280, dev.ops);

	// start measurement
    // set oversampling + power mode (forced), and start sampling
    if(i2c_write_reg(self->i2c, self->address, BMP280_CTRL_MEAS_REG, BMP280_MODE) < 0){
		baro_debug("baro: errstart\n");
		return -EIO;
	}

    uint32_t delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16);
	baro_debug("baro: sleep %dms\n", delay);
    thread_sleep_ms(delay);

    // read data from sensor
    uint8_t data[BMP280_DATA_FRAME_SIZE];
    if(i2c_read_buf(self->i2c, self->address, BMP280_PRESSURE_MSB_REG, data, BMP280_DATA_FRAME_SIZE) < 0){
		baro_debug("baro: noresult\n");
		return -EIO;
	}
    int32_t up = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    int32_t ut = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));

	// compensate temperature
	{
		int32_t var1, var2;
		var1 = ((((ut >> 3) - ((int32_t)self->cal_data.dig_T1 << 1))) * ((int32_t)self->cal_data.dig_T2)) >> 11;
		var2  = (((((ut >> 4) - ((int32_t)self->cal_data.dig_T1)) * ((ut >> 4) - ((int32_t)self->cal_data.dig_T1))) >> 12) * ((int32_t)self->cal_data.dig_T3)) >> 14;
		self->cal_data.t_fine = var1 + var2;
		ut = (self->cal_data.t_fine * 5 + 128) >> 8;
	}

	// compensate pressure
	{
		int64_t var1, var2, p;
		var1 = ((int64_t)self->cal_data.t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)self->cal_data.dig_P6;
		var2 = var2 + ((var1*(int64_t)self->cal_data.dig_P5) << 17);
		var2 = var2 + (((int64_t)self->cal_data.dig_P4) << 35);
		var1 = ((var1 * var1 * (int64_t)self->cal_data.dig_P3) >> 8) + ((var1 * (int64_t)self->cal_data.dig_P2) << 12);
		var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)self->cal_data.dig_P1) >> 33;
		if (var1 == 0)
			return 0;
		p = 1048576 - up;
		p = (((p << 31) - var2) * 3125) / var1;
		var1 = (((int64_t)self->cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((int64_t)self->cal_data.dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)self->cal_data.dig_P7) << 4);
		up = (int32_t)(p / 256);
	}

	out->temperature = (float)ut;
	out->pressure = (float)up;

	return 0;
}

static const struct baro_device_ops _baro_ops = {
	.read = _bmp280_read
};

int _bmp280_probe(void *fdt, int fdt_node){
    // find the i2c device fdt node
	int bus = fdt_find_node_by_ref(fdt, fdt_node, "bus");
	if(bus < 0) {
		dbg_printk("bmp280: nobus!\n");
		return -EINVAL;
	}

    i2c_device_t i2c = i2c_find_by_node(fdt, bus);
    if(!i2c) {
		dbg_printk("bmp280: noi2c!\n");
		return -EINVAL;
	}

    uint8_t address = (uint8_t)fdt_get_int_or_default(fdt, (int)fdt_node, "address", BMP280_DEFAULT_I2C_ADDRESS);

    // try to identify the chip
    uint8_t chip_id = 0;
    if(i2c_read_reg(i2c, address, BMP280_CHIP_ID_REG, &chip_id) != 1){
		dbg_printk("bmp280: errio!\n");
        return -EIO;
    }

    if (chip_id != BMP280_DEFAULT_CHIP_ID) {
		dbg_printk("bmp280: noid (%02x)!\n", chip_id);
        return -1;
    }

    // create the new device
    struct bmp280 *self = kzmalloc(sizeof(struct bmp280));
	baro_device_init(&self->dev, fdt_node, &_baro_ops);
    self->i2c = i2c;
    self->address = address;

    // read calibration
    i2c_read_buf(self->i2c, self->address, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, &self->cal_data, 24);

    // set oversampling + power mode (forced), and start sampling
    //i2c_write_reg(self->i2c, self->address, BMP280_CTRL_MEAS_REG, BMP280_MODE);

	baro_device_register(&self->dev);
/*
    // these are dummy as temperature is measured as part of pressure
    baro->ut_delay = 0;
    baro->get_ut = bmp280_get_ut;
    baro->start_ut = bmp280_start_ut;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = bmp280_start_up;
    baro->get_up = bmp280_get_up;
    baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
    baro->calculate = bmp280_calculate;
*/

	printk("bmp280: ok, id %02x at %02x\n", chip_id, address);
    return 0;
}

int _bmp280_remove(void *fdt, int fdt_node){
    return -1;
}

DEVICE_DRIVER(bmp280, "bosch,bmp280", _bmp280_probe, _bmp280_remove)

/*
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)self->cal_data.dig_T1 << 1))) * ((int32_t)self->cal_data.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)self->cal_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)self->cal_data.dig_T1))) >> 12) * ((int32_t)self->cal_data.dig_T3)) >> 14;
    bmp280_cal.t_fine = var1 + var2;
    T = (self->cal_data.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280_compensate_P(int32_t adc_P){
    int64_t var1, var2, p;
    var1 = ((int64_t)self->cal_data.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)self->cal_data.dig_P6;
    var2 = var2 + ((var1*(int64_t)self->cal_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)self->cal_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)self->cal_data.dig_P3) >> 8) + ((var1 * (int64_t)self->cal_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)self->cal_data.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)self->cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)self->cal_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)self->cal_data.dig_P7) << 4);
    return (uint32_t)p;
}

STATIC_UNIT_TESTED void bmp280_calculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    int32_t t;
    uint32_t p;
    t = bmp280_compensate_T(bmp280_ut);
    p = bmp280_compensate_P(bmp280_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;
}
*/
