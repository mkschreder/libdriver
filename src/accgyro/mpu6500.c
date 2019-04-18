/*
 * Copyright (C) 2017 Martin K. Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <errno.h>

#include <libfirmware/spi.h>
#include <libfirmware/i2c.h>
#include <libfirmware/driver.h>
#include <libfirmware/imu.h>

#include <libfdt/libfdt.h>

#define MPU6500_REQUEST_TIMEOUT 100

#define MPU_DEFAULT_I2C_ADDR    0x68
// MPU6050
#define MPU_RA_WHO_AM_I         0x75
#define MPU_RA_WHO_AM_I_LEGACY  0x00

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU_INQUIRY_MASK   0x7E

// WHO_AM_I register contents for MPU3050, 6050 and 6500
#define MPU6500_ID              (0x70)
#define MPUx0x0_ID              (0x68)
#define MPU9250_ID              (0x71)

#define MPU6500_BIT_RESET                   (0x80)

enum {
    MPU_GYRO_FSR_250DPS = 0,
    MPU_GYRO_FSR_500DPS,
    MPU_GYRO_FSR_1000DPS,
    MPU_GYRO_FSR_2000DPS,
};

enum {
    MPU_CLK_INTERNAL = 0,
    MPU_CLK_PLL
};

enum {
    MPU_ACC_FSR_2G = 0,
    MPU_ACC_FSR_4G,
    MPU_ACC_FSR_8G,
    MPU_ACC_FSR_16G
};

enum {
    MPU_FILTER_256HZ_NOLPF2 = 0,
    MPU_FILTER_188HZ,
    MPU_FILTER_98HZ,
    MPU_FILTER_42HZ,
    MPU_FILTER_20HZ,
    MPU_FILTER_10HZ,
    MPU_FILTER_5HZ,
    MPU_FILTER_2100HZ_NOLPF
};

#define MPU6500_TIMEOUT 100

struct mpu6500 {
    struct imu_device dev;
    i2c_device_t i2c;
    spi_device_t spi;
    uint8_t chip_id;
};

int _spi_read_reg(spi_device_t spi, uint8_t reg, uint8_t *data){
#if 0
    uint8_t tx[2] = {reg | 0x80, 0};
    uint8_t rx[2] = {0, 0};
    // try to read out the id of the chip
    if(spi_transfer(spi, tx, rx, 1, MPU6500_REQUEST_TIMEOUT) < 0) {
        return -EIO;
    }
    *data = rx[1];
#endif
    return 1;
}

static int _mpu6500_write_reg(struct mpu6500 *self, uint8_t reg, uint8_t data){
    if(self->i2c){
		uint8_t buf[2] = {reg, data};
        return i2c_transfer(self->i2c, MPU_DEFAULT_I2C_ADDR, buf, 2, NULL, 0, MPU6500_TIMEOUT);
    }
    return -EINVAL;
}

static int _mpu6500_read_buf(struct mpu6500 *self, uint8_t reg, uint8_t *data, size_t len){
    if(self->i2c){
        return i2c_transfer(self->i2c, MPU_DEFAULT_I2C_ADDR, &reg, 1, data, len, MPU6500_TIMEOUT);
    }
    return -EINVAL;
}

static void _mpu6500_configure(struct mpu6500 *self){
    _mpu6500_write_reg(self, MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    thread_sleep_ms(100);
    _mpu6500_write_reg(self, MPU_RA_SIGNAL_PATH_RESET, 0x07);
    thread_sleep_ms(100);
    _mpu6500_write_reg(self, MPU_RA_PWR_MGMT_1, 0);
    thread_sleep_ms(100);
    _mpu6500_write_reg(self, MPU_RA_PWR_MGMT_1, MPU_CLK_PLL);
    thread_sleep_ms(150);
    _mpu6500_write_reg(self, MPU_RA_GYRO_CONFIG, MPU_GYRO_FSR_2000DPS << 3);
    thread_sleep_ms(150);
    _mpu6500_write_reg(self, MPU_RA_ACCEL_CONFIG, MPU_ACC_FSR_2G << 3);
    thread_sleep_ms(150);
    _mpu6500_write_reg(self, MPU_RA_CONFIG, MPU_FILTER_2100HZ_NOLPF);
    thread_sleep_ms(150);
    _mpu6500_write_reg(self, MPU_RA_SMPLRT_DIV, 0);
    thread_sleep_ms(100);
    _mpu6500_write_reg(self, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
    //_mpu6500_write_reg(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
}

int _mpu6500_read(imu_device_t dev, struct imu_reading *data){
    struct mpu6500 *self = container_of(dev, struct mpu6500, dev.ops);
    uint8_t raw[6];
    _mpu6500_read_buf(self, MPU_RA_ACCEL_XOUT_H, raw, 6);

    int16_t acc[3];
    acc[0] = (int16_t)((raw[0] << 8) | raw[1]);
    acc[1] = (int16_t)((raw[2] << 8) | raw[3]);
    acc[2] = (int16_t)((raw[4] << 8) | raw[5]);

    data->ax = acc[0];
    data->ay = acc[1];
    data->az = acc[2];

	return 0;
}

static const struct imu_device_ops _imu_ops = {
    .read = _mpu6500_read
};

int _mpu6500_probe(void *fdt, int fdt_node){
    // find the spi device fdt node
	int bus = fdt_find_node_by_ref(fdt, fdt_node, "bus");
	if(bus < 0) {
		dbg_printk("mpu6500: nobus!\n");
		return -EINVAL;
	}

    spi_device_t spi = spi_find_by_node(fdt, bus);
    i2c_device_t i2c = i2c_find_by_node(fdt, bus);
    if(!spi && !i2c) {
		dbg_printk("mpu6500: nobus!\n");
		return -EINVAL;
	}

    uint8_t sig = 0;
    if(i2c){
		uint8_t reg = MPU_RA_WHO_AM_I;
        if(i2c_transfer(i2c, MPU_DEFAULT_I2C_ADDR, &reg, 1, &sig, 1, MPU6500_TIMEOUT) < 0){
            dbg_printk("mpu: errio\n");
            return -1;
        }

        if(sig != 0){
            dbg_printk("mpu@i2c: ");
            if(sig == MPU6500_ID) dbg_printk("mpu6500\n");
            else if(sig == MPU9250_ID) dbg_printk("mpu9250\n");
            else dbg_printk("unknown\n");
        } else {
            dbg_printk("mpu: fail\n");
        }
    } else if(spi){
        if(_spi_read_reg(spi, MPU_RA_WHO_AM_I, &sig) < 0){
            dbg_printk("mpu: errio\n");
            return -1;
        }

        if(sig != 0){
            dbg_printk("mpu@spi: ");
            if(sig == MPU6500_ID) dbg_printk("mpu6500\n");
            else if(sig == MPU9250_ID) dbg_printk("mpu9250\n");
            else dbg_printk("unknown\n");
        } else {
            dbg_printk("mpu: fail\n");
        }
    }

    struct mpu6500 *self = kzmalloc(sizeof(struct mpu6500));
    imu_device_init(&self->dev, fdt_node, &_imu_ops);
    self->i2c = i2c;
    self->spi = spi;
    self->chip_id = sig;
    _mpu6500_configure(self);
    imu_device_register(&self->dev);
    return 0;
}

int _mpu6500_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(mpu6500, "inv,mpu6500", _mpu6500_probe, _mpu6500_remove)
