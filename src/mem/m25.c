/*
 * This file is part of martink project.
 *
 * Copyright (c) 2010, Loughborough University - Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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
#include <libfirmware/memory.h>
#include <libfirmware/mtd.h>
#include <libfirmware/mutex.h>

#include <libfdt/libfdt.h>

/* Instruction Set */
#define M25_CMD_WREN 0x06      /* Write Enable */
#define M25_CMD_WRDI 0x04      /* Write Disable */
#define M25_CMD_RDID 0x9F      /* Read Identification */
#define M25_CMD_RDSR 0x05      /* Read Status Register */
#define M25_CMD_WRSR 0x01      /* Write Status Register */
#define M25_CMD_READ 0x03      /* Read Data Bytes */
#define M25_CMD_FAST_READ 0x0B /* Read Data Bytes at Higher Speed */
#define M25_CMD_PAGE_PROGRAM 0x02        /* Page Program */
#define M25_CMD_SECTOR_ERASE 0x20
#define M25_CMD_BLOCK_ERASE_32K 0x52        /* Sector Erase */
#define M25_CMD_BLOCK_ERASE_64K 0xD8        /* Sector Erase */
#define M25_CMD_BULK_ERASE 0xC7        /* Bulk Erase */
#define M25_CMD_DP 0xB9        /* Deep Power-down */
#define M25_CMD_RES 0xAB       /* Release from Deep Power-down */

/* Status Register Bits */
#define M25_SR_BIT_SRWD 0x80 /* Status Register Write Disable */
#define M25_SR_BIT_BP2 0x10  /* Block Protect 2 */
#define M25_SR_BIT_BP1 0x08  /* Block Protect 1 */
#define M25_SR_BIT_BP0 0x04  /* Block Protect 0 */
#define M25_SR_BIT_BP 0x1C   /* All Block Protect Bits */
#define M25_SR_BIT_WEL 0x02  /* Write Enable Latch */
#define M25_SR_BIT_BUSY 0x01  /* Write in Progress */

/* Do we use READ or FAST_READ to read? Fast by default */
#ifdef M25P16_CONF_READ_FAST
#define M25P16_READ_FAST M25P16_CONF_READ_FAST
#else
#define M25P16_READ_FAST 1
#endif

#define M25_SPI_TIMEOUT 200

struct m25 {
	spi_device_t spi;
	gpio_device_t gpio;
	uint32_t cs_pin;

	struct mtd_device_info info;
	struct memory_device dev;

	uint8_t sector[4096];
};

#if 0

static void _flash_wren(struct m25 *self) {
	select();
	write_byte(M25_CMD_WREN);
	deselect();

	while(!write_enabled())
		;
}

uint32_t m25_read(struct m25 *self, uint32_t addr, uint8_t *buff,
                           uint32_t buff_len) {
	uint32_t i;

	// wait for any writes to complete
	while(write_in_progress())
		;

	write_byte(M25_CMD_FAST_READ);

	write_byte(addr >> 16);
	write_byte(addr >> 8);
	write_byte(addr);

	// for fast read send a dummy byte
	write_byte(0);

	for(i = 0; i < buff_len; i++) {
		buff[i] = ~write_byte(0);
	}

	// ENERGEST_OFF(ENERGEST_TYPE_FLASH_READ);
	deselect();

	return i;
}

uint32_t m25_write(struct m25 *self, uint32_t addr,
                            const uint8_t *buff, uint32_t buff_len) {
	uint32_t i;
	// wait for previous write to complete
	while(write_in_progress())
		;

	// write enable
	_flash_wren(self);

	select();
	// ENERGEST_ON(ENERGEST_TYPE_FLASH_WRITE);
	write_byte(M25_CMD_PP);

	// write addr, msb first
	write_byte(addr >> 16);
	write_byte(addr >> 8);
	write_byte(addr);

	// write data
	for(i = 0; i < buff_len; i++) {
		write_byte(~buff[i]);
	}
	// ENERGEST_OFF(ENERGEST_TYPE_FLASH_WRITE);
	deselect();
	return i;
}

void m25_sector_erase(struct m25 *self, uint8_t s) {
	_flash_wren(self);

	select();
	// ENERGEST_ON(ENERGEST_TYPE_FLASH_WRITE);
	write_byte(M25_CMD_SE);
	write_byte(s);
	write_byte(0x00);
	write_byte(0x00);
	deselect();

	while(write_in_progress())
		;

	// ENERGEST_OFF(ENERGEST_TYPE_FLASH_WRITE);
}
#endif

static int _m25_spi_transfer(struct m25 *self, const uint8_t *tx, uint8_t *rx, size_t size){
	if(spi_transfer(self->spi, self->gpio, self->cs_pin, tx, rx, size, M25_SPI_TIMEOUT) < 0){
		return -EIO;
	}
	return (int)size;
}

static uint32_t _m25_read_id(struct m25 *self){
	uint8_t tx[4] = {M25_CMD_RDID, 0, 0, 0};
	uint8_t rx[4] = {0, 0, 0, 0};

	if(_m25_spi_transfer(self, tx, rx, 4) < 0){
		return 0;
	}

	return (uint32_t)(rx[1] << 24 | rx[2] << 16 | rx[3] << 8);
}

static uint8_t _m25_read_status(struct m25 *self) {
	uint8_t tx[2] = {M25_CMD_RDSR, 0};
	uint8_t rx[2] = {0, 0};

	if(_m25_spi_transfer(self, tx, rx, 2) < 0){
		return 0;
	}

	return rx[1];
}

static void _m25_wr_en(struct m25 *self, bool write) {
	uint8_t tx[1] = {(write)?M25_CMD_WREN:M25_CMD_WRDI};
	uint8_t rx[1] = {0};

	_m25_spi_transfer(self, tx, rx, 1);
}

static int _m25_wait_done(struct m25 *self) {
	for(unsigned c = 0; c < 2000; c++){
		uint8_t sts = _m25_read_status(self);
		if(!(sts & M25_SR_BIT_BUSY)){
			return 0;
		}
		thread_sleep_ms(1);
	}
	return -1;
}

/*
static int _m25_bulk_erase(struct m25 *self) {
	uint8_t tx[1] = {M25_CMD_BULK_ERASE};
	uint8_t rx[1] = {0};

	if(spi_transfer(self->spi, self->gpio, self->cs_pin, tx, rx, 1, M25_SPI_TIMEOUT) < 0){
		return -1;
	}

	return 0;
}
*/
static int _m25_read(struct m25 *self, size_t ad, uint8_t *data, size_t len){
	static const size_t sz = 8;
	for(size_t off = 0; off < len; off += sz){
		size_t rdad = ad + off;
		size_t rdsz = ((off + sz) > len)?(len - off):sz;
		uint8_t buf[8 + 4] = {
			M25_CMD_READ,
			(rdad >> 16) & 0xff,
			(rdad >> 8) & 0xff,
			(rdad >> 0) & 0xff};
		memset(buf + 4, 0xFF, rdsz);
		if(_m25_spi_transfer(self, buf, buf, rdsz + 4) < 0){
			printk("m25: could not verify write at %08x\n", off);
			return -1;
		}
		// copy data into output buffer
		memcpy(data + off, buf + 4, rdsz);
	}
	return (int)len;
}

static int _m25_write(struct m25 *self, size_t ad, const uint8_t *data, size_t len){
	static const size_t sz = 8;
	for(size_t off = 0; off < len; off += sz){
		size_t wrad = ad + off;
		size_t wrsz = ((off + sz) > len)?(len - off):sz;
		uint8_t buf[8 + 4] = {
			M25_CMD_PAGE_PROGRAM,
			(wrad >> 16) & 0xff,
			(wrad >> 8) & 0xff,
			(wrad >> 0) & 0xff};
		memcpy(buf + 4, &data[off], wrsz);

		_m25_wr_en(self, true);

		if(_m25_spi_transfer(self, buf, buf, wrsz + 4) < 0){
			printk("m25: could not write at %08x\n", off);
			return -1;
		}

		if(_m25_wait_done(self) < 0){
			printk("m25: write failed!\n");
			break;
		}

		_m25_wr_en(self, false);
	}
	return (int)len;
}

static int _m25_sector_erase(struct m25 *self, size_t addr){
	uint8_t buf[] = {
		M25_CMD_SECTOR_ERASE,
		(addr >> 16) & 0xff,
		(addr >> 8) & 0xff,
		(addr >> 0) & 0xff
	};

	_m25_wr_en(self, true);

	if(_m25_spi_transfer(self, buf, buf, sizeof(buf)) < 0){
		printk("m25: could not erase sector at %08x\n", addr);
		return -1;
	}

	_m25_wait_done(self);

	_m25_wr_en(self, false);

	return (int)self->info.erasesize;
}

static int _m25_memory_write(memory_device_t dev, size_t offset, const void *data, size_t len){
	struct m25 *self = (struct m25*)container_of(dev, struct m25, dev.ops);

	const uint8_t *dat = (const uint8_t*)data;
	for(size_t ptr = 0; ptr < len;){
		uint32_t off = offset + ptr;
		uint32_t sectoraddr = off & ~(self->info.erasesize - 1);
		uint32_t sectoroff = off & (self->info.erasesize - 1);
		uint32_t sectorwrsz = ((self->info.erasesize - sectoroff) > (len - ptr))?(len - ptr):(self->info.erasesize - sectoroff);
		//uint32_t pageaddr = off & ~(self->info.writesize - 1);

		// load the whole sector into memory
		if(_m25_read(self, sectoraddr, self->sector, self->info.erasesize) != (int)self->info.erasesize){
			printk("m25: read failed at %08x\n", sectoraddr);
			return -1;
		}

		// check if sector needs an erase
		for(size_t p = 0; p < sectorwrsz; p++){
			uint8_t sd = self->sector[sectoroff + p];
			uint8_t id = dat[ptr + p];
			if((sd | id) != sd){
				printk("m25: erase sector at %08x\n", offset + ptr);
				_m25_sector_erase(self, offset + ptr);
				break;
			}
		}

		// copy data into buffer
		memcpy(&self->sector[sectoroff], &dat[ptr], sectorwrsz);

		// write sector to flash
		printk("m25: write sector at %08x, size: %u\n", offset + ptr, sectorwrsz);
		if(_m25_write(self, offset + ptr, &self->sector[sectoroff], sectorwrsz) != (int)sectorwrsz){
			printk("m25: write failed at %08x\n", offset + ptr);
			return -1;
		}

		ptr += sectorwrsz;
	}
	return (int)len;
}

static int _m25_memory_read(memory_device_t dev, size_t offset, void *data, size_t len){
	struct m25 *self = (struct m25*)container_of(dev, struct m25, dev.ops);
	return _m25_read(self, offset, data, len);
}

static struct memory_device_ops _m25_memory_ops = {
	.write = _m25_memory_write,
	.read = _m25_memory_read
};

int _m25_probe(void *fdt, int fdt_node){
	spi_device_t spi = spi_find_by_ref(fdt, fdt_node, "spi");
	gpio_device_t gpio = gpio_find_by_ref(fdt, fdt_node, "gpio");
	int cs_pin = fdt_get_int_or_default(fdt, fdt_node, "cs_pin", -1);

	if(!spi){
		printk("m25: missing spi device!\n");
		return -1;
	}

	if(!gpio){
		printk("m25: missing gpio device!\n");
		return -1;
	}

	if(cs_pin < 0){
		printk("m25: missing cs_pin!\n");
		return -1;
	}

	struct m25 *self = kzmalloc(sizeof(struct m25));
	self->spi = spi;
	self->gpio = gpio;
	self->cs_pin = (uint32_t)cs_pin;

	gpio_set(self->gpio, self->cs_pin);

	uint32_t id = _m25_read_id(self);
	self->info.manufacturer = (uint8_t)((id >> 24) & 0xff);
	self->info.type = (uint8_t)((id >> 16) & 0xff);
	self->info.size = (uint8_t)((id >> 8) & 0xff);
	self->info.erasesize = 4096;
	self->info.writesize = 256;

	printk("m25: MFR %02x, TYPE: %02x, CAPACITY: %02x\n",
				 self->info.manufacturer,
				 self->info.type,
				 self->info.size);

	switch(self->info.manufacturer){
		case 0xef:
			printk("m25: found Winband flash!\n");
			break;
		default:
			printk("m25: unknown device!\n");
			return -1;
	}

	memory_device_init(&self->dev, fdt, fdt_node, &_m25_memory_ops);
	memory_device_register(&self->dev);

	return 0;
}

int _m25_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(m25, "fw,m25", _m25_probe, _m25_remove)
