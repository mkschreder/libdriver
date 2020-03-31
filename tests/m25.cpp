
#include "mock/printk.hpp"
#include "mock/thread.hpp"
#include "mock/driver.hpp"
#include "mock/memory.hpp"

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

extern "C" {
#include <libfirmware/spi.h>
#include <libfirmware/gpio.h>
#include "m25-dtb.h"

struct mock_spi {
	struct spi_device dev;
};

static uint8_t _flash[1024 * 1024];

static int _mock_spi_transfer(spi_device_t dev, gpio_device_t gpio, uint32_t cs_pin, const void *tx_data, void *rx_data, size_t size, msec_t timeout){
	struct mock_spi *self = container_of(dev, struct mock_spi, dev.ops);
	uint8_t *rx = (uint8_t*)rx_data;
	const uint8_t *tx = (const uint8_t*)tx_data;
	switch(tx[0]){
		case M25_CMD_WREN: {
			printf("mock m25: write enable\n");
		} break;
		case M25_CMD_WRDI: {
			printf("mock m25: write disable\n");
		} break;
		case M25_CMD_RDSR: {
			printf("mock m25: read status\n");
			EXPECT_EQ(size, 2);
			rx[1] = 0x00;
		} break;
		case M25_CMD_RDID: {
			printf("mock m25: read id\n");
			EXPECT_EQ(size, 4);
			EXPECT_NE(tx_data, nullptr);
			EXPECT_NE(rx_data, nullptr);
			rx[1] = 0x4f;
			rx[2] = 0x20;
			rx[3] = 0x15;
		} break;
		case M25_CMD_READ: {
			uint32_t addr = rx[1] << 16 | rx[2] << 8 | rx[3];
			printf("mock m25: read %u, size: %lu\n", addr, size - 4);
			EXPECT_LE(addr, sizeof(_flash));
			EXPECT_GE(size, 5);
			memcpy(rx + 4, _flash + addr, size - 4);
		} break;
		case M25_CMD_PAGE_PROGRAM: {
			uint32_t addr = rx[1] << 16 | rx[2] << 8 | rx[3];
			printf("mock m25: write %u, size: %lu\n", addr, size - 4);
			EXPECT_LE(addr, sizeof(_flash));
			EXPECT_GE(size, 5);
			memcpy(_flash + addr, tx + 4, size - 4);
		} break;
		case M25_CMD_SECTOR_ERASE: {
			uint32_t addr = rx[1] << 16 | rx[2] << 8 | rx[3];
			printf("mock m25: sector erase %u, size: %lu\n", addr, size - 4);
		} break;
		default:
			printf("mock m25: unknown transfer %lu bytes\n", size);
	};
	return size;
}

static const struct spi_device_ops _mock_spi_ops = {
	.transfer = _mock_spi_transfer
};

static int _mock_spi_probe(void *fdt, int fdt_node){
	struct mock_spi *self = (struct mock_spi*)kzmalloc(sizeof(struct mock_spi));
	spi_device_init(&self->dev, fdt, fdt_node, &_mock_spi_ops);
	spi_device_register(&self->dev);
	memset(_flash, 0xff, sizeof(_flash));
	return 0;
}

static int _mock_spi_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(spi, "mock,spi", _mock_spi_probe, _mock_spi_remove)

struct mock_gpio {
	struct gpio_device dev;
};

static int _mock_gpio_write_pin(gpio_device_t dev, uint32_t pin, bool value){
	printf("gpio write: %d, %d\n", pin, value);
	return 0;
}

static int _mock_gpio_read_pin(gpio_device_t dev, uint32_t pin, bool *value){
	printf("gpio read: %d\n", pin);
	return 0;
}

static const struct gpio_device_ops _mock_gpio_ops = {
    .write_pin = _mock_gpio_write_pin,
    .read_pin = _mock_gpio_read_pin
};

static int _mock_gpio_probe(void *fdt, int fdt_node){
	struct mock_gpio *self = (struct mock_gpio*)kzmalloc(sizeof(struct mock_gpio));
	gpio_device_init(&self->dev, fdt, fdt_node, &_mock_gpio_ops);
	gpio_device_register(&self->dev);
	return 0;
}

static int _mock_gpio_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(gpio, "mock,gpio", _mock_gpio_probe, _mock_gpio_remove)
}

class M25Test : public ::testing::Test {
	protected:
	memory_device_t mem;
	void SetUp() {
		probe_device_drivers(m25_dtb);
		mem = memory_find(m25_dtb, "/m25");
	}
	void TearDown() {
		//remove_device_drivers(m25_dtb);
	}
	void clock(unsigned cycles) {

	}
};

TEST_F(M25Test, check_init) {
	EXPECT_NE(memory_find(m25_dtb, "/m25"), nullptr);
}

TEST_F(M25Test, check_write) {
	uint8_t data[4096 + 10];
	memset(data, 0xFE, sizeof(data));
	memset(_flash, 0x00, sizeof(_flash));
	EXPECT_EQ(memory_write(mem, 4090, data, 12), 12);
	EXPECT_EQ(_flash[4090], 0xFE);
	EXPECT_EQ(_flash[4090 + 11], 0xFE);
}

TEST_F(M25Test, check_read) {
	uint8_t data[32];
	memset(data, 0, sizeof(data));
	EXPECT_EQ(memory_read(mem, 0, data, 32), 32);
	EXPECT_EQ(data[0], 0xff);
	_flash[0] = 0x10;
	_flash[1] = 0x11;
	_flash[2] = 0x12;
	_flash[3] = 0x13;
	EXPECT_EQ(memory_read(mem, 0, data, 4), 4);
	EXPECT_EQ(data[0], 0x10);
	EXPECT_EQ(data[1], 0x11);
	EXPECT_EQ(data[2], 0x12);
	EXPECT_EQ(data[3], 0x13);
}

int main(int argc, char **argv) {
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
