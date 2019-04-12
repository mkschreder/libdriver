#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <libfirmware/driver.h>
#include <libfirmware/memory.h>
#include <libfirmware/gpio.h>
#include <libfirmware/mutex.h>
#include <libfirmware/i2c.h>

#include <libfdt/libfdt.h>

//#define AT24_PAGE_SIZE 32

struct at24 {
    i2c_device_t i2c;
    uint8_t addr;
    struct memory_device dev;
};

static int _at24_read(memory_device_t dev, size_t offset, void *data, size_t size){
    if(!dev) return -EINVAL;
    struct at24 *self = container_of(dev, struct at24, dev.ops);
    return i2c_read8_buf(self->i2c, self->addr, (uint8_t)offset, data, size);
}

static int _at24_write(memory_device_t dev, size_t offset, const void *data, size_t _size){
    if(!dev) return -EINVAL;
    struct at24 *self = container_of(dev, struct at24, dev.ops);
    size_t size = _size;
	uint8_t *buf = (uint8_t*)data;
    // write one byte at a time (guaranteed support on all devices)
    while(size--){
        //if(i2c_write16_reg8(self->i2c, self->addr, (uint16_t)offset, *buf++) < 0){
        if(i2c_write8_reg8(self->i2c, self->addr, (uint8_t)offset, *buf++) < 0){
            return -EFAULT;
        }
        // twr is at least 10ms
        thread_sleep_ms(10);
        offset++;
    }
    return (int)(_size - size);
}

static struct memory_device_ops _at24_memory_ops = {
    .read = _at24_read,
    .write = _at24_write
};

static int _at24_probe(void *fdt, int fdt_node) {
	i2c_device_t i2c = i2c_find_by_ref(fdt, fdt_node, "i2c");
	uint8_t addr = (uint8_t)fdt_get_int_or_default(fdt, fdt_node, "reg", 0x32);

	if(!i2c){
		printk("at24: no i2c\n");
		return -1;
	}

	struct at24 *self = kzmalloc(sizeof(struct at24));
	if(!self){
		printk("at24: nomem\n");
		return -1;
	}

	self->addr = addr;
	self->i2c = i2c;

	memory_device_init(&self->dev, fdt, fdt_node, &_at24_memory_ops);
	memory_device_register(&self->dev);

	printk("at24: ready (addr %02x)\n", addr);

	return 0;
}

static int _at24_remove(void *fdt, int fdt_node) {
	memory_device_t mem = memory_find_by_node(fdt, fdt_node);
	if(!mem) return -1;
	struct at24 *self = container_of(mem, struct at24, dev.ops);
	kfree(self);
	return 0;
}

DEVICE_DRIVER(at24, "fw,at24", _at24_probe, _at24_remove)

