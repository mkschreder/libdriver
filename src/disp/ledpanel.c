#include <errno.h>

#include <libfirmware/driver.h>
#include <libfirmware/gpio.h>
#include <libfirmware/display.h>
#include <libfdt/libfdt.h>

#include "font_8x8.h"
#include "font_5x7.h"
#include "font_16x16.h"

enum {
    LEDPANEL_PIN_A,
    LEDPANEL_PIN_B,
    LEDPANEL_PIN_C,
    LEDPANEL_PIN_D,
    LEDPANEL_PIN_GREEN,
    LEDPANEL_PIN_RED,
    LEDPANEL_PIN_LAT,
    LEDPANEL_PIN_CLK,
    LEDPANEL_PIN_OE,
};

struct ledpanel {
	struct display_device dev;
    gpio_device_t gpio;
    uint8_t *framebuffer;
    uint16_t width, height;
    uint8_t bpp;
    uint16_t fb_width;
};

void _ledpanel_task(void *data){
    struct ledpanel *self = (struct ledpanel*)data;
    int scroll = 0, scroll_cnt = 0;
    while(1){
        for(int line = 0; line < self->height; line++){
            if(line & 0x01) gpio_set(self->gpio, LEDPANEL_PIN_A);
            else gpio_reset(self->gpio, LEDPANEL_PIN_A);
            if(line & 0x02) gpio_set(self->gpio, LEDPANEL_PIN_B);
            else gpio_reset(self->gpio, LEDPANEL_PIN_B);
            if(line & 0x04) gpio_set(self->gpio, LEDPANEL_PIN_C);
            else gpio_reset(self->gpio, LEDPANEL_PIN_C);
            if(line & 0x08) gpio_set(self->gpio, LEDPANEL_PIN_D);
            else gpio_reset(self->gpio, LEDPANEL_PIN_D);

            for(int c = scroll; c < self->width + scroll; c++){
                int x = c % self->fb_width;
                int base = line * (self->fb_width * self->bpp) + x * self->bpp;
                int byte = base / 8;
                int bit = base & 0x07;
                bool r = self->framebuffer[byte] & (1 << bit);
                base++;
                byte = base / 8;
                bit = base & 0x07;
                bool g = self->framebuffer[byte] & (1 << bit);
                if(r)
                    gpio_reset(self->gpio, LEDPANEL_PIN_RED);
                else
                    gpio_set(self->gpio, LEDPANEL_PIN_RED);

                if(g)
                    gpio_reset(self->gpio, LEDPANEL_PIN_GREEN);
                else
                    gpio_set(self->gpio, LEDPANEL_PIN_GREEN);

                gpio_set(self->gpio, LEDPANEL_PIN_CLK);
                gpio_reset(self->gpio, LEDPANEL_PIN_CLK);
            }

            gpio_set(self->gpio, LEDPANEL_PIN_LAT);
            gpio_reset(self->gpio, LEDPANEL_PIN_LAT);

            gpio_reset(self->gpio, LEDPANEL_PIN_OE);
            int time = 500; while(--time) asm volatile ("nop");
            gpio_set(self->gpio, LEDPANEL_PIN_OE);

            // latch output
            //thread_sleeP_Ms(1);
        }
        scroll_cnt = (scroll_cnt + 1);
        if(scroll_cnt % 5 == 0) scroll = (scroll + 1) % self->fb_width;
    }
	thread_sleep_ms(10);
}

int _ledpanel_write_pixel(display_device_t dev, int x, int y, color_t color){
	struct ledpanel *self = container_of(dev, struct ledpanel, dev.ops);

	if(x < 0 || x >= self->fb_width || y < 0 || y >= self->height) return -1;

    for(int c = 0; c < 2; c++){
        int base = y * (self->fb_width * self->bpp) + x * self->bpp + c;
        int byte = base / 8;
        int bit = base & 0x7;
        if(color & (uint32_t)(1 << c)){
            self->framebuffer[byte] = (uint8_t)(self->framebuffer[byte] | (1 << bit));
        } else {
            self->framebuffer[byte] = (uint8_t)(self->framebuffer[byte] & ~(1 << bit));
        }
    }
    return 0;
}

static const struct display_device_ops _display_ops = {
	.write_pixel = _ledpanel_write_pixel
};

int _ledpanel_probe(void *fdt, int fdt_node){
    // find the gpio device for pins
	int node = fdt_find_node_by_ref(fdt, fdt_node, "pins");
	if(node < 0) {
		dbg_printk("ledpanel: nopins!\n");
		return -EINVAL;
	}

    gpio_device_t gpio = gpio_find_by_node(fdt, node);
    if(!gpio) {
		dbg_printk("ledpanel: nogpio!\n");
		return -EINVAL;
	}

	int width = fdt_get_int_or_default(fdt, fdt_node, "width", 64);
	int height = fdt_get_int_or_default(fdt, fdt_node, "height", 8);
	int bpp = fdt_get_int_or_default(fdt, fdt_node, "bpp", 3);
	int fb_width = fdt_get_int_or_default(fdt, fdt_node, "fb_width", width);

    // reset communication lines
    gpio_reset(gpio, LEDPANEL_PIN_A);
    gpio_reset(gpio, LEDPANEL_PIN_B);
    gpio_reset(gpio, LEDPANEL_PIN_C);
    gpio_reset(gpio, LEDPANEL_PIN_D);
    gpio_reset(gpio, LEDPANEL_PIN_GREEN);
    gpio_reset(gpio, LEDPANEL_PIN_RED);
    gpio_reset(gpio, LEDPANEL_PIN_LAT);
    gpio_reset(gpio, LEDPANEL_PIN_CLK);
    gpio_reset(gpio, LEDPANEL_PIN_OE);

    struct ledpanel *self = kzmalloc(sizeof(struct ledpanel));
    if(!self) return -ENOMEM;

	display_device_init(&self->dev, fdt_node, &_display_ops);
	display_device_register(&self->dev);

    self->framebuffer = kzmalloc((size_t)((fb_width * height * bpp) / 8));
    self->gpio = gpio;
    self->width = (uint16_t)width;
    self->height = (uint16_t)height;
    self->bpp = (uint8_t)bpp;
    self->fb_width = (uint16_t)fb_width;

    if(!self->framebuffer){
        dbg_printk("ledpanel: fbfail\n");
        kfree(self);
        return -1;
    }
/*
    const char *str = ">>> Need an engineer? Call now! <<<";
    _ledpanel_write(self, 0, str, (int)strlen(str), 1);
    const char *str2 = "0733387694 | Martin | 0733387694  ";
    _ledpanel_write(self, 1, str2, (int)strlen(str), 3);
    //_ledpanel_write_bitmap(self, 256, 0, _smiley, 16, 16);
*/
    if(thread_create(
		  _ledpanel_task,
		  "lp",
		  180,
		  self,
		  1,
		  NULL) < 0)
        dbg_printk("ledpanel: taskfail\n");
    else
        dbg_printk("ledpanel: ok\n");
    return 0;
}

int _ledpanel_remove(void *fdt, int node){
    return -1;
}

DEVICE_DRIVER(ledpanel, "mk,ledpanel", _ledpanel_probe, _ledpanel_remove)
