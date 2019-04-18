/*
    This file is part of martink project.

    martink firmware project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    martink firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with martink firmware.  If not, see <http://www.gnu.org/licenses/>.

    Author: Martin K. Schröder
    Email: info@fortmax.se
    Github: https://github.com/mkschreder
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <errno.h>

#include <libfirmware/driver.h>
#include <libfirmware/display.h>
#include <libfirmware/spi.h>
#include <libfirmware/gpio.h>

#include <libfdt/libfdt.h>

enum {
    CS_PIN,
    RST_PIN,
    DC_PIN
};

#define CS_HI       gpio_set(self->gpio, CS_PIN)
#define CS_LO       gpio_reset(self->gpio, CS_PIN)
#define RST_HI      gpio_set(self->gpio, RST_PIN)
#define RST_LO      gpio_reset(self->gpio, RST_PIN)
#define DC_HI       gpio_set(self->gpio, DC_PIN)
#define DC_LO       gpio_reset(self->gpio, DC_PIN)

#define ILI9340_TFTWIDTH  240
#define ILI9340_TFTHEIGHT 320

#define ILI9340_NOP     0x00
#define ILI9340_SWRESET 0x01
#define ILI9340_RDDID   0x04
#define ILI9340_RDDST   0x09

#define ILI9340_SLPIN   0x10
#define ILI9340_SLPOUT  0x11
#define ILI9340_PTLON   0x12
#define ILI9340_NORON   0x13

#define ILI9340_RDMODE  0x0A
#define ILI9340_RDMADCTL  0x0B
#define ILI9340_RDPIXFMT  0x0C
#define ILI9340_RDIMGFMT  0x0A
#define ILI9340_RDSELFDIAG  0x0F

#define ILI9340_INVOFF  0x20
#define ILI9340_INVON   0x21
#define ILI9340_GAMMASET 0x26
#define ILI9340_DISPOFF 0x28
#define ILI9340_DISPON  0x29

#define ILI9340_CASET   0x2A
#define ILI9340_PASET   0x2B
#define ILI9340_RAMWR   0x2C
#define ILI9340_RAMRD   0x2E

#define ILI9340_PTLAR   0x30
#define ILI9340_MADCTL  0x36


#define ILI9340_MADCTL_MY  0x80
#define ILI9340_MADCTL_MX  0x40
#define ILI9340_MADCTL_MV  0x20
#define ILI9340_MADCTL_ML  0x10
#define ILI9340_MADCTL_RGB 0x00
#define ILI9340_MADCTL_BGR 0x08
#define ILI9340_MADCTL_MH  0x04

#define ILI9340_PIXFMT  0x3A

#define ILI9340_FRMCTR1 0xB1
#define ILI9340_FRMCTR2 0xB2
#define ILI9340_FRMCTR3 0xB3
#define ILI9340_INVCTR  0xB4
#define ILI9340_DFUNCTR 0xB6

#define ILI9340_PWCTR1  0xC0
#define ILI9340_PWCTR2  0xC1
#define ILI9340_PWCTR3  0xC2
#define ILI9340_PWCTR4  0xC3
#define ILI9340_PWCTR5  0xC4
#define ILI9340_VMCTR1  0xC5
#define ILI9340_VMCTR2  0xC7

#define ILI9340_RDID1   0xDA
#define ILI9340_RDID2   0xDB
#define ILI9340_RDID3   0xDC
#define ILI9340_RDID4   0xDD

#define ILI9340_GMCTRP1 0xE0
#define ILI9340_GMCTRN1 0xE1

// Color definitions
#define ILI9340_BLACK   0x0000
#define ILI9340_BLUE    0x001F // 5 bit
#define ILI9340_RED     0xF800 // 5 bit
#define ILI9340_GREEN   0x07E0 // 6 bit
#define ILI9340_CYAN    0x07FF
#define ILI9340_MAGENTA 0xF81F
#define ILI9340_YELLOW  0xFFE0  
#define ILI9340_WHITE   0xFFFF

#define ILI9340_TRANSFER_TIMEOUT_MS 10

struct ili9340 {
    struct display_device dev;

    spi_device_t spi;
    gpio_device_t gpio; // for control lines

    color_rgb_565_t *framebuffer;
    int width, height, bpp;
};

static int _wr_command(struct ili9340 *self, uint8_t c) {
    uint8_t tx[1] = {c};
    uint8_t rx[1] = {0};
    DC_LO;
    return spi_transfer(self->spi, self->gpio, DC_PIN, tx, rx, 1, ILI9340_TRANSFER_TIMEOUT_MS);
}

static int _wr_data(struct ili9340 *self, uint8_t c) {
    uint8_t tx[1] = {c};
    uint8_t rx[1] = {0};

    DC_HI;
    return spi_transfer(self->spi, self->gpio, DC_PIN, tx, rx, 1, ILI9340_TRANSFER_TIMEOUT_MS);
}

void _ili9340_reinit_display(struct ili9340 *self){
    RST_LO;
    RST_HI;
    thread_sleep_ms(5);
    RST_LO;
    thread_sleep_ms(20);
    RST_HI;
    thread_sleep_ms(150);

    CS_LO;

    _wr_command(self, 0xEF);
    _wr_data(self, 0x03);
    _wr_data(self, 0x80);
    _wr_data(self, 0x02);

    _wr_command(self, 0xCF);  
    _wr_data(self, 0x00); 
    _wr_data(self, 0XC1); 
    _wr_data(self, 0X30); 

    _wr_command(self, 0xED);  
    _wr_data(self, 0x64); 
    _wr_data(self, 0x03); 
    _wr_data(self, 0X12); 
    _wr_data(self, 0X81); 

    _wr_command(self, 0xE8);  
    _wr_data(self, 0x85); 
    _wr_data(self, 0x00); 
    _wr_data(self, 0x78); 

    _wr_command(self, 0xCB);  
    _wr_data(self, 0x39); 
    _wr_data(self, 0x2C); 
    _wr_data(self, 0x00); 
    _wr_data(self, 0x34); 
    _wr_data(self, 0x02); 

    _wr_command(self, 0xF7);  
    _wr_data(self, 0x20); 

    _wr_command(self, 0xEA);  
    _wr_data(self, 0x00); 
    _wr_data(self, 0x00); 

    _wr_command(self, ILI9340_PWCTR1);    //Power control 
    _wr_data(self, 0x23);   //VRH[5:0] 

    _wr_command(self, ILI9340_PWCTR2);    //Power control 
    _wr_data(self, 0x10);   //SAP[2:0];BT[3:0] 

    _wr_command(self, ILI9340_VMCTR1);    //VCM control 
    _wr_data(self, 0x3e); //
    _wr_data(self, 0x28); 

    _wr_command(self, ILI9340_VMCTR2);    //VCM control2 
    _wr_data(self, 0x86);  //--

    _wr_command(self, ILI9340_MADCTL);    // Memory Access Control 
    _wr_data(self, ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);

    _wr_command(self, ILI9340_PIXFMT);    
    _wr_data(self, 0x55); 

    _wr_command(self, ILI9340_FRMCTR1);    
    _wr_data(self, 0x00);  
    _wr_data(self, 0x18); 

    _wr_command(self, ILI9340_DFUNCTR);    // Display Function Control 
    _wr_data(self, 0x08); 
    _wr_data(self, 0x82);
    _wr_data(self, 0x27);  

    _wr_command(self, 0xF2);    // 3Gamma Function Disable 
    _wr_data(self, 0x00); 

    _wr_command(self, ILI9340_GAMMASET);    //Gamma curve selected 
    _wr_data(self, 0x01); 

    _wr_command(self, ILI9340_GMCTRP1);    //Set Gamma 
    _wr_data(self, 0x0F); 
    _wr_data(self, 0x31); 
    _wr_data(self, 0x2B); 
    _wr_data(self, 0x0C); 
    _wr_data(self, 0x0E); 
    _wr_data(self, 0x08); 
    _wr_data(self, 0x4E); 
    _wr_data(self, 0xF1); 
    _wr_data(self, 0x37); 
    _wr_data(self, 0x07); 
    _wr_data(self, 0x10); 
    _wr_data(self, 0x03); 
    _wr_data(self, 0x0E); 
    _wr_data(self, 0x09); 
    _wr_data(self, 0x00); 

    _wr_command(self, ILI9340_GMCTRN1);    //Set Gamma 
    _wr_data(self, 0x00); 
    _wr_data(self, 0x0E); 
    _wr_data(self, 0x14); 
    _wr_data(self, 0x03); 
    _wr_data(self, 0x11); 
    _wr_data(self, 0x07); 
    _wr_data(self, 0x31); 
    _wr_data(self, 0xC1); 
    _wr_data(self, 0x48); 
    _wr_data(self, 0x08); 
    _wr_data(self, 0x0F); 
    _wr_data(self, 0x0C); 
    _wr_data(self, 0x31); 
    _wr_data(self, 0x36); 
    _wr_data(self, 0x0F); 

    _wr_command(self, ILI9340_SLPOUT);    //Exit Sleep 
    thread_sleep_ms(120);
    _wr_command(self, ILI9340_DISPON);    //Display on

    CS_HI;
}

/*
void ili9340_setScrollMargins(struct ili9340 *self, uint16_t top, uint16_t bottom) {
    // Did not pass in VSA as TFA+VSA=BFA must equal 320
    _wr_command(self, 0x33); // Vertical Scroll definition.
    _wr_data16(self, top);
    _wr_data16(self, 320-(top+bottom));
    _wr_data16(self, bottom); 
}
*/
static void _ili9340_setAddrWindow(struct ili9340 *self, int x0, int y0, int x1, int y1){
    _wr_command(self, ILI9340_CASET); // Column addr set
    _wr_data(self, (uint8_t)(x0 >> 8));
    _wr_data(self, (uint8_t)(x0 & 0xFF));     // XSTART 
    _wr_data(self, (uint8_t)(x1 >> 8));
    _wr_data(self, (uint8_t)(x1 & 0xFF));     // XEND

    _wr_command(self, ILI9340_PASET); // Row addr set
    _wr_data(self, (uint8_t)(y0 >> 8));
    _wr_data(self, (uint8_t)(y0 & 0xff));     // YSTART
    _wr_data(self, (uint8_t)(y1 >> 8));
    _wr_data(self, (uint8_t)(y1 & 0xff));     // YEND
}
/*
void ili9340_setRotation(struct ili9340 *self, uint8_t m) {
    _wr_command(self, ILI9340_MADCTL);
    int rotation = m % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            _wr_data(self, ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
            self->screen_width  = ILI9340_TFTWIDTH;
            self->screen_height = ILI9340_TFTHEIGHT;
            break;
        case 1:
            _wr_data(self, ILI9340_MADCTL_MV | ILI9340_MADCTL_BGR);
            self->screen_width  = ILI9340_TFTHEIGHT;
            self->screen_height = ILI9340_TFTWIDTH;
            break;
        case 2:
            _wr_data(self, ILI9340_MADCTL_MY | ILI9340_MADCTL_BGR);
            self->screen_width  = ILI9340_TFTWIDTH;
            self->screen_height = ILI9340_TFTHEIGHT;
            break;
        case 3:
            _wr_data(self, ILI9340_MADCTL_MV | ILI9340_MADCTL_MY | ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
            self->screen_width  = ILI9340_TFTHEIGHT;
            self->screen_height = ILI9340_TFTWIDTH;
            break;
    }
}*/

static void _ili9340_refresh(display_device_t dev) {
    struct ili9340 *self = container_of(dev, struct ili9340, dev.ops);
    CS_LO;

    _ili9340_setAddrWindow(self, 0, 0, self->width, self->height);
    _wr_command(self, ILI9340_RAMWR); // write to RAM

    DC_HI;

    //spi_writereadbyte(color >> 8);
    //spi_writereadbyte(color);

    CS_HI;
}

int _ili9340_write_pixel(display_device_t dev, int x, int y, uint32_t color){
    struct ili9340 *self = container_of(dev, struct ili9340, dev.ops);
    if(!self->framebuffer) return -ENOMEM;
    self->framebuffer[(self->bpp / 8) * (self->width * y + x)] = RGB888_TO_RGB565(color);
    return 0;
}

static const struct display_device_ops _display_ops = {
    .write_pixel = _ili9340_write_pixel
};

int _ili9340_probe(void *fdt, int fdt_node){
    // find the gpio device for pins
	int node = fdt_find_node_by_ref(fdt, fdt_node, "gpio");
	if(node < 0) {
		dbg_printk("ledpanel: nopins!\n");
		return -EINVAL;
	}

    gpio_device_t gpio = gpio_find_by_node(fdt, node);
    if(!gpio) {
		dbg_printk("ledpanel: nogpio!\n");
		return -EINVAL;
	}

    node = fdt_find_node_by_ref(fdt, fdt_node, "spi");
	if(node < 0) {
		dbg_printk("ledpanel: nospi!\n");
		return -EINVAL;
	}

    spi_device_t spi = spi_find_by_node(fdt, node);
    if(!spi) {
		dbg_printk("ledpanel: nospi!\n");
		return -EINVAL;
	}

	int width = fdt_get_int_or_default(fdt, fdt_node, "width", 240);
	int height = fdt_get_int_or_default(fdt, fdt_node, "height", 320);

    struct ili9340 *self = kzmalloc(sizeof(struct ili9340));
    display_device_init(&self->dev, fdt_node, &_display_ops);

    self->spi = spi;
    self->gpio = gpio;
    self->width = width;
    self->height = height;
    self->bpp = 16;

    self->framebuffer = kzmalloc(sizeof(color_rgb_565_t) * (size_t)width * (size_t)height);
    if(!self->framebuffer){
        dbg_printk("ili9340: nofb\n");
        return -ENOMEM;
    }

    display_device_register(&self->dev);

    return 0;
}

int _ili9340_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(ili9340, "mk,ili9340", _ili9340_probe, _ili9340_remove)
