#pragma once

#include <libfirmware/memory.h>

typedef memory_device_t drv8302_t;

typedef enum {
	DRV8302_OC_MODE_CYCLE_BY_CYCLE,
	DRV8302_OC_MODE_SHUTDOWN
} drv8302_oc_mode_t;

typedef enum {
	DRV8302_GAIN_10V,
	DRV8302_GAIN_40V
} drv8302_gain_t;

void drv8302_enable_calibration(drv8302_t dev, bool en);
void drv8302_set_oc_mode(drv8302_t dev, drv8302_oc_mode_t mode);
void drv8302_enable(drv8302_t dev, bool en);
void drv8302_set_gain(drv8302_t dev, drv8302_gain_t gain);
bool drv8302_is_in_error(drv8302_t dev);
bool drv8302_is_in_overcurrent(drv8302_t dev);
int drv8302_get_gain(drv8302_t dev);
