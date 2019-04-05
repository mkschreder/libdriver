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

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <libfirmware/thread.h>
#include <libfirmware/vardir.h>
#include <libfirmware/driver.h>

struct vardir_driver {
	struct vardir_device dev;

	struct vardir dir;
	struct mutex lock;
};

int _vardir_add_static(vardir_device_t dev, uint32_t id, const char *name, vardir_value_type_t type, const void *init_value){
	struct vardir_driver *self = container_of(dev, struct vardir_driver, dev.ops);
	thread_mutex_lock(&self->lock);

	int ret = vardir_add_field(&self->dir, id, name, type, 123);

	thread_mutex_unlock(&self->lock);

	return ret;
}

int _vardir_add_dynamic(vardir_device_t dev, uint32_t id, const char *name, uint8_t type, struct vardir_entry_ops **ops){
	struct vardir_driver *self = container_of(dev, struct vardir_driver, dev.ops);
	thread_mutex_lock(&self->lock);
	
	int ret = vardir_add_entry(&self->dir, id, name, type, ops);

	thread_mutex_unlock(&self->lock);

	return ret;
}

int _vardir_get(vardir_device_t dev, uint32_t id, const char *name, vardir_value_type_t type, void *value, size_t size){
	struct vardir_driver *self = container_of(dev, struct vardir_driver, dev.ops);

	thread_mutex_lock(&self->lock);
	struct vardir_entry *e = NULL;
	if(name){
		if((e = vardir_find_entry_by_name(&self->dir, name))) id = e->id;
		else {
			thread_mutex_unlock(&self->lock);
			return -ENOENT;
		}
	}

	struct vardir_entry *ent = vardir_find_entry_by_id(&self->dir, id);

	int ret = 0;

	uint32_t val = 0;
	vardir_entry_get_u32(ent, &val);

	switch(type){
		case VAR_INT32:
		case VAR_UINT32:
			if(size != sizeof(uint32_t)) return -1;
			*((uint32_t*)value) = val;
			ret = sizeof(uint32_t);
		case VAR_INT16:
		case VAR_UINT16:
			if(size != sizeof(uint16_t)) return -1;
			*((uint16_t*)value) = (uint16_t)val;
			ret = sizeof(uint16_t);
		case VAR_INT8:
		case VAR_UINT8:
			if(size != sizeof(uint8_t)) return -1;
			*((uint8_t*)value) = (uint8_t)val;
			ret = sizeof(uint8_t);
		case VAR_STRING: {
			break;
		}
		default: 
			printk("vardir: unsupported type\n");
	}

	thread_mutex_unlock(&self->lock);
	return ret;
}

int _vardir_set(vardir_device_t dev, uint32_t id, const char *name, vardir_value_type_t type, const void *value){
	struct vardir_driver *self = container_of(dev, struct vardir_driver, dev.ops);
	struct vardir_entry *e = NULL;
	if(name) e = vardir_find_entry_by_name(&self->dir, name);
	else e = vardir_find_entry_by_id(&self->dir, id);
	if(!e) return -ENOENT;

	thread_mutex_lock(&self->lock);
	switch (e->type & VARDIR_VALUE_TYPE_MASK) {
		case VAR_UINT8:
		case VAR_INT8:
		case VAR_UINT16:
		case VAR_INT16:
		case VAR_INT32:
		case VAR_UINT32:
			if(name && vardir_set_int_by_name(&self->dir, name, *((uint32_t*)value))){
				vardir_set_int(&self->dir, id, *((uint32_t*)value));
			}
			break;
		case VAR_FLOAT: 
			if(name && vardir_set_float_by_name(&self->dir, name, *((float*)value))){
				vardir_set_float(&self->dir, id, *((float*)value));
			}
			break;
		case VAR_STRING:

			break;
	};
	thread_mutex_unlock(&self->lock);
	return 0;
}


static struct vardir_device_ops _vardir_ops = {
	.add_static = _vardir_add_static,
	.add_dynamic = _vardir_add_dynamic,
	.get = _vardir_get,
	.set = _vardir_set,
};

static int _vardir_probe(void *fdt, int fdt_node){
	struct vardir_driver *self = kzmalloc(sizeof(struct vardir_driver));
	thread_mutex_init(&self->lock);
	vardir_init(&self->dir);

	vardir_device_init(&self->dev, fdt, fdt_node, &_vardir_ops);
	vardir_device_register(&self->dev);

	return 0;
}

static int _vardir_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(vardir, "fw,vardir", _vardir_probe, _vardir_remove)
