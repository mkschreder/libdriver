#pragma once

extern "C" {
#include <libfirmware/driver.h>

void *kzmalloc(size_t size){
	return calloc(size, 1);
}

#include <libfirmware/memory.h>

DEFINE_DEVICE_CLASS(memory)

}
