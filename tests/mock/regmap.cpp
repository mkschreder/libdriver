#include <stdint.h>

extern "C" {
int regmap_write_u32(regmap_device_t dev, uint32_t id, uint32_t value) {
	printf("fb_test: regmap write %08x %08x\n", id, value);
	return 0;
}

int regmap_mem_to_u16(regmap_value_type_t type, const void *data, size_t size,
                      uint16_t *out) {
	return 0;
}
int regmap_mem_to_u32(regmap_value_type_t type, const void *data, size_t size,
                      uint32_t *out) {
	return 0;
}

int regmap_convert_u32(uint32_t value, regmap_value_type_t type, void *data,
                       size_t size) {
	return 0;
}

int regmap_convert_u16(uint16_t value, regmap_value_type_t type, void *data,
                       size_t size) {
	return 0;
}

void regmap_range_init(struct regmap_range *self, uint32_t start, uint32_t end,
                       const struct regmap_range_ops *ops) {
}
}
