
extern "C" {
#include <libfirmware/thread.h>

uint32_t thread_ticks_count() {
}

int thread_sleep_ms_until(uint32_t *time, uint32_t ms) {
}

int thread_sleep_ms(uint32_t ms) {
}
unsigned long thread_get_total_heap(void) {
	return 1000;
}
unsigned long thread_get_free_heap(void) {
	return 1000;
}
void thread_meminfo() {
}

int thread_create(void (*thread)(void*), const char *name, uint32_t stack, void *ptr, uint8_t prio, thread_t *handle) {
}
}
