/** :ms-top-comment
 *  _____ _       _             ____
 * |  ___| |_   _(_)_ __   __ _| __ )  ___ _ __ __ _ _ __ ___   __ _ _ __
 * | |_  | | | | | | '_ \ / _` |  _ \ / _ \ '__/ _` | '_ ` _ \ / _` | '_ \
 * |  _| | | |_| | | | | | (_| | |_) |  __/ | | (_| | | | | | | (_| | | | |
 * |_|   |_|\__, |_|_| |_|\__, |____/ \___|_|  \__, |_| |_| |_|\__,_|_| |_|
 *          |___/         |___/                |___/
 **/
#include <stdint.h>

extern "C" {
static uint32_t _usec = 0;

timestamp_t _us_to_timestamp(uint32_t us) {
	return (timestamp_t){.sec = _usec / 1000000U, .usec = _usec % 1000000U};
}

uint32_t _timestamp_to_us(timestamp_t ts) {
	return ts.sec * 1000000 + ts.usec;
}

uint32_t micros(){
	return _usec;
}

timestamp_t timestamp() {
	_us_to_timestamp(_usec);
}

timestamp_t timestamp_from_now_us(usec_t us) {
	_us_to_timestamp(_usec + us);
}

timestamp_t timestamp_from_now_ms(usec_t ms) {
	_us_to_timestamp(_usec + ms * 1000);
}

int timestamp_expired(timestamp_t ts) {
	uint32_t us = _timestamp_to_us(ts);
	return us > _usec;
}

	timestamp_t timestamp_add_us(timestamp_t ts, uint32_t us){
		return _us_to_timestamp(_timestamp_to_us(ts) + us);
	}
}
