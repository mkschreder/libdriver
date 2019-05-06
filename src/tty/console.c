/*
 * This file contains a lot of code from Ninjaflight.
 *
 * Authors: Martin Schröder & Cleanflight project
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <math.h>

#include <libfirmware/vardir.h>
#include <libfirmware/console.h>
#include <libfirmware/serial.h>
#include <libfirmware/types.h>
#include <libfirmware/chip.h>
#include <libfirmware/driver.h>

#include <libfdt/libfdt.h>

#include <errno.h>

#define CONSOLE_WRITE_TIMEOUT 10

#define CONSOLE_MAX_PS_TASKS 10
// TODO: make this configurable from the devicetree
#define CONSOLE_MAX_LINE 80
#define CONSOLE_MAX_ARGS 16

#define CONSOLE_KEY_ARROW_UP 0x41

struct console {
	struct console_device dev;

	serial_port_t serial;
	struct list_head commands;
	size_t ncommands;

	struct vardir *vars;

	//! this is used for displaying task stats
#if 0
	TaskStatus_t prev_status[CONSOLE_MAX_PS_TASKS];
#endif
	char printf_buffer[CONSOLE_MAX_LINE];
	char line[CONSOLE_MAX_LINE];
	char prev_line[CONSOLE_MAX_LINE];
	char *argv[CONSOLE_MAX_ARGS];

	vardir_device_t vardir;

	struct mutex lock;
};

static int strtokenize(char *buf, size_t len, char *tokens[], uint8_t ntokens){
	if(!buf || (len <= 0) || !tokens || !ntokens) return -1;

	memset(tokens, 0, ntokens * sizeof(*tokens[0]));

	uint8_t tok = 0;
	size_t c = 0;
	while(c < len){
		// skip leading spaces and special chars
		while((buf[c] == ' ' || buf[c] < 0x0f) && c < len) {
			if(buf[c] == 0) break;
			buf[c] = 0;
			c++;
		}
		// if reached end of string or end of buffer then break
		if(buf[c] == 0 || c >= len) break;
		// store current position in the string and increase token counter
		tokens[tok++] = buf + c;
		// break if we are out of tokens
		if(tok == ntokens) break;
		// skip the word until next space or end of string
		while(buf[c] != ' ' && buf[c] > 0x0f && c < len) c++;
	}
	return tok;
}

static int _console_printf(console_device_t dev, const char *fmt, ...){
	struct console *self = container_of(dev, struct console, dev.ops);

	// lock printf buffer
	//thread_mutex_lock(&self->lock);

	va_list argptr;
	va_start(argptr, fmt);
	int r = vsnprintf(self->printf_buffer, sizeof(self->printf_buffer), fmt, argptr);
	va_end(argptr);
	size_t len = strlen(self->printf_buffer);
	int rr = serial_write(self->serial, self->printf_buffer, len, CONSOLE_WRITE_TIMEOUT);

	//thread_mutex_unlock(&self->lock);

	if(rr < 0) return rr;
	return r;
}

static int _console_read(console_device_t dev, char *data, size_t size, uint32_t timeout){
	struct console *self = container_of(dev, struct console, dev.ops);
	return serial_read(self->serial, data, size, timeout);
}

#if !defined(__linux__)
#if 0
#include <FreeRTOS.h>
#include <task.h>

// TODO: get rid of this ugliness
static int _compare_tasks(const void *a, const void *b){
	TaskStatus_t *ta = (TaskStatus_t*)a;
	TaskStatus_t *tb = (TaskStatus_t*)b;
	return tb->xTaskNumber < ta->xTaskNumber;
}
#endif
#endif

typedef enum {
	HEX_8,
	HEX_32
} hex_format_t;

static void _dump_hex(console_device_t con, void *addr, hex_format_t format){
	console_printf(con, "0x%08x: ", addr);
	switch(format){
		case HEX_8:
			for(int c = 0;c < 16; c++){
				console_printf(con, "%02x ", *((uint8_t*)addr + c));
			}
			break;
		case HEX_32:
			for(int c = 0;c < 4; c++){
				console_printf(con, "%08x ", *((uint32_t*)addr + c));
			}
			break;
	}
	console_printf(con, "\n");
}

static int _cmd_md(console_device_t con, void *ptr, int argc, char **argv){
	hex_format_t format = HEX_8;
	unsigned int addr = 0;
	if(argc == 2){
		int r = sscanf(argv[1], "%x", &addr);
		if(r != 1){
			goto usage;
		}
	} else if(argc == 3){
		int r = sscanf(argv[2], "%x", &addr);
		if(r != 1){
			goto usage;
		}
		if(strcmp(argv[1], "x8") == 0){
			format = HEX_8;
		} else if(strcmp(argv[1], "x32") == 0){
			format = HEX_32;
		}
	}
	_dump_hex(con, (void*)addr, format);
	return 0;
usage:
	console_printf(con, "Usage: %s [format:x32|x8] <hex addr>\n");
	return -1;
}

static int _cmd_ps(console_device_t con, void *ptr, int argc, char **argv){
	(void)con;
	(void)argc;
	(void)argv;
	thread_meminfo();
	// realtime tasks
//#if !defined(__linux__)
#if 0
	TaskStatus_t status[CONSOLE_MAX_PS_TASKS];
	memset(status, 0, sizeof(status));
	uint32_t total_time;
	con_printf(self, "== realtime tasks\n");
	UBaseType_t ret = uxTaskGetSystemState(status, sizeof(status)/sizeof(status[0]), &total_time);
	struct timeval tval;
	time_gettime(&tval);
	if(ret > 0){
		qsort(status, ret, sizeof(status[0]), _compare_tasks);
		uint32_t total_calculated = 0;
		uint32_t prev_total_calculated = 0;
		for(UBaseType_t c = 0; c < ret; c++){
			TaskStatus_t *stat = &status[c];
			TaskStatus_t *prev_stat = &self->prev_status[c];
			total_calculated += stat->ulRunTimeCounter;
			prev_total_calculated += prev_stat->ulRunTimeCounter;
		}
		con_printf(self, "time elapsed: %u:%06u, micros: %u, time usr: %u\n", tval.tv_sec, tval.tv_usec, micros(), total_calculated);
		con_printf(self, "heap: %lu free of %lu bytes\n", xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE);
		con_printf(self, "data: %d\n", chip_get_data_size());
		con_printf(self, "%5s%5s%8s%8s%10s%8s%8s\n", "id", "prio", "name", "stack", "cpu (us)", "cpu (%)", "load");
		for(UBaseType_t c = 0; c < ret; c++){
			TaskStatus_t *stat = &status[c];
			TaskStatus_t *prev_stat = &self->prev_status[c];
			uint32_t dtotal = total_calculated - prev_total_calculated;
			uint32_t run_time = stat->ulRunTimeCounter;
			uint32_t drun_time = stat->ulRunTimeCounter - prev_stat->ulRunTimeCounter;

			uint32_t cpu_percent = 0;
			uint32_t dcpu_percent = 0;

			if(total_calculated && dtotal){
				cpu_percent = (uint32_t)((uint64_t)run_time * 10000 / total_calculated);
				dcpu_percent = (uint32_t)(((uint64_t)drun_time * 10000) / dtotal);
			}

			uint32_t cpu_whole = cpu_percent / 100;
			uint32_t cpu_frac = cpu_percent % 100;

			uint32_t dcpu_whole = dcpu_percent / 100;
			uint32_t dcpu_frac = dcpu_percent % 100;

			con_printf(self, "%5u%5u%8s%8u%10u%5u.%02u%5u.%02u\n",
					stat->xTaskNumber, stat->uxBasePriority, stat->pcTaskName, stat->usStackHighWaterMark, stat->ulRunTimeCounter, cpu_whole, cpu_frac, dcpu_whole, dcpu_frac);
		}
		memcpy(self->prev_status, status, sizeof(self->prev_status));
	} else {
		con_printf(self, "(none)\n");
	}
#endif
	return 0;
}

static int _cmd_set(struct console *con, int argc, char **argv){
	if(!con->vardir){
		_console_printf(&con->dev.ops, PRINT_ERROR "no vardir\n");
		return -EIO;
	}

	if(argc != 3) {
		_console_printf(&con->dev.ops, PRINT_ERROR "set <name> <value>\n");
		return -1;
	}

	return vardir_set(con->vardir, 0, argv[1], VAR_STRING, argv[2]);
}

static int _cmd_get(struct console *con, int argc, char **argv){
	if(!con->vardir){
		_console_printf(&con->dev.ops, PRINT_ERROR "no vardir\n");
		return -EIO;
	}

	if(argc != 2) {
		_console_printf(&con->dev.ops, PRINT_ERROR "get <name>\n");
		return -1;
	}

	char buf[16];

	int ret = 0;
	if((ret = vardir_get(con->vardir, 0, argv[1], VAR_STRING, buf, sizeof(buf))) >= 0){
		_console_printf(&con->dev.ops, "%s=%s\n", argv[1], buf);
	} else {
		_console_printf(&con->dev.ops, PRINT_ERROR "ERROR: variable not found\n");
	}
	return ret;
}

#if 0
static int _cmd_save(struct console *self, int argc, char **argv){
	(void)self;
	(void)argc;
	(void)argv;
	/*
	if(config_save(container_of(self->config, struct config_store, data), ) < 0){
		return -1;
	}
	*/
	return 0;
}

#endif
static int _cmd_help(struct console *self, int argc, char **argv){
	(void)argc; (void)argv;
	struct console_command *cmd;
	list_for_each_entry(cmd, &self->commands, list){
		_console_printf(&self->dev.ops, "%s", cmd->name);
		if(cmd->options) _console_printf(&self->dev.ops, " %s", cmd->options);
		_console_printf(&self->dev.ops, "\n");
		if(cmd->description) _console_printf(&self->dev.ops, "\t%s\n", cmd->description);
	}
#if CONFIG_CMD_PS == 1
	_console_printf(&self->dev.ops, "ps\n");
	_console_printf(&self->dev.ops, "\tshow running tasks\n");
#endif
	_console_printf(&self->dev.ops, "help\n");
	_console_printf(&self->dev.ops, "\tshow this help\n");
	return 0;
}

static int con_readline(struct console *self, char *line, size_t size){
	// buffer must be at least 1 char to accomodate for a \0
	int rd;
	char ch;
	int pos = 0;
	while((rd = serial_read(self->serial, &ch, 1, THREAD_SLEEP_MAX_DELAY)) > 0){
		// emulate backspace correctly
		if(ch == 0x08 || ch == 0x7f){
			//serial_write(self->serial, "\x1b[D \x1b[D", 7);
			if(pos) {
				serial_write(self->serial, "\x08 \x08", 3, CONSOLE_WRITE_TIMEOUT);
				line[--pos] = 0;
			}
			continue;
		} else if(ch == '\n'){
			// skip new lines (we expect carriage returns from now on)
			continue;
		} else if(ch == '\r') {
			// echo a new line
			char nl = '\n';
			serial_write(self->serial, &nl, 1, CONSOLE_WRITE_TIMEOUT);
			break;
		} else if(ch == CONSOLE_KEY_ARROW_UP){
			char *p = strncpy(line, self->prev_line, size);
			size_t len = (size_t)(p - line);
			serial_write(self->serial, line, (size_t)len, CONSOLE_WRITE_TIMEOUT);
			pos = (int)len;
		} else if(ch > 0xf && ch < 127){
			serial_write(self->serial, &ch, 1, CONSOLE_WRITE_TIMEOUT);
			line[pos++] = ch;
		} else {
			printk("%02x ", ch);
		}

		if(size > 0 && ((size_t)pos == (size - 1))) break;
	}
	line[pos] = 0;
	if(rd == 0) return -ETIMEDOUT;
	if(rd < 0) return rd;
	return pos;
}

static void _console_device_task(void *ptr){
	struct console *self = (struct console*)ptr;

	while(1){
		_console_printf(&self->dev.ops, "\x1b[0m# ");
		int rd = 0;
		memset(self->line, 0, sizeof(self->line));

		rd = con_readline(self, self->line, sizeof(self->line));

		if(rd < 0) {
			_console_printf(&self->dev.ops, "Internal error (%d): %s\n", rd, strerror(-rd));
            thread_sleep_ms(100); // to avoid busy loop
			continue;
		}

		memset(self->argv, 0, sizeof(self->argv));
		int argc = strtokenize(self->line, (size_t)rd, self->argv, 8);

		if(argc <= 0)
			continue;

		// reset optind because getopt uses it for first parameter and we need to start from the begining for each command
		optind = 1;
		/*
		for(int c = 0; c < argc; c++){
			con_printf(self, "arg %d: %s\n", c, argv[c]);
		}
		*/
		// find the command and run it
		uint8_t handled = 0;
		struct console_command *cmd;
		int err = 0;
		thread_mutex_lock(&self->lock);

		list_for_each_entry(cmd, &self->commands, list){
			if(strcmp(cmd->name, self->argv[0]) == 0 && cmd->proc){
				err = cmd->proc(&self->dev.ops, cmd->userptr, argc, self->argv);
				handled = 1;
				break;
			}
		}

		thread_mutex_unlock(&self->lock);

		if(handled && err < 0){
			_console_printf(&self->dev.ops, "ERROR (%d): %s\n", -err, strerror(-err));
		} else if(!handled){
			if(0) {}
			else if(strcmp("set", self->argv[0]) == 0){
				_cmd_set(self, argc, self->argv);
			}
			else if(strcmp("get", self->argv[0]) == 0){
				_cmd_get(self, argc, self->argv);
			}
#if 0
			else if(strcmp("save", self->argv[0]) == 0){
				_cmd_save(self, argc, self->argv);
			}
#endif
			else {
				_cmd_help(self, 0, NULL);
			}
		}

        // send end of transmission
		_console_printf(&self->dev.ops, "\x04");
	}
}

static int _console_add_command(console_device_t dev, struct console_command *cmd){
	struct console *self = container_of(dev, struct console, dev.ops);
	thread_mutex_lock(&self->lock);
	list_add_tail(&cmd->list, &self->commands);
	thread_mutex_unlock(&self->lock);
	return 0;
}

static const struct console_device_ops _console_ops = {
	.add_command = _console_add_command,
	.printf = _console_printf,
	.read = _console_read
};

static int _console_file_read(struct _reent *r, void *ptr, char *buf, int size){
	struct console *self = (struct console*)ptr;
	return serial_read(self->serial, buf, (size_t)size, THREAD_SLEEP_MAX_DELAY);
}

static int _console_file_write(struct _reent *r, void *ptr, const char *buf, int size){
	struct console *self = (struct console*)ptr;
	return serial_write(self->serial, buf, (size_t)size, CONSOLE_WRITE_TIMEOUT);
}

int _console_probe(void *fdt, int fdt_node){
	struct console *self = kzmalloc(sizeof(struct console));
	vardir_device_t vardir = vardir_find_by_ref(fdt, fdt_node, "vardir");

	int node = fdt_find_node_by_ref(fdt, fdt_node, "serial");
	if(node < 0){
		printk("console: serial port missing\n");
		return -EINVAL;
	}

	serial_port_t serial = serial_find_by_node(fdt, node);
	if(!serial){
		printk("console: invalid serial port\n");
		return -EINVAL;
	}

	self->serial = serial;
	self->vardir = vardir;
	INIT_LIST_HEAD(&self->commands);
	thread_mutex_init(&self->lock);

	console_device_init(&self->dev, fdt, fdt_node, &_console_ops);
	console_device_register(&self->dev);

	console_add_command(&self->dev.ops, self, "ps", "Show list of processes", "", _cmd_ps);
	console_add_command(&self->dev.ops, self, "md", "Dump raw memory location", "", _cmd_md);

	if(thread_create(
		  _console_device_task,
		  "shell",
		  620,
		  self,
		  1,
		  NULL) < 0){
        dbg_printk("con: fail!\n");
        return -1;
    } else {
        printk("con: started!\n");
    }
	return 0;
}

int _console_remove(void *fdt, int fdt_node){

	return -1;
}

DEVICE_DRIVER(console, "fw,console", _console_probe, _console_remove)
