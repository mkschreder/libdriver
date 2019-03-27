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

#include <libfirmware/console.h>
#include <libfirmware/serial.h>
#include <libfirmware/vardir.h>
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
	char *argv[CONSOLE_MAX_ARGS];
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

static int _console_printf(console_t dev, const char *fmt, ...){
	struct console *self = container_of(dev, struct console, dev.ops);

	va_list argptr;
	va_start(argptr, fmt);
	int r = vsnprintf(self->printf_buffer, sizeof(self->printf_buffer), fmt, argptr);
	va_end(argptr);
	size_t len = strlen(self->printf_buffer);
	int rr = serial_write(self->serial, self->printf_buffer, len, CONSOLE_WRITE_TIMEOUT);
	if(rr < 0) return rr;
	return r;
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
static int _cmd_ps(struct console *self, int argc, char **argv){
	(void)self;
	(void)argc;
	(void)argv;
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
	#if 0
	static char str[32];
	if (argc == 1) {
		struct vardir_entry *entry;
		vardir_for_each_entry(con->vars, entry) {
			vardir_entry_to_string(entry, str, sizeof(str));
			con_printf(con, "%04x (name: %s, type: %02x): %s\n", entry->id, (entry->name)?entry->name:"-", entry->type, str);
		}
	} else if(argc == 2) {
		uint32_t id = (uint32_t)strtoul(argv[1], NULL, 16);
		struct vardir_entry *entry = vardir_find_entry_by_id(con->vars, id);
		if(entry){
			if(vardir_entry_to_string(entry, str, sizeof(str)) < 0){
				con_printf(con, "Could not convert entry to string!\n");
			} else {
				con_printf(con, "%04x (%s): %s\n", entry->id, (entry->name)?entry->name:"-", str);
			}
		} else {
			con_printf(con, "Not found (%08x)\n", id);
		}
	} else if (argc == 3) {
		char *var_name = argv[1];
		char *var_value = argv[2];

		uint32_t id = (uint32_t)strtoul(var_name, NULL, 16);
		uint32_t val = (uint32_t)strtoul(var_value, NULL, 16);
		if(id){
			vardir_set_int(con->vars, id, val);
		} else if(vardir_set_value(con->vars, var_name, var_value) < 0){
			con_printf(con, "ERROR: could not set value!\n");
		} else {
			// print value
		}
	}
	return 0;
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
#endif
#endif
	return 0;
}

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
	while((rd = serial_read(self->serial, &ch, 1, THREAD_QUEUE_MAX_DELAY)) > 0){
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
		}
		serial_write(self->serial, &ch, 1, CONSOLE_WRITE_TIMEOUT);
		if(ch == '\r') {
			// echo a new line
			char nl = '\n';
			serial_write(self->serial, &nl, 1, CONSOLE_WRITE_TIMEOUT);
			break;
		}
		line[pos++] = ch;
		if(size > 0 && ((size_t)pos == (size - 1))) break;
	}
	line[pos] = 0;
	if(rd == 0) return -ETIMEDOUT;
	if(rd < 0) return rd;
	return pos;
}

static void _console_task(void *ptr){
	struct console *self = (struct console*)ptr;

	while(1){
		_console_printf(&self->dev.ops, "# ");
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
		list_for_each_entry(cmd, &self->commands, list){
			if(strcmp(cmd->name, self->argv[0]) == 0 && cmd->proc){
				if(cmd->proc(&self->dev.ops, cmd->userptr, argc, self->argv) < 0){
					_console_printf(&self->dev.ops, "Invalid arguments to command\n");
				}
				handled = 1;
				break;
			}
		}
		if(!handled){
			if(0) {}
			else if(strcmp("ps", self->argv[0]) == 0){
				_cmd_ps(self, argc, self->argv);
			}
			else if(strcmp("set", self->argv[0]) == 0){
				_cmd_set(self, argc, self->argv);
			}
#if 0
			else if(strcmp("get", self->argv[0]) == 0){
				_cmd_get(self, argc, self->argv);
			}
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

static int _console_add_command(console_t dev, struct console_command *cmd){
	struct console *self = container_of(dev, struct console, dev.ops);
	list_add_tail(&cmd->list, &self->commands);
	return 0;
}

static const struct console_ops _console_ops = {
	.add_command = _console_add_command,
	.printf = _console_printf
};

int _console_probe(void *fdt, int fdt_node){
	struct console *self = kzmalloc(sizeof(struct console));
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

	console_init(&self->dev, fdt, fdt_node, &_console_ops);
	self->serial = serial;
	INIT_LIST_HEAD(&self->commands);

	if(thread_create(
		  _console_task,
		  "shell",
		  460,
		  self,
		  1,
		  NULL) < 0){
        dbg_printk("con: fail!\n");
        return -1;
    } else {
        printk("con: started!\n");
    }

	console_register(&self->dev);
    return 0;
}

int _console_remove(void *fdt, int fdt_node){

	return -1;
}

DEVICE_DRIVER(console, "fw,console", _console_probe, _console_remove)
