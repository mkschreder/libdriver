extern "C" {
#include <stdarg.h>
#include <stdlib.h>


int printk(const char *str, ...){
	va_list arg;
  int done;
  va_start (arg, str);
  done = vfprintf(stdout, str, arg);
  va_end (arg);
  return done;
}

};
