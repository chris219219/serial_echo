#ifndef PRINT_H
#define PRINT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

void print_err(const char *fmt, ...);
void print_ok(const char *fmt, ...);
void print_verbose(const char *fmt, ...);
void debug_print_err(const char *fmt, ...);
void debug_print_ok(const char *fmt, ...);

#endif