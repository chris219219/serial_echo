#ifndef SERIAL_H
#define SERIAL_H

#ifdef _WIN32
#include "windows_serial.h"
#endif

#ifdef __linux__
#include "linux_serial.h"
#endif

#endif