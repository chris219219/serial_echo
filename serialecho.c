#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <getopt.h>
#include <signal.h>
#include <time.h>

#define SERIAL_IMPL_H
#include "serial.h"

static volatile sig_atomic_t run = 1;

static void sigint_handler(int _)
{
    (void)_;
    run = 0;
}

void on_conn_lost(serial_device_t *device)
{
    print_err("Connection lost.\n");
    run = 0;
}

void on_recv_overflow(serial_device_t *device, size_t bytes_lost)
{
    print_err("Device '%s' receive overflow by %zu bytes.\n", device->name, bytes_lost);
}

void print_usage(const char *ex_name)
{
    printf(
        "%s:\n"
        "This program continuously takes input from stdin and echoes it to\n"
        "a serial device.\n"
        "Settings:\n"
        "-I   --device-name   name of the input device, e.g. [/dev/ttyS0]\n"
        "-l   --locking       whether the device should have a lockfile in /var/lock\n"
        "-v   --verbose       whether the device should be verbose\n"
        "-n   --buffer-size   how large the send and recv ring buffers are for the device [int]\n"
        "-b   --baudrate      baudrate of the serial connection [int]\n"
        "-d   --databits      number of data bits for the serial connection [5/6/7/8]\n"
        "-p   --parity        parity type for the serial connection [none/even/odd/mark]\n"
        "-s   --stopbits      number of stop bits for the serial connection [1/1.5/2]\n"
        "-a   --async         if the serial connection is asynchronous\n"
        "-c   --hardware-flow whether the device should use hardware flow control\n"
        "-x   --software-flow whether the device should use software flow control\n"
        "-h   --help          this help screen\n",
        ex_name);
}

void init_device_with_args(int argc, char **argv, serial_device_t *device, serial_settings_t *settings)
{
    const enum
    {
        DEVICE_NAME = 'I',
        LOCKING = 'l',
        VERBOSE = 'v',
        RINGBUF_SIZE = 'n',
        BAUDRATE = 'b',
        DATABITS = 'd',
        PARITY = 'p',
        STOPBITS = 's',
        ASYNC = 'a',
        HARDWARE_FLOW_CONTROL = 'c',
        SOFTWARE_FLOW_CONTROL = 'x',
        HELP = 'h'
    } options;

    const struct option long_options[] =
    {
        { "device-name", required_argument, NULL, DEVICE_NAME },
        { "locking", no_argument, NULL, LOCKING },
        { "verbose", no_argument, NULL, VERBOSE },
        { "buffer-size", required_argument, NULL, RINGBUF_SIZE },
        { "baudrate", required_argument, NULL, BAUDRATE },
        { "databits", required_argument, NULL, DATABITS },
        { "parity", required_argument, NULL, PARITY },
        { "stopbits", required_argument, NULL, STOPBITS },
        { "async", no_argument, NULL, ASYNC },
        { "hardware-flow", no_argument, NULL, HARDWARE_FLOW_CONTROL },
        { "software-flow", no_argument, NULL, SOFTWARE_FLOW_CONTROL },
        { "help", no_argument, NULL, HELP }
    };

    char device_name[DEVICE_NAME_MAX];
    bool device_verbose = false;
    bool device_locking = false;
    size_t device_ringbuf_size = 1024;
    
    int opt;
    while ((opt = getopt_long(argc, argv, "I:lvn:b:d:p:s:acxh", long_options, NULL)) != -1)
    {
        switch (opt)
        {
        case DEVICE_NAME:
            strncpy(device_name, optarg, DEVICE_NAME_MAX - 1);
            device_name[DEVICE_NAME_MAX - 1] = '\0';
            if (strlen(device_name) < 6)
            {
                print_err("Device name cannot be less than 6 characters long!\n");
                exit(EXIT_FAILURE);
            }
            break;

        case LOCKING:
            device_locking = true;
            break;

        case VERBOSE:
            device_verbose = true;
            break;

        case RINGBUF_SIZE:
            device_ringbuf_size = (size_t)atol(optarg);
            break;

        case BAUDRATE:
            int speed = atoi(optarg);
            speed_t baudrate = int2baudrate(speed);
            if (baudrate == B0)
            {
                print_err("Invalid baudrate, must be a standard serial baudrate.\n");
                exit(EXIT_FAILURE);
            }
            settings->baudrate = baudrate;
            break;

        case DATABITS:
            int databits = atoi(optarg);
            if (databits < 5 || databits > 8)
            {
                print_err("Invalid number of databits, has to be between 5 and 8.\n");
                exit(EXIT_FAILURE);
            }
            settings->databits = databits;
            break;

        case PARITY:
            if (strcmp(optarg, "none") == 0) settings->parity = PARITY_NONE;
            else if (strcmp(optarg, "even") == 0) settings->parity = PARITY_EVEN;
            else if (strcmp(optarg, "odd") == 0) settings->parity = PARITY_ODD;
            else if (strcmp(optarg, "mark") == 0) settings->parity = PARITY_MARK;
            else
            {
                print_err("Invalid parity setting, has to be none/even/odd/mark.\n");
                exit(EXIT_FAILURE);
            }
            break;

        case STOPBITS:
            if (strcmp(optarg, "1") == 0) settings->stopbits = STOPBITS_ONE;
            else if (strcmp(optarg, "1.5") == 0) settings->stopbits = STOPBITS_ONE_HALF;
            else if (strcmp(optarg, "2") == 0) settings->stopbits = STOPBITS_TWO;
            else
            {
                print_err("Invalid number of stop bits, has to be 1/1.5/2.\n");
                exit(EXIT_FAILURE);
            }

        case ASYNC:
            settings->is_async = true;
            break;
        
        case HARDWARE_FLOW_CONTROL:
            settings->hardware_flow = true;
            break;

        case SOFTWARE_FLOW_CONTROL:
            settings->xon_xoff = XONXOFF_INOUT;
            break;

        case HELP:
            print_usage(argv[0]);
            exit(EXIT_SUCCESS);

        default:
            print_err("Invalid argument %s.\n", optarg);
            exit(EXIT_FAILURE);
        }
    } // while (getopt())

    print_verbose("\nCreating new device...\n");

    if (!new_device_tty(device, device_name, device_verbose, device_locking, device_ringbuf_size))
    {
        print_err(
            "Cannot start device with the specified parameters:\n"
            "name: %s\n"
            "verbosity: %d\n"
            "locking: %d\n"
            "ring buffer size: %zu\n",
            device_name, device_verbose, device_locking, device_ringbuf_size);
        exit(EXIT_FAILURE);
    }

    print_ok("Device successfully created.\n");

    print_verbose("\nApplying settings to device...\n");

    if (!set_settings_tty(device, *settings))
    {
        print_err(
            "Cannot set settings for device %s with the specified parameters:\n"
            "baudrate: %d\n"
            "data bits: %d\n"
            "parity: %s\n"
            "stop bits: %s\n"
            "async: %d\n"
            "hardware flow control: %d\n"
            "software flow control: %s\n",
            device->name, baudrate2int(settings->baudrate), settings->databits, parity2char(settings->parity),
            stopbits2str(settings->stopbits), settings->is_async, settings->hardware_flow, xon_xoff2str(settings->xon_xoff));
        exit(EXIT_FAILURE);
    }

    print_ok(
        "Device settings successfully set. Current settings:\n"
        "name: %s\n"
        "verbosity: %d\n"
        "locking: %d\n"
        "ring buffer size: %zu\n"
        "baudrate: %d\n"
        "data bits: %d\n"
        "parity: %c\n"
        "stop bits: %s\n"
        "async: %d\n"
        "hardware flow control: %d\n"
        "software flow control: %s\n",
        device->name, device->is_verbose, device->is_locking, device->recv_ringbuf->maxlen,
        baudrate2int(device->settings.baudrate), device->settings.databits, parity2char(device->settings.parity),
        stopbits2str(device->settings.stopbits), device->settings.is_async, device->settings.hardware_flow,
        xon_xoff2str(device->settings.xon_xoff));
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        print_usage(argv[0]);
        exit(EXIT_SUCCESS);
    }

    signal(SIGINT, sigint_handler);

    serial_settings_t settings =
    {
        .baudrate = B115200,
        .databits = DATABITS_8,
        .parity = PARITY_NONE,
        .stopbits = STOPBITS_ONE,
        .is_async = false,
        .hardware_flow = false,
        .xon_xoff = XONXOFF_OFF
    };

    serial_device_t device;
    
    init_device_with_args(argc, argv, &device, &settings);
    
    bool success = true;
    success = open_tty(&device, &on_conn_lost, &on_recv_overflow);
    if (!success)
    {
        print_err("Error opening device '%s'.\n", device.name);
        exit(EXIT_FAILURE);
    }

    const struct timespec slowdown = { 0, 10000000 }; // 10ms

    print_verbose("\nEchoing...\n");
    while (run)
    {
        uint8_t buf[1];
        size_t bytes_r = read_tty(&device, buf, 1);
        if (bytes_r == 0)
        {
            nanosleep(&slowdown, NULL);
            continue;
        }
        printf("0x%hhx ", *buf);
        fflush(stdout);
        ssize_t bytes_w = write_tty(&device, buf, 1);
        if (bytes_w < 1)
        {
            print_err("You failed\n");
            run = 0;
        }
    }
    print_ok("\nEcho stopped.\n");

    print_verbose("\nClosing device...\n");
    success = close_tty(&device);
    if (!success)
    {
        print_err("Error closing device '%s'.\n", device.name);
        exit(EXIT_FAILURE);
    }
    print_ok("Successfully closed device.\n");

    return EXIT_SUCCESS;
}