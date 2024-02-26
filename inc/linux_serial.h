#ifndef LINUX_SERIAL_H
#define LINUX_SERIAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <termios.h>
#include <pthread.h>
#include <sys/types.h>

#include "ringbuf.h"
#include "print.h"

#define DATA_BUFFER_SIZE 128
#define DEVICE_NAME_MAX 22
#define LOCKFILE_NAME_MAX DEVICE_NAME_MAX + 15

#ifndef UNBUFFERED_IO
#define UNBUFFERED_IO 1
#endif

#ifndef CMSPAR
#define CMSPAR 010000000000
#endif

#ifndef CRTSCTS
#define CRTSCTS 020000000000
#endif

#define FIELD_INVALID -1

/**
 * @brief The number of databits in a serial connection.
 * 
 * Serial connections can have 5 to 8 bits of data.
 */
typedef enum databits_t databits_t;
enum databits_t
{
    DATABITS_INVALID = FIELD_INVALID, /** Invalid number of data bits. */
    DATABITS_5 = 5, /** 5 data bits. */
    DATABITS_6 = 6, /** 6 data bits. */
    DATABITS_7 = 7, /** 7 data bits. */
    DATABITS_8 = 8 /** 8 data bits. */
};

/**
 * @brief The type of parity in a serial connection.
 * 
 * Serial connections can have no parity, even parity, odd parity, or mark
 * parity. Parity involves sending an extra bit to check the validity of a
 * unit of data (usually a byte). Even and odd parity are the most common. Mark
 * (mark/space) parity is mainly used for low-power bus interrupts.
 */
typedef enum parity_t parity_t;
enum parity_t
{
    PARITY_INVALID = FIELD_INVALID, /** Invalid parity. */
    PARITY_NONE = 0, /** No parity. */
    PARITY_EVEN = 1, /** Even parity. */
    PARITY_ODD = 2, /** Odd parity. */
    PARITY_MARK = 3 /** Mark/space parity. */
};

/**
 * @brief The number of stop bits in a serial connection.
 * 
 * Serial connections have one, one-half, or two stop bits. The stop bit(s)
 * specify how many bits signal the end of the smallest unit of data transfer
 * (typically a byte).
 */
typedef enum stopbits_t stopbits_t;
enum stopbits_t
{
    STOPBITS_INVALID = FIELD_INVALID, /** Invalid number of stop bits. */
    STOPBITS_ONE = 1, /** 1 stop bit */
    STOPBITS_TWO = 2, /** 2 stop bits */
    STOPBITS_ONE_HALF = 3 /** one-half stop bits */
};

/**
 * @brief The mode of software flow control in a serial connection.
 * 
 * Software flow control is implemented with XOFF and XON (transmit off,
 * transmit on) codes transmitted over the serial connection. XON is generally
 * represented with the decimal value 19, or CTRL-Q. XOFF is generally
 * represented with the decimal value 17, or CTRL-S.
*/
typedef enum xon_xoff_t xon_xoff_t;
enum xon_xoff_t
{
    XONXOFF_INVALID = FIELD_INVALID, /** Invalid software flow control. */
    XONXOFF_OFF = 0, /** Disable software flow control. */
    XONXOFF_IN, /** Enable software flow control for input only. */
    XONXOFF_OUT, /** Enable software flow control for output only. */
    XONXOFF_INOUT /** Enable software flow control. */
};

/**
 * @brief Structure to store thread data.
 * 
 * pthreads which need a signal to terminate from another thread are safer
 * doing so by checking a boolean which is mutex locked to be thread safe. This
 * ensures that proper cleanup is done before the thread exits.
*/
typedef struct thread_data_t thread_data_t;
struct thread_data_t
{
    bool run; /** Value to check if thread is running. */
    void *data; /** Data to pass into thread. */
};

/**
 * @brief Structure to store serial connection settings.
 * 
 * Serial connections have many settings, including baudrate (bits per second),
 * number of databits (typically 8 bits), parity bit, number of stop bits, async
 * communication, hardware flow control, and software flow control (XON/XOFF).
 * These are set with set_settings_tty() after a device is created with
 * new_device_tty().
*/
typedef struct serial_settings_t serial_settings_t;
struct serial_settings_t
{
    speed_t baudrate; /** The baudrate of the serial connection. */
    databits_t databits; /** The number of databits for the serial connection. */
    parity_t parity; /** The type of parity for the serial connection. */
    stopbits_t stopbits; /** The number of stop bits for the serial conneciton. */
    bool is_async; /** If the serial connection isn't synchronized by a clock signal. */
    bool hardware_flow; /** If the serial connection uses hardware flow control. */
    xon_xoff_t xon_xoff; /** The software flow control of the serial connection. */
};

/**
 * @brief Structure to store serial device information.
 * 
 * A new serial device can be initialized by declaring a serial_device_t struct
 * variable, then calling new_device_tty() on it. Then a serial_settings_t
 * struct variable can be declared with user-specified fields, and the device
 * can be set with these settings with set_settings_tty(). open_tty() can then
 * be called to activate send/receive operations.
*/
typedef struct serial_device_t serial_device_t;
struct serial_device_t
{
    char name[DEVICE_NAME_MAX]; /** The name of the serial device. */
    serial_settings_t settings; /** The serial connection settings of the device. */
    bool settings_set; /** Whether the settings have been set for the device. */
    bool is_verbose;
    bool is_locking;

    int fd;
    struct termios tty;

    ringbuf_t *recv_ringbuf;
    //ringbuf_t *send_ringbuf;

    pthread_t thread;
    thread_data_t thread_data;
    pthread_mutex_t thread_lock;

    void (*on_conn_lost)(serial_device_t *self);
    //void (*on_send_underflow)(serial_device_t *self, size_t bytes_lost);
    void (*on_recv_overflow)(serial_device_t *self, size_t bytes_lost);
};

bool new_device_tty(serial_device_t *device, const char *name, bool is_verbose, bool is_locking, size_t ringbuf_size);
bool set_settings_tty(serial_device_t *device, serial_settings_t settings);
serial_settings_t get_settings_tty(serial_device_t *device);

bool open_tty(serial_device_t *device,
    void (*on_conn_lost)(serial_device_t *self),
    void (*on_recv_overflow)(serial_device_t *self, size_t bytes_lost));
bool close_tty(serial_device_t *device);
size_t read_tty(serial_device_t *device, uint8_t *buf, size_t n);
ssize_t write_tty(serial_device_t *device, uint8_t *buf, size_t n);

int32_t baudrate2int(speed_t baudrate);
speed_t int2baudrate(int32_t speed);
char parity2char(parity_t parity);
parity_t char2parity(char c);
const char *stopbits2str(stopbits_t stopbits);
const char *xon_xoff2str(xon_xoff_t xon_xoff);

#endif
