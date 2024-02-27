#ifndef LINUX_SERIAL_H
#define LINUX_SERIAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <termios.h>
#include <pthread.h>
#include <sys/types.h>

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

/**
 * @brief Structure to store ring buffer information.
 * 
 * The ring buffer structure is dynamically allocated with ringbuf_new(). It
 * stores a byte buffer, the size of the byte buffer, and the start and end
 * indexes of the ring buffer. The current amount of bytes stored in the ring
 * buffer can be retrieved with ringbuf_len().
 */
typedef struct ringbuf_t ringbuf_t;
struct ringbuf_t
{
    size_t maxlen; /** Length of the byte buffer. */
    size_t start; /** Start index of the ring buffer. */
    size_t end; /** End index of the ring buffer. */
    uint8_t buf[]; /** The byte buffer. */
};

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

void print_err(const char *fmt, ...);
void print_ok(const char *fmt, ...);
void print_verbose(const char *fmt, ...);
void debug_print_err(const char *fmt, ...);
void debug_print_ok(const char *fmt, ...);

ringbuf_t *ringbuf_new(size_t size);
bool ringbuf_push(ringbuf_t *ringbuf, uint8_t data);
bool ringbuf_pop(ringbuf_t *ringbuf, uint8_t *data);
size_t ringbuf_push_buf(ringbuf_t *ringbuf, uint8_t *buf, size_t n);
size_t ringbuf_pop_buf(ringbuf_t *ringbuf, uint8_t *buf, size_t n);
size_t ringbuf_len(ringbuf_t *ringbuf);
void ringbuf_print_all(ringbuf_t *ringbuf);

#ifdef SERIAL_IMPL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

void *run_data_thread_tty(void *device_ptr);

unsigned int databits2flag(databits_t databits);
unsigned int parity2flag(parity_t parity);
unsigned int xon_xoff2flag(xon_xoff_t xon_xoff);

bool new_device_tty(serial_device_t *device, const char *name, bool is_verbose, bool is_locking, size_t ringbuf_size)
{
    // set device name
    strncpy(device->name, name, DEVICE_NAME_MAX - 1);
    device->name[DEVICE_NAME_MAX - 1] = '\0';
    if (strlen(device->name) < 6)
    {
        print_err("Device name cannot be less than 6 characters long!\n");
        return false;
    }

    // initialize device serial settings
    serial_settings_t settings =
    {
        .baudrate = 0,
        .databits = FIELD_INVALID,
        .parity = FIELD_INVALID,
        .stopbits = FIELD_INVALID,
        .is_async = false,
        .hardware_flow = false,
        .xon_xoff = FIELD_INVALID
    };
    device->settings = settings;

    // set verbosity of device
    device->is_verbose = is_verbose;
    // set if device is locking
    device->is_locking = is_locking;

    // open with read/write, no CTRL-C, non-blocking
    device->fd = open(device->name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (device->fd < 0)
    {
        print_err("%s:%d open_tty: Error opening device '%s': %s\n", __FILE__, __LINE__, device->name, strerror(errno));
    }

    //device->send_ringbuf = ringbuf_new(ringbuf_size);
    device->recv_ringbuf = ringbuf_new(ringbuf_size);

    device->settings_set = false;

    if (device->is_verbose)
    {
        print_verbose("Device created with name '%s' on fd %d.\n", device->name, device->fd);
    }

    return true;
}

bool set_settings_tty(serial_device_t *device, serial_settings_t settings)
{
    bool success = true;

    success &= device->fd >= 0;
    if (!success)
    {
        print_err("Can't set settings while device '%s' is not initialized.\n", device->name);
        return false;
    }

    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcgetattr(device->fd, &device->tty) == 0;
    if (!success)
    {
        print_err("Error getting initial attributes for device '%s': %s\n", device->name, strerror(errno));
        return false;
    }

    // turn on read
    // ignore ctrl lines
    device->tty.c_cflag |= CREAD | CLOCAL;

    // turn off canonical mode and echo
    device->tty.c_cflag &= ~(ICANON | ECHO | ECHOE | ECHONL | IEXTEN);

    // disable signal characters
    device->tty.c_lflag &= ~(ISIG);

    // disable handling of special characters
    device->tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // disable output processing
    device->tty.c_oflag &= ~OPOST;

    // no blocking
    device->tty.c_cc[VMIN] = 0;
    device->tty.c_cc[VTIME] = 0;

    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcsetattr(device->fd, TCSANOW, &device->tty) == 0;
    if (!success)
    {
        print_err("Can't set device '%s' to default settings.\n", device->name);
        return false;
    }

    // set input/output speed
    success &= cfsetispeed(&device->tty, settings.baudrate) == 0;
    success &= cfsetospeed(&device->tty, settings.baudrate) == 0;
    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcsetattr(device->fd, TCSANOW, &device->tty) == 0;
    if (!success)
    {
        print_err("Can't set device '%s' to speed %d.\n", device->name, baudrate2int(settings.baudrate));
        return false;
    }
    device->settings.baudrate = settings.baudrate;
    if (device->is_verbose)
        print_verbose("Device '%s' set to speed %d.\n", device->name, baudrate2int(device->settings.baudrate));

    // set number of databits
    success &= settings.databits != DATABITS_INVALID;
    device->tty.c_cflag &= ~CSIZE; // clear data bits flags
    unsigned int databits_flag = databits2flag(settings.databits);
    device->tty.c_cflag |= databits_flag;
    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcsetattr(device->fd, TCSANOW, &device->tty) == 0;
    if (!success)
    {
        print_err("Can't set device '%s' to %d databits.\n", device->name, settings.databits);
        return false;
    }
    device->settings.databits = settings.databits;
    if (device->is_verbose)
        print_verbose("Device '%s' databits set to %d.\n", device->name, device->settings.databits);

    // set parity
    success &= settings.parity != PARITY_INVALID;
    device->tty.c_cflag = ~(PARENB | PARODD | CMSPAR); // clear parity flags
    unsigned int parity_flag = parity2flag(settings.parity);
    device->tty.c_cflag = parity_flag;
    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcsetattr(device->fd, TCSANOW, &device->tty) == 0;
    if (!success)
    {
        print_err("Can't set device '%s' to parity %c.\n", device->name, parity2char(settings.parity));
        return false;
    }
    device->settings.parity = settings.parity;
    if (device->is_verbose)
        print_verbose("Device '%s' set with parity %c.\n", device->name, parity2char(device->settings.parity));

    // set number of stop bits
    success &= settings.stopbits != STOPBITS_INVALID;
    device->tty.c_cflag &= ~CSTOPB; // clear stop bits flag
    switch (settings.stopbits)
    {
    case STOPBITS_ONE:
        break;
    case STOPBITS_ONE_HALF:
        if (!(device->tty.c_cflag & CSIZE)) device->tty.c_cflag |= CSTOPB;
        break;
    case STOPBITS_TWO:
        device->tty.c_cflag |= CSTOPB;
        break;
    }
    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcsetattr(device->fd, TCSANOW, &device->tty) == 0;
    if (!success)
    {
        print_err("Can't set device '%s' stop bits to %d.\n", device->name, stopbits2str(settings.stopbits));
        return false;
    }
    device->settings.stopbits = settings.stopbits;
    if (device->is_verbose)
        print_verbose("Device '%s' set with %s stop bits.\n", device->name, stopbits2str(device->settings.stopbits));

    // set async
    // WIP

    // set hardware flow control
    device->tty.c_cflag &= ~CRTSCTS;
    if (settings.hardware_flow)
        device->tty.c_cflag |= CRTSCTS;
    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcsetattr(device->fd, TCSANOW, &device->tty) == 0;
    device->settings.hardware_flow = settings.hardware_flow;
    if (device->is_verbose)
        print_verbose("Device '%s' set with hardware flow control %d.\n", device->name, device->settings.hardware_flow);

    // set xon_xoff
    success &= settings.xon_xoff != XONXOFF_INVALID;
    unsigned int xon_xoff_flag = xon_xoff2flag(settings.xon_xoff);
    device->tty.c_cflag &= ~(IXON | IXOFF);
    device->tty.c_cflag |= xon_xoff_flag;
    success &= tcflush(device->fd, TCIFLUSH) == 0;
    success &= tcsetattr(device->fd, TCSANOW, &device->tty) == 0;
    if (!success)
    {
        print_err("Can't set device '%s' to software flow control %s.\n", device->name, xon_xoff2str(settings.xon_xoff));
        return false;
    }
    device->settings.xon_xoff = settings.xon_xoff;
    if (device->is_verbose)
        print_verbose("Device '%s' set with software flow control %s.\n", device->name, xon_xoff2str(settings.xon_xoff));

    device->settings_set = true;
    if (device->is_verbose)
        print_verbose("Device '%s' settings successfully set.\n", device->name);

    return true;
}

serial_settings_t get_settings_tty(serial_device_t *device)
{
    return device->settings;
}

bool open_tty(serial_device_t *device,
    void (*on_conn_lost)(serial_device_t *self),
    void (*on_recv_overflow)(serial_device_t *self, size_t bytes_lost))
{
    if (device->fd < 0)
    {
        print_err("%s:%d open_tty: Cannot open device '%s': %s\n", __FILE__, __LINE__, device->name, strerror(errno));
        return false;
    }
    if (!device->settings_set)
    {
        print_err("%s:%d open_tty: Cannot open device '%s' because its serial settings are not set.\n", __FILE__, __LINE__, device->name);
        return false;
    }
    if (!isatty(device->fd))
    {
        print_err("%s:%d open_tty: Device '%s' is not a tty device.\n", __FILE__, __LINE__, device->name);
        close_tty(device);
        return false;
    }

    if (device->is_locking)
    {
        char lockfile_name[LOCKFILE_NAME_MAX];
        memset(lockfile_name, 0, LOCKFILE_NAME_MAX);
        snprintf(lockfile_name, LOCKFILE_NAME_MAX, "/var/lock/%s.lock", &device->name[5]);
        FILE *fd;

        if (access(lockfile_name, F_OK) == 0)
        {
            print_err("%s:%d open_tty: Lockfile '%s' exists, maybe another process is using this device.\n", __FILE__, __LINE__, lockfile_name);
            close(device->fd);
            return false;
        }
        if ((fd = fopen(lockfile_name, "w+")) == NULL)
        {
            print_err("%s:%d open_tty: The lockfile '%s' cannot be created.\n", __FILE__, __LINE__, lockfile_name);
            close(device->fd);
            return false;
        }
        if (fprintf(fd, "%d", (int)getpid()) < 0)
        {
            print_err("%s:%d open_tty: Cannot write pid for lockfile '%s'.\n", __FILE__, __LINE__, lockfile_name);
            close_tty(device);
            return false;
        }
        if (device->is_verbose)
            print_verbose("Pid written to lockfile '%s'.\n", lockfile_name);

        if (ioctl(device->fd, TIOCEXCL) < 0) // lock file so no other processes can use it
        {
            print_err("%s:%d open_tty: Error locking device '%s': %s \n", __FILE__, __LINE__, lockfile_name, strerror(errno));
            close_tty(device);
            return false;
        }
    }

    device->on_conn_lost = on_conn_lost;
    //device->on_send_underflow = on_send_underflow;
    device->on_recv_overflow = on_recv_overflow;

    // start thread for receiving data from the ring buffer
    device->thread_data.run = true;
    device->thread_data.data = (void*)device;
    pthread_mutex_init(&device->thread_lock, NULL);
    pthread_create(&device->thread, NULL, run_data_thread_tty, (void*)&device->thread_data);

    if (device->is_verbose)
        print_verbose("Device '%s' successfully opened with fd %d.\n", device->name, device->fd);

    return true;
}

bool close_tty(serial_device_t *device)
{
    // if device not open and is not locking, nothing to do
    if (device->fd < 0 && !device->is_locking)
    {
        return true;
    }

    // if device is locking, delete the lock file if it exists
    if (device->is_locking)
    {
        char lockfile_name[LOCKFILE_NAME_MAX];
        memset(lockfile_name, 0, LOCKFILE_NAME_MAX);
        snprintf(lockfile_name, LOCKFILE_NAME_MAX, "/var/lock/%s.lock", &device->name[5]);

        if (unlink(lockfile_name) < 0)
        {
            print_err("%s:%d close_tty: Error removing lockfile '%s': %s\n", __FILE__, __LINE__, lockfile_name, strerror(errno));
            return false;
        }
        if (device->is_verbose)
            print_verbose("Lockfile '%s' removed sucessfully. \n", lockfile_name);
    }

    // if device is closed, nothing to do
    if (device->fd < 0)
    {
        return true;
    }

    // stop the data thread
    pthread_mutex_lock(&device->thread_lock);
    device->thread_data.run = false;
    pthread_mutex_unlock(&device->thread_lock);
    pthread_join(device->thread, NULL);
    pthread_mutex_destroy(&device->thread_lock);

    // free the ring buffers
    free(device->recv_ringbuf);
    //free(device->send_ringbuf);

    // close the device
    if (close(device->fd) < 0)
    {
        print_err("%s:%d close_tty: Error closing device '%s': %s\n", __FILE__, __LINE__, device->name, strerror(errno));
        return false;
    }
    device->fd = -1;

    if (device->is_verbose)
        print_verbose("Device '%s' sucessfully closed. \n", device->name);
    return true;
}

size_t read_tty(serial_device_t *device, uint8_t *buf, size_t n)
{
    pthread_mutex_lock(&device->thread_lock);
    bool thread_running = device->thread_data.run;
    pthread_mutex_unlock(&device->thread_lock);
    if (!thread_running)
    {
        print_err("%s:%d read_tty: Cannot read from device '%s' because its data thread isn't running.\n", __FILE__, __LINE__, device->name);
        return 0;
    }

    pthread_mutex_lock(&device->thread_lock);
    size_t bytes_r = ringbuf_pop_buf(device->recv_ringbuf, buf, n);
    pthread_mutex_unlock(&device->thread_lock);
    return bytes_r;
}

ssize_t write_tty(serial_device_t *device, uint8_t *buf, size_t n)
{
    pthread_mutex_lock(&device->thread_lock);
    bool thread_running = device->thread_data.run;
    pthread_mutex_unlock(&device->thread_lock);
    if (!thread_running)
    {
        print_err("%s:%d write_tty: Cannot write to device '%s' because its data thread isn't running.\n", __FILE__, __LINE__, device->name);
        return 0;
    }
    ssize_t bytes_w = write(device->fd, buf, n);
    if (bytes_w < 0)
    {
        print_err("Device '%s' on fd '%d' failed to send data.\n", device->name, device->fd);
        return bytes_w;
    }
    tcdrain(device->fd);
    if (device->is_verbose)
        print_verbose("Device '%s' sent %zd bytes.\n", device->name, bytes_w);
    return bytes_w;
}

void *run_data_thread_tty(void *thread_data_ptr)
{
    thread_data_t *tdata = (thread_data_t*)thread_data_ptr; // thread data from serial device
    serial_device_t *device = (serial_device_t*)tdata->data; // serial device

    struct pollfd pfd; // struct to store device file descriptor and poll events
    pfd.fd = device->fd;
    pfd.events = POLLIN;
    int pollval; // return value of poll() call

    uint8_t buf[DATA_BUFFER_SIZE];
    bool run_loop = true;

    while (run_loop)
    {
        pthread_mutex_lock(&device->thread_lock);
        // read
        if ((pollval = poll(&pfd, 1, 0)) > 0) // data available to read
        {
            ssize_t bytes_r = read(device->fd, buf, DATA_BUFFER_SIZE);
            if (bytes_r < 0) // fd error, connection lost
            {
                device->on_conn_lost(device);
                pthread_mutex_unlock(&device->thread_lock);
                break;
            }
            ssize_t bytes_pushed = ringbuf_push_buf(device->recv_ringbuf, buf, bytes_r);
            if (bytes_pushed < bytes_r) // receive overflow
            {
                ssize_t bytes_lost = bytes_r - bytes_pushed;
                device->on_recv_overflow(device, bytes_lost);
                debug_print_err("Device '%s' receive overflow by %zd bytes.\n", device->name, bytes_lost);
            }
            // read succeeded
            if (device->is_verbose)
                print_verbose("Device '%s' received %zd bytes.\n", device->name, bytes_pushed);
        }

        run_loop = tdata->run;
        pthread_mutex_unlock(&device->thread_lock);
    } // while (run_loop)

    pthread_mutex_lock(&device->thread_lock);
    tdata->run = false;
    pthread_mutex_unlock(&device->thread_lock);
}

unsigned int databits2flag(databits_t databits)
{
    switch (databits)
    {
    case DATABITS_5: return CS5;
    case DATABITS_6: return CS6;
    case DATABITS_7: return CS7;
    case DATABITS_8: return CS8;
    default: return 0;
    }
}

unsigned int parity2flag(parity_t parity)
{
    switch (parity)
    {
    case PARITY_NONE: return ~PARENB;
    case PARITY_EVEN: return PARENB;
    case PARITY_ODD: return PARENB | PARODD;
    case PARITY_MARK: return PARENB | PARODD | CMSPAR;
    default: return 0;
    }
}

unsigned int xon_xoff2flag(xon_xoff_t xon_xoff)
{
    switch (xon_xoff)
    {
    case XONXOFF_OFF: return ~(IXON | IXOFF);
    case XONXOFF_IN: return IXOFF;
    case XONXOFF_OUT: return IXON;
    case XONXOFF_INOUT: return IXON | IXOFF;
    default: return 0;
    }
}

int32_t baudrate2int(speed_t baudrate)
{
    switch (baudrate)
    {
    case B0: return 0;
    case B50: return 50;
    case B75: return 75;
    case B110: return 110;
    case B134: return 134;
    case B150: return 150;
    case B200: return 200;
    case B300: return 300;
    case B600: return 600;
    case B1200: return 1200;
    case B1800: return 1800;
    case B2400: return 2400;
    case B4800: return 4800;
    case B9600: return 9600;
    case B19200: return 19200;
    case B38400: return 38400;
    case B57600: return 57600;
    case B115200: return 115200;
    case B230400: return 230400;
    case B460800: return 460800;
    case B500000: return 500000;
    case B576000: return 576000;
    case B921600: return 921600;
    case B1000000: return 1000000;
    case B1152000: return 1152000;
    case B1500000: return 1500000;
    case B2000000: return 2000000;
    //case B2500000: return 2500000;
    //case B3000000: return 3000000;
    //case B3500000: return 3500000;
    //case B4000000: return 4000000;
    default: return 0;
    }
}

speed_t int2baudrate(int32_t speed)
{
    switch (speed)
    {
    case 0: return B0;
    case 50: return B50;
    case 75: return B75;
    case 110: return B110;
    case 134: return B134;
    case 150: return B150;
    case 200: return B200;
    case 300: return B300;
    case 600: return B600;
    case 1200: return B1200;
    case 1800: return B1800;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    //case 2500000: return B2500000;
    //case 3000000: return B3000000;
    //case 3500000: return B3500000;
    //case 4000000: return B4000000;
    default: return B0;
    }
}

const char parity2char(parity_t parity)
{
    switch (parity)
    {
    case PARITY_NONE: return 'N';
    case PARITY_EVEN: return 'E';
    case PARITY_ODD: return 'O';
    case PARITY_MARK: return 'M';
    default: return '-';
    }
}

parity_t char2parity(char c)
{
    switch (c)
    {
    case 'N': return PARITY_NONE;
    case 'E': return PARITY_EVEN;
    case 'O': return PARITY_ODD;
    case 'M': return PARITY_MARK;
    default: return PARITY_INVALID;
    }
}

const char *stopbits2str(stopbits_t stopbits)
{
    switch (stopbits)
    {
    case STOPBITS_ONE: return "1";
    case STOPBITS_ONE_HALF: return "1/2";
    case STOPBITS_TWO: return "2";
    default: return "INVALID";
    }
}

const char *xon_xoff2str(xon_xoff_t xon_xoff)
{
    switch (xon_xoff)
    {
    case XONXOFF_OFF: return "off";
    case XONXOFF_IN: return "input";
    case XONXOFF_OUT: return "output";
    case XONXOFF_INOUT: return "input/output";
    default: return "INVALID";
    }
}

#define COLOR_VERBOSE "\033[90m"
#define COLOR_SUCCESS "\033[92m"
#define COLOR_ERROR "\033[91m"
#define COLOR_RESET "\033[0m"

void print_err(const char *fmt, ...)
{
    fprintf(stderr, COLOR_ERROR);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, COLOR_RESET);
    fflush(stderr);
}

void print_ok(const char *fmt, ...)
{
    fprintf(stdout, COLOR_SUCCESS);
    va_list args;
    va_start(args, fmt);
    vfprintf(stdout, fmt, args);
    va_end(args);
    fprintf(stdout, COLOR_RESET);
    fflush(stderr);
}
void print_verbose(const char *fmt, ...)
{
    fprintf(stdout, COLOR_VERBOSE);
    va_list args;
    va_start(args, fmt);
    vfprintf(stdout, fmt, args);
    va_end(args);
    fprintf(stdout, COLOR_RESET);
    fflush(stderr);
}

void debug_print_err(const char *fmt, ...)
{
    #if DEBUG == 1
    fprintf(stderr, COLOR_ERROR);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, COLOR_RESET);
    fflush(stderr);
    #endif
}

void debug_print_ok(const char *fmt, ...)
{
    #if DEBUG == 1
    fprintf(stdout, COLOR_SUCCESS);
    va_list args;
    va_start(args, fmt);
    vfprintf(stdout, fmt, args);
    va_end(args);
    fprintf(stdout, COLOR_RESET);
    fflush(stderr);
    #endif
}

ringbuf_t *ringbuf_new(size_t size)
{
    ringbuf_t *ringbuf = (ringbuf_t*)malloc(sizeof(ringbuf_t) + size);
    ringbuf->maxlen = size;
    ringbuf->start = 0;
    ringbuf->end = 0;
    memset(ringbuf->buf, 0, size);
    return ringbuf;
}

bool ringbuf_push(ringbuf_t *ringbuf, uint8_t data)
{
    size_t next = ringbuf->end + 1;

    if (next >= ringbuf->maxlen) // loop back
        next = 0;

    if (next == ringbuf->start) // buffer is full
        return false;

    ringbuf->buf[ringbuf->end] = data;
    ringbuf->end = next;
    return true;
}

bool ringbuf_pop(ringbuf_t *ringbuf, uint8_t *data)
{
    if (ringbuf->start == ringbuf->end) // no data
        return false;

    size_t next = ringbuf->start + 1;
    if (next >= ringbuf->maxlen) // loop back
        next = 0;
    
    *data = ringbuf->buf[ringbuf->start];
    ringbuf->start = next;
    return true;
}

size_t ringbuf_push_buf(ringbuf_t *ringbuf, uint8_t *buf, size_t n)
{
    size_t i = 0;
    while (i < n)
    {
        if (!ringbuf_push(ringbuf, buf[i]))
            return i;
        ++i;
    }
    return i;
}

size_t ringbuf_pop_buf(ringbuf_t *ringbuf, uint8_t *buf, size_t n)
{
    size_t i = 0;
    while (i < n)
    {
        if (!ringbuf_pop(ringbuf, buf + i))
            return i;
        ++i;
    }
    return i;
}

size_t ringbuf_len(ringbuf_t *ringbuf)
{
    if (ringbuf->end >= ringbuf->start)
        return ringbuf->end - ringbuf->start;
    else
        return ringbuf->maxlen - ringbuf->start + ringbuf->end;
}

void ringbuf_print_all(ringbuf_t *ringbuf)
{
    size_t curr = ringbuf->start;
    while (curr != ringbuf->end)
    {
        printf("%c ", ringbuf->buf[curr]);

        ++curr;
        if (curr >= ringbuf->maxlen)
            curr = 0;
    }
    printf("\n");
}

#endif

#endif
