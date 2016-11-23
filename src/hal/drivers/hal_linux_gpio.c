/*
 * hal_linux_gpio.c
 *
 *  Created on: 23 апр. 2016 г.
 *      Author: krtkr
 */


#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_bitops.h"
#include "rtapi_app.h"		/* RTAPI realtime module decls */
                                /* this also includes config.h */
#include "hal.h"		/* HAL public API decls */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

/* module information */
MODULE_AUTHOR("Kirill Kranke");
MODULE_DESCRIPTION("Standard Linux GPIO driver connector (aka /sys/class/gpio");
MODULE_LICENSE("GPL");

/* module parameters */
#define MAX_PINS_COUNT 32
int gpio_pins[] = { [0 ... MAX_PINS_COUNT-1] = -1 } ;
RTAPI_MP_ARRAY_INT(gpio_pins, MAX_PINS_COUNT, "Used GPIO pins array, for example: '3,12,8' will use 3 corresponding GPIOs, mapping 3 GPIO to 0 module pin etc");

int gpio_dirs[] = { [0 ... MAX_PINS_COUNT-1] = -1 } ;
RTAPI_MP_ARRAY_INT(gpio_dirs, MAX_PINS_COUNT, "Corresponding to 'gpio_pins' directions: 1 is output, 0 - input");

static int npins;
static int comp_id;		/* component ID */
hal_bit_t **port_data;

int gpio_value_fds[] = { [0 ... MAX_PINS_COUNT-1] = -1 } ;

static int export_pins() {
	char buffer[32];
	int n, retval;
	size_t bytes;
	int export_fd = open("/sys/class/gpio/export", O_WRONLY);

	errno = 0;
	if (export_fd > 0) {
		retval = 0;
		for (n = 0; n < npins; n++) {
			bytes = snprintf(buffer, 31, "%i", gpio_pins[n]);
			if (write(export_fd, buffer, bytes) != (ssize_t)bytes) {
				rtapi_print_msg(RTAPI_MSG_ERR, "Unable to export pin #%i mapped to gpio #%i, errno = %i\n", n, gpio_pins[n], errno);
				retval = -2;
				break;
			}
		}
		close(export_fd);
	} else {
		rtapi_print_msg(RTAPI_MSG_ERR, "Unable to open gpio export file, errno = %i\n", errno);
		retval = -1;
	}

	return retval;
}

static void unexport_pins() {
	char buffer[32];
	int n;
	size_t bytes;
	int export_fd = open("/sys/class/gpio/unexport", O_WRONLY);

	errno = 0;
	if (export_fd > 0) {
		for (n = 0; n < npins; n++) {
			bytes = snprintf(buffer, 31, "%i", gpio_pins[n]);
			if (write(export_fd, buffer, bytes) != (ssize_t)bytes) {
				break;
			}
		}
		close(export_fd);
	}

}

static int set_directions() {
	char path_buffer[512];
	int n, retval;
	size_t bytes;
	int gpio_fd;

	errno = 0;
	for (n = 0; n < npins; n++) {
		snprintf(path_buffer, 511, "/sys/class/gpio/gpio%i/direction", gpio_pins[n]);
		gpio_fd = open(path_buffer, O_WRONLY);
		if (gpio_fd > 0) {
			bytes = gpio_dirs[n] ? 3 : 2;
			if (write(gpio_fd, gpio_dirs[n] ? "out" : "in", bytes) != (ssize_t)bytes) {
				rtapi_print_msg(RTAPI_MSG_ERR, "Unable to set direction (%i) for pin #%i mapped to gpio #%i, errno = %i\n", gpio_dirs[n], n, gpio_pins[n], errno);
				retval = -2;
				break;
			}
			close(gpio_fd);
		} else {
			rtapi_print_msg(RTAPI_MSG_ERR, "Unable to open direction file for pin #%i mapped to gpio #%i, errno = %i\n", n, gpio_pins[n], errno);
			retval = -1;
			break;
		}
	}

	return retval;
}

static int open_pin_files() {
	char path_buffer[512];
	int n, retval;

	errno = 0;
	for (n = 0; n < npins; n++) {
		snprintf(path_buffer, 511, "/sys/class/gpio/gpio%i/value", gpio_pins[n]);
		gpio_value_fds[n] = open(path_buffer, O_RDWR);

		if (gpio_value_fds[n] <= 0) {
			rtapi_print_msg(RTAPI_MSG_ERR, "Unable to open value file for pin #%i mapped to gpio #%i, errno = %i\n", n, gpio_pins[n], errno);
			retval = -1;
			break;
		}
	}

	return retval;
}

static void close_pin_files() {
	int n;

	for (n = 0; n < npins; n++) {
		if (gpio_value_fds[n] > 0) {
			close(gpio_value_fds[n]);
			gpio_value_fds[n] = -1;
		}
	}
}

static void read_port(void *arg, long period) {
	int n;
	char buffer;

	for (n = 0; n < npins; n++) {
		if (!gpio_dirs[n]) {
			read(gpio_value_fds[n], &buffer, 1);
			*(port_data[n]) = buffer - '0';
		}
	}
}

static void write_port(void *arg, long period) {
	int n;
	char buffer;

	for (n = 0; n < npins; n++) {
		if (gpio_dirs[n]) {
			write(gpio_value_fds[n], *(port_data[n]) ? "1" : "0", 1);
		}
	}
}

int rtapi_app_main(void) {
	int retval;
	int n;

	for (npins = 0; npins < MAX_PINS_COUNT; npins++) {
		if ((gpio_pins[npins] < 0) || (gpio_dirs[npins] < 0) || (gpio_dirs[npins] > 1)) {
			break;
		}
	}

	if (!npins) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: ERROR: no pins configured found, forgot to add 'gpio_pins' and 'gpio_dirs' module parameters?\n");
		retval = -2;
		goto cleanup_nothing;
	} else {
		rtapi_print_msg(RTAPI_MSG_INFO, "HAL_LINUX_GPIO: found configured %i GPIO's\n", npins);
	}

	port_data = hal_malloc(npins * sizeof(void *));
	if (port_data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: ERROR: hal_malloc() failed\n");
		retval = -3;
		goto cleanup_nothing;
	}

	comp_id = hal_init("hal_gpio");
	if (comp_id < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: ERROR: hal_init() failed\n");
		retval = -4;
		goto cleanup_nothing;
	}

	retval = export_pins();
	if (retval) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: unable to export pins\n");
		retval = -5;
		goto cleanup_hal_comp;
	}

	retval = set_directions();
	if (retval) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: unable to set directions, maybe you tried to use GPIO that does not exist?\n");
		retval = -6;
		goto cleanup_unexport;
	}

	retval = open_pin_files();
	if (retval) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: unable to open one of pins files, I guess this should not happen\n");
		retval = -7;
		goto cleanup_close_pins;
	}

	for (n = 0; n < npins; n++) {
		if (gpio_dirs[n]) {
			retval = hal_pin_bit_newf(HAL_OUT, &port_data[n],
					comp_id, "hal_linux_gpio.pin-%02d-out", n);
		} else {
			retval = hal_pin_bit_newf(HAL_IN, &port_data[n],
					comp_id, "hal_linux_gpio.pin-%02d-in", n);
		}
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: ERROR: unable to create pin #%i\n, retval = %i", n, retval);
			retval = -8;
			goto cleanup_close_pins;
		}
	}

	retval = hal_export_funct("hal_gpio.read", read_port, 0, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: ERROR: 'read' function export failed\n");
		retval = -9;
		goto cleanup_close_pins;
	}

	retval = hal_export_funct("hal_gpio.write", write_port, 0, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LINUX_GPIO: ERROR: 'write' function export failed\n");
		retval = -10;
		goto cleanup_close_pins;
	}

	return 0;

cleanup_close_pins:
	close_pin_files();

cleanup_unexport:
	unexport_pins();

cleanup_hal_comp:
	hal_exit(comp_id);

cleanup_nothing:
	return retval;
}

void rtapi_app_exit(void) {
	close_pin_files();
	unexport_pins();
	hal_exit(comp_id);
}
