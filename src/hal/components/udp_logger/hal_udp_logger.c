/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.
*/

#include "udp_logger.h"

#include "rtapi.h"            /* RTAPI realtime OS API */
#include "rtapi_app.h"        /* RTAPI realtime module decls */
#include "hal.h"            /* HAL public API decls */

#include <unistd.h>

// Linux
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>

#define MAX_ELEMENTS 64

/* module information */
MODULE_AUTHOR("Kirill Kranke");
MODULE_DESCRIPTION("Simple UDP logger interface");
MODULE_LICENSE("GPL");

char *header[MAX_ELEMENTS] = { [0 ... MAX_ELEMENTS - 1] = 0 };
RTAPI_MP_ARRAY_STRING(header, MAX_ELEMENTS, "header for elements, elements processed in following order: bits, uint, sint, float");

int counts[4] = { [0 ... 3] = -1 } ;
RTAPI_MP_ARRAY_INT(counts, 4, "how many elements of each type (bits, uint, sint, float) to grab");

char *host;
RTAPI_MP_STRING(host, "host name or IP address");

char *port;
RTAPI_MP_STRING(port, "UDP port");

static void send_log(void *arg, long period);

typedef struct {
    hal_bit_t *bit_in[MAX_ELEMENTS];          /* pins: input bit data */
    hal_u32_t *u32_in[MAX_ELEMENTS];          /* pins: input u32 data */
    hal_s32_t *s32_in[MAX_ELEMENTS];          /* pins: input s32 data */
    hal_float_t *float_in[MAX_ELEMENTS];      /* pins: input float data */
} hal_udp_logger_t;

static hal_udp_logger_t *hal_udp_logger;
static int send_header;
static int total;
static int comp_id;                       /* component ID */

static int sockfd;
static struct addrinfo *linfo;
static udp_logger_buffer_t log_buf;

int rtapi_app_main(void)
{
    int n, retval = -500;

    int total_elements = 0;

    total = 0;

    struct addrinfo hints;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_flags = AI_PASSIVE;
	hints.ai_socktype = SOCK_DGRAM;

    linfo = 0;
    sockfd = 0;

    for (n = 0; n < 4; n++) {
        if (counts[n] < MAX_ELEMENTS && counts[n] >= 0) {
            total_elements += counts[n];
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "UDP_LOGGER: ERROR: provide count for every type of element within [0:%i]\n", MAX_ELEMENTS);
            retval = -1;
            goto clean_up_nothing;
        }
    }

    if (total_elements > MAX_ELEMENTS && total_elements < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "UDP_LOGGER: ERROR: total count should be [1:%i]\n", MAX_ELEMENTS);
        retval = -2;
        goto clean_up_nothing;
    }

    for (n = 0; n < MAX_ELEMENTS; n++) {
        if (header[n]) {
            total++;
        } else {
            break;
        }
    }

    if (total != total_elements) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "UDP_LOGGER: ERROR: total elements at header (%i) and summ of counts (%i) should agree\n", total, total_elements);
        retval = -3;
        goto clean_up_nothing;
    }

    send_header = 1;

    /* Connect to the HAL */
    comp_id = hal_init("hal_udp_logger");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "UDP_LOGGER: ERROR: hal_init() failed\n");
        retval = -4;
        goto clean_up_nothing;
    }

    /* Allocate shared memory for data */
    hal_udp_logger = hal_malloc(sizeof(hal_udp_logger_t));
    if (hal_udp_logger == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "UDP_LOGGER: ERROR: hal_malloc() failed\n");
        retval = -5;
        goto clean_up_component;
    }

    // Export pins
    for (n = 0; n < counts[0]; n++) {
        retval = hal_pin_bit_newf(HAL_IN, &(hal_udp_logger->bit_in[n]), comp_id, "hal_udp_logger.bit_in.%d", n);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "UDP_LOGGER: ERROR: failed to export bit_in[%i] variables\n", n);
            retval = -6;
            goto clean_up_memory;
        }
    }

    for (n = 0; n < counts[1]; n++) {
        retval = hal_pin_u32_newf(HAL_IN, &(hal_udp_logger->u32_in[n]), comp_id, "hal_udp_logger.u32_in.%d", n);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "UDP_LOGGER: ERROR: failed to export u32_in[%i] variables\n", n);
            retval = -7;
            goto clean_up_memory;
        }
    }

    for (n = 0; n < counts[2]; n++) {
        retval = hal_pin_s32_newf(HAL_IN, &(hal_udp_logger->s32_in[n]), comp_id, "hal_udp_logger.s32_in.%d", n);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "UDP_LOGGER: ERROR: failed to export s32_in[%i] variables\n", n);
            retval = -8;
            goto clean_up_memory;
        }
    }

    for (n = 0; n < counts[3]; n++) {
        retval = hal_pin_float_newf(HAL_IN, &(hal_udp_logger->float_in[n]), comp_id, "hal_udp_logger.float_in.%d", n);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "UDP_LOGGER: ERROR: failed to export float_in[%i] variables\n", n);
            retval = -9;
            goto clean_up_memory;
        }
    }

    /* Export functions */
    retval = hal_export_funct("hal_udp_logger.send", send_log, hal_udp_logger, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "UDP_LOGGER: ERROR: send function export failed\n");
        retval = -10;
        goto clean_up_memory;
    }

    // Convert host and port strings to socket info
    if ((retval = getaddrinfo(host, port, &hints, &linfo)) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "UDP_LOGGER: ERROR: unable to parse address/port getaddrinfo: %s\n", gai_strerror(retval));
        retval = -11;
        goto clean_up_memory;
    }

    errno = 0;
    sockfd = socket(linfo->ai_family, linfo->ai_socktype, linfo->ai_protocol);
    if (sockfd == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "UDP_LOGGER: ERROR: Unable to open listen socket, posix error: %i \n", errno);
        retval = -12;
        goto clean_up_linfo;
    }

    return 0;

clean_up_linfo:
    freeaddrinfo(linfo);
    linfo = 0;

clean_up_memory:
    // No hal_free, ignore

clean_up_component:
    hal_exit(comp_id);

clean_up_nothing:
    return retval;
}

void rtapi_app_exit(void) {
    if (sockfd > 0) {
        close(sockfd);
        sockfd = 0;
    }
    if (linfo) {
        freeaddrinfo(linfo);
    }
    hal_exit(comp_id);
}

static void send_log(void *arg, long period) {
    int n;

    if (send_header) {
        send_header = 0;

        init_header(&log_buf);

        for (n = 0; n < total; n++) {
            append_name(&log_buf, header[n]);
        }

        sendto(sockfd, log_buf.buffer, log_buf.bytes_to_send, 0, linfo->ai_addr, linfo->ai_addrlen);
    }

    init_data(&log_buf);

    for (n = 0; n < counts[0]; n++) {
        append_bit(&log_buf, *(hal_udp_logger->bit_in[n]));
    }

    for (n = 0; n < counts[1]; n++) {
        append_uint(&log_buf, *(hal_udp_logger->u32_in[n]));
    }

    for (n = 0; n < counts[2]; n++) {
        append_int(&log_buf, *(hal_udp_logger->s32_in[n]));
    }

    for (n = 0; n < counts[3]; n++) {
        append_double(&log_buf, *(hal_udp_logger->float_in[n]));
    }

    sendto(sockfd, log_buf.buffer, log_buf.bytes_to_send, 0, linfo->ai_addr, linfo->ai_addrlen);
}

