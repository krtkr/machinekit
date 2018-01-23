/*
 * udp_logger.h
 *
 *  Created on: 6 янв. 2017 г.
 *      Author: krtkr
 */

#ifndef UDP_LOGGER_H_
#define UDP_LOGGER_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UDP_LOGGER_HEADER      0xaf82cd48
#define UDP_LOGGER_DATA        0xf750b1e6

#define UDP_LOGGER_HEADER_INV  0x48cd82af
#define UDP_LOGGER_DATA_INV    0xe6b150f7

#define UDP_LOGGER_MAX_DATA_SIZE 256

#define UDP_LOGGER_BUFFER_SIZE 2048

typedef struct {
	// Should be one of: UDP_LOGGER_HEADER or UDP_LOGGER_DATA
	uint32_t prefix;
	// Should be monotonically incremented from any initial number
	uint32_t seqnum;
} __attribute__ ((packed)) udp_logger_header_t;

typedef enum {
	UDP_LOGGER_BIT = 0,
	UDP_LOGGER_UINT32,
	UDP_LOGGER_INT32,
	UDP_LOGGER_FLOAT32,
	UDP_LOGGER_FLOAT64,
} udp_logger_types_t;

typedef struct {
	// One of udp_logger_types_t
	uint16_t type;
	// Elements count
	uint16_t count;
	// Keep any data of specified data here
	union {
		uint32_t bit_flags[UDP_LOGGER_MAX_DATA_SIZE/4];
		uint32_t uint_v[UDP_LOGGER_MAX_DATA_SIZE/4];
		int32_t int_v[UDP_LOGGER_MAX_DATA_SIZE/4];
		float float_v[UDP_LOGGER_MAX_DATA_SIZE/4];
		double double_v[UDP_LOGGER_MAX_DATA_SIZE/8];
	} vec;
} __attribute__ ((packed)) udp_logger_vector_t;

typedef struct {
	// Use bytes_to_send and buffer variables at sendto
	size_t bytes_to_send;
	char buffer[UDP_LOGGER_BUFFER_SIZE];
	// Following are internal variables, do not touch them
	char *buffer_cur;
	char *buffer_end;
	udp_logger_header_t *header;
	udp_logger_vector_t *vec_cur;
} udp_logger_buffer_t;

int init_header(udp_logger_buffer_t *buf);
int append_name(udp_logger_buffer_t *buf, const char *name);

int init_data(udp_logger_buffer_t *buf);
int append_bit(udp_logger_buffer_t *buf, int value);
int append_uint(udp_logger_buffer_t *buf, uint32_t value);
int append_int(udp_logger_buffer_t *buf, int32_t value);
int append_float(udp_logger_buffer_t *buf, float value);
int append_double(udp_logger_buffer_t *buf, double value);

#ifdef __cplusplus
}
#endif

#endif /* UDP_LOGGER_H_ */
