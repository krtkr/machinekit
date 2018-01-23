/*
 * udp_logger_lib.c
 *
 *  Created on: 7 янв. 2017 г.
 *      Author: krtkr
 */

#include <string.h>

#include "udp_logger.h"

static int get_vector(udp_logger_buffer_t *buf, udp_logger_types_t type) {
	if (buf->header->prefix != UDP_LOGGER_DATA) {
		return 1;
	}

	if (!(buf->vec_cur && buf->vec_cur->type == type)) {
		buf->vec_cur = (udp_logger_vector_t*)buf->buffer_cur;

		buf->buffer_cur += 4;

		if (buf->buffer_cur >= buf->buffer_end) {
			buf->vec_cur = 0;
			return 1;
		}

		buf->bytes_to_send += 4;

		buf->vec_cur->type = type;
		buf->vec_cur->count = 0;
	}

	return 0;
}

int init_header(udp_logger_buffer_t *buf) {
	buf->buffer_cur = buf->buffer + sizeof(*buf->header);
	buf->buffer_end = buf->buffer + UDP_LOGGER_BUFFER_SIZE;
	buf->header = (udp_logger_header_t*)buf->buffer;
	buf->header->prefix = UDP_LOGGER_HEADER;
	buf->header->seqnum = 0;
	buf->bytes_to_send = sizeof(*buf->header);
	return 0;
}

int append_name(udp_logger_buffer_t *buf, const char *name) {
	if (buf->header->prefix != UDP_LOGGER_HEADER) {
		return 1;
	}

	size_t len = strnlen(name, UDP_LOGGER_BUFFER_SIZE) + 1;

	if (buf->buffer_cur + len >= buf->buffer_end) {
		return 1;
	}

	memcpy(buf->buffer_cur, name, len);

	buf->buffer_cur += len;
	buf->bytes_to_send += len;

	return 0;
}


int init_data(udp_logger_buffer_t *buf) {
	buf->buffer_cur = buf->buffer + sizeof(*buf->header);
	buf->buffer_end = buf->buffer + UDP_LOGGER_BUFFER_SIZE;
	buf->header = (udp_logger_header_t*)buf->buffer;
	buf->header->prefix = UDP_LOGGER_DATA;
	buf->header->seqnum++;
	buf->vec_cur = 0;
	buf->bytes_to_send = sizeof(*buf->header);
	return 0;
}

int append_bit(udp_logger_buffer_t *buf, int value) {
	int pos;
	uint32_t *p_value;
	if (get_vector(buf, UDP_LOGGER_BIT)) {
		return 1;
	}

	p_value = (uint32_t*)buf->buffer_cur;

	if (buf->vec_cur->count % 32 == 0) {
		buf->buffer_cur += 4;

		if (buf->buffer_cur >= buf->buffer_end) {
			return 1;
		}

		buf->bytes_to_send += 4;
	}

	pos = 1 << (buf->vec_cur->count % 32);

	buf->vec_cur->count++;

	if (value) {
		*p_value |= pos;
	} else {
		*p_value &= ~pos;
	}

	return 0;
}

int append_uint(udp_logger_buffer_t *buf, uint32_t value) {
	uint32_t *p_value;
	if (get_vector(buf, UDP_LOGGER_UINT32)) {
		return 1;
	}

	p_value = (uint32_t*)buf->buffer_cur;

	buf->buffer_cur += 4;

	if (buf->buffer_cur >= buf->buffer_end) {
		return 1;
	}

	buf->bytes_to_send += 4;

	buf->vec_cur->count++;
	*p_value = value;

	return 0;
}

int append_int(udp_logger_buffer_t *buf, int32_t value) {
	int32_t *p_value;
	if (get_vector(buf, UDP_LOGGER_INT32)) {
		return 1;
	}

	p_value = (int32_t*)buf->buffer_cur;

	buf->buffer_cur += 4;

	if (buf->buffer_cur >= buf->buffer_end) {
		return 1;
	}

	buf->bytes_to_send += 4;

	buf->vec_cur->count++;
	*p_value = value;

	return 0;
}

int append_float(udp_logger_buffer_t *buf, float value) {
	float *p_value;
	if (get_vector(buf, UDP_LOGGER_FLOAT32)) {
		return 1;
	}

	p_value = (float*)buf->buffer_cur;

	buf->buffer_cur += 4;

	if (buf->buffer_cur >= buf->buffer_end) {
		return 1;
	}

	buf->bytes_to_send += 4;

	buf->vec_cur->count++;
	*p_value = value;

	return 0;
}

int append_double(udp_logger_buffer_t *buf, double value) {
	double *p_value;
	if (get_vector(buf, UDP_LOGGER_FLOAT64)) {
		return 1;
	}

	p_value = (double*)buf->buffer_cur;

	buf->buffer_cur += 8;

	if (buf->buffer_cur >= buf->buffer_end) {
		return 1;
	}

	buf->bytes_to_send += 8;

	buf->vec_cur->count++;
	*p_value = value;

	return 0;
}

