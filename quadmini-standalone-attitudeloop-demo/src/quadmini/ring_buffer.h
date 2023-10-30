#ifndef _RING_BUFFER_H
#define _RING_BUFFER_H

#include <atomic.h>

struct ring_buffer_ty
{
	unsigned char * ring_buf;
	int start;
	int end;
	int size; //buf capacity
	int len;
	int last_operation; //0:write 1:read
	corelock_t core_lock;
};

int ring_buffer_init(struct ring_buffer_ty * buf, int _size);
int push(struct ring_buffer_ty * buf, unsigned char * date, int _size);
int pop(struct ring_buffer_ty * buf, unsigned char * data, int _size);
int pop_with_block(struct ring_buffer_ty * buf, unsigned char * data, int _size);
int is_empty(struct ring_buffer_ty * buf);
int available_pop_bytes(struct ring_buffer_ty * buf);
int available_push_bytes(struct ring_buffer_ty * buf);

#endif
