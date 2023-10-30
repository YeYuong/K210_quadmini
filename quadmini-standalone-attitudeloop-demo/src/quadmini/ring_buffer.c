#include <stdlib.h>

#include <sysctl.h>
#include <ring_buffer.h>

int ring_buffer_init(struct ring_buffer_ty * buf, int _size)
{
	buf->size = _size;
    buf->start = 0;
    buf->end = 0;
    buf->last_operation = 1;
    buf->len = 0;
    buf->ring_buf = (unsigned char *)malloc(_size);

    return 0;
}

int is_empty(struct ring_buffer_ty * buf)
{
	return (buf->len == 0);
}

int available_pop_bytes(struct ring_buffer_ty * buf)
{
	return buf->len;
}

int available_push_bytes(struct ring_buffer_ty * buf)
{
	return buf->size - buf->len;
}

int _push(struct ring_buffer_ty * buf, unsigned char data)
{
	int flag = 0;
	if(buf->start == buf->end && buf->last_operation == 1)	//ring buffer is empty
	{
		buf->ring_buf[buf->end] = data;
		buf->len++;
		if(buf->end == buf->size - 1)
		{
			buf->end = 0;
		}
		else
		{
			buf->end++;
		}
	}
	else if((flag = ((buf->start == buf->end) && (buf->last_operation == 0)))) //ring buffer is full, do nothing
	{
//		buf->ring_buf[buf->end] = data; //discard an old byte
//		if(buf->end == buf->size - 1)
//		{
//			buf->start = buf->end = 0;
//		}
//		else
//		{
//			buf->start = ++(buf->end);
//		}
		;
	}
	else	//ring buffer has space
	{
		buf->ring_buf[buf->end] = data;
		buf->len++;
		if(buf->end == buf->size - 1)
		{
			buf->end = 0;
		}
		else
		{
			buf->end++;
		}
	}

	buf->last_operation = 0; //last operation is write

	return -flag; //return whether buf is full
}

int push(struct ring_buffer_ty * buf, unsigned char * date, int _size)
{
	int p = 0;
	for(; p < _size; p++)
	{
		if(_push(buf, *(date + p)) == -1)
			break;
	}

	return p; //return size of write
}

// int push(struct ring_buffer_ty * buf, unsigned char * data, int _size)
// {
// 	int p = 0;
// 	int availd_len = buf->size - buf->len;
// 	int push_len = availd_len > _size ? _size : availd_len;
// 	buf->len += push_len;

// 	if(buf->start > buf->end)
// 	{
// 		for(p = 0; p < push_len; p++)
// 			buf->ring_buf[buf->end++] = data[p];
// 	}
// 	else if(buf->start <= buf->end && push_len > 0)
// 	{
// 		int tmp = buf->size - buf->end;
// 		tmp = tmp > push_len ? push_len : tmp;
// 		for(p = 0; p < tmp; p++)
// 			buf->ring_buf[buf->end++] = data[p];
// 		if(buf->end == buf->size)
// 		{
// 			buf->end = 0;
// 			for(; p < push_len; p++)
// 				buf->ring_buf[buf->end++] = data[p];
// 		}
// 	}

// 	buf->last_operation = 0; //last operation is write

// 	return push_len; //return size of write
// }

int _pop(struct ring_buffer_ty * buf, unsigned char * data)
{
	int flag = 0;
	sysctl_disable_irq();
	if((flag = ((buf->start == buf->end) && (buf->last_operation == 1))))	//ring buffer is empty, do nothing
	{
		//cout << "Failed to pop data : ring buffer is empty" << endl;
	}
	else if(buf->start == buf->end && buf->last_operation == 0) //ring buffer is full
	{
		*data = buf->ring_buf[buf->start]; //pop an old byte
		buf->len--;
		if(buf->start == buf->size - 1)
		{
			buf->start = 0;
		}
		else
		{
			buf->start++;
		}
	}
	else	//ring buffer has data
	{
		*data = buf->ring_buf[buf->start]; //pop an old byte
		buf->len--;
		if(buf->start == buf->size - 1)
		{
			buf->start = 0;
		}
		else
		{
			buf->start++;
		}
	}

	buf->last_operation = 1; //last operation is read

	sysctl_enable_irq();

	return -flag;
}

int pop(struct ring_buffer_ty * buf, unsigned char * data, int _size)
{
	int p = 0;
	for(; p < _size; p++)
	{
		if(_pop(buf, data + p) == -1)
			break;
	}

	return p; //return size of read
}

// int pop(struct ring_buffer_ty * buf, unsigned char * data, int _size)
// {
// 	int p = 0;
// 	int availd_len = buf->len;
// 	int pop_len = availd_len > _size ? _size : availd_len;
// 	buf->len -= pop_len;

// 	if(buf->start < buf->end)
// 	{
// 		for(p = 0; p < pop_len; p++)
// 			data[p] = buf->ring_buf[buf->start++];
// 	}
// 	else if(buf->start >= buf->end && pop_len > 0)
// 	{
// 		int tmp = buf->size - buf->start;
// 		tmp = tmp > pop_len ? pop_len : tmp;
// 		for(p = 0; p < tmp; p++)
// 			data[p] = buf->ring_buf[buf->start++];
// 		if(buf->start == buf->size)
// 		{
// 			buf->start = 0;
// 			for(; p < pop_len; p++)
// 				data[p] = buf->ring_buf[buf->start++];
// 		}
// 	}

// 	buf->last_operation = 1; //last operation is read

// 	return pop_len; //return size of read
// }

#include <sysctl.h>

int pop_with_block(struct ring_buffer_ty * buf, unsigned char * data, int _size)
{
	int p = 0;

	for(; p < _size; p++)
	{
		while (1)
		{
			if (_pop(buf, data + p) == 0)
			{
				break;
			}
		}
	}


	return p; //return size of read
}
