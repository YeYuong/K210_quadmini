#include "data_exchange_printf.h"
#include "data_exchange.h"

#include "printf.h"

typedef __gnuc_va_list _va_list;
#define UNSIGNED 0
#define SIGNED 1
#define FLOAT 0
#define DOUBLE 1
#define __INTSIZEOF(t)	((sizeof(t) + sizeof(int) - 1) & ~(sizeof(int) - 1))
// #define va_start(ap,v)	(ap = (_va_list)&v + __INTSIZEOF(v))
// #define va_arg(ap,t)	(*(t *)((ap += (((unsigned int)ap + sizeof(t) - 1) & ~(sizeof(t) - 1)) - (unsigned int)ap + __INTSIZEOF(t)) - __INTSIZEOF(t)))
// #define va_end(ap) (ap = (_va_list)0)
#define va_start(v,l)	__builtin_va_start(v,l)
#define va_end(v)	__builtin_va_end(v)
#define va_arg(v,l)	__builtin_va_arg(v,l)
#define MAX_OUT_CHAR 200
#define MAX_NUMBER_BYTES 200

char hex_tab[16] = { '0','1','2','3','4','5','6','7',
'8','9','A','B','C','D','E','F' };
int char_position = 0;

void putsw(char *);
int printw(const char * fmt, ...);
static int double_to_str(double n, char lead, int maxwidth, int decimalwidth, char * data_buf);
static int int_to_str(long n, int radix, char lead, int maxwidth, int sign, char * data_buf);

void putsw(char * s)
{
    str_pack_send(s);
}

static int double_to_str(double n, char lead, int maxwidth, int decimalwidth, char * data_buf)
{
	int i;
	double m = (n>0?n:-n);
	int x;
	int count = 0;
	int size = 0;
	char buf[MAX_OUT_CHAR], *s = buf + MAX_OUT_CHAR - 1;
	*s = '\0';
	if (decimalwidth == -1)
		decimalwidth = 6;
	for (i = decimalwidth; i; i--)
		m *= 10;
	if ((int)(m * 10) % 10 >= 5)
		m += 1;
	x = (int)m;
	i = decimalwidth;
	do
	{
		*--s = hex_tab[x % 10];
		if (--i == 0)
			*--s = '.';
	} while ((x /= 10) != 0 || i > 0);
	if ((int)n == 0)
		*--s = '0';
	count = buf + MAX_OUT_CHAR - s - 1;
	if (n < 0 && lead == ' ')
	{
		*--s = '-';
		count++;
	}
	for (; count<maxwidth; count++)
	{
		*--s = lead;
	}
	if(n<0 && lead == '0')
		*s = '-';
	while(*s)
	{
		*data_buf++ = *s++;
		size++;
	}
	return size;
}

static int int_to_str(long n, int radix, char lead, int maxwidth, int sign, char * data_buf)
{
	unsigned long m;
	int size = 0;;
	if (sign == 1)
		m = (n > 0) ? n : -n;
	else
		m = n;
	int count = 0;
	char buf[MAX_OUT_CHAR];
	char * s = buf + MAX_OUT_CHAR - 1;
	*s = '\0';
	do
	{
		*--s = hex_tab[m%radix];
		count++;
	} while ((m /= radix) != 0);
	if (n < 0 && lead == ' ' && sign)
	{
		*--s = '-';
		count++;
	}
	for (; count<maxwidth; count++)
	{
		*--s = lead;
	}
	if (n<0 && lead == '0' && sign)
		*s = '-';
	while(*s)
	{
		*data_buf++ = *s++;
		size++;
	}
	return size;
}

int printw(const char * fmt, ...)
{
	char lead = ' ';
	static char out_data[MAX_OUT_CHAR];
	char * s;
	int out_data_cnt = 0;
	int maxwidth;
	int decimalwidth;
	_va_list ap;
	va_start(ap, fmt);
	for (; *fmt; fmt++)
	{
		if (*fmt != '%')
		{
			out_data[out_data_cnt++] = *fmt;
			continue;
		}
		fmt++;
		lead = ' ';
		if (*fmt == '0')
		{
			lead = '0';
			fmt++;
		}
		maxwidth = 0;
		while (*fmt >= '0'&&*fmt <= '9')
		{
			maxwidth *= 10;
			maxwidth += (*fmt - '0');
			fmt++;
		}
		decimalwidth = -1;
		if (*fmt == '.')
		{
			fmt++;
			decimalwidth = 0;
		}
		while (*fmt >= '0'&&*fmt <= '9')
		{
			if (decimalwidth<0)	decimalwidth = 0;
			decimalwidth *= 10;
			decimalwidth += (*fmt - '0');
			fmt++;
		}
		switch (*fmt)
		{
		case 'd':
			out_data_cnt += int_to_str(va_arg(ap, int), 10, lead, maxwidth, 1, &out_data[out_data_cnt]);
			break;
		case 'o':
			out_data_cnt += int_to_str(va_arg(ap, unsigned int), 8, lead, maxwidth, 0, &out_data[out_data_cnt]);
			break;
		case 'u':
			out_data_cnt += int_to_str(va_arg(ap, unsigned int), 10, lead, maxwidth, 0, &out_data[out_data_cnt]);
			break;
		case 'x':
			out_data_cnt += int_to_str(va_arg(ap, unsigned int), 16, lead, maxwidth, 0, &out_data[out_data_cnt]);
			break;
		case 'f':
			out_data_cnt += double_to_str(va_arg(ap, double), lead, maxwidth, decimalwidth, &out_data[out_data_cnt]);
			break;
		case 'c':
			out_data[out_data_cnt++] = va_arg(ap, int);
			break;
		case 's':
			s = va_arg(ap, char *);
			while(*s)
				out_data[out_data_cnt++] = *s++;
			break;
		default:
			out_data[out_data_cnt++] = *fmt;
			break;
		}
	}
	out_data[out_data_cnt] = 0;
	putsw(out_data);
	return 0;
}

