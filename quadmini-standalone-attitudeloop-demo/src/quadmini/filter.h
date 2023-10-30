#ifndef _FILTER_H
#define _FILTER_H

struct moving_median_filter_ty{
	float * array;
	int array_size;
	int array_cnt;
	int data_update_position;
	float filter_out;
};

void moving_median_filter_init(struct moving_median_filter_ty * median_data, float size);
float moving_median_filter(struct moving_median_filter_ty * median_data, float data);
void first_order_lpf(float * lpf, float cut_off_freq, float period, float data);

#endif
