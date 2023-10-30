#include <stdlib.h>
#include <filter.h>

/*	滑动中位数滤波 
 */
//***************************************************
/*	滑动中位数滤波器初始化
 *	median_data : 滤波数据结构体
 *	size : 滑动深度，即滑动窗口中的数据个数
 */
void moving_median_filter_init(struct moving_median_filter_ty * median_data, float size)
{
	median_data->array = malloc(size * sizeof(float));
 	median_data->array_size = size;
	median_data->array_cnt = 0;
	median_data->data_update_position = 0;
}
/*	滑动中位数滤波
 *	median_data : 滤波数据结构体
 *	data : 新数据
 *	返回中位数
 */
float moving_median_filter(struct moving_median_filter_ty * median_data, float data)
{
	float * array = median_data->array;
	int * array_cnt = &(median_data->array_cnt);
	int * array_size = &(median_data->array_size);
	int * data_update_posi = &(median_data->data_update_position);
	float * filter_out = &(median_data->filter_out);

	if(*array_cnt < *array_size)
	{
		array[(*array_cnt)++] = data;
		(*data_update_posi)++;
		if(*array_size == *data_update_posi)
			*data_update_posi = 0;
	}
	else
	{
		array[(*data_update_posi)++] = data;
		if(*array_size == *data_update_posi)
			*data_update_posi = 0;
	}

	for(int i = 0; i < *array_cnt - 1; i++)
	{
		for(int j = 0; j < *array_cnt - 1 - i; j++)
		{
			if(array[j] > array[j + 1])
			{	
				float tmp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = tmp;
			}
		}

	}

	*filter_out = array[(int)(*array_cnt / 2)];

	return *filter_out;
}
//***************************************************

/*	低通滤波器
 */
//***************************************************
void first_order_lpf(float * lpf, float cut_off_freq, float period, float data)
{
	*lpf += (1 / (1 + 1/(2.0f * 3.1415f * period * cut_off_freq))) * (data - *lpf);

	return;
}

//***************************************************