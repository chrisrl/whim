/******************************************************************************
File: algo.c

This source file contains the functions and variables responsible for determining
the severity of impact experience by a wearer of the WHIM device

******************************************************************************/


/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/

#include <string.h>
#include <math.h>
#include "sdk_config.h"
#include "ADXL375.h"
#include "algo.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*******************************************************************************
															TYPE DEFINITIONS
*******************************************************************************/

/**
* Queue data structure to contain the samples that are integrated to determine the HIC score
*/
typedef struct{
	int head;
	int tail;
	int size;
	float data[QUEUE_CAPACITY];
} sample_queue_t;


/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
#define ALGO_DEBUG

/** 
 * Uncomment one of these depending on what type of numerical integration method is 
 * desired for the HIC score calculation.
*/
#define RECTANGULAR 
//#define TRAPEZOID

static sample_queue_t sample_queue = { // A sample queue structure that is used to store data in a FIFO manner
	.head = -1,
	.tail = -1,
	.size = 0,
	.data = {0},
};

/*******************************************************************************
																	PROCEDURES
*******************************************************************************/

/**
 * @brief Function determines the linear magnitude of a single x,y,z acceleration sample
 * @param[in] data_in Pointer to the data struct whose magnitude is being determined
 */
static float algo_get_linear_mag(accel_xyz_data_t *data_in)
{
	return sqrt(pow(data_in->out_x, 2) + pow(data_in->out_y,2) + pow(data_in->out_z,2));
}


/**
* @brief Function computes the numerical integration of y, using the left-rectangular method
* @param[in] y The data to be integrated
* @param[in] start,stop The beginning and endpoint of integration (inclusive)
*/
//static double algo_left_rectangular(float y[], double dx)
//{
//	double sum = 0;
//	for(int i = 0; i <= stop; i++)
//	{
//	 sum += y[i];
//	}
//	return dx * sum;
//}


/**
* @brief Function computes the numerical integration of y, using the trapezoidal method
* @param[in] y The data to be integrated
* @param[in] start,stop The beginning and endpoint of integration (inclusive)
*/
//static double algo_trapezoidal(float y[], double dx)
//{
//	double sum = 0;
//	for(int i = 0; i <= stop; i++)
//	{
//		sum += 2.0 * y[i];
//	}
//	return dx * sum/2.0;
//}


/**
* Function returns true if the sample queue is empty
*/
static bool algo_queue_is_empty()
{
	return sample_queue.size == 0;
}


/**
* Function returns true if the sample queue is full
*/
static bool algo_queue_is_full()
{
	return sample_queue.size == QUEUE_CAPACITY;
}


/**
* @brief This function dequeues the oldest element from the global algorithm sampling queue
*/
static void algo_queue_dequeue()
{
		sample_queue.size--; // Size is decremented, head is set the the next element in queue
		sample_queue.data[sample_queue.head] = 0; // Zero out the dequeued element for easier integration (Can be modified later)
		sample_queue.head++;
}


/**
* @brief This function enqueues a new element into the global algorithm sampling queue
*/
static void algo_queue_enqueue(float val)
{
		if (algo_queue_is_empty()) // If queue is empty, place data in first element. Head and tail point to that element.
		{
			sample_queue.data[0] = val;
			sample_queue.tail++;
			sample_queue.head++;
			sample_queue.size++;
		}
		else if(sample_queue.tail == QUEUE_CAPACITY -1) // If tail is at the last element, wrap around to first element. Tail is 0, head is unchanged.
		{
			sample_queue.tail = 0;
			sample_queue.size++;
			sample_queue.data[sample_queue.tail] = val;
		}
		else // Normal enqueue operation. Data is placed at index tail + 1.
		{
			sample_queue.tail++;
			sample_queue.size++;
			sample_queue.data[sample_queue.tail] = val;
		}
}


/**
 * @brief Function checks whether any of the linear magnitudes determined from the FIFO buffer lie above a constant threshold
 * @param[in] data_in The accelerometer xyz data being parsed
 * @param[out] impact_data The magnitudes of the xyz data for all samples in the FIFO
 */
bool ALGO_threshold_check(accel_xyz_data_t data_in[], float impact_data[])
{
	uint8_t batch_flag = 0; // This flag is used to indicate whether an impact has been detected in this batch of data
	
	for(int i = 0; i < ACCEL_FIFO_LENGTH; i++)
	{
		impact_data[i] = algo_get_linear_mag(&data_in[i]);
		
		if(impact_data[i] > IMPACT_THRESHOLD)
		{
			batch_flag = 1;
		}
	}
	
	return batch_flag == 1; // If batch flag == 1, an impact has been detected. Return true
}


/**
 * @brief Function determines the HIC score using numerical integration
 * @param[in] data_in The accelerometer xyz data being parsed
 */
float ALGO_impact_score(accel_xyz_data_t data_in[])
{
	float HIC_score = 0;
	
	#ifdef RECTANGULAR
		//TODO add integration on data in queue
	#endif
	
	#ifdef TRAPEZOIDAL
		//TODO add integration on data in queue
	#endif
	
	return HIC_score;
}
