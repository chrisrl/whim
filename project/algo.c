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

/*******************************************************************************
															TYPE DEFINITIONS
*******************************************************************************/

/**
* Queue data structure to contain the samples that are integrated to determine the HIC score
*/
typedef struct{
	int head;
	int tail;
	float data[QUEUE_CAPACITY];
} sample_queue_t;


/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/

static sample_queue_t sample_queue = { // A sample queue structure that is used to store data in a FIFO manner
	.head = -1,
	.tail = -1,
};

/*******************************************************************************
																	PROCEDURES
*******************************************************************************/


static void algo_enqueue(float val)
{
	//TODO Implement enqueue functionaility and checks
}

static void algo_dequeue(float val)
{
	//TODO Implement dequeue functionaility and checks
}


/**
* @brief Function computes the numerical integration of y, using the trapezoidal method
* @param[in] y The data to be integrated
* @param[in] start,stop The beginning and endpoint of integration (inclusive)
*/
static double algo_trapezoidal(float y[], int start, int stop, double dx)
{
	double sum = 0;
	for(int i = start; i <= stop; i++)
	{
		sum += 2.0 * y[i];
	}
	return dx * sum/2.0;
}


/**
 * @brief Function determines the linear magnitude of a single x,y,z acceleration sample
 * @param[in] data_in Pointer to the data struct whose magnitude is being determined
 */
static float algo_get_linear_mag(accel_xyz_data_t *data_in)
{
	return sqrt(pow(data_in->out_x, 2) + pow(data_in->out_y,2) + pow(data_in->out_z,2));
}


/**
 * @brief Function initializes the accelerometer
 * This function writes the neccessary values to the desired CTRL registers to initialize the LIS2DH12
 * @param[in] accel_inst: accel instance
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
 * @brief Function initializes the accelerometer
 * This function writes the neccessary values to the desired CTRL registers to initialize the LIS2DH12
 * @param[in] accel_inst: accel instance
 */
uint16_t ALGO_impact_score(accel_xyz_data_t data_in[], float impact_data[])
{
	return 1;
}
