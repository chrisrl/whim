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
#include "whim_queue.h"

/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
#define ALGO_DEBUG

WHIM_QUEUE_DEF(algo_queue, QUEUE_CAPACITY, WHIM_QUEUE_MODE_OVERFLOW); // Global queue for impact data

uint8_t impact_score_flag = 0;

/*******************************************************************************
																	PROCEDURES
*******************************************************************************/

/**
 * @brief Function determines the linear magnitudes of the x,y,z acceleration samples
 * @param[in] data_in Pointer to the data struct whose magnitude is being determined
 */
static void algo_get_linear_mag(accel_xyz_data_t data_in[], float impact_mag_data[])
{
	for(int i = 0; i < ACCEL_FIFO_LENGTH; i++)
	{
		impact_mag_data[i] = sqrt(pow(data_in[i].out_x, 2) + pow(data_in[i].out_y,2) + pow(data_in[i].out_z,2)); // Take the magnitude of a single XYZ sample
	}
}


/**
 * @brief Function determines the max HIC score within the provided samples using numerical integration
 * @param[in] data_in The accelerometer xyz data being parsed
 */
float ALGO_get_impact_score(accel_xyz_data_t data_in[], float impact_mag_data[], float *linear_acc)
{
	float curr_score, curr_linear_acc;
	float max_score = 0;
	float peak_linear_acc = 0;
	
	algo_get_linear_mag(data_in, impact_mag_data); // Populate the impact_mag_data array with the most current linear acceleration magnitudes
	
	for(int i = 0; i < ACCEL_FIFO_LENGTH; i++)
	{
		curr_linear_acc = impact_mag_data[i]; // Set the current linear_acc to the current element in the impact_mag array
		
		whim_queue_push(&algo_queue, &curr_linear_acc);
		whim_queue_integrate(&algo_queue, &curr_score, SAMPLE_DX);
		
		curr_score = pow((1/HIC_TIME_INTERVAL)*curr_score, 2.5) * HIC_TIME_INTERVAL;
		
		if(curr_score < IMPACT_SCORE_THRESH)
			impact_score_flag = 0;
		
		if(!impact_score_flag)
		{
		    if(curr_score > max_score)
		    {
			    max_score = curr_score;
		    }
		}
		
		if(curr_linear_acc > peak_linear_acc)
		{
			peak_linear_acc = curr_linear_acc;
		}
		
	}
	
	*linear_acc = peak_linear_acc;

	
	return max_score;
}

