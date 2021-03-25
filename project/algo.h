/******************************************************************************
File: algo.h

This header file contains the macros and function declarations for the algorithm
implementation

******************************************************************************/

#ifndef ALGO_H //Guard statement
#define ALGO_H

/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/
#include "ADXL375.h"

/*******************************************************************************
																	  MACROS
*******************************************************************************/
#define QUEUE_CAPACITY (16) // Buffer size for the FIFO samples used 
#define SAMPLE_DX ((float) 0.00125) // Time delta consecutive samples in the queue
#define IMPACT_SCORE_THRESH_FLOAT ((float) 150)
#define IMPACT_SCORE_THRESH_INT ((uint16_t) 150)
#define HIC_TIME_INTERVAL (0.02) // Approx. 20ms time interval for HIC score
	
/*******************************************************************************
															   PROCEDURES
*******************************************************************************/

float ALGO_get_impact_score(accel_xyz_data_t data_in[], float impact_mag_data[], float *linear_acc);
#endif /*ALGO_H*/
