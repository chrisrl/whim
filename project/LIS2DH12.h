/******************************************************************************
File: LIS2DH12.h

This header file contains the macros and function declarations for the initialization
and communication with the LIS2DH12 accelerometer module

******************************************************************************/

#ifndef LIS2DH12_H //Guard statement
#define LIS2DH12_H

/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/

#include "boards.h"


/*******************************************************************************
															     TYPEDEFS
*******************************************************************************/

typedef struct {
	int16_t out_x;
	int16_t out_y;
	int16_t out_z;
} AccelXYZDataStruct;


/*******************************************************************************
															   PROCEDURES
*******************************************************************************/

void ACCEL_read_xyz(AccelXYZDataStruct* xyz_data);
bool ACCEL_init(void);


#endif /*LIS2DH12_H*/
