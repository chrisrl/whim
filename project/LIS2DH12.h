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
#include "LIS2DH12_registers.h"

/*******************************************************************************
																	  MACROS
*******************************************************************************/
//#define SPI_DEBUG_INFO
//#define ACCEL_DEBUG_INFO

#define get_name(var) #var

#define SCALE_SELECT_MASK ((uint8_t) 0x30)
#define SCALE_2G ((uint8_t) 2)
#define SCALE_4G ((uint8_t) 4)
#define SCALE_8G ((uint8_t) 8)
#define SCALE_16G ((uint8_t) 16)

#define OP_MODE_MASK 0x08
#define HIGH_RES_BITS 12
#define NORMAL_RES_BITS 10
#define LOW_RES_BITS 8

#define XYZ_REG_SIZE 16
#define MAGNITUDE_MASK 0x0FFF

/*******************************************************************************
															     TYPEDEFS
*******************************************************************************/

typedef struct {
	int16_t out_x;
	int16_t out_y;
	int16_t out_z;
} accel_xyz_raw_data_t;

typedef struct {
	float out_x;
	float out_y;
	float out_z;
} accel_xyz_impact_data_t;

enum {
	MAX_2G = 0x00,
	MAX_4G = 0x10,
	MAX_8G = 0x20,
	MAX_16G = 0x30,
};

enum {
	LOW_POWER = 0x10,
	NORMAL = 0x00,
	HIGH_RES = 0x08,
};
	
/*******************************************************************************
															   PROCEDURES
*******************************************************************************/

void ACCEL_read_xyz(accel_xyz_raw_data_t* xyz_data, lis2dh12_instance_t* accel_inst);
bool ACCEL_init(lis2dh12_instance_t* accel_inst);
void ACCEL_display_instance(lis2dh12_instance_t * accel_inst);
void ACCEL_get_xyz_impact(accel_xyz_raw_data_t* xyz_data_in, accel_xyz_impact_data_t* xyz_data_out, lis2dh12_instance_t* accel_inst);

#endif /*LIS2DH12_H*/
