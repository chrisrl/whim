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
#include "nrf_drv_gpiote.h"

/*******************************************************************************
																	  MACROS
*******************************************************************************/
//#define SPI_DEBUG_INFO
//#define ACCEL_DEBUG_INFO

#define SCALE_SELECT_MASK (CTRL_REG4_FS0 | CTRL_REG4_FS1)

#define OP_MODE_MASK 0x08
#define SENSITIVITY ((float) 0.048) //We can change this to make it dyanmically assigned based off of the FS bit settings/op mode

#define XYZ_REG_SIZE sizeof(uint16_t)
#define MAGNITUDE_MASK 0x0FFF

#define LIS2DH12_DEVICE_ID 33

/*******************************************************************************
															     TYPEDEFS
*******************************************************************************/
typedef struct {
	lis2dh12_reg_map_t reg_map;
	uint8_t full_scale_value;
	uint8_t resolution;
} lis2dh12_instance_t;

typedef struct {
	float out_x;
	float out_y;
	float out_z;
} accel_xyz_data_t;

enum {
	SCALE_2G = 2,
	SCALE_4G = 4,
	SCALE_8G = 8,
	SCALE_16G = 16,
};

enum {
	HIGH_RES_BITS = 12,
	NORMAL_RES_BITS = 10,
	LOW_RES_BITS = 8
};

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

void ACCEL_read_xyz(accel_xyz_data_t* xyz_data, lis2dh12_instance_t* accel_inst);
void ACCEL_disable_fifo(void);
bool ACCEL_enable_fifo(void);
bool ACCEL_init(lis2dh12_instance_t* accel_inst);
void ACCEL_pwrdn(void);

#endif /*LIS2DH12_H*/
