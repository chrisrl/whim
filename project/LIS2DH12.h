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

/*NOTE: Uncomment these lines for more debug info*/
//#define SPI_DEBUG_INFO
//#define ACCEL_DEBUG_INFO

#define SCALE_SELECT_MASK (CTRL_REG4_FS0 | CTRL_REG4_FS1)

// This value represents the measurement sensitivity of the accelerometer device's X, Y, and Z registers.
// The value is determined based on the resolution and operating mode that the device is set to in its initialization function.
// The unit of the variable is in g/digit, where digit is the two's compliment value output by the X, Y, and Z registers.
// More information on the sensitivity of the accelerometer device can be found in Table 4. of the device datasheet.
// https://www.st.com/content/ccc/resource/technical/document/datasheet/12/c0/5c/36/b9/58/46/f2/DM00091513.pdf/files/DM00091513.pdf/jcr:content/translations/en.DM00091513.pdf
#define SENSITIVITY ((float) 0.048)

// This value represents the number of bits that the xyz data are stored in. I.e, when the X_OUT_L & X_OUT_H registers are concatenated, they are stored in a uint16_t variable.
// Therefore 16 bits is used to store a value read from any of the X, Y, and Z  registers
#define XYZ_FULL_DATA_SIZE (16) 

#define ACCEL_FIFO_LENGTH (32)
/*******************************************************************************
															     TYPEDEFS
*******************************************************************************/
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

typedef enum {
	HIGH_RES_BITS = 12,
	NORMAL_RES_BITS = 10,
	LOW_RES_BITS = 8,
} resolution_mode_t;

enum {
	LOW_POWER = 0x10,
	NORMAL = 0x00,
	HIGH_RES = 0x08,
};

typedef struct {
	lis2dh12_reg_map_t reg_map;
	resolution_mode_t resolution;
} lis2dh12_instance_t;
	
/*******************************************************************************
															   PROCEDURES
*******************************************************************************/

void ACCEL_read_xyz(accel_xyz_data_t* xyz_data);
void ACCEL_read_xyz_fifo(accel_xyz_data_t data[]);
bool ACCEL_init(void);
bool ACCEL_fifo_init(void);
void ACCEL_pwrdn(void);

#endif /*LIS2DH12_H*/
