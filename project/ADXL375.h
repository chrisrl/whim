/******************************************************************************
File: LIS2DH12.h

This header file contains the macros and function declarations for the initialization
and communication with the LIS2DH12 accelerometer module

******************************************************************************/

#ifndef ADXL375_H //Guard statement
#define ADXL375_H

/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/

#include "boards.h"
#include "ADXL375_registers.h"
#include "nrf_drv_gpiote.h"

/*******************************************************************************
																	  MACROS
*******************************************************************************/

#define SCALE_SELECT_MASK (CTRL_REG4_FS0 | CTRL_REG4_FS1)

// This value represents the measurement sensitivity of the accelerometer device's X, Y, and Z registers.
// The value is determined based on the resolution and operating mode that the device is set to in its initialization function.
// The unit of the variable is in g/LSB, where digit is the two's compliment value output by the X, Y, and Z registers.
// More information on the sensitivity of the accelerometer device can be found in Table 1. of the device datasheet.
// https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.pdf
#define SENSITIVITY_LE_800Hz ((float) 0.049)
#define SENSITIVITY_GE_1600Hz ((float) 0.098)

// This value represents the number of bits that the xyz data are stored in. I.e, when the X_OUT_L & X_OUT_H registers are concatenated, they are stored in a uint16_t variable.
// Therefore 16 bits is used to store a value read from any of the X, Y, and Z  registers
#define XYZ_FULL_DATA_SIZE (16) 

#define ACCEL_FIFO_LENGTH (32)
#define BYTES_PER_DATA_SAMPLE (6)

#define LED_PIN 23

#define IMPACT_LINEAR_ACC_THRESHOLD ((float) 10) // The threshold in g's that if surpassed indicates that an impact level event has occured
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
	adxl375_reg_map_t reg_map;
	resolution_mode_t resolution;
} adxl375_instance_t;
	
/*******************************************************************************
															   PROCEDURES
*******************************************************************************/

void ACCEL_read_xyz(accel_xyz_data_t* xyz_data);
void ACCEL_read_xyz_fifo(accel_xyz_data_t data[]);
//bool ACCEL_analyze_xyz(accel_xyz_data_t data[], float impact_data[]);
void ACCEL_set_fifo_mode_trigger(void);
void ACCEL_read_int_source_reg(void);
bool ACCEL_init(void);
#endif /*ADXL375_H*/
