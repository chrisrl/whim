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
															     MACROS
*******************************************************************************/
#define I_AM_LIS2DH     0x33
#define POWER_DOWN      0x00
#define DUMMY_COMMAND   0xFF


/*******************************************************************************
															   PROCEDURES
*******************************************************************************/
uint8_t ACCEL_read_who_am_i(void);
int16_t ACCEL_read_x(void);
int16_t ACCEL_read_y(void);
int16_t ACCEL_read_z(void);
void ACCEL_power_down(void);
void ACCEL_init(void);

static void accel_send_data_spi(uint8_t data[], size_t sz);
static uint8_t accel_read_data_spi(uint8_t data[], size_t sz);


#endif /*LIS2DH12_H*/
