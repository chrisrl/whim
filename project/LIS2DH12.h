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
#define READ_BIT        0x80
#define DUMMY_COMMAND   0xFF

#define CTRL_REG_1_INIT 0x37
#define CTRL_REG_2_INIT 0x00
#define CTRL_REG_3_INIT 0x04
#define CTRL_REG_4_INIT 0xB8
#define CTRL_REG_5_INIT 0x48



/*******************************************************************************
															   PROCEDURES
*******************************************************************************/
uint8_t ACCEL_read_who_am_i(void);
void ACCEL_power_down(void);
void ACCEL_init(void);

static void accel_send_data_spi(uint8_t data[], size_t sz);
static uint8_t accel_read_data_spi(uint8_t data[], size_t sz);


#endif /*LIS2DH12_H*/
