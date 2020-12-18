/******************************************************************************
File: LIS2DH12_registers.h

This header file contains the macro definitions for the registers of the LIS2DH12
accelerometer module

******************************************************************************/

#ifndef LIS2DH12_REGISTERS_H //Guard statement
#define LIS2DH12_REGISTERS_H

/*******************************************************************************
															     MACROS
*******************************************************************************/
#define WHO_AM_I        0x0F
#define CTRL_REG_0      0x1E
#define CTRL_REG_1      0x20
#define CTRL_REG_2      0x21
#define CTRL_REG_3      0x22
#define CTRL_REG_4      0x23
#define CTRL_REG_5      0x24
#define CTRL_REG_6      0x25
#define REFERENCE       0x26
#define STATUS_REG      0x27
#define OUT_X_L         0x28
#define OUT_X_H         0x29
#define OUT_Y_L         0x2A
#define OUT_Y_H         0x2B
#define OUT_Z_L         0x2C
#define OUT_Z_H         0x2D

#endif /*LIS2DH12_REGISTERS_H*/
