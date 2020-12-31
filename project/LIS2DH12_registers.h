/******************************************************************************
File: LIS2DH12_registers.h

This header file contains the macro definitions for the registers of the LIS2DH12
accelerometer module

******************************************************************************/

#include "sdk_common.h"
#include "nrf_assert.h"

#ifndef LIS2DH12_REGISTERS_H //Guard statement
#define LIS2DH12_REGISTERS_H

/*******************************************************************************
															     MACROS
*******************************************************************************/

typedef struct {
	uint8_t reserved0[7];
	uint8_t STATUS_REG_AUX;
	uint8_t reserved1[4];
	uint8_t OUT_TEMP_L;
	uint8_t OUT_TEMP_H;
	uint8_t INT_COUNTER_REG;
	uint8_t WHO_AM_I;
	uint8_t reserved2[15];
	uint8_t TEMP_CFG_REG;
	uint8_t CTRL_REG1;
	uint8_t CTRL_REG2;
	uint8_t CTRL_REG3;
	uint8_t CTRL_REG4;
	uint8_t CTRL_REG5;
	uint8_t CTRL_REG6;
	uint8_t REFERENCE;
	uint8_t STATUS_REG2;
	uint8_t OUT_X_L;
	uint8_t OUT_X_H;
	uint8_t OUT_Y_L;
	uint8_t OUT_Y_H;
	uint8_t OUT_Z_L;
	uint8_t OUT_Z_H;
	uint8_t FIFO_CTRL_REG;
	uint8_t FIFO_SRC_REG;
	uint8_t INT1_CFG;
	uint8_t INT1_SOURCE;
	uint8_t INT1_THS;
	uint8_t INT1_DURATION;
	uint8_t INT2_CFG;
	uint8_t INT2_SOURCE;
	uint8_t INT2_THS;
	uint8_t INT2_DURATION;
	uint8_t CLICK_CFG;
	uint8_t CLICK_SRC;
	uint8_t CLICK_THS;
	uint8_t TIME_LIMIT;
	uint8_t TIME_LATENCY;
	uint8_t TIME_WINDOW;
	uint8_t Act_THS;
	uint8_t Act_DUR;
} LIS2DH12_REGISTER_MAP;

STATIC_ASSERT(sizeof(LIS2DH12_REGISTER_MAP) != 0x3F);

enum {
	STATUS_REG_AUX = offsetof(LIS2DH12_REGISTER_MAP, STATUS_REG_AUX),
	OUT_TEMP_L = offsetof(LIS2DH12_REGISTER_MAP, OUT_TEMP_L),
	OUT_TEMP_H = offsetof(LIS2DH12_REGISTER_MAP, OUT_TEMP_H),
	INT_COUNTER_REG = offsetof(LIS2DH12_REGISTER_MAP, INT_COUNTER_REG),
	WHO_AM_I = offsetof(LIS2DH12_REGISTER_MAP, WHO_AM_I),
	TEMP_CFG_REG = offsetof(LIS2DH12_REGISTER_MAP, TEMP_CFG_REG),
	CTRL_REG1 = offsetof(LIS2DH12_REGISTER_MAP, CTRL_REG1),
	CTRL_REG2 = offsetof(LIS2DH12_REGISTER_MAP, CTRL_REG2),
	CTRL_REG3 = offsetof(LIS2DH12_REGISTER_MAP, CTRL_REG3),
	CTRL_REG4 = offsetof(LIS2DH12_REGISTER_MAP, CTRL_REG4),
	CTRL_REG5 = offsetof(LIS2DH12_REGISTER_MAP, CTRL_REG5),
	CTRL_REG6 = offsetof(LIS2DH12_REGISTER_MAP, CTRL_REG6),
	REFERENCE = offsetof(LIS2DH12_REGISTER_MAP, REFERENCE),
	STATUS_REG2 = offsetof(LIS2DH12_REGISTER_MAP, STATUS_REG2),
	OUT_X_L = offsetof(LIS2DH12_REGISTER_MAP, OUT_X_L),
	OUT_X_H = offsetof(LIS2DH12_REGISTER_MAP, OUT_X_H),
	OUT_Y_L = offsetof(LIS2DH12_REGISTER_MAP, OUT_Y_L),
	OUT_Y_H = offsetof(LIS2DH12_REGISTER_MAP, OUT_Y_H),
	OUT_Z_L = offsetof(LIS2DH12_REGISTER_MAP, OUT_Z_L),
	OUT_Z_H = offsetof(LIS2DH12_REGISTER_MAP, OUT_Z_H),
	FIFO_CTRL_REG = offsetof(LIS2DH12_REGISTER_MAP, FIFO_CTRL_REG),
	FIFO_SRC_REG = offsetof(LIS2DH12_REGISTER_MAP, FIFO_SRC_REG),
	INT1_CFG = offsetof(LIS2DH12_REGISTER_MAP, INT1_CFG),
	INT1_SOURCE = offsetof(LIS2DH12_REGISTER_MAP, INT1_SOURCE),
	INT1_THS = offsetof(LIS2DH12_REGISTER_MAP, INT1_THS),
	INT1_DURATION = offsetof(LIS2DH12_REGISTER_MAP, INT1_DURATION),
	INT2_CFG = offsetof(LIS2DH12_REGISTER_MAP, INT2_CFG),
	INT2_SOURCE = offsetof(LIS2DH12_REGISTER_MAP, INT2_SOURCE),
	INT2_THS = offsetof(LIS2DH12_REGISTER_MAP, INT2_THS),
	INT2_DURATION = offsetof(LIS2DH12_REGISTER_MAP, INT2_DURATION),
	CLICK_CFG = offsetof(LIS2DH12_REGISTER_MAP, CLICK_CFG),
	CLICK_SRC = offsetof(LIS2DH12_REGISTER_MAP, CLICK_SRC),
	CLICK_THS = offsetof(LIS2DH12_REGISTER_MAP, CLICK_THS),
	TIME_LIMIT = offsetof(LIS2DH12_REGISTER_MAP, TIME_LIMIT),
	TIME_LATENCY = offsetof(LIS2DH12_REGISTER_MAP, TIME_LATENCY),
	TIME_WINDOW = offsetof(LIS2DH12_REGISTER_MAP, TIME_WINDOW),
	Act_THS = offsetof(LIS2DH12_REGISTER_MAP, Act_THS),
	Act_DUR = offsetof(LIS2DH12_REGISTER_MAP, Act_DUR),
};

#define CTRL_REG1_Xen		(uint8_t)(1 << 0)
#define CTRL_REG1_Yen		(uint8_t)(1 << 1)
#define CTRL_REG1_Zen		(uint8_t)(1 << 2)
#define CTRL_REG1_LPen	(uint8_t)(1 << 3)
#define CTRL_REG1_ODR0	(uint8_t)(1 << 4)
#define CTRL_REG1_ODR1	(uint8_t)(1 << 5)
#define CTRL_REG1_ODR2	(uint8_t)(1 << 6)
#define CTRL_REG1_ODR3	(uint8_t)(1 << 7)

#define CTRL_REG2_HPIS1 	(uint8_t)(1 << 0)
#define CTRL_REG2_HPIS2 	(uint8_t)(1 << 1)
#define CTRL_REG2_HPCLICK (uint8_t)(1 << 2)
#define CTRL_REG2_FDS 		(uint8_t)(1 << 3)
#define CTRL_REG2_HPCF1 	(uint8_t)(1 << 4)
#define CTRL_REG2_HPCF2 	(uint8_t)(1 << 5)
#define CTRL_REG2_HPM0 		(uint8_t)(1 << 6)
#define CTRL_REG2_HPM1 		(uint8_t)(1 << 7)

#define CTRL_REG3_I1_OVERRUN	(uint8_t)(1 << 1)
#define CTRL_REG3_I1_WTM			(uint8_t)(1 << 2)
#define CTRL_REG3_I1_DRDY2		(uint8_t)(1 << 3)
#define CTRL_REG3_I1_DRDY1		(uint8_t)(1 << 4)
#define CTRL_REG3_I1_AOI2			(uint8_t)(1 << 5)
#define CTRL_REG3_I1_AOI1			(uint8_t)(1 << 6)
#define CTRL_REG3_I1_CLICK		(uint8_t)(1 << 7)

#define CTRL_REG4_SIM	(uint8_t)(1 << 0)
#define CTRL_REG4_ST0	(uint8_t)(1 << 1)
#define CTRL_REG4_ST1	(uint8_t)(1 << 2)
#define CTRL_REG4_HR	(uint8_t)(1 << 3)
#define CTRL_REG4_FS0	(uint8_t)(1 << 4)
#define CTRL_REG4_FS1	(uint8_t)(1 << 5)
#define CTRL_REG4_BLE	(uint8_t)(1 << 6)
#define CTRL_REG4_BDU	(uint8_t)(1 << 7)

#define CTRL_REG5_D4D_INT2	(uint8_t)(1 << 0)
#define CTRL_REG5_LIR_INT2	(uint8_t)(1 << 1)
#define CTRL_REG5_D4D_INT1	(uint8_t)(1 << 2)
#define CTRL_REG5_LIR_INT1	(uint8_t)(1 << 3)
#define CTRL_REG5_FIFO_EN		(uint8_t)(1 << 6)
#define CTRL_REG5_BOOT			(uint8_t)(1 << 7)

#define CTRL_REG6_H_LACTIVE		(uint8_t)(1 << 1)
#define CTRL_REG6_P2_ACT			(uint8_t)(1 << 3)
#define CTRL_REG6_BOOT_I2			(uint8_t)(1 << 4)
#define CTRL_REG6_I2_INT2			(uint8_t)(1 << 5)
#define CTRL_REG6_I2_INT1			(uint8_t)(1 << 6)
#define CTRL_REG6_I2_CLICKen	(uint8_t)(1 << 7)

#define WRITE_BIT	(uint8_t)(0 << 7)
#define READ_BIT	(uint8_t)(1 << 7)
#define MS_BIT		(uint8_t)(1 << 6)


#endif /*LIS2DH12_REGISTERS_H*/
