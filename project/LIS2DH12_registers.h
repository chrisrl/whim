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

// Any additions to this type def must be at the bottom in order to preserve the memory addresses of the registers defined in the enum below
typedef struct {
	uint8_t reserved0[7];
	uint8_t status_reg_aux;
	uint8_t reserved1[4];
	uint8_t out_temp_l;
	uint8_t out_temp_h;
	uint8_t reserved2;
	uint8_t who_am_i;
	uint8_t reserved3[14];
	uint8_t ctrl0;
	uint8_t temp_cfg;
	uint8_t ctrl1;
	uint8_t ctrl2;
	uint8_t ctrl3;
	uint8_t ctrl4;
	uint8_t ctrl5;
	uint8_t ctrl6;
	uint8_t reference;
	uint8_t status_reg2;
	uint8_t out_x_l;
	uint8_t out_x_h;
	uint8_t out_y_l;
	uint8_t out_y_h;
	uint8_t out_z_l;
	uint8_t out_z_h;
	uint8_t fifo_ctrl;
	uint8_t fifo_src;
	uint8_t int1_cfg;
	uint8_t int1_src;
	uint8_t int1_ths;
	uint8_t int1_dur;
	uint8_t int2_cfg;
	uint8_t int2_src;
	uint8_t int2_ths;
	uint8_t int2_dur;
	uint8_t click_cfg;
	uint8_t click_src;
	uint8_t click_ths;
	uint8_t time_limit;
	uint8_t time_latency;
	uint8_t time_window;
	uint8_t act_ths;
	uint8_t act_dur;
} lis2dh12_reg_map_t;

STATIC_ASSERT(sizeof(lis2dh12_reg_map_t) != 0x3F);

enum {
	STATUS_REG_AUX = offsetof(lis2dh12_reg_map_t, status_reg_aux),
	OUT_TEMP_L = offsetof(lis2dh12_reg_map_t, out_temp_l),
	OUT_TEMP_H = offsetof(lis2dh12_reg_map_t, out_temp_h),
	WHO_AM_I = offsetof(lis2dh12_reg_map_t, who_am_i),
	CTRL_REG0 = offsetof(lis2dh12_reg_map_t, ctrl0),
	TEMP_CFG_REG = offsetof(lis2dh12_reg_map_t, temp_cfg),
	CTRL_REG1 = offsetof(lis2dh12_reg_map_t, ctrl1),
	CTRL_REG2 = offsetof(lis2dh12_reg_map_t, ctrl2),
	CTRL_REG3 = offsetof(lis2dh12_reg_map_t, ctrl3),
	CTRL_REG4 = offsetof(lis2dh12_reg_map_t, ctrl4),
	CTRL_REG5 = offsetof(lis2dh12_reg_map_t, ctrl5),
	CTRL_REG6 = offsetof(lis2dh12_reg_map_t, ctrl6),
	REFERENCE = offsetof(lis2dh12_reg_map_t, reference),
	STATUS_REG2 = offsetof(lis2dh12_reg_map_t, status_reg2),
	OUT_X_L = offsetof(lis2dh12_reg_map_t, out_x_l),
	OUT_X_H = offsetof(lis2dh12_reg_map_t, out_x_h),
	OUT_Y_L = offsetof(lis2dh12_reg_map_t, out_y_l),
	OUT_Y_H = offsetof(lis2dh12_reg_map_t, out_y_h),
	OUT_Z_L = offsetof(lis2dh12_reg_map_t, out_z_l),
	OUT_Z_H = offsetof(lis2dh12_reg_map_t, out_z_h),
	FIFO_CTRL_REG = offsetof(lis2dh12_reg_map_t, fifo_ctrl),
	FIFO_SRC_REG = offsetof(lis2dh12_reg_map_t, fifo_src),
	INT1_CFG = offsetof(lis2dh12_reg_map_t, int1_cfg),
	INT1_SOURCE = offsetof(lis2dh12_reg_map_t, int1_src),
	INT1_THS = offsetof(lis2dh12_reg_map_t, int1_ths),
	INT1_DURATION = offsetof(lis2dh12_reg_map_t, int1_dur),
	INT2_CFG = offsetof(lis2dh12_reg_map_t, int2_cfg),
	INT2_SOURCE = offsetof(lis2dh12_reg_map_t, int2_src),
	INT2_THS = offsetof(lis2dh12_reg_map_t, int2_ths),
	INT2_DURATION = offsetof(lis2dh12_reg_map_t, int2_dur),
	CLICK_CFG = offsetof(lis2dh12_reg_map_t, click_cfg),
	CLICK_SRC = offsetof(lis2dh12_reg_map_t, click_src),
	CLICK_THS = offsetof(lis2dh12_reg_map_t, click_ths),
	TIME_LIMIT = offsetof(lis2dh12_reg_map_t, time_limit),
	TIME_LATENCY = offsetof(lis2dh12_reg_map_t, time_latency),
	TIME_WINDOW = offsetof(lis2dh12_reg_map_t, time_window),
	ACT_THS = offsetof(lis2dh12_reg_map_t, act_ths),
	ACT_DUR = offsetof(lis2dh12_reg_map_t, act_dur),
};

#define CTRL_REG0_VALID_MASK (uint8_t)(0x10)
#define CTRL_REG0_SDO_PU_DISC (uint8_t)(1 << 7)

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

#define FIFO_CTRL_FTH0 ((uint8_t) (1 << 0))
#define FIFO_CTRL_FTH1 ((uint8_t) (1 << 1))
#define FIFO_CTRL_FTH2 ((uint8_t) (1 << 2))
#define FIFO_CTRL_FTH3 ((uint8_t) (1 << 3))
#define FIFO_CTRL_FTH4 ((uint8_t) (1 << 4))
#define FIFO_CTRL_FM0  ((uint8_t) (1 << 6))
#define FIFO_CTRL_FM1  ((uint8_t) (1 << 7))

#define FIFO_SRC_EMPTY ((uint8_t) (1 << 5))

#define INT1_PIN ((nrf_drv_gpiote_pin_t) 25)

#define WRITE_BIT	(uint8_t)(0 << 7)
#define READ_BIT	(uint8_t)(1 << 7)
#define MS_BIT		(uint8_t)(1 << 6)


#endif /*LIS2DH12_REGISTERS_H*/
