/******************************************************************************
File: LIS2DH12_registers.h

This header file contains the macro definitions for the registers of the LIS2DH12
accelerometer module

******************************************************************************/

#include "sdk_common.h"
#include "nrf_assert.h"

#ifndef ADXL375_REGISTERS_H //Guard statement
#define ADXL375_REGISTERS_H

/*******************************************************************************
															     MACROS
*******************************************************************************/

// Any additions to this type def must be at the bottom in order to preserve the memory addresses of the registers defined in the enum below
typedef struct {
	uint8_t devid;
	uint8_t reserved0[28];
	uint8_t thresh_shock;
	uint8_t ofsx;
	uint8_t ofsy;
	uint8_t ofsz;
	uint8_t dur;
	uint8_t latent;
	uint8_t window;
	uint8_t thresh_act;
	uint8_t thresh_inact;
	uint8_t time_inact;
	uint8_t act_inact_ctl;
	uint8_t reserved1[2];
	uint8_t shock_axes;
	uint8_t act_shock_status;
	uint8_t bw_rate;
	uint8_t power_ctl;
	uint8_t int_enable;
	uint8_t int_map;
	uint8_t int_source;
	uint8_t data_format;
	uint8_t datax0;
	uint8_t datax1;
	uint8_t datay0;
	uint8_t datay1;
	uint8_t dataz0;
	uint8_t dataz1;
	uint8_t fifo_ctl;
	uint8_t fifo_status;
} adxl375_reg_map_t;

STATIC_ASSERT(sizeof(adxl375_reg_map_t) != 0x39);

enum {
	DEVID = offsetof(adxl375_reg_map_t, devid),
	THRESH_SHOCK = offsetof(adxl375_reg_map_t, thresh_shock),
	OFSX = offsetof(adxl375_reg_map_t, ofsx),
	OFSY = offsetof(adxl375_reg_map_t, ofsy),
	OFSZ = offsetof(adxl375_reg_map_t, ofsz),
	DUR = offsetof(adxl375_reg_map_t, dur),
	LATENT = offsetof(adxl375_reg_map_t, latent),
	WINDOW = offsetof(adxl375_reg_map_t, window),
	THRESH_ACT = offsetof(adxl375_reg_map_t, thresh_act),
	THRESH_INACT = offsetof(adxl375_reg_map_t, thresh_inact),
	TIME_INACT = offsetof(adxl375_reg_map_t, time_inact),
	ACT_INACT_CTL = offsetof(adxl375_reg_map_t, act_inact_ctl),
	SHOCK_AXES = offsetof(adxl375_reg_map_t, shock_axes),
	ACT_SHOCK_STATUS = offsetof(adxl375_reg_map_t, act_shock_status),
	BW_RATE = offsetof(adxl375_reg_map_t, bw_rate),
	POWER_CTL = offsetof(adxl375_reg_map_t, power_ctl),
	INT_ENABLE = offsetof(adxl375_reg_map_t, int_enable),
	INT_MAP = offsetof(adxl375_reg_map_t, int_map),
	INT_SOURCE = offsetof(adxl375_reg_map_t, int_source),
	DATA_FORMAT = offsetof(adxl375_reg_map_t, data_format),
	DATAX0 = offsetof(adxl375_reg_map_t, datax0),
	DATAX1 = offsetof(adxl375_reg_map_t, datax1),
	DATAY0 = offsetof(adxl375_reg_map_t, datay0),
	DATAY1 = offsetof(adxl375_reg_map_t, datay1),
	DATAZ0 = offsetof(adxl375_reg_map_t, dataz0),
	DATAZ1 = offsetof(adxl375_reg_map_t, dataz1),
	FIFO_CTL = offsetof(adxl375_reg_map_t, fifo_ctl),
	FIFO_STATUS = offsetof(adxl375_reg_map_t, fifo_status),
};

#define ADXL375_DEVICE_ID 0xE5 // ADXL375 DEVID register value.

#define CTRL_REG0_VALID_MASK (uint8_t)(0x10)
#define CTRL_REG0_SDO_PU_DISC (uint8_t)(1 << 7)

#define CTRL_REG1_X_EN		(uint8_t)(1 << 0)
#define CTRL_REG1_Y_EN		(uint8_t)(1 << 1)
#define CTRL_REG1_Z_EN  	(uint8_t)(1 << 2)
#define CTRL_REG1_LPEN	(uint8_t)(1 << 3)
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


#endif /*ADXL375_REGISTERS_H*/
