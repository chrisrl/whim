/******************************************************************************
File: LIS2DH12.c

This source file contains the functions and variables responsible for initializing
and communicating with the LIS2DH12 acclerometer module

******************************************************************************/


/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include <string.h>
#include "LIS2DH12.h"
#include "LIS2DH12_registers.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
extern const nrf_drv_spi_t spi; //SPI instance
extern volatile bool spi_xfer_done; //Flag used to indicate that SPI instance completed the transfer

extern uint8_t m_tx_buf[10]; //Tx buffer
extern uint8_t m_rx_buf[sizeof(m_tx_buf)+1]; //Rx buffer
extern const uint8_t m_length; //Transfer length

typedef struct {
	uint8_t address;
	uint8_t value;
} RegisterCommandStruct;

static RegisterCommandStruct command = {0};

/*******************************************************************************
															    PROCEDURES
*******************************************************************************/

/**
 * @brief Function sends a command to LIS2DH12
 * This function sends a one-byte command to the LIS2DH12 via SPI
 * @param[in] RegisterCommandStruct* cmd
 */
static void accel_write_register(RegisterCommandStruct* cmd)
{
	memcpy(m_tx_buf, cmd, sizeof(RegisterCommandStruct));

	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sizeof(RegisterCommandStruct), m_rx_buf, m_length));
	
	while (!spi_xfer_done) //Check for successful transfer
	{
		__WFE();
	}
}


/**
 * @brief Function sends a read command to LIS2DH12
 * This function sends a one-byte read command to the LIS2DH12 via SPI
 * @param[in] RegisterCommandStruct* cmd
 */
static uint8_t accel_read_register(RegisterCommandStruct* cmd)
{
	cmd->address |= READ_BIT; 
	accel_write_register(cmd);
	return m_rx_buf[1];
}


/**
 * @brief Function sends a command to read the WHO_AM_I register
 * This function sends a one-byte read command to the LIS2DH12 WHO_AM_I register via SPI
 * @param[in] none
 */
uint8_t ACCEL_read_who_am_i(void)
{
	command.address = WHO_AM_I;
	command.value = DUMMY_COMMAND;
	
	accel_read_register(&command);

	return m_rx_buf[1]; //Confirm we have a connection, should be I_AM_LIS2DH
}

/**
 * @brief Function gets X-axis data
 * This function reads the OUT_X_L and OUT_X_H registers to get the current X-axis g-force reading
 * @param[in] none
 */
int16_t ACCEL_read_x(void)
{
	int16_t out_x;
	uint8_t out_x_l, out_x_h; 
	command.value = DUMMY_COMMAND;
	
	command.address = OUT_X_L;
	out_x_l = accel_read_register(&command);
	
	command.address = OUT_X_H;
	out_x_h = accel_read_register(&command);
	
	out_x = (((int16_t)out_x_h << 8) | ((int16_t)out_x_l << 0)) >> 8;
	return out_x;
}

/**
 * @brief Function gets Y-axis data
 * This function reads the OUT_Y_L and OUT_Y_H registers to get the current Y-axis g-force reading
 * @param[in] none
 */
int16_t ACCEL_read_y(void)
{
	int16_t out_y;
	uint8_t out_y_l, out_y_h; 
	command.value = DUMMY_COMMAND;
	
	command.address = OUT_Y_L;
	out_y_l = accel_read_register(&command);
	
	command.address = OUT_Y_H;
	out_y_h = accel_read_register(&command);
	
	out_y = (((int16_t)out_y_h << 8) | ((int16_t)out_y_l << 0)) >> 8;
	return out_y;
}

/**
 * @brief Function gets Z-axis data
 * This function reads the OUT_Z_L and OUT_Z_H registers to get the current Z-axis g-force reading
 * @param[in] none
 */
int16_t ACCEL_read_z(void)
{
	int16_t out_z;
	uint8_t out_z_l, out_z_h; 
	command.value = DUMMY_COMMAND;
	
	command.address = OUT_Z_L;
	out_z_l = accel_read_register(&command);
	
	command.address = OUT_Z_H;
	out_z_h = accel_read_register(&command);
	
	out_z = (((int16_t)out_z_h << 8) | ((int16_t)out_z_l << 0)) >> 8;
	return out_z;
}

/**
 * @brief Function sends a power down command to LIS2DH12
 * This function sends the command to power down the LIS2DH12. It delays for 20ms to ensure 
 * a successful operation.
 * @param[in] none
 */
void ACCEL_power_down(void)
{
	// If this function is called after power on, then the LIS2DH12
	// will still be in boot mode, allow time for boot to complete. 
	nrf_delay_ms(20);
	
	command.address = CTRL_REG1;	//Put device into power-down
	command.value = POWER_DOWN;
	accel_write_register(&command);
	nrf_delay_ms(20);
}


/**
 * @brief Function initializes the CTRL registers of the LS2DH12
 * This function writes the neccessary values to the desired CTRL registers to initialize the LIS2DH12
 * @param[in] none
 */
void ACCEL_init(void)
{	
	nrf_delay_ms(150);
	
	uint8_t register_value = 0;
	
	// Set valid mask in CTRL_REG0
	command.address = CTRL_REG0;
	command.value = CTRL_REG0_VALID_MASK;
	NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
	NRF_LOG_FLUSH();
	accel_write_register(&command);
	nrf_delay_ms(100);
	
	
	// Enable all axes and set ODR
	command.address = CTRL_REG1;
	command.value = CTRL_REG1_Xen | CTRL_REG1_Yen | CTRL_REG1_Zen;// | CTRL_REG1_ODR0 | CTRL_REG1_ODR1;
	NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
	NRF_LOG_FLUSH();
	accel_write_register(&command);
	nrf_delay_ms(100);
	
	// Clear HR in CTRL_REG4
//	command.address = CTRL_REG4;
//	command.value = DUMMY_COMMAND;
//	register_value = accel_read_register(&command);
//	// Only write to the register if HR bit is high
//	if((register_value & CTRL_REG4_HR) == CTRL_REG4_HR)
//	{
//		NRF_LOG_INFO("HR bit was found to be enabled.");
		command.address = CTRL_REG4;
		command.value = CTRL_REG4_BDU;
		NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
		NRF_LOG_FLUSH();
		//accel_write_register(&command);
		nrf_delay_ms(100);
//	}
//	else
//	{
//		NRF_LOG_INFO("HR bit was found to be disabled.");
//		NRF_LOG_FLUSH();
//	}
	
//	// Clear LPen
//	command.address = CTRL_REG1;
//	command.value = CTRL_REG1_LPen;
//	NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
//	NRF_LOG_FLUSH();
//	accel_write_register(&command);
//	
//	nrf_delay_ms(150);
	
//	// Enable all axes and set ODR
//	command.address = CTRL_REG1;
//	command.value = CTRL_REG1_Xen | CTRL_REG1_Yen | CTRL_REG1_Zen;// | CTRL_REG1_ODR0 | CTRL_REG1_ODR1;
//	NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
//	NRF_LOG_FLUSH();
//	accel_write_register(&command);
//	nrf_delay_ms(100);
	
	// Enable watermark interrupt for INT1
//	command.address = CTRL_REG3;
//	command.value = DUMMY_COMMAND;
//	register_value = accel_read_register(&command);
//	
//	command.address = CTRL_REG3;
//	command.value = register_value | CTRL_REG3_I1_WTM;
//	NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
//	NRF_LOG_FLUSH();
//	accel_write_register(&command);
//	nrf_delay_ms(100);
//	
//	// Enable +/-16g sampling
//	command.address = CTRL_REG4;
//	command.value = DUMMY_COMMAND;
//	register_value = accel_read_register(&command);
//	
//	command.address = CTRL_REG4;
//	command.value = register_value | CTRL_REG4_FS0 | CTRL_REG4_FS1;
//	NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
//	NRF_LOG_FLUSH();
////	accel_write_register(&command);
//	
//	// Enable the FIFO
//	command.address = CTRL_REG5;
//	command.value = DUMMY_COMMAND;
//	register_value = accel_read_register(&command);
//	
//	command.address = CTRL_REG5;
//	command.value = register_value | CTRL_REG5_FIFO_EN;
//	NRF_LOG_INFO("Writing 0x%02X to 0x%02X...", command.value, command.address);
//	NRF_LOG_FLUSH();
//	accel_write_register(&command);
//	nrf_delay_ms(100);
//	
//	// Read REFERENCE
//	command.address = REFERENCE;
//	command.value = DUMMY_COMMAND;
//	NRF_LOG_INFO("Reading 0x%02X...", command.address);
//	NRF_LOG_FLUSH();		
	
	nrf_delay_ms(1000);
	command.value = DUMMY_COMMAND;
	command.address = CTRL_REG0;
	NRF_LOG_INFO("CTRL_REG1: 0x%02X", accel_read_register(&command));
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	command.address = CTRL_REG1;
	NRF_LOG_INFO("CTRL_REG1: 0x%02X", accel_read_register(&command));
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	command.address = CTRL_REG2;
	NRF_LOG_INFO("CTRL_REG2: 0x%02X", accel_read_register(&command));
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	command.address = CTRL_REG3;
	NRF_LOG_INFO("CTRL_REG3: 0x%02X", accel_read_register(&command));
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	command.address = CTRL_REG4;
	NRF_LOG_INFO("CTRL_REG4: 0x%02X", accel_read_register(&command));
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	command.address = CTRL_REG5;
	NRF_LOG_INFO("CTRL_REG5: 0x%02X", accel_read_register(&command));
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	command.address = WHO_AM_I;
	NRF_LOG_INFO("WHO_AM_I: 0x%02X", accel_read_register(&command));
	NRF_LOG_FLUSH();
	nrf_delay_ms(1000);
}
