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


/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
extern const nrf_drv_spi_t spi; //SPI instance
extern volatile bool spi_xfer_done; //Flag used to indicate that SPI instance completed the transfer

extern uint8_t m_tx_buf[10]; //Tx buffer
extern uint8_t m_rx_buf[sizeof(m_tx_buf)+1]; //Rx buffer
extern const uint8_t m_length; //Transfer length

static uint8_t command[2];

/*******************************************************************************
															    PROCEDURES
*******************************************************************************/

/**
 * @brief Function sends a command to LIS2DH12
 * This function sends a one-byte command to the LIS2DH12 via SPI
 * @param[in] none
 */
static void accel_send_data_spi(uint8_t data[], size_t sz)
{
	memcpy(m_tx_buf,data,sz);

	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sz, m_rx_buf, m_length));
	
	while (!spi_xfer_done) //Check for successful transfer
	{
		__WFE();
	}
}


/**
 * @brief Function sends a read command to LIS2DH12
 * This function sends a one-byte read command to the LIS2DH12 via SPI
 * @param[in] none
 */
static uint8_t accel_read_data_spi(uint8_t data[], size_t sz)
{
	accel_send_data_spi(data,sz);
	return m_rx_buf[1];
}


/**
 * @brief Function sends a command to read the WHO_AM_I register
 * This function sends a one-byte read command to the LIS2DH12 WHO_AM_I register via SPI
 * @param[in] none
 */
uint8_t ACCEL_read_who_am_i(void)
{
	command[0] = READ_BIT|WHO_AM_I;
	command[1] = DUMMY_COMMAND;
	
	accel_read_data_spi(command,sizeof(command));

	return m_rx_buf[1]; //Confirm we have a connection, should be I_AM_LIS2DH
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
	
	command[0] = CTRL_REG1;	//Put device into power-down
	command[1] = POWER_DOWN;
	accel_send_data_spi(command,sizeof(command));
	nrf_delay_ms(20);
}


/**
 * @brief Function initializes the CTRL registers of the LS2DH12
 * This function writes the neccessary values to the desired CTRL registers to initialize the LIS2DH12
 * @param[in] none
 */
void ACCEL_init(void)
{
	command[0] = CTRL_REG1;
	command[1] = CTRL_REG1_Xen | CTRL_REG1_Yen |  CTRL_REG1_Zen | CTRL_REG1_ODR0 | CTRL_REG1_ODR1;
	accel_send_data_spi(command,sizeof(command));
	
	command[0] = CTRL_REG2;
	command[1] = 0;
	accel_send_data_spi(command,sizeof(command));
	
	command[0] = CTRL_REG3;
	command[1] = CTRL_REG3_I1_WTM;
	accel_send_data_spi(command,sizeof(command));
	
	command[0] = CTRL_REG4;
	command[1] = CTRL_REG4_BDU | CTRL_REG4_FS1 | CTRL_REG4_FS0;
	//accel_send_data_spi(command,sizeof(command));
	
	command[0] = CTRL_REG5;
	command[1] = CTRL_REG5_FIFO_EN | CTRL_REG5_LIR_INT1;
	accel_send_data_spi(command,sizeof(command));
}
