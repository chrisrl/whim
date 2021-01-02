/******************************************************************************
File: LIS2DH12.c

This source file contains the functions and variables responsible for initializing
and communicating with the LIS2DH12 acclerometer module

******************************************************************************/


/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/
#include <string.h>
#include "boards.h"
#include "sdk_common.h"

#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "LIS2DH12.h"
#include "LIS2DH12_registers.h"


/*******************************************************************************
															TYPE DEFINITIONS
*******************************************************************************/

typedef struct {
	uint8_t OUT_X_L;
	uint8_t OUT_X_H;
	uint8_t OUT_Y_L;
	uint8_t OUT_Y_H;
	uint8_t OUT_Z_L;
	uint8_t OUT_Z_H;
} AccelXYZOutDataStruct;

typedef struct {
	uint8_t CTRL_REG0;
	uint8_t TEMP_CFG_REG;
	uint8_t CTRL_REG1;
	uint8_t CTRL_REG2;
	uint8_t CTRL_REG3;
	uint8_t CTRL_REG4;
	uint8_t CTRL_REG5;
	uint8_t CTRL_REG6;
} ControlBlockStruct;

typedef struct {
	uint8_t address;
	uint8_t value;
} RegisterCommandStruct;

typedef struct {
	uint8_t start_address;
	uint8_t *buffer;
	uint8_t buffer_length;
} BlockCommandStruct;


/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/

/*NOTE: Uncomment these lines for more debug info*/
//#define SPI_DEBUG_INFO
//#define ACCEL_DEBUG_INFO

#define SPI_INSTANCE  0 //SPI instance index
const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); //SPI instance
volatile bool spi_xfer_done; //Flag used to indicate that SPI instance completed the transfer

#define SPI_BUFFER_LENGTH (128)
uint8_t m_tx_buf[SPI_BUFFER_LENGTH]; //Tx buffer
uint8_t m_rx_buf[SPI_BUFFER_LENGTH]; //Rx buffer

static RegisterCommandStruct reg_command = {0};
static BlockCommandStruct block_command = {0};

/*******************************************************************************
															    PROCEDURES
*******************************************************************************/

/**
 * @brief Function sends a single byte write command to the accelerometer
 * This function sends a one-byte write register command to the accelerometer via SPI
 * @param[in] RegisterCommandStruct* cmd
 */
static void accel_write_register(RegisterCommandStruct* cmd)
{
	memcpy(m_tx_buf, cmd, sizeof(RegisterCommandStruct));

	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sizeof(RegisterCommandStruct), m_rx_buf, sizeof(RegisterCommandStruct)));
	
	while (!spi_xfer_done) //Check for successful transfer
	{
		__WFE();
	}
}

/**
 * @brief Function sends a single byte read command to the accelerometer
 * This function sends a one-byte read register command to the accelerometer via SPI
 * @param[in] cmd: RegisterCommandStructpointer containing the desired register address
 * @param[out] Returns the value contained in the specified register
 */
static uint8_t accel_read_register(RegisterCommandStruct* cmd)
{
	cmd->address |= READ_BIT; 
	cmd->value = 0xFF; // Insert dummy value to read during that SPI frame
	
	accel_write_register(cmd);
	
	return m_rx_buf[1];
}

/**
 * @brief Function writes a block to a range of addresses
 * Send a block of data to the accelerometer over SPI with the MS bit set
 * @param[in] cmd: BlockCommandStruct pointer containing the start address, 
 * data buffer and buffer length for the block transmission
 */
static void accel_write_block(BlockCommandStruct* cmd)
{
	if(cmd->buffer_length >= SPI_BUFFER_LENGTH)
	{
		NRF_LOG_INFO("ERROR: Attempting to send too much data (%d) in accel_write_block()! (MAX %d)", cmd->buffer_length, SPI_BUFFER_LENGTH);
		return;
	}
	spi_xfer_done = false;
	
	m_tx_buf[0] = cmd->start_address | MS_BIT;
	memcpy(&m_tx_buf[1], cmd->buffer, cmd->buffer_length);
	
	#ifdef SPI_DEBUG_INFO
	NRF_LOG_INFO("Writing block to 0x%02X - 0x%02X:", cmd->start_address, cmd->start_address + cmd->buffer_length-1)
	NRF_LOG_HEXDUMP_INFO(cmd->buffer, cmd->buffer_length);
	#endif
	
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, cmd->buffer_length+1, m_rx_buf, cmd->buffer_length+1));
	
	while (!spi_xfer_done) //Check for successful transfer
	{
		__WFE();
	}
}

/**
 * @brief Function reads a block to a range of addresses
 * Read a block of data to the accelerometer over SPI with the MS bit set
 * @param[in] cmd: BlockCommandStruct pointer containing the start address, 
 * receive data buffer and buffer length for the block transmission
 */
static void accel_read_block(BlockCommandStruct* cmd)
{
	if(cmd->buffer_length >= SPI_BUFFER_LENGTH)
	{
		NRF_LOG_INFO("ERROR: Attempting to send too much data (%d) in accel_write_block()! (MAX %d)", cmd->buffer_length, SPI_BUFFER_LENGTH);
		return;
	}
	spi_xfer_done = false;
	
	m_tx_buf[0] = cmd->start_address | READ_BIT | MS_BIT;
	
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, cmd->buffer_length+1, m_rx_buf, cmd->buffer_length+1));
	
	while (!spi_xfer_done) //Check for successful transfer
	{
		__WFE();
	}
	
	memcpy(cmd->buffer, &m_rx_buf[1], cmd->buffer_length);
	
	#ifdef SPI_DEBUG_INFO
	NRF_LOG_INFO("Read block from 0x%02X - 0x%02X:", cmd->start_address, cmd->start_address + cmd->buffer_length-1)
	NRF_LOG_HEXDUMP_INFO(cmd->buffer, cmd->buffer_length);
	#endif
	
}

/**
 * @brief Functions reads X Y Z data from the accelerometer
 * Get the X Y Z OUT register values scaled to int16_t data types
 * @param[in] data: AccelXYZDataStruct pointer to receive the x y and z values to
 */
void ACCEL_read_xyz(AccelXYZDataStruct* data)
{
	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Reading accel XYZ data registers...");
	#endif
	
	AccelXYZOutDataStruct xyz_out_registers = {0};
	
	block_command.start_address = OUT_X_L;
	block_command.buffer = (uint8_t*)&xyz_out_registers;
	block_command.buffer_length = sizeof(xyz_out_registers);
	
	accel_read_block(&block_command);
	
	data->out_x = (((int16_t)xyz_out_registers.OUT_X_H << 8) | ((int16_t)xyz_out_registers.OUT_X_L << 0)) >> 6;
	data->out_y = (((int16_t)xyz_out_registers.OUT_Y_H << 8) | ((int16_t)xyz_out_registers.OUT_Y_L << 0)) >> 6;
	data->out_z = (((int16_t)xyz_out_registers.OUT_Z_H << 8) | ((int16_t)xyz_out_registers.OUT_Z_L << 0)) >> 6;
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
	spi_xfer_done = true;
}

/**
 * @brief Intialize the SPI perihperal
 * Set the MOSI, MISO, SS and SCK pins and SPI Driver Mode to 3
 */
static void spi_init(void)
{
	#ifdef SPI_DEBUG_INFO
	NRF_LOG_INFO("SPI Initializing...");
	#endif
	
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.sck_pin      = SPI_SCK_PIN;
  spi_config.mosi_pin     = SPI_MOSI_PIN;
  spi_config.miso_pin     = SPI_MISO_PIN;
  spi_config.ss_pin       = SPI_SS_PIN;
  spi_config.mode         = NRF_DRV_SPI_MODE_3;
	
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
	#ifdef SPI_DEBUG_INFO
	NRF_LOG_INFO("SPI Initialized.");
	#endif
}

/**
 * @brief Function initializes the accelerometer
 * This function writes the neccessary values to the desired CTRL registers to initialize the LIS2DH12
 * @param[in] none
 */
bool ACCEL_init(void)
{
	// Initialize the SPI peripheral
	spi_init();
	nrf_delay_ms(100);
	
	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Accelerometer Initializing...");
	#endif
	
	ControlBlockStruct config_block_w = {
		CTRL_REG0_VALID_MASK,
		0x00,
		CTRL_REG1_Xen | CTRL_REG1_Yen | CTRL_REG1_Zen | CTRL_REG1_ODR0 | CTRL_REG1_ODR1,
		0x00,
		0x00,
		CTRL_REG4_BDU | CTRL_REG4_FS0 | CTRL_REG4_FS1,
		0x00
	};
	block_command.start_address = CTRL_REG0;
	block_command.buffer = (uint8_t*)&config_block_w;
	block_command.buffer_length = sizeof(config_block_w);
	
	accel_write_block(&block_command);
	
	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Accelerometer Initializing...");
	#endif
	
	ControlBlockStruct config_block_r = {0};
	block_command.start_address = CTRL_REG0;
	block_command.buffer = (uint8_t*)&config_block_r;
	block_command.buffer_length = sizeof(config_block_r);
	accel_read_block(&block_command);
	
	if(memcmp(&config_block_r, &config_block_w, sizeof(ControlBlockStruct)) != 0)
	{
		#ifdef ACCEL_DEBUG_INFO
		NRF_LOG_INFO("Accelerometer Initialization failed!");
		#endif
		
		return false;
	}
	
	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Accelerometer Initializated.");
	#endif
	
	return true;
}
