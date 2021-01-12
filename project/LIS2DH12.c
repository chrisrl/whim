/******************************************************************************
File: LIS2DH12.c

This source file contains the functions and variables responsible for initializing
and communicating with the LIS2DH12 acclerometer module

******************************************************************************/


/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/
#include <string.h>
#include <math.h>
#include "boards.h"
#include "sdk_common.h"

#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_gpiote.h"

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
} accel_xyz_out_t;

typedef struct {
	uint8_t CTRL_REG0;
	uint8_t TEMP_CFG_REG;
	uint8_t CTRL_REG1;
	uint8_t CTRL_REG2;
	uint8_t CTRL_REG3;
	uint8_t CTRL_REG4;
	uint8_t CTRL_REG5;
	uint8_t CTRL_REG6;
} control_block_t;

typedef struct {
	uint8_t address;
	uint8_t value;
} register_command_t;

typedef struct {
	uint8_t start_address;
	uint8_t *buffer;
	uint16_t buffer_length;
} block_command_t;

/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
lis2dh12_instance_t accel_inst = {0}; // Instance of the LIS2DH12

#define SPI_INSTANCE  0 //SPI instance index
const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); //SPI instance
volatile bool spi_xfer_done; //Flag used to indicate that SPI instance completed the transfer

#define SPI_BUFFER_LENGTH (256)
uint8_t m_tx_buf[SPI_BUFFER_LENGTH]; //Tx buffer
uint8_t m_rx_buf[SPI_BUFFER_LENGTH]; //Rx buffer

static register_command_t reg_command = {0};
static block_command_t block_command = {0};

volatile uint8_t fifo_wtm_flag = 0;
uint32_t read_index = 0;

/*******************************************************************************
															    PROCEDURES
*******************************************************************************/

/**
 * @brief Function sends a single byte write command to the accelerometer
 * This function sends a one-byte write register command to the accelerometer via SPI
 * @param[in] register_command_t* cmd
 */
static void accel_write_register(register_command_t* cmd)
{
	memcpy(m_tx_buf, cmd, sizeof(register_command_t));

	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sizeof(register_command_t), m_rx_buf, sizeof(register_command_t)));
	
	while (!spi_xfer_done) //Check for successful transfer
	{
		__WFE();
	}
}

/**
 * @brief Function sends a single byte read command to the accelerometer
 * This function sends a one-byte read register command to the accelerometer via SPI
 * @param[in] cmd: register_command_tpointer containing the desired register address
 * @param[out] Returns the value contained in the specified register
 */
static uint8_t accel_read_register(register_command_t* cmd)
{
	cmd->address |= READ_BIT; 
	cmd->value = 0xFF; // Insert dummy value to read during that SPI frame
	
	accel_write_register(cmd);
	
	return m_rx_buf[1];
}

/**
 * @brief Function writes a block to a range of addresses
 * Send a block of data to the accelerometer over SPI with the MS bit set
 * @param[in] cmd: block_command_t pointer containing the start address, 
 * data buffer and buffer length for the block transmission
 */
static void accel_write_block(block_command_t* cmd)
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
 * @param[in] cmd: block_command_t pointer containing the start address, 
 * receive data buffer and buffer length for the block transmission
 */
static void accel_read_block(block_command_t* cmd)
{
	if(cmd->buffer_length > SPI_BUFFER_LENGTH)
	{
		NRF_LOG_INFO("ERROR: Attempting to send too much data (%d) in accel_read_block()! (MAX %d)", cmd->buffer_length, SPI_BUFFER_LENGTH);
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
 * @brief Function checks if the FIFO is empty
 * This function checks if the FIFO buffer is empty, it resets the FIFO if true
 * @param[in] none
 */
static bool accel_fifo_check(void)
{
	reg_command.address = FIFO_SRC_REG;
	if((accel_read_register(&reg_command) & FIFO_SRC_EMPTY) == FIFO_SRC_EMPTY)
	{
		fifo_wtm_flag = 0;
		return true;
	}
	return false;
}

/**
 * @brief Function enables the FIFO
 * This function writes the neccessary values to registers enable the FIFO
 * @param[in] none
 */
static bool accel_fifo_enable(void)
{
	uint8_t fifo_config_w = CTRL_REG5_FIFO_EN;
	reg_command.address = CTRL_REG5;
	reg_command.value = fifo_config_w;
	accel_write_register(&reg_command);
	
	uint8_t fifo_config_r = accel_read_register(&reg_command);
	
	if(fifo_config_w != fifo_config_r)
	{
		#ifdef ACCEL_DEBUG_INFO
		NRF_LOG_INFO("FIFO Enable failed!");
		#endif
		
		return false;
	}
	
	return true;
}

/**
 * @brief Function sets the FIFO to bypass mode
 * This function writes the neccessary values to registers put the FIFO into bypass mode
 * @param[in] none
 */
static void accel_fifo_set_bypass_mode(void) // Optional: this could return bool for init purposes, and remain unused in other cases
{
	// Put the FIFO into bypass mode and clear the register
	reg_command.address = FIFO_CTRL_REG;
	reg_command.value = 0x00;
	accel_write_register(&reg_command);	
}

/**
 * @brief Function sets the FIFO to fifo mode
 * This function writes the neccessary values to registers put the FIFO into fifo mode
 * @param[in] none
 */
static void accel_fifo_set_fifo_mode(void) // Optional: this could return bool for init purposes, and remain unused in other cases
{
	// Put the FIFO into fifo mode and set the fifo ctrl values
	reg_command.address = FIFO_CTRL_REG;
	reg_command.value = FIFO_CTRL_FM0 | FIFO_CTRL_FTH4 | FIFO_CTRL_FTH3 | FIFO_CTRL_FTH2 | FIFO_CTRL_FTH1 | FIFO_CTRL_FTH0;
	accel_write_register(&reg_command);	
}

/**
 * @brief Event handler for INT1 watermark interrupt initializes the accelerometer
 * This function sets a flag to begin acquiring the samples from the XYZ registers once an interrupt has occured
 * @param[in] pin: The pin that drives the interrupt
* @param[in] polarity: The transition polarity that causes this interrupt
 */
static void accel_wtm_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity)
{
	fifo_wtm_flag = 1; // Set wtm_flag
}

/**
 * @brief Function initializes the GPIO necessary for the LIS2DH12 interrupt functionality
 * This function sets up the INT1 pin for interrupts using the specified event handler function
 * @param[in] none
 */
static void accel_pin_int_init(void)
{
	ret_code_t err_code;
	
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	
	err_code = nrf_drv_gpiote_in_init(INT1_PIN, &in_config, accel_wtm_event_handler);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_in_event_enable(INT1_PIN, true);
}

/**
 * @brief Function that checks the WHO_AM_I register
 * This function reads the WHO_AM_I register to confirm the device ID of 33
 * @param[in] none
 */
static bool accel_probe(void)
{
	reg_command.address = WHO_AM_I;
	
	uint8_t reg_val = accel_read_register(&reg_command);
	
	if(reg_val != LIS2DH12_WHO_AM_I)
	{
		#ifdef ACCEL_DEBUG_INFO
		NRF_LOG_INFO("Accel Probe Failed!");
		#endif
		
		return false;
	}
	
	return true;
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
 * @brief Function reads X Y Z data from the accelerometer
 * Get the X Y Z OUT register values and convert them to their readable floating point representations
 * @param[in] data: accel_xyz_data_t pointer to receive the x y and z values
 * @param[in] accel_inst: pointer to the lis2dh12 instance variable
 */
void ACCEL_read_xyz(accel_xyz_data_t* data)
{
	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Reading accel XYZ data registers...");
	#endif
	
	accel_xyz_out_t xyz_out_registers = {0};
	
	block_command.start_address = OUT_X_L;
	block_command.buffer = (uint8_t*)&xyz_out_registers;
	block_command.buffer_length = sizeof(xyz_out_registers);
	
	accel_read_block(&block_command);
	
	int16_t x_out_temp = (((int16_t)xyz_out_registers.OUT_X_H << 8) | ((int16_t)xyz_out_registers.OUT_X_L << 0)) >> (XYZ_FULL_DATA_SIZE - accel_inst.resolution);
	int16_t y_out_temp = (((int16_t)xyz_out_registers.OUT_Y_H << 8) | ((int16_t)xyz_out_registers.OUT_Y_L << 0)) >> (XYZ_FULL_DATA_SIZE - accel_inst.resolution);
	int16_t z_out_temp = (((int16_t)xyz_out_registers.OUT_Z_H << 8) | ((int16_t)xyz_out_registers.OUT_Z_L << 0)) >> (XYZ_FULL_DATA_SIZE - accel_inst.resolution);
	
	uint16_t negative_bit_mask = 1 << (accel_inst.resolution - 1);
		
	if((x_out_temp & negative_bit_mask) == negative_bit_mask) // If x_out is (-)
	{
		data->out_x = SENSITIVITY * -((~x_out_temp + 1) & ((1 << accel_inst.resolution) - 1));
	}
	else
	{
		data->out_x = SENSITIVITY * x_out_temp;
	}
	
	if((y_out_temp & negative_bit_mask) == negative_bit_mask) // If y_out is (-)
	{
		data->out_y = SENSITIVITY * -((~y_out_temp + 1) & ((1 << accel_inst.resolution) - 1));
	}
	else
	{
		data->out_y = SENSITIVITY * y_out_temp;
	}
	
	if((z_out_temp & negative_bit_mask) == negative_bit_mask) // If z_out is (-)
	{
		data->out_z = SENSITIVITY * -((~z_out_temp + 1) & ((1 << accel_inst.resolution) - 1));
	}
	else
	{
		data->out_z = SENSITIVITY * z_out_temp;
	}
	
	if(accel_fifo_check())
	{
		accel_fifo_set_bypass_mode();
		accel_fifo_set_fifo_mode();
	}
}

/**
 * @brief Function reads all X Y Z data from the accelerometer fifo buffer
 * Get the X Y Z OUT register values and convert them to their readable floating point representations
 * @param[in] data: accel_xyz_data_t pointer to receive the x y and z values
 * @param[in] accel_inst: pointer to the lis2dh12 instance variable
 */
void ACCEL_read_xyz_fifo(accel_xyz_data_t data[])
{
	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Reading accel XYZ block data...");
	#endif

	accel_xyz_out_t xyz_out_fifo[ACCEL_FIFO_LENGTH] = {0};	//initialize array of structs to all 0s

	block_command.start_address = OUT_X_L;
	block_command.buffer = (uint8_t*)xyz_out_fifo;
	block_command.buffer_length = ACCEL_FIFO_LENGTH*BYTES_PER_DATA_SAMPLE;

	accel_read_block(&block_command);

	int16_t x_out_temp;
	int16_t y_out_temp;
	int16_t z_out_temp;

	for(int i = 0; i < ACCEL_FIFO_LENGTH; i++)
	{
		x_out_temp = (((int16_t)xyz_out_fifo[i].OUT_X_H << 8) | ((int16_t)xyz_out_fifo[i].OUT_X_L << 0)) >> (XYZ_FULL_DATA_SIZE - accel_inst.resolution);
		y_out_temp = (((int16_t)xyz_out_fifo[i].OUT_Y_H << 8) | ((int16_t)xyz_out_fifo[i].OUT_Y_L << 0)) >> (XYZ_FULL_DATA_SIZE - accel_inst.resolution);
		z_out_temp = (((int16_t)xyz_out_fifo[i].OUT_Z_H << 8) | ((int16_t)xyz_out_fifo[i].OUT_Z_L << 0)) >> (XYZ_FULL_DATA_SIZE - accel_inst.resolution);
		
		uint16_t mask = 1 << (accel_inst.resolution - 1);
			
		if((x_out_temp & mask) == mask) // If x_out is (-)
		{
			data[i].out_x = SENSITIVITY * -((~x_out_temp + 1) & ((1 << accel_inst.resolution) - 1));
		}
		else
			data[i].out_x = SENSITIVITY * x_out_temp;
		
		if((y_out_temp & mask) == mask) // If y_out is (-)
		{
			data[i].out_y = SENSITIVITY * -((~y_out_temp + 1) & ((1 << accel_inst.resolution) - 1));
		}
		else
			data[i].out_y = SENSITIVITY * y_out_temp;
		
		if((z_out_temp & mask) == mask) // If z_out is (-)
		{
			data[i].out_z = SENSITIVITY * -((~z_out_temp + 1) & ((1 << accel_inst.resolution) - 1));
		}
		else
			data[i].out_z = SENSITIVITY * z_out_temp;
	}
	
	if(accel_fifo_check())
	{
		accel_fifo_set_bypass_mode();
		accel_fifo_set_fifo_mode();
	}
}

bool ACCEL_analyze_xyz(accel_xyz_data_t data_in[], float impact_data[])
{
	uint8_t batch_flag = 0; // This flag is used to indicate whether an impact has been detected in this batch of data
	
	for(int i = 0; i < ACCEL_FIFO_LENGTH; i++)
	{
		impact_data[i] = sqrt(pow(data_in[i].out_x, 2) + pow(data_in[i].out_y,2) + pow(data_in[i].out_z,2)); // Take the magnitude of a single XYZ sample
		
		if(impact_data[i] > IMPACT_THRESHOLD)
		{
			batch_flag = 1;
		}
	}
	
	return batch_flag == 1; // If batch flag == 1, an impact has been detected. Return true
}

/**
 * @brief Function initializes and enables the FIFO
 * This function writes the neccessary values to the desired CTRL registers to initialize the FIFO
 * @param[in] none
 */
bool ACCEL_fifo_init(void)
{
	accel_fifo_set_bypass_mode(); // Set bypass mode to clear fifo buffer of any existing samples
	nrf_delay_ms(10);

	// Initialize the FIFO
	uint8_t fifo_config_w = FIFO_CTRL_FM0 | FIFO_CTRL_FTH4 | FIFO_CTRL_FTH3 | FIFO_CTRL_FTH2 | FIFO_CTRL_FTH1 | FIFO_CTRL_FTH0;
	reg_command.address = FIFO_CTRL_REG;
	reg_command.value = fifo_config_w;
	accel_write_register(&reg_command);

	uint8_t fifo_config_r = accel_read_register(&reg_command);

	if((fifo_config_w != fifo_config_r) | !accel_fifo_enable())
	{
		#ifdef ACCEL_DEBUG_INFO
		NRF_LOG_INFO("FIFO Initialization failed!");
		#endif

		return false;
	}	
	
	return true;
}

/**
 * @brief Function initializes the accelerometer
 * This function writes the neccessary values to the desired CTRL registers to initialize the LIS2DH12
* @param[in] accel_inst: accel instance
 */
bool ACCEL_init(void)
{
	// Initialize the SPI peripheral
	spi_init();

	// Check device ID
	if(!accel_probe())
	{
		return false;
	}

	accel_pin_int_init(); // Not sure if we would like to throw some debug capabilities around this

	nrf_delay_ms(100);

	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Accelerometer Initializing...");
	#endif

	// Initialize the CTRL registers
	control_block_t config_block_w = {
		CTRL_REG0_VALID_MASK,
		0x00,
		CTRL_REG1_X_EN | CTRL_REG1_Y_EN | CTRL_REG1_Z_EN | CTRL_REG1_ODR0 | CTRL_REG1_ODR1,
		0x00,
		CTRL_REG3_I1_WTM,
		CTRL_REG4_BDU | CTRL_REG4_FS0 | CTRL_REG4_FS1,
	};		

	block_command.start_address = CTRL_REG0;
	block_command.buffer = (uint8_t*)&config_block_w;
	block_command.buffer_length = sizeof(config_block_w);

	accel_write_block(&block_command);

	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Accelerometer Initializing...");
	#endif

	control_block_t config_block_r = {0};
	block_command.start_address = CTRL_REG0;
	block_command.buffer = (uint8_t*)&config_block_r;
	block_command.buffer_length = sizeof(config_block_r);
	accel_read_block(&block_command);
	
	if(memcmp(&config_block_r, &config_block_w, sizeof(control_block_t)) != 0)
	{
		#ifdef ACCEL_DEBUG_INFO
		NRF_LOG_INFO("Accelerometer Initialization failed!");
		#endif

		return false;
	}

	// Set the resolution of the samples from the X Y Z registers
	uint8_t op_mode = ((config_block_r.CTRL_REG1 & CTRL_REG1_LPEN) << 1) | (config_block_r.CTRL_REG4 & CTRL_REG4_HR);
	if(op_mode == LOW_POWER)
		accel_inst.resolution = LOW_RES_BITS;
	else if(op_mode == NORMAL)
		accel_inst.resolution = NORMAL_RES_BITS;
	else
		accel_inst.resolution = HIGH_RES_BITS;

	#ifdef ACCEL_DEBUG_INFO
	NRF_LOG_INFO("Accelerometer Initializated.");
	#endif

	return true;
}

/**
 * @brief Function that puts the ACCEL in power-down mode
 * This function writes to the necessary registers to put the accel into power-down mode
 * @param[in] none
 */
void ACCEL_pwrdn(void)
{
	// Set Accel to power-down mode and disable all axes
	reg_command.address = CTRL_REG1;
	reg_command.value = 0x00;

	accel_write_register(&reg_command);
}
