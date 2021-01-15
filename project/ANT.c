/******************************************************************************
File: ANT.c

This source file contains the functions and variables responsible for initializing
and communicating over ANT wireless protocol

******************************************************************************/


/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/
#include "ANT.h"
#include <stdint.h>
#include "string.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "ant_error.h"
#include "boards.h"
#include "sdk_config.h"
#include "ant_channel_config.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "LIS2DH12.h"

extern accel_xyz_data_t xyz_data;

/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/

/*NOTE: Uncomment these lines for more debug info*/
#define ANT_DEBUG_INFO

#define DIGITALIO_DATA_PID              1u                      /**< Page number: digital data. */
#define APP_ANT_OBSERVER_PRIO           1                       /**< Application's ANT observer priority. You shouldn't need to modify this value. */

static uint8_t m_broadcast_data[ANT_STANDARD_DATA_PAYLOAD_SIZE];    /**< Primary data transmit buffer. */
static uint8_t m_tx_input_pin_state = 0;                         /**< State of digital inputs in this node, for transmission. */


/*******************************************************************************
															    PROCEDURES
*******************************************************************************/

/**@brief Formats page with current button state and sends data
 * Byte 0   = Page number (Digital I/O Data)
 * Byte 1-6 = Reserved
 * Byte 7   = State of digital inputs
 */
static void ant_handle_transmit()
{
	uint32_t err_code;


	m_broadcast_data[0] = DIGITALIO_DATA_PID;
	m_broadcast_data[1] = m_tx_input_pin_state;
	m_broadcast_data[2] = 0;
	m_broadcast_data[3] = 0;
	m_broadcast_data[4] = 0;
	m_broadcast_data[5] = 0;
	m_broadcast_data[6] = 0;
	m_broadcast_data[7] = 0;

	err_code = sd_ant_broadcast_message_tx(ANT_CHANNEL_NUM,
																				 ANT_STANDARD_DATA_PAYLOAD_SIZE,
																				 m_broadcast_data);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for ANT stack initialization.
 */
static void softdevice_setup(void)
{
	#ifdef ANT_DEBUG_INFO
	NRF_LOG_INFO("Softdevice Initializing...");
	#endif
	
	ret_code_t err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	ASSERT(nrf_sdh_is_enabled());

	err_code = nrf_sdh_ant_enable();
	APP_ERROR_CHECK(err_code);
	
	#ifdef ANT_DEBUG_INFO
	NRF_LOG_INFO("Softdevice Initialized.");
	#endif
}

void ANT_init(void)
{
	softdevice_setup();
	
	uint32_t err_code;

	ant_channel_config_t channel_config =
	{
		.channel_number    = ANT_CHANNEL_NUM,
		.channel_type      = CHANNEL_TYPE_MASTER,
		.ext_assign        = 0x00,
		.rf_freq           = RF_FREQ,
		.transmission_type = CHAN_ID_TRANS_TYPE,
		.device_type       = CHAN_ID_DEV_TYPE,
		.device_number     = NRF_FICR->DEVICEID[0],
		.channel_period    = CHAN_PERIOD,
		.network_number    = ANT_NETWORK_NUM,
	};

	#ifdef ANT_DEBUG_INFO
	NRF_LOG_INFO("Opening ANT channel...");
	#endif
	
	// Configure channel parameters
	err_code = ant_channel_init(&channel_config);
	APP_ERROR_CHECK(err_code);

	// Open channel.
	err_code = sd_ant_channel_open(ANT_CHANNEL_NUM);
	APP_ERROR_CHECK(err_code);
	
	#ifdef ANT_DEBUG_INFO
	NRF_LOG_INFO("ANT channel ");
	#endif
}

/**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
static void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
	switch (p_ant_evt->event)
	{
		case EVENT_RX:
			NRF_LOG_INFO("RX");
		//TODO: Handle a received message
			break;

		case EVENT_TX:
			// Transmit data on the reverse direction every channel period
			ant_handle_transmit();
			break;

		default:
			break;
	}
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);
