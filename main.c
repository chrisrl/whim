/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Dynastream Innovations, Inc. 2015
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1) Redistributions of source code must retain the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2) Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * 3) Neither the name of Dynastream nor the names of its
 *    contributors may be used to endorse or promote products
 *    derived from this software without specific prior
 *    written permission.
 * 
 * The following actions are prohibited:
 * 1) Redistribution of source code containing the ANT+ Network
 *    Key. The ANT+ Network Key is available to ANT+ Adopters.
 *    Please refer to http://thisisant.com to become an ANT+
 *    Adopter and access the key.
 * 
 * 2) Reverse engineering, decompilation, and/or disassembly of
 *    software provided in binary form under this license.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 * 
 */
 
/*******************************************************************************
																		INCLUDES
*******************************************************************************/

// std includes
#include <stdint.h>
#include <string.h>

// sdk config
#include "sdk_config.h"

// nrf includes
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"

// misc includes
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "hardfault.h"

// whim includes
#include "ANT.h"
#include "ADXL375.h"
#include "ADXL375_registers.h"

/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
//#define DISPLAY_ACCEL_DATA // Uncomment if the raw accelerometer data needs to be displayed
#define DISPLAY_IMPACT_DATA // Uncomment if the impact and magnitude data needs to be displayed 

extern volatile uint8_t fifo_wtm_flag;
extern uint32_t read_index;

//static accel_xyz_data_t accel_data[ACCEL_FIFO_LENGTH]; //One fifo worth of data
//static float impact_data[ACCEL_FIFO_LENGTH]; // One fifo worth of analyzed data
//static bool impact_detected = false; // Boolean used to indicate if an impact has occured

#ifdef DISPLAY_ACCEL_DATA
static char disp_string[100]; // String used to display desired output data
#endif

volatile uint8_t impact_count = 0; // Variable to hold the overall impact count of this device

/*******************************************************************************
															      PROCEDURES
*******************************************************************************/

/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
		NRF_LOG_INFO("%d", err_code);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = app_timer_init();
		NRF_LOG_INFO("%d", err_code);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_LED, NULL);
		NRF_LOG_INFO("%d", err_code);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
		NRF_LOG_INFO("%d", err_code);
    APP_ERROR_CHECK(err_code);
}


/*******************************************************************************
															      PROCEDURES
*******************************************************************************/


int main(void)
{
	utils_setup();
	
	NRF_LOG_INFO("----- WHIM Sensor -----");
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	
//	ANT_init();
//	
//	if(!ACCEL_init())
//	{
//		NRF_LOG_INFO("ERROR: Accelerometer Initialization failed!");
//		NRF_LOG_FLUSH();
//		nrf_delay_ms(100);
//		return 0;
//	}

//	if(!ACCEL_fifo_init())
//	{
//		NRF_LOG_INFO("ERROR: FIFO Initialization failed!");
//		NRF_LOG_FLUSH();
//		nrf_delay_ms(100);
//		return 0;
//	}
//	NRF_LOG_FLUSH();
//	nrf_delay_ms(100);
	

	while (1)
	{	
//		if(fifo_wtm_flag == 1)
//		{				
//			//bsp_board_led_invert(BSP_BOARD_LED_0);
//			ACCEL_read_xyz_fifo(accel_data);
//			impact_detected = ACCEL_analyze_xyz(accel_data, impact_data);
//			
//			if(impact_detected)
//			{
//				++impact_count;
//				NRF_LOG_INFO("...Impact level event(s) detected...");
//			}
//			
//			for (uint8_t i = 0; i < ACCEL_FIFO_LENGTH; i++)
//			{
//			  #ifdef DISPLAY_ACCEL_DATA
//			    sprintf(disp_string, "X = %.2f, Y = %.2f, Z = %.2f", accel_data[i].out_x, accel_data[i].out_y, accel_data[i].out_z);
//				  NRF_LOG_INFO("%s",disp_string); // Display the interpretted accel data
//				  NRF_LOG_FLUSH(); //flush often so that the buffer doesnt overflow
//			  #endif
//				
//				#ifdef DISPLAY_IMPACT_DATA
//				//TODO throw some printy bois in here
//				if(impact_data[i] > IMPACT_THRESHOLD)
//				{
//					NRF_LOG_INFO("Impact Value: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(impact_data[i]));
//					NRF_LOG_FLUSH();	
//				}			
//				#endif
//			}
//			
//		  NRF_LOG_FLUSH();
//		}		
//		
//    nrf_pwr_mgmt_run();

	  //bsp_board_led_invert(BSP_BOARD_LED_3);
		//nrf_delay_ms(1000);
	}
}

