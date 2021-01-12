/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
 
/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/
#include "sdk_config.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "LIS2DH12.h"

/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
//#define DISPLAY_ACCEL_DATA // Uncomment if the raw accelerometer data needs to be displayed
//#define DISPLAY_IMPACT_DATA // Uncomment if the impact and magnitude data needs to be displayed 

extern volatile uint8_t fifo_wtm_flag;
extern uint32_t read_index;

static accel_xyz_data_t accel_data[ACCEL_FIFO_LENGTH]; //One fifo worth of data
static float impact_data[ACCEL_FIFO_LENGTH]; // One fifo worth of analyzed data
static bool impact_detected = false; // Boolean used to indicate if an impact has occured
static char disp_string[100]; // String used to display desired output data

/*******************************************************************************
															      PROCEDURES
*******************************************************************************/

int main(void)
{
	bsp_board_leds_init();

	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();

	NRF_LOG_INFO("----- WHIM Sensor -----");
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);

	if(!ACCEL_init())
	{
		NRF_LOG_INFO("ERROR: Accelerometer Initialization failed!");
		NRF_LOG_FLUSH();
		nrf_delay_ms(100);
		return 0;
	}

	if(!ACCEL_fifo_init())
	{
		NRF_LOG_INFO("ERROR: FIFO Initialization failed!");
		NRF_LOG_FLUSH();
		nrf_delay_ms(100);
		return 0;
	}
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	
	while (1)
	{	
		if(fifo_wtm_flag == 1)
		{				
			//bsp_board_led_invert(BSP_BOARD_LED_0);
			ACCEL_read_xyz_fifo(accel_data);
			impact_detected = ACCEL_analyze_xyz(accel_data, impact_data);
			
			for (uint8_t i = 0; i < ACCEL_FIFO_LENGTH; i++)
			{
			  #ifdef DISPLAY_ACCEL_DATA
			    sprintf(disp_string, "X = %.2f, Y = %.2f, Z = %.2f", accel_data[i].out_x, accel_data[i].out_y, accel_data[i].out_z);
				  NRF_LOG_INFO("%s",disp_string); // Display the interpretted accel data
				  NRF_LOG_FLUSH(); //flush often so that the buffer doesnt overflow
			  #endif
				
				#ifdef DISPLAY_IMPACT_DATA
				//TODO throw some printy bois in here
				NRF_LOG_INFO("Impact Value: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(impact_data[i]));
				NRF_LOG_FLUSH();				
				#endif
			}
			
			if(impact_detected)
			{
				NRF_LOG_INFO("...Impact level event(s) detected...");
			}
			
		  NRF_LOG_FLUSH();
		}		
		
	  //bsp_board_led_invert(BSP_BOARD_LED_3);
		//nrf_delay_ms(1000);
	}
}
