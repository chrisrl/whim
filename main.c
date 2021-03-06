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
#include "nrf_fstorage_sd.h"

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
#include "fstorage_manager.h"
#include "whim_queue.h"
#include "algo.h"

/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
//#define DISPLAY_IMPACT_SCORE // Uncomment to display the impact score and impact count
//#define DISPLAY_FIFO_DATA // Uncomment to use any of the options below
//#define DISPLAY_ACCEL_AXIS_DATA // Uncomment if the raw accelerometer data needs to be displayed
//#define DISPLAY_ACCEL_GFORCE_DATA // Uncomment if the overall gforce data needs to be displayed

// Board Defines
#define BAT_LOW_INPUT 2
#define BAT_LED_PIN 23
#define BONK_LED_PIN 24
#define BAT_LOW 0
#define LED_ON 1
#define LED_OFF 0

extern volatile uint8_t fifo_wtm_flag;
extern volatile uint8_t impact_reset_flag;
extern uint8_t impact_score_flag;

static accel_xyz_data_t accel_data[ACCEL_FIFO_LENGTH]; //One fifo worth of data
static float impact_data[ACCEL_FIFO_LENGTH]; // One fifo worth of analyzed data

#ifdef DISPLAY_ACCEL_AXIS_DATA
static char disp_string[100];// String used to display desired output data
#endif

uint8_t impact_count = 0; // Variable to hold the overall impact count of this device
uint16_t impact_score_latest = 0; // Variable to hold the most recent HIC impact score that is transmitted
uint16_t impact_score_max = 0; // Variable to hold the largest HIC
static float impact_linear_acc = 0; // Variable to hold the most recent peak linear acceleration
/*******************************************************************************
															      PROCEDURES
*******************************************************************************/

#ifdef DISPLAY_IMPACT_SCORE
static void display_impact_data(uint16_t impact_score) 
{
		NRF_LOG_INFO("...Impact level event detected...");
		NRF_LOG_INFO("HIC Score: %u --- Peak LA: " NRF_LOG_FLOAT_MARKER, impact_score, NRF_LOG_FLOAT(impact_linear_acc));
		NRF_LOG_INFO("Max HIC Score: %d", impact_score_max);
		NRF_LOG_INFO("Impact Count: %d", impact_count);
		NRF_LOG_FLUSH();
}
#endif

#ifdef DISPLAY_FIFO_DATA
static void display_fifo_data(void)
{
	for (uint8_t i = 0; i < ACCEL_FIFO_LENGTH; i++)
	{
		#ifdef DISPLAY_ACCEL_AXIS_DATA
		sprintf(disp_string, "X = %.2f, Y = %.2f, Z = %.2f", accel_data[i].out_x, accel_data[i].out_y, accel_data[i].out_z);
		NRF_LOG_INFO("%s",disp_string); // Display the interpretted accel data
		NRF_LOG_FLUSH(); //flush often so that the buffer doesnt overflow
		#endif
		
		#ifdef DISPLAY_ACCEL_GFORCE_DATA
		NRF_LOG_INFO("G-Force Value: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(impact_data[i]));
		NRF_LOG_FLUSH();	
		#endif
	}
}
#endif

/**
 * @brief Function for the Timer and BSP initialization.
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

		// LED and Battery Indicator GPIO Initializations
		nrf_gpio_cfg_input(BAT_LOW_INPUT, NRF_GPIO_PIN_PULLUP);	// Configure battery indicator as input
		nrf_gpio_cfg_output(BAT_LED_PIN);	//set led to output
		nrf_gpio_pin_write(BAT_LED_PIN, 0);	//turn off led
		nrf_gpio_cfg_output(BONK_LED_PIN);	//set led to output
		nrf_gpio_pin_write(BONK_LED_PIN, 0);	//turn off led

    err_code = nrf_pwr_mgmt_init();
		NRF_LOG_INFO("%d", err_code);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
	utils_setup();
	
	NRF_LOG_INFO("----- WHIM Sensor -----");
	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	fstorage_init();
	ANT_init();
	
	if(!ACCEL_init())
	{
		NRF_LOG_INFO("ERROR: Accelerometer Initialization failed!");
		NRF_LOG_FLUSH();
		nrf_delay_ms(100);
		return 0;
	}
	
	/* Reading stored impact value from flash */
	fstorage_read_impact();

	NRF_LOG_FLUSH();
	nrf_delay_ms(100);
	
	uint16_t impact_score = 0; // Temporary variable to hold the most recent HIC impact score
	while (1)
	{
		if(fifo_wtm_flag == 1)
		{
			ACCEL_read_xyz_fifo(accel_data);
			impact_score = (uint16_t) ALGO_get_impact_score(accel_data, impact_data, &impact_linear_acc);
			
			if(impact_score > IMPACT_SCORE_THRESH_INT && impact_linear_acc > IMPACT_LINEAR_ACC_THRESHOLD)
			{ 
				impact_score_latest = impact_score;
				
				if(impact_score > impact_score_max)
				{
					impact_score_max = impact_score;
				}
				++impact_count;
			  fstorage_write_impact();
				
				#ifdef DISPLAY_IMPACT_SCORE
				display_impact_data(impact_score);
			  #endif
				
			  impact_score_flag = 1;		
			}
			
			#ifdef DISPLAY_FIFO_DATA
			display_fifo_data();
			#endif
			
		  NRF_LOG_FLUSH();
		}		
		
		if(impact_reset_flag == 1)
		{
			/* Reset Impact count to 0 */
			impact_count = 0;
			impact_score_latest = 0;
			impact_score_max = 0; 
		
			fstorage_write_impact();
			NRF_LOG_INFO("Impact reset.");
			
			/* Reset impact reset flag */
			impact_reset_flag = 0;
		}
		
		// Battery Low LED Indicator
		if(nrf_gpio_pin_read(BAT_LOW_INPUT) == BAT_LOW){
			if(nrf_gpio_pin_out_read(BAT_LED_PIN) == LED_OFF)	// If LED off
				nrf_gpio_pin_write(BAT_LED_PIN, LED_ON);	// Turn on led
		}
		else{
			if(nrf_gpio_pin_out_read(BAT_LED_PIN) == LED_ON)	// If LED on
				nrf_gpio_pin_write(BAT_LED_PIN, LED_OFF);	// Turn off led
		}
		
    nrf_pwr_mgmt_run();
	}
}
