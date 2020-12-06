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

#define WHO_AM_I        0x0F
#define I_AM_LIS2DH     0x33

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

//static uint8_t       who_am_i[] = {0x8F, 0x00};			/**< TX buffer. */
//static uint8_t			 config1[] = {0x20,0x37};
static uint8_t			 command[2];
static uint8_t       m_tx_buf[10];						/**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(m_tx_buf)+1];			/**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

static uint8_t received = 0;
static char str[10];



static void vSend_Data(uint8_t data[], size_t sz);
static uint8_t read_Data(uint8_t data[], size_t sz);

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
	spi_xfer_done = true;
	NRF_LOG_INFO("Transfer completed.");
	if (m_rx_buf[0] != 0)
	{
		NRF_LOG_INFO(" Received:");
		NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
	}
}

/**
 * @brief Function Send a command to read Id
 * This function sends a one-byte command to ACC via SPI
 * @param[in] none
 */
static uint8_t AccWhoAmI(void)
{
	command[0] = 0x80|WHO_AM_I;
	command[1] = 0xFF;
	
	read_Data(command,sizeof(command));
	//vSend_Data(who_am_i,sizeof(who_am_i));
	// Confirm we have a connection, should be I_AM_LIS2DH
	return m_rx_buf[1];
}

static void vSend_Data(uint8_t data[], size_t sz)
{
	memcpy(m_tx_buf,data,sz);

	//memset(rx_buf, '?', m_tx_buf);
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sz, m_rx_buf, m_length));
	// Check for successful transfer
	while (!spi_xfer_done)
	{
		__WFE();
	}
}

static uint8_t read_Data(uint8_t data[], size_t sz)
{
	vSend_Data(data,sz);
	return m_rx_buf[1];
}

static void vPower_Down(void)
{
	// If this function is called after power on, then the LIS2DH12
	// will still be in boot mode, allow time for boot to complete. 
	nrf_delay_ms(20);
	
	// Put device into power-down
	command[0] = 0x20;
	command[1] = 0x00;
	vSend_Data(command,sizeof(command));
	nrf_delay_ms(20);
}


static void vInit_LIS2DH12(void)
{
	
	command[0] = 0x20;
	command[1] = 0x37;
	vSend_Data(command,sizeof(command));
	command[0] = 0x21;
	command[1] = 0x00;
	vSend_Data(command,sizeof(command));
	command[0] = 0x22;
	command[1] = 0x04;
	vSend_Data(command,sizeof(command));
	command[0] = 0x23;
	command[1] = 0x80|0x08|0x30;
	//vSend_Data(command,sizeof(command));
	command[0] = 0x24;
	command[1] = 0x40|0x08;
	vSend_Data(command,sizeof(command));
	
	

	//received = m_rx_buf[1];
	//sprintf(str,"%x",received);
	//NRF_LOG_INFO("Config1: %s",(uint32_t)str);
	//NRF_LOG_FLUSH();
}

int main(void)
{
	bsp_board_leds_init();

	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	
	NRF_LOG_INFO("SPI example.");

	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin  = SPI_SCK_PIN;
	spi_config.mode     = NRF_DRV_SPI_MODE_0;
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
	
	vInit_LIS2DH12();
	//vPower_Down();
	
	//received = AccWhoAmI();
	//sprintf(str,"%x",received);
	//NRF_LOG_INFO("I am: %s",(uint32_t)str);
	//if(received == I_AM_LIS2DH){
	//	bsp_board_led_on(BSP_BOARD_LED_3);
	//}
	//NRF_LOG_FLUSH();

	while (1)
	{
		// Reset rx buffer and transfer done flag
		memset(m_rx_buf, 0, m_length);
		spi_xfer_done = false;
			
		received = AccWhoAmI();
		sprintf(str,"%x",received);
		NRF_LOG_INFO("I am: %s",(uint32_t)str);
		if(received == I_AM_LIS2DH){
			bsp_board_led_invert(BSP_BOARD_LED_3);
		}
		NRF_LOG_FLUSH();
		//command[0] = 0x80|0x07;
		//command[1] = 0xFF;
		//if(read_Data(command,sizeof(command)) & 0x44){
		//	command[0] = 0x80|0x28;
		//	command[1] = 0xFF;
		//	sprintf(str,"%x",read_Data(command,sizeof(command)));
		//	NRF_LOG_INFO("Temp: %s",(uint32_t)str);
		//	NRF_LOG_FLUSH();
		//}
		//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

		//while (!spi_xfer_done)
		//{
		//    __WFE();
		//}

		NRF_LOG_FLUSH();

		bsp_board_led_invert(BSP_BOARD_LED_0);
		nrf_delay_ms(200);
	}
}
