/******************************************************************************
File: fstorage_manager.c

This source file contains the functions and variables responsible for handling
Flash Storage (fstorage).

******************************************************************************/


/*******************************************************************************
															GENERAL INCLUDES
*******************************************************************************/
#include "nrf_sdh.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "fstorage_manager.h"
/*******************************************************************************
															VARIABLES AND CONSTANTS
*******************************************************************************/
//#define DISPLAY_FSTORAGE_INFO // Uncomment if the fstorage info needs to be displayed

nrf_fstorage_api_t *p_fs_api;
extern volatile uint32_t impact_count;
/*******************************************************************************
															      PROCEDURES
*******************************************************************************/
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x7f000,
    .end_addr   = 0x7ffff,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
	#ifdef DISPLAY_FSTORAGE_INFO
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
	#endif
}

/**
 * @brief Function that waits for fstorage instance
 * This sleeps until the fstorage instance is not busy.
 */
static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        (void) sd_app_evt_wait();
    }
}

/**
 * @brief Function initalizes fstorage instance
 * Initialize an fstorage instance using the nrf_fstorage_sd backend.
 * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
 * used whenever there is a SoftDevice, regardless of its status (enabled/disabled)
 */
void fstorage_init(void)
{
	ret_code_t rc;
	
	p_fs_api = &nrf_fstorage_sd;
	rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
	APP_ERROR_CHECK(rc);
}

/**
 * @brief Function writes current impact count to flash
 * This function overwrites in flash the current value of impact_count.
 */
void fstorage_write_impact(void)
{
	ret_code_t rc;
	uint32_t impact_curr;
	 
	/* Erase single flash page */
	rc = nrf_fstorage_erase(&fstorage, 0x7f000, 1, NULL);
	APP_ERROR_CHECK(rc);

	wait_for_flash_ready(&fstorage);
	
	/* Copy impact_count to meet program size requirements */
	impact_curr = impact_count;
				
	/* Writing to flash. */
	rc = nrf_fstorage_write(&fstorage, 0x7f000, (const void *)&impact_curr, sizeof(impact_curr), NULL);
	APP_ERROR_CHECK(rc);

	wait_for_flash_ready(&fstorage);
}

/**
 * @brief Function reads current impact count from flash
 * This function reads flash storage and stores the value in impact_count.
 */
void fstorage_read_impact(void)
{
	ret_code_t rc;
	uint32_t impact_curr;

	/* Copy impact_count to meet program size requirements */
	impact_curr = impact_count;
	
	/* Reading stored impact value from flash */
	rc = nrf_fstorage_read(&fstorage, 0x7f000, (void *)&impact_curr, sizeof(impact_curr));
	APP_ERROR_CHECK(rc);
	
	wait_for_flash_ready(&fstorage);
}
