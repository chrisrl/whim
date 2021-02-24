/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#include "whim_queue.h"
#include "app_util_platform.h"

/**@brief Function gets the next element index.
 * @param[in] p_queue Pointer to the queue instance.
 * @param[in] idx Current index.
 * @return Next element index.
 */
__STATIC_INLINE size_t whim_queue_next_idx(whim_queue_t const * p_queue, size_t idx)
{
    ASSERT(p_queue != NULL);
    return (idx < p_queue->size) ? (idx + 1) : 0;
}


/**
 * @brief Function gets current queue utilization. This function assumes that this process will not be interrupted.
 * @param[in] p_queue Pointer to the queue instance.
 * @return Current queue utilization.
 */
__STATIC_INLINE size_t queue_utilization_get(whim_queue_t const * p_queue)
{
    size_t front    = p_queue->p_cb->front;
    size_t back     = p_queue->p_cb->back;
    return (back >= front) ? (back - front) : (p_queue->size + 1 - front + back);
}


/**
* Function checks to see if the queue is empty
	@param[in] p_queue Pointer to the queue instance
*	@return true if the queue is empty
*/
static bool whim_queue_is_empty(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    size_t front    = p_queue->p_cb->front;
    size_t back     = p_queue->p_cb->back;
    return (front == back);
}


/**
* Function checks to see if the queue is full
	@param[in] p_queue Pointer to the queue instance
*	@return true if the queue is full
*/
static bool whim_queue_is_full(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    size_t front    = p_queue->p_cb->front;
    size_t back     = p_queue->p_cb->back;

    return (whim_queue_next_idx(p_queue, back) == front);
}


/**
* @brief Function pops element from the queue
* @param[in] p_queue Pointer to the queue instance
* @param[in] p_element Pointer to the element being inserted
* @param[in] just_peek Bool to indicate whether to remove item from queue when popping
* @return NRF_SUCCESS if no errors were encountered
*/
ret_code_t whim_queue_generic_pop(whim_queue_t const * p_queue, float * p_element, bool just_peek)                                                                
{
    ret_code_t status = NRF_SUCCESS;

    ASSERT(p_queue      != NULL);
    ASSERT(p_element    != NULL);

    CRITICAL_REGION_ENTER();

    if (!whim_queue_is_empty(p_queue))
    {
        // Get read position.
        size_t read_pos = p_queue->p_cb->front;

        // Update next read position.
        if (!just_peek)
        {
            p_queue->p_cb->front = whim_queue_next_idx(p_queue, p_queue->p_cb->front);
        }

				*((float *)p_element) = ((float *)p_queue->p_buffer)[read_pos];
    }
    else
    {
        status = NRF_ERROR_NOT_FOUND;
    }

    CRITICAL_REGION_EXIT();

    return status;
}


/**
* @brief Function pushes element into the queue
* @param[in] p_queue Pointer to the queue instance
* @param[in] p_element Pointer to the element being inserted
* @return NRF_SUCCESS if no errors were encountered
*/
ret_code_t whim_queue_push(whim_queue_t const * p_queue, float const * p_element)
{
    ret_code_t status = NRF_SUCCESS;

    ASSERT(p_queue != NULL);
    ASSERT(p_element != NULL);

    CRITICAL_REGION_ENTER();
    bool is_full = whim_queue_is_full(p_queue);

    if (!is_full || (p_queue->mode == WHIM_QUEUE_MODE_OVERFLOW))
    {
        // Get write position.
        size_t write_pos = p_queue->p_cb->back;
        p_queue->p_cb->back = whim_queue_next_idx(p_queue, p_queue->p_cb->back);
        if (is_full)
        {
            // Overwrite the oldest element.
            p_queue->p_cb->front = whim_queue_next_idx(p_queue, p_queue->p_cb->front);
        }
				
				((float *)p_queue->p_buffer)[write_pos] = *((float *)p_element);

        // Update utilization.
        size_t utilization = queue_utilization_get(p_queue);
        if (p_queue->p_cb->max_utilization < utilization)
        {
            p_queue->p_cb->max_utilization = utilization;
        }
    }
    else
    {
        status = NRF_ERROR_NO_MEM;
    }

    CRITICAL_REGION_EXIT();

    return status;
}


/**
* Function integrates over the current values in the queue using left-rectangular integration
* @param[in] p_queue Pointer to the queue instance
* @param[in] dx The width of the rectangles used for integration
* @param[out] sum The integral product
*/
ret_code_t whim_queue_integrate(whim_queue_t const * p_queue, float * sum, double dx)																									
{
	ret_code_t status = NRF_SUCCESS;

	ASSERT(p_queue      != NULL);
	*sum = 0;
	CRITICAL_REGION_ENTER();
	if (!whim_queue_is_empty(p_queue))
	{
		size_t read_pos = p_queue->p_cb->front;
		do{
			*sum += (p_queue->p_buffer[read_pos]);
			read_pos = whim_queue_next_idx(p_queue, read_pos);
		}while(read_pos != p_queue->p_cb->back);
		*sum *= dx;
	}
	else
	{
		status = NRF_ERROR_NOT_FOUND;
	}

	CRITICAL_REGION_EXIT();

	return status;
}


/**
* Function resets the contents of the queue
	@param[in] p_queue Pointer to the queue instance
*/
void whim_queue_reset(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    CRITICAL_REGION_ENTER();
    memset(p_queue->p_cb, 0, sizeof(whim_queue_cb_t));
    CRITICAL_REGION_EXIT();
}

