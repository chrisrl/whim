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

/**@brief Get next element index.
 *
 * @param[in]   p_queue     Pointer to the queue instance.
 * @param[in]   idx         Current index.
 *
 * @return      Next element index.
 */
__STATIC_INLINE size_t whim_queue_next_idx(whim_queue_t const * p_queue, size_t idx)
{
    ASSERT(p_queue != NULL);
    return (idx < p_queue->size) ? (idx + 1) : 0;
}

/**@brief Get current queue utilization. This function assumes that this process will not be interrupted.
 *
 * @param[in]   p_queue     Pointer to the queue instance.
 *
 * @return      Current queue utilization.
 */
__STATIC_INLINE size_t queue_utilization_get(whim_queue_t const * p_queue)
{
    size_t front    = p_queue->p_cb->front;
    size_t back     = p_queue->p_cb->back;
    return (back >= front) ? (back - front) : (p_queue->size + 1 - front + back);
}

bool whim_queue_is_full(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    size_t front    = p_queue->p_cb->front;
    size_t back     = p_queue->p_cb->back;

    return (whim_queue_next_idx(p_queue, back) == front);
}

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
//        // Write a new element.
//        switch (p_queue->element_size)
//        {
//            case sizeof(uint8_t):
//                ((uint8_t *)p_queue->p_buffer)[write_pos] = *((uint8_t *)p_element);
//                break;

//            case sizeof(uint16_t):
//                ((uint16_t *)p_queue->p_buffer)[write_pos] = *((uint16_t *)p_element);
//                break;

//            case sizeof(uint32_t):
//                ((uint32_t *)p_queue->p_buffer)[write_pos] = *((uint32_t *)p_element);
//                break;

//            case sizeof(uint64_t):
//                ((uint64_t *)p_queue->p_buffer)[write_pos] = *((uint64_t *)p_element);
//                break;

//            default:
//                memcpy((void *)((size_t)p_queue->p_buffer + write_pos * p_queue->element_size),
//                       p_element,
//                       p_queue->element_size);
//                break;
//        }

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

ret_code_t whim_queue_generic_pop(whim_queue_t const * p_queue,
                                 float              * p_element,
                                 bool                just_peek)
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
//        // Read element.
//        switch (p_queue->element_size)
//        {
//            case sizeof(uint8_t):
//                *((uint8_t *)p_element) = ((uint8_t *)p_queue->p_buffer)[read_pos];
//                break;

//            case sizeof(uint16_t):
//                *((uint16_t *)p_element) = ((uint16_t *)p_queue->p_buffer)[read_pos];
//                break;

//            case sizeof(uint32_t):
//                *((uint32_t *)p_element) = ((uint32_t *)p_queue->p_buffer)[read_pos];
//                break;

//            case sizeof(uint64_t):
//                *((uint64_t *)p_element) = ((uint64_t *)p_queue->p_buffer)[read_pos];
//                break;

//            default:
//                memcpy(p_element,
//                       (void const *)((size_t)p_queue->p_buffer + read_pos * p_queue->element_size),
//                       p_queue->element_size);
//                break;
//        }
    }
    else
    {
        status = NRF_ERROR_NOT_FOUND;
    }

    CRITICAL_REGION_EXIT();

    return status;
}

/**@brief Write elements to the queue. This function assumes that there is enough room in the queue
 *        to write the requested number of elements and that this process will not be interrupted.
 *
 * @param[in]   p_queue             Pointer to the whim_queue_t instance.
 * @param[in]   p_data              Pointer to the buffer with elements to write.
 * @param[in]   element_count       Number of elements to write.
 */
static void queue_write(whim_queue_t const * p_queue, float const * p_data, uint32_t element_count)
{
    size_t prev_available = whim_queue_available_get(p_queue);
    size_t continuous     = p_queue->size + 1 - p_queue->p_cb->back;
    void * p_write_ptr    = (void *)((size_t)p_queue->p_buffer
                          + p_queue->p_cb->back * p_queue->element_size);
    if (element_count <= continuous)
    {
        memcpy(p_write_ptr,
               p_data,
               element_count * p_queue->element_size);

        p_queue->p_cb->back = ((p_queue->p_cb->back + element_count) <= p_queue->size)
                            ? (p_queue->p_cb->back + element_count)
                            : 0;
    }
    else
    {
        size_t first_write_length = continuous * p_queue->element_size;
        memcpy(p_write_ptr,
               p_data,
               first_write_length);

        size_t elements_left = element_count - continuous;
        memcpy(p_queue->p_buffer,
               (void const *)((size_t)p_data + first_write_length),
               elements_left * p_queue->element_size);

        p_queue->p_cb->back = elements_left;
        if (prev_available < element_count)
        {
            // Overwrite the oldest elements.
            p_queue->p_cb->front = whim_queue_next_idx(p_queue, p_queue->p_cb->back);
        }
    }

    // Update utilization.
    size_t utilization = queue_utilization_get(p_queue);
    if (p_queue->p_cb->max_utilization < utilization)
    {
        p_queue->p_cb->max_utilization = utilization;
    }
}

ret_code_t whim_queue_write(whim_queue_t const * p_queue,
                           float const        * p_data,
                           size_t              element_count)
{
    ret_code_t status = NRF_SUCCESS;

    ASSERT(p_queue != NULL);
    ASSERT(p_data != NULL);
    ASSERT(element_count <= p_queue->size);

    if (element_count == 0)
    {
        return NRF_SUCCESS;
    }

    CRITICAL_REGION_ENTER();

    if ((whim_queue_available_get(p_queue) >= element_count)
     || (p_queue->mode == WHIM_QUEUE_MODE_OVERFLOW))
    {
        queue_write(p_queue, p_data, element_count);
    }
    else
    {
        status = NRF_ERROR_NO_MEM;
    }

    CRITICAL_REGION_EXIT();

    return status;
}


size_t whim_queue_in(whim_queue_t const * p_queue,
                    float const        * p_data,
                    size_t              element_count)
{
    ASSERT(p_queue != NULL);
    ASSERT(p_data != NULL);

    if (element_count == 0)
    {
        return 0;
    }

    CRITICAL_REGION_ENTER();

    if (p_queue->mode == WHIM_QUEUE_MODE_OVERFLOW)
    {
        element_count = MIN(element_count, p_queue->size);
    }
    else
    {
        size_t available = whim_queue_available_get(p_queue);
        element_count    = MIN(element_count, available);
    }

    queue_write(p_queue, p_data, element_count);

    CRITICAL_REGION_EXIT();

    return element_count;
}

/**@brief Read elements from the queue. This function assumes that there are enough elements
 *        in the queue to read and that this process will not be interrupted.
 *
 * @param[in]   p_queue             Pointer to the whim_queue_t instance.
 * @param[out]  p_data              Pointer to the buffer where elements will be copied.
 * @param[in]   element_count       Number of elements to read.
 */
static void queue_read(whim_queue_t const * p_queue, float * p_data, uint32_t element_count)
{
    size_t front        = p_queue->p_cb->front;
    size_t back         = p_queue->p_cb->back;
    size_t continuous   = (front <= back) ? (back - front) : (p_queue->size + 1 - front);
    void const * p_read_ptr = (void const *)((size_t)p_queue->p_buffer
                                           + front * p_queue->element_size);

    if (element_count <= continuous)
    {
        memcpy(p_data,
               p_read_ptr,
               element_count * p_queue->element_size);

        p_queue->p_cb->front = ((front + element_count) <= p_queue->size)
                             ? (front + element_count)
                             : 0;
    }
    else
    {
        size_t first_read_length = continuous * p_queue->element_size;
        memcpy(p_data,
               p_read_ptr,
               first_read_length);

        size_t elements_left = element_count - continuous;
        memcpy((void *)((size_t)p_data + first_read_length),
               p_queue->p_buffer,
               elements_left * p_queue->element_size);

        p_queue->p_cb->front = elements_left;
    }
}

ret_code_t whim_queue_read(whim_queue_t const * p_queue,
                          float               * p_data,
                          size_t              element_count)
{
    ret_code_t status = NRF_SUCCESS;

    ASSERT(p_queue != NULL);
    ASSERT(p_data != NULL);

    if (element_count == 0)
    {
        return NRF_SUCCESS;
    }

    CRITICAL_REGION_ENTER();

    if (element_count <= queue_utilization_get(p_queue))
    {
        queue_read(p_queue, p_data, element_count);
    }
    else
    {
        status = NRF_ERROR_NOT_FOUND;
    }

    CRITICAL_REGION_EXIT();

    return status;
}

size_t whim_queue_out(whim_queue_t const * p_queue,
                     float               * p_data,
                     size_t              element_count)
{
    ASSERT(p_queue != NULL);
    ASSERT(p_data != NULL);

    if (element_count == 0)
    {
        return 0;
    }

    CRITICAL_REGION_ENTER();

    size_t utilization = queue_utilization_get(p_queue);
    element_count      = MIN(element_count, utilization);

    queue_read(p_queue, p_data, element_count);

    CRITICAL_REGION_EXIT();

    return element_count;
}

void whim_queue_reset(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);

    CRITICAL_REGION_ENTER();

    memset(p_queue->p_cb, 0, sizeof(whim_queue_cb_t));

    CRITICAL_REGION_EXIT();
}

size_t whim_queue_utilization_get(whim_queue_t const * p_queue)
{
    size_t utilization;
    ASSERT(p_queue != NULL);

    CRITICAL_REGION_ENTER();

    utilization = queue_utilization_get(p_queue);

    CRITICAL_REGION_EXIT();

    return utilization;
}

bool whim_queue_is_empty(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    size_t front    = p_queue->p_cb->front;
    size_t back     = p_queue->p_cb->back;
    return (front == back);
}

size_t whim_queue_available_get(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    return p_queue->size - whim_queue_utilization_get(p_queue);
}

size_t whim_queue_max_utilization_get(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    return p_queue->p_cb->max_utilization;
}

void whim_queue_max_utilization_reset(whim_queue_t const * p_queue)
{
    ASSERT(p_queue != NULL);
    p_queue->p_cb->max_utilization = 0;
}

ret_code_t whim_queue_integrate(whim_queue_t const * p_queue,
															double               * sum,
															double								 dx)
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

