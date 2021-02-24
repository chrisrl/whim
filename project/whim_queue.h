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
/**
* @defgroup whim_queue Queue module
* @{
* @ingroup app_common
* @brief Functions that handle the queue instances.
*/

#ifndef WHIM_QUEUE_H__
#define WHIM_QUEUE_H__

#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include "nrf_assert.h"
#include "sdk_errors.h"
#include "app_util.h"
#include "app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Queue control block. */
typedef struct
{
    volatile size_t front;          //!< Queue front index.
    volatile size_t back;           //!< Queue back index.
    size_t max_utilization;         //!< Maximum utilization of the queue.
} whim_queue_cb_t;

/**@brief Supported queue modes. */
typedef enum
{
    WHIM_QUEUE_MODE_OVERFLOW,        //!< If the queue is full, new element will overwrite the oldest.
    WHIM_QUEUE_MODE_NO_OVERFLOW,     //!< If the queue is full, new element will not be accepted.
} whim_queue_mode_t;

/**@brief Instance of the queue. */
typedef struct
{
    whim_queue_cb_t * p_cb;          //!< Pointer to the instance control block.
    float           * p_buffer;      //!< Pointer to the memory that is used as storage.
    size_t           size;          //!< Size of the queue.
    size_t           element_size;  //!< Size of one element.
    whim_queue_mode_t mode;          //!< Mode of the queue.
} whim_queue_t;

/**@brief Create a queue instance.
 *
 * @note  This macro reserves memory for the given queue instance.
 *
 * @param[in]   _name       Name of the queue.
 * @param[in]   _size       Size of the queue.
 * @param[in]   _mode       Mode of the queue.
 */
#define WHIM_QUEUE_DEF(_name, _size, _mode)                             \
    static float             CONCAT_2(_name, _whim_queue_buffer[(_size) + 1]); \
    static whim_queue_cb_t    CONCAT_2(_name, _whim_queue_cb);                  \
    static const whim_queue_t _name =                                          \
        {                                                                     \
            .p_cb           = &CONCAT_2(_name, _whim_queue_cb),                \
            .p_buffer       = CONCAT_2(_name,_whim_queue_buffer),              \
            .size           = (_size),                                        \
            .element_size   = sizeof(float),                                  \
            .mode           = _mode,                                          \
        }

/**@brief Declare a queue interface.
 *
 * @param[in]   _name    Name of the queue.
 */
#define WHIM_QUEUE_INTERFACE_DEC(_name)               \
    ret_code_t  _name##_push(float const * p_element);      \
    ret_code_t  _name##_pop(float * p_element);             \
    ret_code_t  _name##_peek(float * p_element);            \
    ret_code_t  _name##_write(float const * p_data,         \
                              size_t        element_count); \
    ret_code_t  _name##_read(float * p_data,                \
                             size_t  element_count);        \
    size_t      _name##_out(float * p_data,                 \
                            size_t  element_count);         \
    size_t      _name##_in(float const * p_data,            \
                            size_t element_count);          \
    bool        _name##_is_full(void);                      \
    bool        _name##_is_empty(void);                     \
    size_t      _name##_utilization_get(void);              \
    size_t      _name##_available_get(void);                \
    size_t      _name##_max_utilization_get(void);          \
    void        _name##_reset(void)

/**@brief Define a queue interface.
 *
 * @param[in]   _name    Name of the queue.
 * @param[in]   _p_queue Queue instance.
 */
#define WHIM_QUEUE_INTERFACE_DEF(_name, _p_queue)                 \
    ret_code_t _name##_push(float const * p_element)                    \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        ASSERT((_p_queue)->element_size == sizeof(float));              \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_push((_p_queue), p_element);                   \
    }                                                                   \
    ret_code_t _name##_pop(float * p_element)                           \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        ASSERT((_p_queue)->element_size == sizeof(float));              \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_pop((_p_queue), p_element);                    \
    }                                                                   \
    ret_code_t _name##_peek(float * p_element)                          \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        ASSERT((_p_queue)->element_size == sizeof(float));              \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_peek((_p_queue), p_element);                   \
    }                                                                   \
    ret_code_t _name##_write(float const * p_data,                      \
                             size_t        element_count)               \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        ASSERT((_p_queue)->element_size == sizeof(float));              \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_write((_p_queue), p_data, element_count);      \
    }                                                                   \
    ret_code_t _name##_read(float * p_data,                             \
                            size_t  element_count)                      \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        ASSERT((_p_queue)->element_size == sizeof(float));              \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_read((_p_queue), p_data, element_count);       \
    }                                                                   \
    size_t _name##_in(float const * p_data,                             \
                      size_t  element_count)                            \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        ASSERT((_p_queue)->element_size == sizeof(float));              \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_in((_p_queue), p_data, element_count);         \
    }                                                                   \
    size_t _name##_out(float * p_data,                                  \
                       size_t  element_count)                           \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        ASSERT((_p_queue)->element_size == sizeof(float));              \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_out((_p_queue), p_data, element_count);        \
    }                                                                   \
    bool _name##_is_full(void)                                          \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        return whim_queue_is_full(_p_queue);                             \
        GCC_PRAGMA("GCC diagnostic pop")                                \
    }                                                                   \
    bool _name##_is_empty(void)                                         \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_is_empty(_p_queue);                            \
    }                                                                   \
    size_t _name##_utilization_get(void)                                \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_utilization_get(_p_queue);                     \
    }                                                                   \
    size_t _name##_available_get(void)                                  \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_available_get(_p_queue);                       \
    }                                                                   \
    size_t _name##_max_utilization_get(void)                            \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        return whim_queue_max_utilization_get(_p_queue);                 \
    }                                                                   \
    void _name##_reset(void)                                            \
    {                                                                   \
        GCC_PRAGMA("GCC diagnostic push")                               \
        GCC_PRAGMA("GCC diagnostic ignored \"-Waddress\"")              \
        ASSERT((_p_queue) != NULL);                                     \
        GCC_PRAGMA("GCC diagnostic pop")                                \
        whim_queue_reset(_p_queue);                                      \
    }


#define whim_queue_pop(_p_queue, _p_element) whim_queue_generic_pop((_p_queue), (_p_element), false)

void whim_queue_reset(whim_queue_t const * p_queue);
ret_code_t whim_queue_push(whim_queue_t const * p_queue, float const * p_element);
ret_code_t whim_queue_generic_pop(whim_queue_t const * p_queue, float * p_element, bool just_peek);
ret_code_t whim_queue_integrate(whim_queue_t const * p_queue, float * sum, double dx);

#ifdef __cplusplus
}
#endif

#endif // WHIM_QUEUE_H__
/** @} */
