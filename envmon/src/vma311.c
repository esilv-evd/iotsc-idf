/*
 * BSD 2-Clause License
 * 
 * Copyright (c) 2021, ESILV
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file vma311.c
 * @author Frédéric Fauberteau
 * @brief Implementation of VMA311 (based on DHT11) sensor driver.
 * 
 * This code is inspired by:
 * @see https://github.com/0nism/esp32-DHT11/blob/master/dht11.c
 */

#include "vma311.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static vma311_t vma311;

static        void            vma311_send_start_signal();
static        int             vma311_wait(uint16_t, int);
static        int             vma311_check_response();
static inline vma311_status_t vma311_read_byte(uint8_t *);
static        vma311_status_t vma311_check_crc(uint8_t *);

void vma311_init(gpio_num_t num)
{
    vma311.num = num;
    vma311.last_read_time = -2000000;
    gpio_reset_pin(vma311.num);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

vma311_data_t vma311_get_values()
{
    vma311_data_t error_data = {VMA311_TIMEOUT_ERROR, -1, -1, -1, -1};
    uint8_t data[5] = {0, 0, 0, 0, 0};

    if (esp_timer_get_time() - 2000000 < vma311.last_read_time)
    {
        return vma311.data;
    }
    vma311.last_read_time = esp_timer_get_time();
    vma311_send_start_signal();
    if (vma311_check_response() == VMA311_TIMEOUT_ERROR)
    {
        return error_data;
    }
    for (int i = 0; i < 5; ++i)
    {
        if (vma311_read_byte(&data[i]))
        {
            return error_data;
        }
    }
    if(vma311_check_crc(data) != VMA311_CRC_ERROR)
    {
        vma311.data.status = VMA311_OK;
        vma311.data.rh_int = data[0];
        vma311.data.rh_dec = data[1];
        vma311.data.t_int = data[2];
        vma311.data.t_dec = data[3];
    }
    return vma311.data;
}

static void vma311_send_start_signal()
{
    gpio_set_direction(vma311.num, GPIO_MODE_OUTPUT);
    gpio_set_level(vma311.num, 0);
    ets_delay_us(20 * 1000);
    gpio_set_level(vma311.num, 1);
    ets_delay_us(40);
}

static int vma311_wait(uint16_t us, int level)
{
    int us_ticks = 0;

    while(gpio_get_level(vma311.num) == level)
    { 
        if(us_ticks++ > us) 
        {
            return VMA311_TIMEOUT_ERROR;
        }
        ets_delay_us(1);
    }
    return us_ticks;
}

static int vma311_check_response()
{
    gpio_set_direction(vma311.num, GPIO_MODE_INPUT);
    if(vma311_wait(80, 0) == VMA311_TIMEOUT_ERROR)
    {
        return VMA311_TIMEOUT_ERROR;
    }
    if(vma311_wait(80, 1) == VMA311_TIMEOUT_ERROR) 
    {
        return VMA311_TIMEOUT_ERROR;
    }
    return VMA311_OK;
}

static vma311_status_t vma311_read_byte(uint8_t *byte)
{
    for (int i = 0; i < 8; ++i)
    {
        if(vma311_wait(50, 0) == VMA311_TIMEOUT_ERROR)
        {
            return VMA311_TIMEOUT_ERROR;
        }
        if(vma311_wait(70, 1) > 28)
        {
            *byte |= (1 << (7 - i));
        }
    }
    return VMA311_OK;
}

static vma311_status_t vma311_check_crc(uint8_t data[]) {
    if(data[4] != (data[0] + data[1] + data[2] + data[3]))
    {
        return VMA311_CRC_ERROR;
    }
    return VMA311_OK;
}