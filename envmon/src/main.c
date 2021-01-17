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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "led.h"
#include "mcp9700.h"
#include "vma311.h"
#include "bme680.h"

/**
 * These macros define characteristics of devices
 */
#define BUILTIN_LED_GPIO    GPIO_NUM_2
#define MCP9700_ADC_UNIT    ADC_UNIT_1
#define MCP9700_ADC_CHANNEL ADC_CHANNEL_4
#define VMA311_GPIO         GPIO_NUM_5

/**
 * This function is the main function of the application.
 */
void app_main()
{
    uint32_t mcp_temp;
    vma311_data_t vma311_data;

    /* Device initialization */
    led_init(BUILTIN_LED_GPIO);
    mcp9700_init(MCP9700_ADC_UNIT, MCP9700_ADC_CHANNEL);
    vma311_init(VMA311_GPIO);
    bme680_init();
    while(1)
    {
        /* Data collection */
        mcp_temp = mcp9700_get_value();
        printf("mcp9700:temp:%d\n", mcp_temp);
        vma311_data = vma311_get_values();
        if (vma311_data.status == VMA311_OK)
        {
            printf("vma311:temp:%d.%d\n", vma311_data.t_int, vma311_data.t_dec);
            printf("vma311:humidity:%d.%d\n", vma311_data.rh_int, vma311_data.rh_dec);
        }
        else
        {
            printf("vma311:error\n");
        }
        bme680_get_values();
        printf("bme680:\n");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
