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

#include <stdlib.h>
#include "mcp9700.h"

static mcp9700_t mcp9700;

void mcp9700_init(adc_unit_t unit, adc_channel_t channel)
{
    mcp9700.unit = unit;
    mcp9700.channel = channel;
    if (mcp9700.unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH);
        adc1_config_channel_atten(mcp9700.channel, ADC_ATTEN);
    }
    else /* mcp9700.unit == ADC_UNIT_2 */
    {
        adc2_config_channel_atten(mcp9700.channel, ADC_ATTEN);
    }
    esp_adc_cal_characterize(mcp9700.unit, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, &mcp9700.adc_chars);
}

int32_t mcp9700_get_value()
{
    uint32_t adc_reading = 0;
    uint32_t voltage;

    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (mcp9700.unit == ADC_UNIT_1)
        {
            adc_reading += adc1_get_raw(mcp9700.channel);
        }
        else /* mcp9700.unit == ADC_UNIT_2 */
        {
            int raw;
            adc2_get_raw((adc2_channel_t)mcp9700.channel, ADC_WIDTH, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    voltage = esp_adc_cal_raw_to_voltage(adc_reading, &mcp9700.adc_chars);
    return voltage - 500;
}
