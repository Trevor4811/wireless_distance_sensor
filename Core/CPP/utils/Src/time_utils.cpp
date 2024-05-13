/*
 * time_utils.cpp
 *
 *  Created on: Apr 26, 2024
 *      Author: trevor
 */

#include "time_utils.hpp"
#include <stm32l4xx_hal.h>

// blood line will end before this overflows
uint32_t g_uptimeHours = 0;

TimeHoursUs getUptime()
{
    return {g_uptimeHours, __HAL_TIM_GET_COUNTER(&htim2)};
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    g_uptimeHours++;
}

ErrorCode sleepUs(uint32_t usecs)
{
    if (usecs >= 1000)
    {
        sleepMs(usecs / 1000);
    }
    uint32_t start_cycle = SysTick->VAL;
    usecs = usecs % 1000;
    uint32_t cycles_to_sleep = usecs * ((SysTick->LOAD + 1) / 1000);

    if (cycles_to_sleep > start_cycle)
    {
        uint32_t target_cycle = SysTick->LOAD - (cycles_to_sleep - start_cycle - 1);
        while (SysTick->VAL <= start_cycle || SysTick->VAL >= target_cycle)
        {
        }
    }
    else
    {
        uint32_t target_cycle = start_cycle - cycles_to_sleep;
        while (SysTick->VAL <= start_cycle && SysTick->VAL >= target_cycle)
        {
        }
    }
    return ErrorCode::OKAY;
}

ErrorCode sleepMs(uint32_t msecs)
{
    uint32_t start_cycle = SysTick->VAL;
    while (msecs > 0)
    {
    	// delay 1 ms
    	uint32_t startTick = HAL_GetTick();
    	    uint32_t tick = startTick;
    	    uint32_t cycle = SysTick->VAL;
    	    while ((cycle < start_cycle) && (tick == startTick))
    	    {
    	        cycle = SysTick->VAL;
    	        tick = HAL_GetTick();
    	    }
    	    startTick = HAL_GetTick();
    	    tick = startTick;
    	    cycle = SysTick->VAL;
    	    while ((cycle > start_cycle) && (tick == startTick))
    	    {
    	        cycle = SysTick->VAL;
    	        tick = HAL_GetTick();
    	    }
        --msecs;
    }
    return ErrorCode::OKAY;
}
