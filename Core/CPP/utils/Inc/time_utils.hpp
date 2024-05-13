/*
 * time_utils.hpp
 *
 *  Created on: Apr 26, 2024
 *      Author: trevor
 */

#ifndef CPP_UTILS_INC_TIME_UTILS_HPP_
#define CPP_UTILS_INC_TIME_UTILS_HPP_

#include "stm32l4xx_hal.h"
#include "tim.h"
#include <enums.hpp>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern uint32_t g_uptimeHours;

struct TimeHoursUs
{
    uint32_t hours;
    uint32_t us;
};

TimeHoursUs getUptime();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

class Stopwatch
{
public:
    void reset()
    {
        m_startTime = getUptime();
    }
    void start() { reset(); }

    TimeHoursUs getTime()
    {
        TimeHoursUs currTime = getUptime();
        return {currTime.hours - m_startTime.hours, currTime.us - m_startTime.us};
    }

private:
    TimeHoursUs m_startTime = {0, 0};
};

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/**
 * Sleeps for the specified time in us.
 *
 * @param usecs
 *      The number of us to sleep.
 */
ErrorCode sleepUs(uint32_t usecs);

/**
 * Sleeps for the specified time in ms
 *
 * @param msecs
 *      The number of ms to sleep.
 */
ErrorCode sleepMs(uint32_t msecs);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif // CPP_UTILS_INC_TIME_UTILS_HPP_
