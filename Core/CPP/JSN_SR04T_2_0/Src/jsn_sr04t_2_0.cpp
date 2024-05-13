/*
 * jsn_sr04t_2_0.cpp
 *
 *  Created on: Apr 25, 2024
 *      Author: trevor
 */

#include "jsn_sr04t_2_0.hpp"

ErrorCode JsnSr04t2::startRanging()
{
    if (m_state != State::INIT)
    {
        return ErrorCode::BUSY;
    }

    // trigger ranging sample
    HAL_GPIO_WritePin(m_triggerPort, m_triggerPin, GPIO_PIN_SET);
    sleepUs(10);
    HAL_GPIO_WritePin(m_triggerPort, m_triggerPin, GPIO_PIN_RESET);

    measurementTimeoutTimer.start();

    m_state = State::WAIT_FOR_PULSE_START;
    return ErrorCode::OKAY;
}

void JsnSr04t2::interruptHandler()
{
    TimeHoursUs time;
    switch (m_state)
    {
    case State::WAIT_FOR_PULSE_START:
        m_pulseTimer.start();
        m_state = State::WAIT_FOR_PULSE_END;
        break;

    case State::WAIT_FOR_PULSE_END:
        time = m_pulseTimer.getTime();
        if (time.hours == 0) // should not pulse for longer than an hour
        {
            updateDistance(time.us);
        }
        m_state = State::NEW_DATA_READY;
        break;

    case State::INIT:
    case State::NEW_DATA_READY:
    default:
        break;
    }
}

bool JsnSr04t2::newDistanceReady() {
    // check timeout
    if ((m_state == State::WAIT_FOR_PULSE_START || m_state == State::WAIT_FOR_PULSE_END) && measurementTimeoutTimer.getTime().us >= MEASUREMENT_TIMEOUT_MS*1000) {
    	m_state = State::INIT;
    }
	return m_state == State::NEW_DATA_READY;
}

float JsnSr04t2::getLatestDistance_in()
{
    // reset state machine if reading new data
    if (m_state==State::NEW_DATA_READY) {
        m_state=State::INIT;
    }
    return m_distance_in;
}

void JsnSr04t2::updateDistance(uint32_t pulse_us)
{
	m_distance_in = pulse_us / 148.0;
//    m_distance_in = pulse_us * SPEED_SOUND_M_PER_S / 2 / 1000;
}
