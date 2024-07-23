/*
 * jsn_sr04t_2_0.h
 *
 *  Created on: Apr 25, 2024
 *      Author: trevor
 */

#ifndef JSN_SR04T_2_0_INC_JSN_SR04T_2_0_HPP_
#define JSN_SR04T_2_0_INC_JSN_SR04T_2_0_HPP_

#include "enums.hpp"
#include "time_utils.hpp"
#include "gpio.h"
#include <stdint.h>

class JsnSr04t2
{
public:
	JsnSr04t2(GPIO_TypeDef *triggerPort, uint16_t triggerPin) : m_triggerPort{triggerPort}, m_triggerPin{triggerPin} {};

	// start a distance measurement. returns busy if in the middle of
	// a measurement
	ErrorCode startRanging();

	// Call every time the echo interrupt pin transitions
	void interruptHandler();

	// is a new distance ready
	bool newDistanceReady();
	// get the latest distance data. un-sets new distance ready as it
	// returns the new distance data
	float getLatestDistance_in();

private:
	static constexpr uint16_t SPEED_SOUND_M_PER_S = 340;

	void updateDistance(uint32_t pulse_us);

	enum class State : uint8_t
	{
		INIT,
		WAIT_FOR_PULSE_START,
		WAIT_FOR_PULSE_END,
		NEW_DATA_READY,
	} m_state = State::INIT;

	Stopwatch m_pulseTimer;

	GPIO_TypeDef *m_triggerPort;
	uint16_t m_triggerPin;

	float m_distance_in = 0;

	// 38ms is max signal duration
	static constexpr uint16_t MEASUREMENT_TIMEOUT_MS = 50;
	Stopwatch measurementTimeoutTimer;
};

#endif /* JSN_SR04T_2_0_INC_JSN_SR04T_2_0_HPP_ */
