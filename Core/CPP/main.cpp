/*
 * main.cpp
 *
 *  Created on: Apr 25, 2024
 *      Author: trevor
 */

#include "main.h"
#include "gpio.h"
#include "usart.h"
#include <cstdio>
#include <cstring>
#include <iostream>

#include <jsn_sr04t_2_0.hpp>
#include "nrf24l01p.hpp"
#include <time_utils.hpp>

class MainClass
{
public:
	// init function runs after peripheral initialization
	void init()
	{
		radioDriver.tx_init(2500, _1Mbps);
		m_state = State::WAIT_FOR_START_ACK;
		blinkTimer.start();
	}

	// service function called every period
	void service()
	{
		switch (m_state)
		{
		case State::WAIT_FOR_START_ACK:
			// transmit start packet
			DistanceSensorPacket_t pkt;
			memcpy(&pkt, &syncPacket, sizeof(DistanceSensorPacket_t));
			radioDriver.tx_transmit(reinterpret_cast<uint8_t *>(&pkt));
			sleepUs(500); // distance node must receive within 10 ms of wakeup
			break;
		case State::RECEIVING_DISTANCES:
			sleepMs(100); // interrupt based
			// timeout if distance not received in a while
			if (lastRxPacketTimer.getTime().us == DISTANCE_NOT_RECEIVED_TIMEOUT_MS * 1000)
			{
				// assume distance node turned off
				m_state = State::WAIT_FOR_START_ACK;
				radioDriver.flush_rx_fifo();
				radioDriver.power_down();
				radioDriver.tx_init(2500, _1Mbps);
			}
			break;
		}
		if (blinkTimer.getTime().us >= BLINK_PERIOD_US) {
			blinkTimer.reset();
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}

	void interruptCallback(uint16_t gpioPin)
	{
		char buffer[100];
		switch (gpioPin)
		{
		case NRF_IRQ_Pin:
			if (m_state == State::WAIT_FOR_START_ACK)
			{
				if (isOk(radioDriver.tx_irq())) {
				// ack received
				m_state = State::RECEIVING_DISTANCES;
				radioDriver.flush_tx_fifo();
				radioDriver.power_down();
				radioDriver.rx_init(2500, _1Mbps);
				lastRxPacketTimer.start();
				}
			}
			else if (m_state == State::RECEIVING_DISTANCES)
			{
				radioDriver.rx_receive(reinterpret_cast<uint8_t *>(&latestPacket));
				sprintf(buffer, "Received Distance: %.4f, Sensor: %u\r\n", latestPacket.distance_in, latestPacket.distanceSensorNum);
				HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
				lastRxPacketTimer.start();
			}
			break;

		default:
			sprintf(buffer, "Default: %u\r\n", gpioPin);
			HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
			break;
		}
	}

private:
	static constexpr uint32_t BLINK_PERIOD_US = 500*1000;
	Stopwatch blinkTimer;

	nrf24l01p radioDriver;

	DistanceSensorPacket_t latestPacket = {0, 0};

	static constexpr uint32_t DISTANCE_NOT_RECEIVED_TIMEOUT_MS = 5000;
	Stopwatch lastRxPacketTimer;

	enum class State : uint8_t
	{
		WAIT_FOR_START_ACK,	 // wait for the ack to the start command
		RECEIVING_DISTANCES, // receive distance measurements
	} m_state = State::WAIT_FOR_START_ACK;
} g_mainClass;

void CppMain()
{
	g_mainClass.init();
	while (1)
	{
		g_mainClass.service();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	g_mainClass.interruptCallback(GPIO_Pin);
}
