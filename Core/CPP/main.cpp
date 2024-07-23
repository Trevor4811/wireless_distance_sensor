/*
 * main.cpp
 *
 *  Created on: Apr 25, 2024
 *      Author: trevor
 */

#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "lptim.h"
#include <cstdio>
#include <cstring>
#include <iostream>

#include <jsn_sr04t_2_0.hpp>
#include <nrf24l01p.hpp>
#include <time_utils.hpp>

class MainClass
{
public:
	MainClass() : distanceSensors{{DistanceTrigger1_GPIO_Port, DistanceTrigger1_Pin}}, distanceSensorEchoPins{DistanceEcho1_Pin}
	{
		// WARNING: Runs before HAL initialization!!!
	}

	// init function runs after peripheral initialization
	void init()
	{
		distanceIntervalTimer.start();
		radioDriver.rx_init(2500, _1Mbps);
		m_state = State::WAIT_FOR_RX;
		enterLowPowerTimer.start();
	}

	// service function called every period
	void service()
	{
		if (blinkTimer.getTime().us >= BLINK_PERIOD_US) {
			blinkTimer.reset();
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}

		switch (m_state)
		{
		case State::WAIT_FOR_RX:
			serviceWaitForRx();
			break;
		case State::MEASUREMENT:
			serviceMeasurement();
			break;
		case State::ENTER_STOP1:
			serviceEnterStop1();
			break;
		}
	}

	// wait for the signal to start ranging
	void serviceWaitForRx()
	{
		// check if latest packet was a start command
		if (latestPacket == syncPacket)
		{
			radioDriver.flush_rx_fifo();
			radioDriver.power_down();
			radioDriver.tx_init(2500, _1Mbps);
			distanceIntervalTimer.start();
			enterLowPowerTimer.start();
			m_state = State::MEASUREMENT;
			return;
		}

		// check for timeout
		if (enterLowPowerTimer.getTime().us > RX_NOT_RECEIVED_PERIOD_MS * 1000)
		{
			m_state = State::ENTER_STOP1;
			return;
		}
	}

	// do ranging measurements
	void serviceMeasurement()
	{
		// collect distance sample
		for (uint8_t i = 0; i < NUM_DISTANCE_SENSORS; i++)
		{
			distanceSensors[i].startRanging();
			if (distanceSensors[i].newDistanceReady())
			{
				distanceMeasurementCounts[i]++;
				distanceSums[i] += distanceSensors[i].getLatestDistance_in();
			}
		}
		// publish average distance over the sample period
		if (distanceIntervalTimer.getTime().us >= SAMPLE_PERIOD_MS * 1000)
		{
			distanceIntervalTimer.start();
			char buffer[100];
			for (uint8_t i = 0; i < NUM_DISTANCE_SENSORS; i++)
			{
				float avg_dist = distanceSums[i] / distanceMeasurementCounts[i];
				sprintf(buffer, "distance %u: %.4f, sample count = %lu\r\n", i, avg_dist, distanceMeasurementCounts[i]);
				HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
				DistanceSensorPacket_t pkt = {i, avg_dist};
				radioDriver.tx_transmit(reinterpret_cast<uint8_t *>(&pkt));
				distanceSums[i] = 0;
				distanceMeasurementCounts[i] = 0;
			}
		}

		// check for timeout
		if (enterLowPowerTimer.getTime().us > RANGING_TIMEOUT_PERIOD_MS * 1000)
		{
			m_state = State::ENTER_STOP1;
			return;
		}
	}

	// enter low power mode
	void serviceEnterStop1()
	{
		char buffer[] = "Sleep\r\n";
		HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
		sleepMs(1);

		// power down radio
		radioDriver.flush_tx_fifo();
		radioDriver.power_down();
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

		// distance sensors run based on trigger pin, no need to power down

		// setup wake interrupt and enter stm32 into low power mode
	    // Set the timeout register to 10 seconds
	    if (HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 0xFFFF, WAKEUP_PERIOD_S * 250) != HAL_OK)
	    {
	        Error_Handler();
	    }
		HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
		// Wait for interrupt to reset processor and reinitialize everything
		while(1);
	}

	void interruptCallback(uint16_t gpioPin)
	{
//		char buffer[100];

		// check distance sensor echo pins
		if (m_state == State::MEASUREMENT)
					{
		for (uint8_t i = 0; i < NUM_DISTANCE_SENSORS; i++)
		{
			if (gpioPin == distanceSensorEchoPins[i])
			{
				distanceSensors[i].interruptHandler();
			}
		}
					}

		// check other interrupt pins
		switch (gpioPin)
		{
		case NRF_IRQ_Pin:
			if (m_state == State::MEASUREMENT)
			{
				radioDriver.tx_irq();
			}
			else if (m_state == State::WAIT_FOR_RX)
			{
				radioDriver.rx_receive(reinterpret_cast<uint8_t *>(&latestPacket));
				sleepMs(10);
			}
			break;
		default:
			// TODO figure out why triggering
			// 512=0x200 is 2nd distance sensor IRQ
//			sprintf(buffer, "Default: %u\r\n", gpioPin);
//			HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
			break;
		}
	}

private:
	static constexpr uint32_t BLINK_PERIOD_US = 500*1000;
	Stopwatch blinkTimer;

	// distance sensing
	static constexpr uint8_t NUM_DISTANCE_SENSORS = 1;
	JsnSr04t2 distanceSensors[NUM_DISTANCE_SENSORS];
	uint16_t distanceSensorEchoPins[NUM_DISTANCE_SENSORS];
	uint32_t distanceMeasurementCounts[NUM_DISTANCE_SENSORS] = {0};
	float distanceSums[NUM_DISTANCE_SENSORS] = {0};
	static constexpr uint32_t SAMPLE_PERIOD_MS = 250;
	Stopwatch distanceIntervalTimer;

	nrf24l01p radioDriver;

	// how long to wait for rx start ranging command before entering low-power mode
	static constexpr uint32_t RX_NOT_RECEIVED_PERIOD_MS = 1000;
	// how long to range for before going to sleep
	static constexpr uint32_t RANGING_TIMEOUT_PERIOD_MS = 300'000;
	Stopwatch enterLowPowerTimer;

	// how long to be in low power-mode before waking up and checking for start ranging command
	static constexpr uint32_t WAKEUP_PERIOD_S = 10;

	DistanceSensorPacket_t latestPacket = {0, 0};

	enum class State : uint8_t
	{
		WAIT_FOR_RX, // wait for the start ranging command
		MEASUREMENT, // measure and transmit distances for set amount of time
		ENTER_STOP1, // enter power save for a set amount of time
	} m_state = State::WAIT_FOR_RX;

} g_mainClass;

void CppMain()
{
	g_mainClass.init();
	char buffer[] = "Init\r\n";
			HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
			sleepMs(1);
	while (1)
	{
		g_mainClass.service();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	g_mainClass.interruptCallback(GPIO_Pin);
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	// reset ensures everything initializes properly
	NVIC_SystemReset();
}
