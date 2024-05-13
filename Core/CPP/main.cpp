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
#include "time_utils.hpp"

class MainClass
{
public:
	MainClass() : distanceSensor1{DistanceTrigger1_GPIO_Port, DistanceTrigger1_Pin}, distanceSensor2{DistanceTrigger2_GPIO_Port, DistanceTrigger2_Pin}
	{
		distanceIntervalTimer.start();
	}

	// service function called every period
	void service()
	{
//		sprintf(buffer, "distance ready 1: %.6f\r\n", distanceSensor1.getLatestDistance_in());
		distanceSensor1.startRanging();
		if (distanceSensor1.newDistanceReady())
		{
			distanceMeasurementCount1++;
			distanceSum1 += distanceSensor1.getLatestDistance_in();
		}
		distanceSensor2.startRanging();
		if (distanceSensor2.newDistanceReady())
				{
					distanceMeasurementCount2++;
					distanceSum2 += distanceSensor2.getLatestDistance_in();
				}
		if (distanceIntervalTimer.getTime().us >= SAMPLE_PERIOD_MS*1000) {
			distanceIntervalTimer.start();
			char buffer[100];
			sprintf(buffer, "distance 1: %.4f, sample count = %lu\tdistance 2: %.4f, sample count = %lu\r\n", distanceSum1/distanceMeasurementCount1, distanceMeasurementCount1,distanceSum2/distanceMeasurementCount2, distanceMeasurementCount2);
			HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
			distanceSum1=0;
			distanceMeasurementCount1=0;
			distanceSum2=0;
						distanceMeasurementCount2=0;
		}
	}

	void interruptCallback(uint16_t gpioPin)
	{
		char buffer[100];
		switch (gpioPin)
		{
		case DistanceEcho1_Pin:
			// signal distance sensor 1 interrupt
			distanceSensor1.interruptHandler();
			break;
		case DistanceEcho2_Pin:
			// signal distance sensor 2 interrupt
			distanceSensor2.interruptHandler();
			break;
		default:
			sprintf(buffer, "Default: %u\n", gpioPin);
			HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), 1000);
			break;
		}
	}

private:
	// distance sensing
	JsnSr04t2 distanceSensor1;
	JsnSr04t2 distanceSensor2;
	uint32_t distanceMeasurementCount1 = 0;
	float distanceSum1 = 0;
	uint32_t distanceMeasurementCount2 = 0;
	float distanceSum2 = 0;
	static constexpr uint32_t SAMPLE_PERIOD_MS = 250;
	Stopwatch distanceIntervalTimer;
} g_mainClass;

void CppMain()
{
	while (1)
	{
		g_mainClass.service();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	g_mainClass.interruptCallback(GPIO_Pin);
}
