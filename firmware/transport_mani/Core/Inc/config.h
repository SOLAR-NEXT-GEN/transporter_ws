/*
 * config.h
 *
 *  Created on: Jul 17, 2025
 *      Author: b
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"
#include "gpio.h"
#include <string.h>
#include "math.h"
#include "PWM.h"
#include "Cytron_MDXX.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// front left linear actuator
extern TIM_HandleTypeDef htim3;
extern MDXX motor1;
#define MOTOR1_TIM &htim3
#define MOTOR1_TIM_CH TIM_CHANNEL_1
#define MOTOR1_GPIOx GPIOB
#define MOTOR1_GPIO_Pin GPIO_PIN_8

// back left linear actuator
extern TIM_HandleTypeDef htim3;
extern MDXX motor2;
#define MOTOR1_TIM &htim3
#define MOTOR1_TIM_CH TIM_CHANNEL_2
#define MOTOR1_GPIOx GPIOB
#define MOTOR1_GPIO_Pin GPIO_PIN_9

void tarnsport_mani_begin();

#endif /* INC_CONFIG_H_ */
