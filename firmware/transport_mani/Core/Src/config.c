/*
 * config.c
 *
 *  Created on: Jul 17, 2025
 *      Author: b
 */
#include <config.h>

MDXX motor1;
MDXX motor2;

void tarnsport_mani_begin() {
	MDXX_GPIO_init(&motor1, MOTOR1_TIM, MOTOR1_TIM_CH, MOTOR1_GPIOx, MOTOR1_GPIO_Pin);
	MDXX_GPIO_init(&motor2, MOTOR2_TIM, MOTOR2_TIM_CH, MOTOR2_GPIOx, MOTOR2_GPIO_Pin);
}
