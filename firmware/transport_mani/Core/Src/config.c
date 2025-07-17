/*
 * config.c
 *
 *  Created on: Jul 17, 2025
 *      Author: b
 */
#include <config.h>

MDXX FL_motor;
MDXX BL_motor;

void tarnsport_mani_begin() {

	MDXX_GPIO_init(&FL_motor, MOTOR1_TIM, MOTOR1_TIM_CH, MOTOR1_GPIOx, MOTOR1_GPIO_Pin);
	MDXX_GPIO_init(&BL_motor, MOTOR2_TIM, MOTOR2_TIM_CH, MOTOR2_GPIOx, MOTOR2_GPIO_Pin);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}
