/*
 * config.c
 *
 *  Manipulator GPIO driver. See config.h for the truth table.
 */
#include "config.h"

typedef struct {
	GPIO_TypeDef *up_port;
	uint16_t up_pin;
	GPIO_TypeDef *down_port;
	uint16_t down_pin;
} ManiSidePins;

static const ManiSidePins side_pins[MANI_SIDE_COUNT] = {
	[MANI_LEFT]  = { LEFT_UP_GPIO_Port,  LEFT_UP_Pin,  LEFT_DOWN_GPIO_Port,  LEFT_DOWN_Pin  },
	[MANI_RIGHT] = { RIGHT_UP_GPIO_Port, RIGHT_UP_Pin, RIGHT_DOWN_GPIO_Port, RIGHT_DOWN_Pin },
};

static volatile int8_t side_cmd[MANI_SIDE_COUNT] = { MANI_CMD_HOLD, MANI_CMD_HOLD };

static void init_output(GPIO_TypeDef *port, uint16_t pin) {
	GPIO_InitTypeDef init = { 0 };
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	init.Pin = pin;
	init.Mode = GPIO_MODE_OUTPUT_PP;
	init.Pull = GPIO_PULLDOWN;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(port, &init);
}

static void apply_outputs(void) {
	uint8_t any_moving = 0;

	/* Release every active output first so UP and DOWN of a side are never
	 * high at the same time, even for one instruction while switching. */
	for (int s = 0; s < MANI_SIDE_COUNT; s++) {
		if (side_cmd[s] != MANI_CMD_UP)
			HAL_GPIO_WritePin(side_pins[s].up_port, side_pins[s].up_pin, GPIO_PIN_RESET);
		if (side_cmd[s] != MANI_CMD_DOWN)
			HAL_GPIO_WritePin(side_pins[s].down_port, side_pins[s].down_pin, GPIO_PIN_RESET);
		if (side_cmd[s] != MANI_CMD_HOLD)
			any_moving = 1;
	}

	if (!any_moving) {
		HAL_GPIO_WritePin(PUMP_EN_GPIO_Port, PUMP_EN_Pin, GPIO_PIN_RESET);
		return;
	}

	HAL_GPIO_WritePin(PUMP_EN_GPIO_Port, PUMP_EN_Pin, GPIO_PIN_SET);
	for (int s = 0; s < MANI_SIDE_COUNT; s++) {
		if (side_cmd[s] == MANI_CMD_UP)
			HAL_GPIO_WritePin(side_pins[s].up_port, side_pins[s].up_pin, GPIO_PIN_SET);
		else if (side_cmd[s] == MANI_CMD_DOWN)
			HAL_GPIO_WritePin(side_pins[s].down_port, side_pins[s].down_pin, GPIO_PIN_SET);
	}
}

void transport_mani_begin(void) {
	/* Clocks for every port that may be used by the placeholder pins. */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	init_output(PUMP_EN_GPIO_Port, PUMP_EN_Pin);
	for (int s = 0; s < MANI_SIDE_COUNT; s++) {
		init_output(side_pins[s].up_port, side_pins[s].up_pin);
		init_output(side_pins[s].down_port, side_pins[s].down_pin);
	}
	mani_hold_all();
}

void mani_set(ManiSide side, int8_t cmd) {
	if (side >= MANI_SIDE_COUNT)
		return;
	if (cmd != MANI_CMD_UP && cmd != MANI_CMD_DOWN)
		cmd = MANI_CMD_HOLD;
	side_cmd[side] = cmd;
	apply_outputs();
}

void mani_hold_all(void) {
	side_cmd[MANI_LEFT] = MANI_CMD_HOLD;
	side_cmd[MANI_RIGHT] = MANI_CMD_HOLD;
	apply_outputs();
}

int8_t mani_get(ManiSide side) {
	return (side < MANI_SIDE_COUNT) ? side_cmd[side] : MANI_CMD_HOLD;
}
