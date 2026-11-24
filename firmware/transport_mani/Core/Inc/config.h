/*
 * config.h
 *
 *  Manipulator GPIO configuration.
 *
 *  Each side (Left / Right) is driven by two outputs (Up / Down) and both
 *  sides share one pump enable:
 *
 *    command | PUMP_EN | <SIDE>_UP | <SIDE>_DOWN
 *    --------+---------+-----------+------------
 *    UP      |  HIGH   |   HIGH    |    LOW
 *    DOWN    |  HIGH   |   LOW     |    HIGH
 *    HOLD    |  LOW*   |   LOW     |    LOW
 *
 *  * PUMP_EN is LOW only when BOTH sides hold. While any side is moving the
 *    pump stays enabled.
 *
 *  PIN CONFIG: when the pins are set up in CubeMX, give them the user labels
 *  PUMP_EN, LEFT_UP, LEFT_DOWN, RIGHT_UP and RIGHT_DOWN. CubeMX then generates
 *  <LABEL>_Pin / <LABEL>_GPIO_Port in main.h, and those override the
 *  placeholder pins below automatically.
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"
#include "gpio.h"
#include <stdint.h>

/* ---------- Placeholder pins (TODO: replace via CubeMX pin config) ------- */
#ifndef PUMP_EN_Pin
#define PUMP_EN_GPIO_Port   GPIOC
#define PUMP_EN_Pin         GPIO_PIN_0
#endif

#ifndef LEFT_UP_Pin
#define LEFT_UP_GPIO_Port   GPIOC
#define LEFT_UP_Pin         GPIO_PIN_1
#endif

#ifndef LEFT_DOWN_Pin
#define LEFT_DOWN_GPIO_Port GPIOC
#define LEFT_DOWN_Pin       GPIO_PIN_4
#endif

#ifndef RIGHT_UP_Pin
#define RIGHT_UP_GPIO_Port  GPIOA
#define RIGHT_UP_Pin        GPIO_PIN_10
#endif

#ifndef RIGHT_DOWN_Pin
#define RIGHT_DOWN_GPIO_Port GPIOB
#define RIGHT_DOWN_Pin       GPIO_PIN_6
#endif

/* ---------- Command values (same as the ROS side, std_msgs/Int8MultiArray) */
#define MANI_CMD_DOWN  (-1)
#define MANI_CMD_HOLD  (0)
#define MANI_CMD_UP    (1)

/* If no command is received for this long, both sides go to HOLD. */
#define MANI_CMD_TIMEOUT_MS  500U

typedef enum {
	MANI_LEFT = 0,
	MANI_RIGHT = 1,
	MANI_SIDE_COUNT
} ManiSide;

void transport_mani_begin(void);

/* Set the command for one side (MANI_CMD_UP / DOWN / HOLD). Anything else
 * is treated as HOLD. Outputs are updated immediately. */
void mani_set(ManiSide side, int8_t cmd);

/* Force both sides to HOLD (pump off). */
void mani_hold_all(void);

int8_t mani_get(ManiSide side);

#endif /* INC_CONFIG_H_ */
