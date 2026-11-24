/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int8_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <stdio.h>
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCLSOFTCHECK(fn) if ((fn) != RCL_RET_OK) {};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rclc_executor_t executor;
rcl_timer_t timer;
const int timeout_ms = 10;
const unsigned int timer_period = RCL_MS_TO_NS(10);

/* manipulator/hw_cmd   : std_msgs/Int8MultiArray data = [left, right]
 *               1 = up, -1 = down, 0 = hold
 * manipulator/hw_state : std_msgs/Int8MultiArray data = [left, right] (applied state) */
rcl_subscription_t cmd_sub;
std_msgs__msg__Int8MultiArray cmd_msg;
rcl_publisher_t state_pub;
std_msgs__msg__Int8MultiArray state_msg;

#define CMD_DATA_CAPACITY 8
static int8_t cmd_data_buf[CMD_DATA_CAPACITY];
static std_msgs__msg__MultiArrayDimension cmd_dim_buf[1];
static char cmd_label_buf[16];
static int8_t state_data_buf[MANI_SIDE_COUNT];

volatile uint32_t last_cmd_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	static uint8_t cnt = 0;
	if (timer == NULL)
		return;

	// Sync micro-ROS session
	rmw_uros_sync_session(timeout_ms);

	// Command timeout -> hold both manipulators
	if ((HAL_GetTick() - last_cmd_tick) > MANI_CMD_TIMEOUT_MS) {
		if (mani_get(MANI_LEFT) != MANI_CMD_HOLD
				|| mani_get(MANI_RIGHT) != MANI_CMD_HOLD) {
			mani_hold_all();
		}
	}

	// Publish applied state at 10 Hz
	if (cnt % 10 == 0) {
		state_msg.data.data[MANI_LEFT] = mani_get(MANI_LEFT);
		state_msg.data.data[MANI_RIGHT] = mani_get(MANI_RIGHT);
		RCLSOFTCHECK(rcl_publish(&state_pub, &state_msg, NULL));
	}

	// Toggle LED every 0.5 s
	if (cnt == 0)
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	cnt = (cnt + 1) % 50;

	// Refresh watchdog
	HAL_IWDG_Init(&hiwdg);
}

void cmd_callback(const void *msgin) {
	const std_msgs__msg__Int8MultiArray *msg =
			(const std_msgs__msg__Int8MultiArray*) msgin;

	if (msg->data.size >= 2) {
		mani_set(MANI_LEFT, msg->data.data[0]);
		mani_set(MANI_RIGHT, msg->data.data[1]);
	} else {
		mani_hold_all();
	}
	last_cmd_tick = HAL_GetTick();
}

void StartDefaultTask(void *argument) {
	// micro-ROS configuration
	rmw_uros_set_custom_transport(
	true, (void*) &hlpuart1, cubemx_transport_open, cubemx_transport_close,
			cubemx_transport_write, cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	allocator = rcl_get_default_allocator();

	// create init_options
	init_options = rcl_get_zero_initialized_init_options();
	RCLSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCLSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 124));

	rclc_support_init_with_options(&support, 0, NULL, &init_options,
			&allocator);

	// create node
	rclc_node_init_default(&node, "mani_node", "", &support);

	// Subscription message memory (static, no heap)
	cmd_msg.data.data = cmd_data_buf;
	cmd_msg.data.capacity = CMD_DATA_CAPACITY;
	cmd_msg.data.size = 0;
	cmd_dim_buf[0].label.data = cmd_label_buf;
	cmd_dim_buf[0].label.capacity = sizeof(cmd_label_buf);
	cmd_dim_buf[0].label.size = 0;
	cmd_msg.layout.dim.data = cmd_dim_buf;
	cmd_msg.layout.dim.capacity = 1;
	cmd_msg.layout.dim.size = 0;
	cmd_msg.layout.data_offset = 0;

	// State message memory
	state_msg.data.data = state_data_buf;
	state_msg.data.capacity = MANI_SIDE_COUNT;
	state_msg.data.size = MANI_SIDE_COUNT;
	state_msg.layout.dim.data = NULL;
	state_msg.layout.dim.capacity = 0;
	state_msg.layout.dim.size = 0;
	state_msg.layout.data_offset = 0;

	rclc_publisher_init_default(&state_pub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
			"manipulator/hw_state");

	rclc_subscription_init_default(&cmd_sub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
			"manipulator/hw_cmd");

	rclc_timer_init_default(&timer, &support, timer_period, timer_callback);

	// executor handles: 1 timer + 1 subscription
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 2, &allocator);
	rclc_executor_add_timer(&executor, &timer);
	rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
			&cmd_callback, ON_NEW_DATA);

	last_cmd_tick = HAL_GetTick();

	while (1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		osDelay(1);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	transport_mani_begin();   // all manipulator outputs LOW (hold)
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
