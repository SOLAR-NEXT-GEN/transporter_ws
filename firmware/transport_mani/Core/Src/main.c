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
#include "adc.h"
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

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>

#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

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
#define RCLSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
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

rcl_publisher_t publisher;
std_msgs__msg__Float64MultiArray pub_msg;

rcl_subscription_t subscriber;
std_msgs__msg__Float64MultiArray sub_msg;

int cmd_FL_hinge, cmd_BL_hinge;
uint8_t FL_photo1, FL_photo2, BL_photo1, BL_photo2;
uint8_t FL_limit1, FL_limit2, BL_limit1, BL_limit2;
int FL_hinge_state, BL_hinge_state;
uint32_t FL_timer_up, FL_timer_down;
uint32_t BL_timer_up, BL_timer_down;

int32_t cmd_vel1, cmd_vel2;
uint16_t FlexData[2] = { 0 };
float flexdata_converted[2];
float flex_threshold = 0.9;
int flex_output[2];

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
	if (timer != NULL) {
		// Sync micro-ROS session
		rmw_uros_sync_session(timeout_ms);

		// Prepare and publish multi-array message with motor data
		if (pub_msg.data.data != NULL) {
            pub_msg.data.data[0] = FL_hinge_state;
            pub_msg.data.data[1] = BL_hinge_state;
            pub_msg.data.data[2] = flex_output[0];  // 0 or 1
            pub_msg.data.data[3] = flex_output[1];  // 0 or 1

			// Publish the multi-array message
			RCLSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
		}

		// Reinitialize watchdog timer
		HAL_IWDG_Init(&hiwdg);
	}
}

void subscription_callback(const void *msgin) {
	const std_msgs__msg__Float64MultiArray *msg =
			(const std_msgs__msg__Float64MultiArray*) msgin;

	// Extract commands: 1 = down, 0 = stop, -1 = up
	if (msg->data.size >= 2) {
		cmd_FL_hinge = (int) msg->data.data[0];
		cmd_BL_hinge = (int) msg->data.data[1];
	}
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

	//create init_options
	init_options = rcl_get_zero_initialized_init_options();
	RCLSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCLSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 124));

	rclc_support_init_with_options(&support, 0, NULL, &init_options,
			&allocator);

	// create node
	rclc_node_init_default(&node, "mani_node", "", &support);

	pub_msg.layout.dim.capacity = 1;
	pub_msg.layout.dim.size = 1;
	pub_msg.layout.dim.data = malloc(
			sizeof(std_msgs__msg__MultiArrayDimension) * 1);

	pub_msg.layout.dim.data[0].label.data = malloc(10);
	pub_msg.layout.dim.data[0].label.capacity = 10;
	pub_msg.layout.dim.data[0].label.size = strlen("mani_data");
	strcpy(pub_msg.layout.dim.data[0].label.data, "mani_data");

	pub_msg.layout.data_offset = 0;

	pub_msg.data.capacity = 4;
	pub_msg.data.size = 4;
	pub_msg.data.data = malloc(4 * sizeof(double));

	// Allocate layout for sub_msg
	sub_msg.layout.dim.capacity = 1;
	sub_msg.layout.dim.size = 1;
	sub_msg.layout.dim.data = malloc(
			sizeof(std_msgs__msg__MultiArrayDimension));

	sub_msg.layout.dim.data[0].label.data = malloc(10);
	sub_msg.layout.dim.data[0].label.capacity = 10;
	sub_msg.layout.dim.data[0].label.size = strlen("cmd_data");
	strcpy(sub_msg.layout.dim.data[0].label.data, "cmd_data");

	sub_msg.layout.data_offset = 0;

	// Allocate data array for sub_msg
	sub_msg.data.capacity = 2;
	sub_msg.data.size = 2;
	sub_msg.data.data = malloc(sizeof(double) * 2);
	sub_msg.data.data[0] = 0.0;
	sub_msg.data.data[1] = 0.0;

	// create publisher
	rclc_publisher_init_default(&publisher, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
			"hinge_state");

	// Create subscriber
	rclc_subscription_init_default(&subscriber, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
			"cmd_hinge");

	// create timer
	rclc_timer_init_default(&timer, &support, timer_period, timer_callback);

	// create executor
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 2, &allocator); // total number of handles = #subscriptions + #timers
	rclc_executor_add_timer(&executor, &timer);
	rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
			&subscription_callback, ON_NEW_DATA);
	rclc_executor_spin(&executor);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	tarnsport_mani_begin();
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, FlexData, 2);
//  HAL_ADC_Start(&hadc1);
//  HAL_ADC_PollForConversion(&hadc1, 10);
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
	if (htim == &htim2) {

		// photo 0 = found
		FL_photo1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
		FL_photo2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		BL_photo1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		BL_photo2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);

		// limit 1 = trick
		FL_limit1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
		FL_limit2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
		BL_limit1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
		BL_limit2 = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

//		MDXX_set_range(&FL_motor, 2000, cmd_vel1);
//		MDXX_set_range(&BL_motor, 2000, cmd_vel2);

		// FL_hinge down 8 s, up 10 s
		if (cmd_FL_hinge == 0) {
			FL_hinge_state = 0; // manual stop
		}

		if (FL_hinge_state == 0) { // stopped (manual or initial)
			if (cmd_FL_hinge == 1) {
				FL_timer_down = 0;
				FL_timer_up = 0;
				FL_hinge_state = 1; // start going down
			} else if (cmd_FL_hinge == -1) {
				FL_timer_down = 0;
				FL_timer_up = 0;
				FL_hinge_state = -1; // start going up
			}
		}

		else if (FL_hinge_state == 2) { // stopped by limit
			if (cmd_FL_hinge == 0) {
				FL_hinge_state = 0;
			} else if (cmd_FL_hinge == -1) {
				FL_hinge_state = -1;
			}
		}

		else if (FL_hinge_state == 3) { // stopped by timeout
			if (cmd_FL_hinge == 0) {
				FL_hinge_state = 0;
			}
		}

		else if (FL_hinge_state == 1) { // down
			FL_timer_down++;

			if (FL_limit1 == 0 && FL_limit2 == 0) {
				MDXX_set_range(&FL_motor, 2000, 65535);
			} else {
				FL_hinge_state = 2; // stopped by limit
				MDXX_set_range(&FL_motor, 2000, 0);

				if (FL_timer_down < 4800) {
					FL_timer_up = 0;
					FL_hinge_state = -1;
				}
			}
		}

		else if (FL_hinge_state == -1) { // up
			FL_timer_up++;

			if (FL_timer_up > 7000) {
				FL_hinge_state = 3; // stop after timeout
				MDXX_set_range(&FL_motor, 2000, 0);
			} else {
				MDXX_set_range(&FL_motor, 2000, -65535);
			}
		}

		if (FL_hinge_state == 0 || FL_hinge_state == 2 || FL_hinge_state == 3) {
			MDXX_set_range(&FL_motor, 2000, 0);
			FL_timer_down = 0;
			FL_timer_up = 0;
		}
		///////////////////////////////////////////////

		// BL_hinge down 6 s, up 8 s
		if (cmd_BL_hinge == 0) {
			BL_hinge_state = 0; // manual stop
		}

		if (BL_hinge_state == 0) { // stopped (manual or initial)
			if (cmd_BL_hinge == 1) {
				BL_timer_down = 0;
				BL_timer_up = 0;
				BL_hinge_state = 1; // start going down
			} else if (cmd_BL_hinge == -1) {
				BL_timer_down = 0;
				BL_timer_up = 0;
				BL_hinge_state = -1; // start going up
			}
		}

		else if (BL_hinge_state == 2) { // stopped by limit
			if (cmd_BL_hinge == 0) {
				BL_hinge_state = 0;
			} else if (cmd_BL_hinge == -1) {
				BL_hinge_state = -1;
			}
		}

		else if (BL_hinge_state == 3) { // stopped by timeout
			if (cmd_BL_hinge == 0) {
				BL_hinge_state = 0;
			}
		}

		else if (BL_hinge_state == 1) { // down
			BL_timer_down++;

			if (BL_limit1 == 0 && BL_limit2 == 0) {
				MDXX_set_range(&BL_motor, 2000, 65535);
			} else {
				BL_hinge_state = 2; // stopped by limit
				MDXX_set_range(&BL_motor, 2000, 0);

				if (BL_timer_down < 4800) {
					BL_timer_up = 0;
					BL_hinge_state = -1;
				}
			}
		}

		else if (BL_hinge_state == -1) { // up
			BL_timer_up++;

			if (BL_timer_up > 7000) {
				BL_hinge_state = 3; // stop after timeout
				MDXX_set_range(&BL_motor, 2000, 0);
			} else {
				MDXX_set_range(&BL_motor, 2000, -65535);
			}
		}

		if (BL_hinge_state == 0 || BL_hinge_state == 2 || BL_hinge_state == 3) {
			MDXX_set_range(&BL_motor, 2000, 0);
			BL_timer_down = 0;
			BL_timer_up = 0;
		}
		///////////////////////////////////////////////

		// flex sensor //
		flexdata_converted[0] = FlexData[0] / 255.0 * 3.3;
		flexdata_converted[1] = FlexData[1] / 255.0 * 3.3;
		if (flexdata_converted[0] >= flex_threshold) {
			flex_output[0] = 1;
		}
		else{
			flex_output[0] = 0;
		}

		if (flexdata_converted[1] >= flex_threshold) {
			flex_output[1] = 1;
		}
		else{
			flex_output[1] = 0;
		}
		///////////////////////////////////////////////

	}
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
