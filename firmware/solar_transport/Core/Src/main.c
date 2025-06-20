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

#include <std_msgs/msg/int32.h>

#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include "transporter_config.h"
#include "signal_generator.h"
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
uint32_t Fs = 0;
float setpointL = 0, setpointR = 0;
;
float filteredValue1 = 0, filteredValue2 = 0, filteredValue3 = 0,
		filteredValue4 = 0;
float cmd_vel1 = 0, cmd_vel2 = 0, cmd_vel3 = 0, cmd_vel4 = 0;

SignalGenerator sine_sg;
SignalGenerator chirp_linear_sg;
SignalGenerator chirp_log_sg;
SignalGenerator square_sg;
SignalGenerator ramp_sg;

float sine_sample = 0.0f;
float chirp_Linear_sample = 0.0f;
float chirp_log_sample = 0.0f;
float square_sample = 0.0f;
float ramp_sample = 0.0f;

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rclc_executor_t executor;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Float64MultiArray pub_msg;
geometry_msgs__msg__Twist sub_msg;

rcl_timer_t timer;
const unsigned int timer_period = RCL_MS_TO_NS(10);
const int timeout_ms = 1000;

float linear_x, linear_y, linear_z, angular_x, angular_y, angular_z;
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

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void subscription_callback(const void *msgin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	static uint8_t cnt = 0;

	if (timer != NULL) {
		// Sync micro-ROS session
		rmw_uros_sync_session(timeout_ms);

		// Toggle LED every 50 cycles (approximately every 0.5 seconds)
		if (cnt == 0)
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		cnt = (cnt + 1) % 50;

		// Prepare and publish multi-array message with motor data
		if (pub_msg.data.data != NULL) {
			pub_msg.data.data[0] = setpointL;
			pub_msg.data.data[1] = setpointR;
			pub_msg.data.data[2] = filteredValue1;
			pub_msg.data.data[3] = filteredValue2;
			pub_msg.data.data[4] = filteredValue3;
			pub_msg.data.data[5] = filteredValue4;

			// Publish the multi-array message
			RCLSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
		}

		// Reinitialize watchdog timer
		HAL_IWDG_Init(&hiwdg);
	}
}

void subscription_callback(const void *msgin) {
	const geometry_msgs__msg__Twist *twist_msg =
			(const geometry_msgs__msg__Twist*) msgin;

	linear_x = twist_msg->linear.x;
	linear_y = twist_msg->linear.y;
	linear_z = twist_msg->linear.z;

	angular_x = twist_msg->angular.x;
	angular_y = twist_msg->angular.y;
	angular_z = twist_msg->angular.z;

	SKID_inverse_kinematic(&robot_kinematics, linear_x, angular_z);

	setpointL = robot_kinematics.left_cmd;
	setpointR = robot_kinematics.right_cmd;
}

void StartDefaultTask(void *argument) {

	// micro-ROS configuration
	rmw_uros_set_custom_transport(true, (void*) &hlpuart1,
			cubemx_transport_open, cubemx_transport_close,
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

	//create init
	init_options = rcl_get_zero_initialized_init_options();
	RCLSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCLSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 99));

	//create support
	rclc_support_init_with_options(&support, 0, NULL, &init_options,
			&allocator);

	// create node
	rclc_node_init_default(&node, "uros_motor_node", "", &support);

	pub_msg.layout.dim.capacity = 1;
	pub_msg.layout.dim.size = 1;
	pub_msg.layout.dim.data = malloc(
			sizeof(std_msgs__msg__MultiArrayDimension) * 1);

	pub_msg.layout.dim.data[0].label.data = malloc(10);
	pub_msg.layout.dim.data[0].label.capacity = 10;
	pub_msg.layout.dim.data[0].label.size = strlen("motor_data");
	strcpy(pub_msg.layout.dim.data[0].label.data, "motor_data");

	pub_msg.layout.data_offset = 0;

	pub_msg.data.capacity = 6;
	pub_msg.data.size = 6;
	pub_msg.data.data = malloc(6 * sizeof(double));

	// Create publisher
	rclc_publisher_init_default(&publisher, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
			"robot_pos");

	// Create subscriber
	rclc_subscription_init_best_effort(&subscriber, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

	//create timer
	rclc_timer_init_default(&timer, &support, timer_period, timer_callback);

	//create executor
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
int main(void) {

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
	MX_IWDG_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_TIM20_Init();
	/* USER CODE BEGIN 2 */
	transporter_begin();

	//	HAL_ADC_Start_DMA(&hadc1, pData, Length);

	// 1. Sine Wave Example
	SIGNAL_init(&sine_sg, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg, 0.7f / 3.0f,    // Amplitude
	0.05f,         // Frequency
			0.0f,         // Phase
			0.0f,         // Offset
			-0.7f / 3.0f,   // Min Setpoint
			0.7f / 3.0f     // Max Setpoint
					);

	// 2. Chirp Wave Example (Linear)
	SIGNAL_init(&chirp_linear_sg, SIGNAL_CHIRP);
	SIGNAL_config_chirp(&chirp_linear_sg, 0.7f / 3.0f,        // Amplitude
	1.0f,             // Start Frequency
			10.0f,            // End Frequency
			5.0f,             // Duration (seconds)
			CHIRP_LINEAR,     // Chirp Type
			-0.7f / 3.0f,       // Min Setpoint
			0.7f / 3.0f         // Max Setpoint
					);

	// 3. Chirp Wave Example (Logarithmic)
	SIGNAL_init(&chirp_log_sg, SIGNAL_CHIRP);
	SIGNAL_config_chirp(&chirp_log_sg, 0.7f / 3.0f,            // Amplitude
	1.0f,                 // Start Frequency
			10.0f,                // End Frequency
			5.0f,                 // Duration (seconds)
			CHIRP_LOGARITHMIC,    // Chirp Type
			-0.7f / 3.0f,           // Min Setpoint
			0.7f / 3.0f             // Max Setpoint
					);

	// 4. Square Wave Example
	SIGNAL_init(&square_sg, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg, 0.7f / 3.0f,    // Amplitude
	2.0f,         // Frequency
			0.3f,         // Duty Cycle (30%)
			0.0f,         // Phase
			0.0f,         // Offset
			-0.7f / 3.0f,   // Min Setpoint
			0.7f / 3.0f     // Max Setpoint
					);

	// 5. Ramp Wave Example
	SIGNAL_init(&ramp_sg, SIGNAL_RAMP);
	SIGNAL_config_ramp(&ramp_sg, 0.7f / 3.0f,    // Amplitude
	1.0f,         // Frequency
			-0.7f / 3.0f,   // Ramp Start
			0.7f / 3.0f,    // Ramp End
			0.0f,         // Phase
			0.0f,         // Offset
			-0.7f / 3.0f,   // Min Setpoint
			0.7f / 3.0f     // Max Setpoint
					);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* Call init function for freertos objects (in cmsis_os2.c) */
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim == &htim2) {
		QEI_get_diff_count(&encoder1);
		QEI_get_diff_count(&encoder2);
		QEI_get_diff_count(&encoder3);
		QEI_get_diff_count(&encoder4);

		QEI_compute_data(&encoder1);
		QEI_compute_data(&encoder2);
		QEI_compute_data(&encoder3);
		QEI_compute_data(&encoder4);

		sine_sample = SIGNAL_generate(&sine_sg, 0.001f);
		chirp_Linear_sample = SIGNAL_generate(&chirp_linear_sg, 0.001f);
		chirp_log_sample = SIGNAL_generate(&chirp_log_sg, 0.001f);
		square_sample = SIGNAL_generate(&square_sg, 0.001f);
		ramp_sample = SIGNAL_generate(&ramp_sg, 0.001f);

		filteredValue1 = FIR_process(&lowPassFilter1, encoder1.radps);
		filteredValue2 = FIR_process(&lowPassFilter2, encoder2.radps);
		filteredValue3 = FIR_process(&lowPassFilter3, encoder3.radps);
		filteredValue4 = FIR_process(&lowPassFilter4, encoder4.radps);

		cmd_vel1 = PWM_Satuation(PID_CONTROLLER_Compute(&pid1, setpointL - filteredValue1),
				65535, -65535);
		cmd_vel2 = PWM_Satuation(PID_CONTROLLER_Compute(&pid2, setpointL - filteredValue2),
				65535, -65535);
		cmd_vel3 = PWM_Satuation(PID_CONTROLLER_Compute(&pid3, setpointR - filteredValue3),
				65535, -65535);
		cmd_vel4 = PWM_Satuation(PID_CONTROLLER_Compute(&pid4, setpointR - filteredValue4),
				65535, -65535);

		if (setpointL == 0.0 && setpointR == 0.0) {
			cmd_vel1 = 0.0;
			cmd_vel2 = 0.0;
			cmd_vel3 = 0.0;
			cmd_vel4 = 0.0;
		}

		MDXX_set_range(&motor1, 2000, cmd_vel1);
		MDXX_set_range(&motor2, 2000, cmd_vel2);
		MDXX_set_range(&motor3, 2000, cmd_vel3);
		MDXX_set_range(&motor4, 2000, cmd_vel4);

		uint32_t current_time = HAL_GetTick();
		SKID_calculate_pose(&robot_kinematics, cmd_vel1, cmd_vel2, cmd_vel3,
				cmd_vel4, current_time);
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
