/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include<stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_SPEED 5000  // divide by 9 gets rpm  // 3000 before
#define REBOUNCE_TIME 1000  // in ms
#define ROBOT_ID 0  // the id of the motor, 0-3
#define MAX_CHANGE_TIME 2000  // in ms, decide when to change TODO: make this bigger
// pid for motor, p small so smooth control
#define RPM_P 1.5  // 1.5 works fine
#define RPM_I 0.1
#define RPM_D 0
// change of rpm per 5ms, smaller transition will be faster but more abrupt (for smooth_transitino)
#define INC_RPM 50
// transition time in ms*10 (for transition())
#define INC_TIME 50
// to enable transition, 0 for none, 1 for fixed time transition, 2 for smooth transition
#define TRANSITION 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// PID struct
PID_TypeDef motor_pid[4];

// speed of motor in rpm
float speed;
// count random time
uint16_t random_timer;
// randomly change direction
int8_t direction;
// counter for sensor turning
uint16_t sensor_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// make a smooth transition of speed
void smooth_transition(float pre_speed, float set_speed);
// with transition but not smooth
void transition(float pre_speed, float set_speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
    can_filter_init();
		
		for(int i=0; i<4; i++)
  {	

    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0, RPM_P, RPM_I, RPM_D);  //pid初始化后面三个是PID系数。
	}
  /* USER CODE BEGIN 2 */
    
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// set random speed
	srand((uint32_t)(HAL_GetTick()));
	// random direction
	direction = rand()%2;
	if (!direction){
		direction = -1;
	}
	
	// init speed to random speed
	speed = MAX_SPEED * (float)(rand()%50 + 50)/100 * direction;  // 50% to 100% * random direction
	
	// set random timer
	random_timer = MAX_CHANGE_TIME/10 * (float)(rand()%80 + 20)/100;  // ticks every 10 ms
  
  while (1)
  {
		HAL_Delay(10);
		
		// update sensor counter for censor time
		if (sensor_counter != 0){
			sensor_counter--;
			
			// set led red as indicator that it's in sensor state
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, (GPIO_PinState)1);
			HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, (GPIO_PinState)0);
			HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, (GPIO_PinState)0);
		}
		// update counter if not in sensor fall back
		else if (!random_timer){
			// set random speed
			srand((uint32_t)(HAL_GetTick()));
			int8_t direction = rand()%2;
			if (!direction){
				direction = -1;
			}
			
			float prev_speed = speed;
			speed = MAX_SPEED * (float)(rand()%80 + 20)/100 * direction;  // 50% to 100% * random direction
			
			// transit the speed
			if (TRANSITION == 0){
				motor_pid[ROBOT_ID].target = speed;
				motor_pid[ROBOT_ID].f_cal_pid(&motor_pid[ROBOT_ID],get_chassis_motor_measure_point(ROBOT_ID)->speed_rpm);
				CAN_cmd_chassis(motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output);
			 
			}
			else if (TRANSITION == 1){
				transition(prev_speed, speed);
			}
			else{
				smooth_transition(prev_speed, speed);
			}
			
			// randomly change speed
			random_timer = MAX_CHANGE_TIME/10 * (float)(rand()%50 + 50)/100;  // ticks every 10 ms
		}
		else{			
			// inc counter
			--random_timer;
		}
		
		
		// user key pressed
		if (!HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin)){			
//			// randomly change the speed
//			srand((uint32_t)(HAL_GetTick()));
//			int8_t direction = rand()%2;
//			if (!direction){
//				direction = -1;
//			}
//			speed = MAX_SPEED * (float)(rand()%50 + 50)/100 * direction;  // 50% to 100% * random direction
			
			// set led red as indicator
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, (GPIO_PinState)1);
			HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, (GPIO_PinState)0);
			HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, (GPIO_PinState)0);
		}
		
		//left sensor
		else if (HAL_GPIO_ReadPin(RIGHT_SENSOR_GPIO_Port, RIGHT_SENSOR_Pin)){
			HAL_Delay(20);  // eliminate pulse
			if (HAL_GPIO_ReadPin(RIGHT_SENSOR_GPIO_Port, RIGHT_SENSOR_Pin)){
				// set led green as indicator
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, (GPIO_PinState)0);
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, (GPIO_PinState)1);
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, (GPIO_PinState)0);
				
				// move right
				// update random speed when exit the move right
				random_timer = 0;
				
				// transit the speed
				if (TRANSITION == 0){
					// do nothing, send directly
				}
				else if (TRANSITION == 1){
					transition(speed, MAX_SPEED);
				}
				else{
					smooth_transition(speed, MAX_SPEED);
				}
				
				// still need to send speed otherwise will stop
				speed = MAX_SPEED;
				motor_pid[ROBOT_ID].target = speed;
				motor_pid[ROBOT_ID].f_cal_pid(&motor_pid[ROBOT_ID],get_chassis_motor_measure_point(ROBOT_ID)->speed_rpm);
				CAN_cmd_chassis(motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output);
				
				// delay for a while to go the other direction
				sensor_counter = REBOUNCE_TIME/10;
			}
		}
		
		// right sensor
		else if (HAL_GPIO_ReadPin(LEFT_SENSOR_GPIO_Port, LEFT_SENSOR_Pin)){
			HAL_Delay(20);  // eliminate pulse
			if (HAL_GPIO_ReadPin(LEFT_SENSOR_GPIO_Port, LEFT_SENSOR_Pin)){
				// set led blue as indicator
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, (GPIO_PinState)0);
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, (GPIO_PinState)0);
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, (GPIO_PinState)1);
				
				// move left
				// update random speed when exit the move right
				random_timer = 0;
				
				// transit the speed
				if (TRANSITION == 0){
					// do nothing, send directly
				}
				else if (TRANSITION == 1){
					transition(speed, -MAX_SPEED);
				}
				else{
					smooth_transition(speed, -MAX_SPEED);
				}
				
				// set speed
				speed = -MAX_SPEED;
				motor_pid[ROBOT_ID].target = speed;
				motor_pid[ROBOT_ID].f_cal_pid(&motor_pid[ROBOT_ID],get_chassis_motor_measure_point(ROBOT_ID)->speed_rpm);
				CAN_cmd_chassis(motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output);
				
				// delay for a while to go the other direction
				sensor_counter = REBOUNCE_TIME/10;
			}
		}
		
		else{
			// set led white when in random state
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, (GPIO_PinState)1);
			HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, (GPIO_PinState)1);
			HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, (GPIO_PinState)1);
			
			// stop motor
			// pid to control motor speed
			motor_pid[ROBOT_ID].target = speed;
			motor_pid[ROBOT_ID].f_cal_pid(&motor_pid[ROBOT_ID],get_chassis_motor_measure_point(ROBOT_ID)->speed_rpm);    //把第二个参数改成读取的速度?
			CAN_cmd_chassis(motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output);
			 
		}
			

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void smooth_transition(float pre_speed, float set_speed){
	// one transition took 500ms, transform from previous speed to set speed
	int16_t inc = (set_speed - pre_speed)/INC_RPM;
	
	// if inc == 0, then no need to transition
	if (inc > 0){
		for(size_t i = 0; i < inc; ++i){
			HAL_Delay(5);
			// cal pid and send to motor
			motor_pid[ROBOT_ID].target = pre_speed + INC_RPM * i;
			motor_pid[ROBOT_ID].f_cal_pid(&motor_pid[ROBOT_ID],get_chassis_motor_measure_point(ROBOT_ID)->speed_rpm);
			CAN_cmd_chassis(motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output);
		}
	}
	else if (inc < 0){
		for(size_t i = 0; i < -inc; ++i){
			HAL_Delay(5);
			// cal pid and send to motor
			motor_pid[ROBOT_ID].target = pre_speed - INC_RPM * i;
			motor_pid[ROBOT_ID].f_cal_pid(&motor_pid[ROBOT_ID],get_chassis_motor_measure_point(ROBOT_ID)->speed_rpm);
			CAN_cmd_chassis(motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output);
		}
	}
}

void transition(float pre_speed, float set_speed){
	// TODO: change to a better transition
	// one transition took 500ms, transform from previous speed to set speed
	int16_t inc = (set_speed - pre_speed)/INC_TIME;
	if (inc == 0){
		return;
	}
	
	for(size_t i = 0; i < INC_TIME; ++i){
		HAL_Delay(10);
		// cal pid and send to motor
		motor_pid[ROBOT_ID].target = pre_speed + (float)inc * i;
		motor_pid[ROBOT_ID].f_cal_pid(&motor_pid[ROBOT_ID],get_chassis_motor_measure_point(ROBOT_ID)->speed_rpm);
		CAN_cmd_chassis(motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output, motor_pid[ROBOT_ID].output);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
