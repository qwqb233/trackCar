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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "OLEDMenu.h"
#include "MotorClass.h"
#include "colorRe.h"
#include "rgb.h"
#include "button.h"
#include "trailing.h"
#include "project_exit.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void task_trailing_1straight(void);
void task_trailing_2straight(void);
void task_turn_left(void);
void task_turn_back(void);
typedef enum {
    STATE_WAIT_SW1,
    STATE_WAIT_GREEN,
    STATE_MOVING,
	STATE_TURNTOLEFT,
    STATE_TURNBACK,
    STATE_WAIT_RED,
    STATE_RETURNING,
	  STATE_OVER
} DeliveryState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
DeliveryState deliveryState = STATE_WAIT_SW1;
static double __time = 0;

static uint8_t green_blink_flag = 0;  // 闪烁状态标志
static double blink_timer = 0;        // 闪烁计时器
static uint8_t errorleft = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motorlist_init(MotorClass * motorlist[4])
{
	MotorClass ** motorlist_it = motorlist;
	MotorClass * motor1 = *motorlist_it;motorlist_it++;
	MotorClass * motor2 = *motorlist_it;motorlist_it++;
	MotorClass * motor3 = *motorlist_it;motorlist_it++;
	MotorClass * motor4 = *motorlist_it;
	motorlist_it = motorlist;
	*motorlist_it = motor1;motorlist_it++;
	*motorlist_it = motor2;motorlist_it++;
	*motorlist_it = motor3;motorlist_it++;
	*motorlist_it = motor4;
	motor_init(motorlist);
	
	motor1->kP = 0.9f;
	motor1->kI = 0.0f;
	motor1->kD = 0.2f;
	motor1->arr = 1000;
	motor1->motor_tim_1 = &htim5;
	motor1->motor_tim_ch_1 = TIM_CHANNEL_1;
	motor1->motor_tim_2 = &htim5;
	motor1->motor_tim_ch_2 = TIM_CHANNEL_2;
	motor1->motor_encoder_tim = &htim1;
	
	motor2->kP = 0.8f;
	motor2->kI = 0.0f;
	motor2->kD = 0.3f;
	motor2->arr = 1000;
	motor2->motor_tim_1 = &htim8;
	motor2->motor_tim_ch_1 = TIM_CHANNEL_1;
	motor2->motor_tim_2 = &htim8;
	motor2->motor_tim_ch_2 = TIM_CHANNEL_2;
	motor2->motor_encoder_tim = &htim2;
	
	motor3->kP = 0.8f;
	motor3->kI = 0.0f;
	motor3->kD = 0.1f;
	motor3->arr = 1000;
	motor3->motor_tim_1 = &htim9;
	motor3->motor_tim_ch_1 = TIM_CHANNEL_1;
	motor3->motor_tim_2 = &htim9;
	motor3->motor_tim_ch_2 = TIM_CHANNEL_2;
	motor3->motor_encoder_tim = &htim3;
	
	motor4->kP = 0.98f;
	motor4->kI = 0.01f;
	motor4->kD = 0.2f;
	motor4->arr = 1000;
	motor4->motor_tim_1 = &htim10;
	motor4->motor_tim_ch_1 = TIM_CHANNEL_1;
	motor4->motor_tim_2 = &htim11;
	motor4->motor_tim_ch_2 = TIM_CHANNEL_1;
	motor4->motor_encoder_tim = &htim4;

}

CarClass car;

	MotorClass motor1;
	MotorClass motor2;
	MotorClass motor3;
	MotorClass motor4;

//电机迭代器
	MotorClass ** motorlist_it;
	MotorClass * motorlist[4] = {&motor1,&motor2,&motor3,&motor4};
	
	float target_speed[4] = {5,5,-5,-5};
	
	TrailingClass pathFinding1;
	TrailingClass pathFinging2;
	
	ButtonClass button_1;
	ButtonState button_1_state = Button_Pressed;
	
	uint8_t max(uint8_t a,uint8_t b)
	{
		return a > b?a:b;
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
  
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
  
	HAL_GPIO_WritePin(RGB1_GPIO_Port,RGB1_Pin|RGB2_Pin|RGB3_Pin,GPIO_PIN_SET);
	OLED_Init(&hi2c2);
	
	//初始化4电机
	trailing_init(&pathFinding1,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_13,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_15);
	trailing_init(&pathFinging2,GPIOC,GPIO_PIN_0,GPIOC,GPIO_PIN_1,GPIOC,GPIO_PIN_2,GPIOC,GPIO_PIN_3);
	motorlist_init(motorlist);
	
	
	//电机设定速度

	OLED_ShowWelcomeMenu(&hi2c2);
	HAL_Delay(1000);
	OLED_Clear(&hi2c2);
	
	
	car.Vw = 0;
	car.Vx = 0;
	car.Vy = 0;
	
	CarClass car_result;
	TCS34725_Init(&hi2c2);
	TCS34725_Enable(&hi2c2);
	COLOR_RGBC rgb;
	button_init(&button_1,BUTTON1_GPIO_Port,BUTTON1_Pin,1000);
//	button_1.button_pin = ;
//	button_1.button_port = ;
	
	HAL_TIM_Base_Start_IT(&htim13);
	
	RGBClass LED; // 声明RGB对象
    RGB_init(&LED,RGB1_GPIO_Port,RGB1_Pin,RGB2_GPIO_Port,RGB2_Pin,RGB3_GPIO_Port,RGB3_Pin);
		
	for(int i=0; i<6; i++){
        HAL_GPIO_TogglePin(BUZZER_GPIO_Port,BUZZER_Pin);
		HAL_Delay(100);
    }

	

    double _time = 0;
    static uint8_t middleline_times = 0;
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 switch (deliveryState) {
		 case STATE_WAIT_SW1:
		{
			OLED_ShowWelcomeMenu(&hi2c2);
		}
			break;
		 case STATE_WAIT_GREEN:
		 {
			 if(TCS34725_GetRawData(&hi2c2,&rgb))
			 {
					uint8_t max_color = max(max(rgb.r,rgb.g),rgb.b);
					if(max_color > 200){
					if(max_color == rgb.g)
					{
							LED.rgb_light = GREEN_LIGHT; // 设置颜色为绿色
							LED.setRGB(&LED);
							//设置rgb颜色为绿色
						deliveryState = STATE_MOVING;
					}	
				 }	
			 }
		 }
			break;
		 case STATE_MOVING:
		 {
			 if(__time - blink_timer > 0.2) 
				{
        green_blink_flag = !green_blink_flag;
        blink_timer = __time;
        // 设置闪烁效果
        if(green_blink_flag) {
				
          LED.rgb_light = GREEN_LIGHT;
        }
				else 
				{
          LED.rgb_light = 7;  
        }
        LED.setRGB(&LED);
       }
			if ((pathFinding1.x1_state == 0 && pathFinding1.x2_state == 0 && pathFinding1.x3_state == 0 )||(pathFinding1.x1_state == 0 && pathFinding1.x2_state == 0 && pathFinding1.x4_state == 0 )||(pathFinding1.x1_state == 0 && pathFinding1.x4_state == 0 && pathFinding1.x3_state == 0 )
		           ||(pathFinding1.x4_state == 0 && pathFinding1.x2_state == 0 && pathFinding1.x3_state == 0 )||(pathFinding1.x1_state == 0 && pathFinding1.x2_state == 0 && pathFinding1.x3_state == 0 && pathFinding1.x4_state == 0))
			{
				
				car.Vw = 0;
				car.Vx = 0;
				car.Vy = 0;
				
				
				if( middleline_times == 0)
				{
					car.Vw = 0;
				  car.Vx = 10;
			  	car.Vy = 0;
				  _time = __time;
				  while(__time < _time+0.5);
					deliveryState = STATE_MOVING;
					middleline_times = 1;
				}
				else if(middleline_times < 5)
				{
					car.Vw = 0;
				  car.Vx = 10;
			  	car.Vy = 0;
				  _time = __time;
				  while(__time < _time+0.5);
					car.Vw = 0;
				  car.Vx = 0;
			  	car.Vy = 0;
				  _time = __time;
				  while(__time < _time+0.3);
					middleline_times += 1;
					
					deliveryState = STATE_TURNTOLEFT;
				}
				else if(middleline_times == 5)
				{
					deliveryState = STATE_RETURNING;
					middleline_times = 0;
				}
			}
			else if (get_distance_cm_nT(TR_GPIO_Port,TR_Pin) <= 10 )
			{
				car.Vw = 0;
				car.Vx = 0;
				car.Vy = 0;
				deliveryState = STATE_WAIT_RED;
			}
			else
              task_trailing_1straight();
         }
			break;
    case STATE_TURNTOLEFT:
    { 
			LED.rgb_light = MAGENTA_LIGHT; 
      LED.setRGB(&LED);
      task_turn_left();
    }
      break;
    case STATE_WAIT_RED:
    {
      LED.rgb_light = RED_LIGHT; 
      LED.setRGB(&LED);
      if(TCS34725_GetRawData(&hi2c2,&rgb))
      {
        uint8_t max_color = max(max(rgb.r,rgb.g),rgb.b);
        if(max_color > 200){
          if(max_color == rgb.r)
          {
            LED.rgb_light = YELLOW_LIGHT; 
            LED.setRGB(&LED);
            deliveryState = STATE_TURNBACK;
          }
        }
      }
    }
      break;
    case STATE_TURNBACK:
	 {
		task_turn_back();
		 
	 }
		break;
	case STATE_RETURNING:
	{
		car.Vw = 0;
        car.Vx = 10;
        car.Vy = 0;
		_time = __time;
		while(__time < _time+1.15);
		
				car.Vw = 0.7;
        car.Vx = 0;
        car.Vy = 0;
		_time = __time;
		while(__time < _time+2.25);
		
		car.Vw = 0;
        car.Vx = 0;
        car.Vy = 0;
		LED.rgb_light = BLUE_LIGHT;
        LED.setRGB(&LED);
		deliveryState = STATE_OVER;
	}
	    break;
	case STATE_OVER:
	{
		HAL_TIM_Base_Stop_IT(&htim13);
	}
	break;
	 }
	 
    OLED_ShowRunMenu(&hi2c2,car_result.Vx,car_result.Vy,car_result.Vw,get_distance_cm_nT(TR_GPIO_Port,TR_Pin),(uint32_t)__time,middleline_times);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 72-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 72-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 72-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 72-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 72-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 72-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 10000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RGB1_Pin|RGB2_Pin|RGB3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TR_Pin|LED_Pin|BUZZER_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RGB1_Pin RGB2_Pin RGB3_Pin TR_Pin
                           LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = RGB1_Pin|RGB2_Pin|RGB3_Pin|TR_Pin
                          |LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : F11_Pin F12_Pin F13_Pin F14_Pin
                           ec_Pin */
  GPIO_InitStruct.Pin = F11_Pin|F12_Pin|F13_Pin|F14_Pin
                          |ec_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON1_Pin BUTTON2_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin|BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : F21_Pin F22_Pin F23_Pin F24_Pin */
  GPIO_InitStruct.Pin = F21_Pin|F22_Pin|F23_Pin|F24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_SDA_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if(htim == &htim13)
	{
		   button_1.button_scan(&button_1);
			 if (button_1.button_state==button_1_state)
			 {
				 deliveryState = STATE_WAIT_GREEN;
				 HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			 }
		  car_speed_to_motor_speed(&car,motorlist);
		  target_speed[0] = motor1.motor_speed_target;
		  target_speed[1] = motor2.motor_speed_target;
		  target_speed[2] = motor3.motor_speed_target;
		  target_speed[3] = motor4.motor_speed_target;
		  motor_control_AllInOne(motorlist,target_speed);
		  pathFinding1.Trailing_scan(&pathFinding1);
		  __time += 0.01;
	}

}


void task_trailing_1straight(void)
{   
	 if((pathFinding1.x2_state==0)&&(pathFinding1.x3_state==0))
			 {
				 car.Vw = 0;
				 car.Vx = 13;
				 car.Vy = 0;
				 return;
			 }
			 else if(pathFinding1.x1_state==0)
			 {
//				 car.Vw = -0;
//				 car.Vx = 10;
//				 car.Vy = -5;
				 car.Vw = 0.3;
				 car.Vx = 10;
				 car.Vy = 0;
				 pathFinding1.x1_last_state = 0;
				  pathFinding1.x2_last_state=0;
				 return;
			 }
			 else if(pathFinding1.x4_state==0)
			 {
			   car.Vw = -0.3;
				 car.Vx = 10;
				 car.Vy = 0;
				 pathFinding1.x4_last_state = 0;
				 pathFinding1.x3_last_state = 0;
				 return;
			 }
			 else if((pathFinding1.x1_state==1)&&(pathFinding1.x2_state==1)&&(pathFinding1.x3_state==1)&&(pathFinding1.x4_state==1))
			 {
				 if(pathFinding1.x1_last_state == 0 || pathFinding1.x2_last_state==0)
				 {
					 car.Vw = 0.3;
					 car.Vx = 10;
					 car.Vy = 0;
					 pathFinding1.x1_last_state = 1;
					 return;
				 }
				else if(pathFinding1.x3_last_state==0 || pathFinding1.x4_last_state==0)
			     {
					 car.Vw = -0.3;
					 car.Vx = 10;
					 car.Vy = 0;
					 pathFinding1.x4_last_state = 1;
					 return;
			     }
				 else
				 {
					 car.Vw = 0;
					 car.Vx = 13;
					 car.Vy = 0;
					 return;
				 }
				
			 }
}
void task_turn_left(void)
{
	static uint8_t left = 0;
	static uint8_t turn = 0;
	if(pathFinding1.x2_state == 1 && pathFinding1.x3_state == 1 && pathFinding1.x1_state == 1 && pathFinding1.x4_state == 1)
		left = 1;
	if((pathFinding1.x2_state == 1 && pathFinding1.x3_state == 1 && pathFinding1.x1_state == 0 && pathFinding1.x4_state == 1) && left == 1)
	  turn = 1;
	else
	{
		car.Vw = 0.5;
        car.Vx = 0;
        car.Vy = 0;
	}
	if( pathFinding1.x2_state == 0  && turn)
	{
		deliveryState = STATE_MOVING;
		turn = 0;
		left = 0;
	}
      else
      {
        car.Vw = 0.5;
        car.Vx = 0;
        car.Vy = 0;
      }
    
}

void task_turn_back(void)
{
	static uint8_t turn = 0;
	static uint8_t back = 0;
	if(pathFinding1.x2_state == 1 && pathFinding1.x3_state == 1 && pathFinding1.x1_state == 1 && pathFinding1.x4_state == 0)
	  turn = 1;
	if((pathFinding1.x2_state == 1 && pathFinding1.x3_state == 1 && pathFinding1.x1_state == 1 && pathFinding1.x4_state == 1) && turn == 1)
	  turn = 2;
	if((pathFinding1.x2_state == 1 && pathFinding1.x3_state == 1 && pathFinding1.x1_state == 0 && pathFinding1.x4_state == 1) && turn == 2)
	  back = 1;
	else
	{
		    car.Vw = 0.5;
        car.Vx = 0;
        car.Vy = 0;
	}
    if( ( pathFinding1.x2_state == 0) && ( pathFinding1.x3_state == 0) && back==1)
    {
      deliveryState = STATE_MOVING;
		  turn = 0;
			back = 0;
    }
    else
    {
	    car.Vw = 0.5;
		  car.Vx = 0;
		  car.Vy = 0;
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
  __disable_irq();
  while (1)
  {
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
