/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "arm_math.h"
#include "pid.h"
#include "ModBusRTU.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/*
List to do
-Merge IOC
-vacuum

List to test
-set home
-set shelve
*/

uint16_t timerange = 2;
uint64_t upper = 0;
uint64_t read = 0;
float linearspeed[2] ;
float linearacc = 0;
uint8_t relay[4];
uint32_t WaitGripper;
uint8_t GripperFlag;
uint32_t CountGripper;

uint32_t QEIReadRaw;
float velodegree;
float CurrentPos;
typedef struct {
// for record New / Old value to calculate dx / dt
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float QEIPostion_1turn[2];
	float QEIAngularVelocity;
	float Angle;
	float TotalPos;
	float QEIRound;

} QEI_StructureTypeDef;

QEI_StructureTypeDef QEIdata = { 0 };
uint64_t _micros = 0;

enum {
	NEW, OLD
};

//motor
uint16_t duty_cycle = 1000;

//LogicConv
uint8_t Lo1 = 0; // foreward leedswitch
uint8_t Lo2 = 0;// backward
uint8_t Lo3 = 0; //joy vs pid
uint8_t Lo4 = 1; //emer

//Botton
uint8_t bt1 = 0;
uint8_t bt2 = 0;
uint8_t bt3 = 0;
uint8_t bt4 = 0;
uint8_t bt5 = 0;
uint8_t bt5prev = 0;

//Limit
uint8_t LimitTop = 0;
uint8_t LimitBottom = 0;
uint8_t x = 0;
//RelayWrite


//PID
uint8_t mode = 2;
float Vfeedback;
float RealVfeedback;
float Goal = 0;
uint16_t duty_cycle_pid;
PID pid_control;
uint8_t test_change = 1;
uint8_t test_change_prev = 0;
float pid_p = 0.5;
float pid_i = 5.5e-3;
float pid_d = 3.5e-6;
uint8_t PercentDis;

///BaseSys
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];

float ShelvePos[5];
float HomePos = 0;

float Z[4];	//Z[] = ZPos ZSpeed ZAccel XPos
uint8_t BaseVacuum;
uint8_t BaseGripper;
uint8_t ActualVacuum;
uint8_t ActualGripper;
char PickOrder[6];
char PlaceOrder[6];
uint16_t GoalPick[5];
uint16_t GoalPlace[5];
int j = 0;
uint8_t Arrived = 0;
uint8_t i;
uint8_t a;
float b_check[10];
uint8_t MotorDriveFlag = 0;
float MotorDriveDampDistance = 0;
float MotorDriveTravelDistance;
float StartTotalPos;
float PIDVFeedback;
//flag
uint8_t LimitBottomFlag=0;
uint8_t flagpick = 0;
uint8_t flagpickend = 0;
uint8_t flagplace = 0;
uint8_t flagplaceend = 0;
uint8_t flagEmer = 0;
uint8_t flagstart =0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HomeGoTo();
void JogModeGoTo();
void PointModeGoTo();
void convert_to_string(uint16_t number, char* buffer, int buffer_size);

uint64_t Micros();
void QEIEncoderPosVel_Update();
void ReadLogicConv();
void ReadButton();
void ReadLimit();
void WritePins();
void MotorDrive();
void MotorDrivePoint();
void RelayDrive();
void GoPick();
void GoPlace();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //pwm setup
  	 relay[0] = 1;
  	 relay[1] = 0;
  	 relay[2] = 0;

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim4);
	upper = 0;
	PID_init(&pid_control, pid_p, pid_i, pid_d ,timerange);

	hmodbus.huart = &huart2;
	hmodbus.htim = &htim16;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize =200;
	Modbus_init(&hmodbus, registerFrame);

	HAL_TIM_Base_Start_IT(&htim5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(flagstart == 0){
		relay[0] = 0;
		relay[1] = 0;
		relay[2] = 0;
		flagstart = 1;
	}


	Modbus_Protocal_Worker();
	registerFrame[0x11].U16 = QEIdata.TotalPos*10; //ZPos
	//registerFrame[0x11].U16 = b_check[0];
	registerFrame[0x12].U16 = fabs(linearspeed[NEW]*10); //ZSpeed
	registerFrame[0x13].U16 = fabs(linearacc); //ZAccel
	registerFrame[0x40].U16 = Z[3]; //XPos
	BaseVacuum = registerFrame[2].U16; // 0 = off , 1 = on
	BaseGripper = registerFrame[3].U16; // 0 = Backward , 1 = Forward

	//re counter
	if (LimitBottomFlag == 1) {
		memset(&QEIdata, 0, sizeof(QEIdata));

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
		HAL_Delay(200);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COUNTER(&htim3,0);
		registerFrame[0x10].U16 = 0;
		LimitBottomFlag = 0;
	}

	if(Lo4 == 0  && flagEmer==0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		relay[0] = 0; //Gripper pull
		relay[1] = 0;
		relay[2] = 0;
		flagEmer = 1;
	}
	if((Lo4 == 1) && (flagEmer == 1)){
		MotorDriveFlag = 1;
		flagEmer = 0;
	}

	static uint64_t timestamp = 0;
	int64_t currentTime = Micros();
	if (currentTime > timestamp) {
		timestamp = currentTime + timerange;	 //us
		QEIEncoderPosVel_Update();
	}

	RelayDrive();
	ReadButton();
	ReadLogicConv();
	ReadLimit();
	if (mode == 0){
		MotorDrive();
	}


	if (LimitBottom == 0) {
		LimitBottomFlag = 1;
	}

	if (LimitTop == 0) {
		mode = 1;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}

	if (Lo3 == 1 && mode !=0) { //joy manual
			if (bt3 == 0) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
			} else if (bt2 == 0) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
			} else if (bt1 == 0){
				Z[3] = Z[3] + 1 ;
			}else if(bt4 == 0){
				Z[3] = Z[3] - 1 ;
			}else {
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			}

			relay[3] = 0;
		} else if (Lo3 == 0 && mode!=0) {
			relay[3] = 1;
			 if (bt1 == 0){
				 Z[3] = Z[3] + 1 ;
			}else if(bt4 == 0){
				Z[3] = Z[3] - 1 ;
			}
			if(registerFrame[0x10].U16 == 0){

				if(BaseGripper==1){
					relay[0] = 0; //Gripper push
					relay[1] = 1;
				}else if(BaseGripper == 0){
					relay[0] = 1; //Gripper push
					relay[1] = 0;
				}
				if(BaseVacuum==1){
					relay[2] = 1;
				}else if(BaseVacuum == 0){
					relay[2] = 0;
				}
			}


			//Set Home
			if(registerFrame[0x01].U16 == 2){
				registerFrame[0x10].U16 = 2;
				registerFrame[0x01].U16 = 0;
			}
			//Set Shelves
			if (registerFrame[0x01].U16 == 1){
				registerFrame[0x01].U16 = 0;
				registerFrame[0x10].U16 = 1;
			}


			//Set Shelves
			if(registerFrame[0x10].U16 == 1){

				if ((bt2 == 0) && (bt1 == 1)) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0); //Go Up
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 700);
				} else if((bt2 == 1) && (bt1 == 0)){
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); //Go Down
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
				} else{
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				}

				// Handle bt5 press, to ensure that this function only trigger once.
				static uint8_t flagbt5 = 0;
				if(bt5 == 0) {
					static uint64_t timestampbt5 = 0;
					if(HAL_GetTick() > timestampbt5 && flagbt5 == 0) {
						timestampbt5 = HAL_GetTick() + 1000;
						ShelvePos[i] = QEIdata.TotalPos;
						i+=1;
						flagbt5 = 1;
					}
				} else {
					flagbt5 = 0;
				}

				// Set registerFrame 0x10 to 0 (idle) if finish running
				if(i > 4){
					i = 0;
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
					registerFrame[0x10].U16 = 0;
				}


				bt5prev = bt5;
				registerFrame[0x23].U16 = ShelvePos[0]*10;
				registerFrame[0x24].U16 = ShelvePos[1]*10;
				registerFrame[0x25].U16 = ShelvePos[2]*10;
				registerFrame[0x26].U16 = ShelvePos[3]*10;
				registerFrame[0x27].U16 = ShelvePos[4]*10;
				//timestamp = HAL_GetTick()+2000;
			}


			//Set Home Run To limit switch
			if(registerFrame[0x10].U16 == 2){

				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 800);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); //End effector Go Down

			}

			//Run Point Mode
				if(registerFrame[0x01].U16 == 8) {
					registerFrame[0x01].U16 = 0;
					registerFrame[0x10].U16 = 16;
					Arrived = 0;
					Goal = registerFrame[48].U16/10;
				}

				if(registerFrame[0x10].U16 == 16){
					MotorDrivePoint();
				}
				if(Arrived == 1 && registerFrame[0x10].U16 == 16) {
					registerFrame[0x10].U16 = 0;
					MotorDriveFlag = 0;
				}

			//Run Jog Mode
			if(registerFrame[0x01].U16 == 4){
				convert_to_string(registerFrame[0x21].U16, PickOrder, sizeof(PickOrder));
				convert_to_string(registerFrame[0x22].U16, PlaceOrder, sizeof(PlaceOrder));
				registerFrame[0x01].U16 = 0;
				registerFrame[0x10].U16 = 4;
				for(int i = 0;i<=4;i++){

					GoalPick[i] = ShelvePos[PickOrder[4-i]-'0'-1];
					GoalPlace[i] = ShelvePos[PlaceOrder[4-i]-'0'-1];
//					GoalPick[i] = i + 10;
//					GoalPlace[i] =(i+1)*100 +10;
				}
			}

		/////////////////START JOG////////////////////////////////////////////////////////////
			if(registerFrame[0x10].U16 == 4 && j < 5){
				GoPick();
			}else if(registerFrame[0x10].U16 == 8 && j < 5){
				GoPlace();
			}else if(j==5){
				registerFrame[0x10].U16 = 0;
				for(int i = 0;i<=4;i++){
					GoalPick[i] = 0;
					GoalPlace[i] = 0;
			}


				j = 0;
				a = 7;
			}
		///////////////////END JOG///////////////////////////////////////////////////////////

		}




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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  htim3.Init.Period = 64799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 3;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1145;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|Relay4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 Relay4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|Relay4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == GPIO_PIN_7){ //limitBottom
//		LimitBottomFlag = 1;
//	}
//
//	if(GPIO_Pin == GPIO_PIN_6){
////		mode = 1;
////		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	}
//
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		registerFrame[0].U16 = 22881;
		CountGripper += 1;
	}
	if (htim == &htim5) {
		upper += 1;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}

void convert_to_string(uint16_t number, char* buffer, int buffer_size) {
  if (buffer_size < 6) { // Ensure buffer size is at least 6 (for 5 digits + null terminator)    return; // Handle error (insufficient buffer size)
  }

  int index = 0;
  do {
    uint8_t digit = number % 10;
    buffer[index++] = digit + '0';
    number /= 10;
  } while (number > 0);
  buffer[index] = '\0';
}

void GoPick() {
	//a=0;
	b_check[5] = 1;
	Arrived = 0;

	Goal = GoalPick[j];
	MotorDrive();

	static uint64_t timestampVacuum = 0;
  //Gripper FW Vacuum On
	if((Arrived == 1)&&(flagpickend ==0)){
		relay[2] = 1; //Vacuum On

		relay[0] = 0; //Gripper push
		relay[1] = 1;

		a=1;
		if (GripperFlag == 0) {
			timestampVacuum = HAL_GetTick()+500;
			WaitGripper = CountGripper+150;
			GripperFlag = 1;
			flagpickend = 0;
		}
	}
	if((ActualGripper == 1) && (flagpickend == 0) && (HAL_GetTick()>= timestampVacuum)){ //leed switch Out And wait 200 ms
		relay[0] = 1; //Gripper pull
		relay[1] = 0;
		a=2;
		flagpick = 1;
		flagpickend = 1;
	}

	if((ActualGripper == 0) && (flagpick == 1) && (flagpickend == 1)){
		GripperFlag = 0;
		registerFrame[0x10].U16 = 8;
		flagpick = 0;
		a=3;
		MotorDriveFlag = 0;
	}

}

void GoPlace() {

	static uint64_t timestampVacuum = 0;
	if((ActualGripper == 0)){//Gripper BW before move
		Arrived = 0;
		Goal = GoalPlace[j]+10;
		MotorDrive();
		a = 4;
	}//Gripper FW Vacuum Off
	if((Arrived == 1)&&(flagplaceend == 0)){
		relay[2] = 0; //Vacuum Off

		relay[0] = 0;
		relay[1] = 1; //Gripper push
		if (GripperFlag == 0) {
			timestampVacuum = HAL_GetTick()+500;
			WaitGripper = CountGripper+8;
			GripperFlag = 1;
			flagplaceend = 0;
		}
	}
	if((ActualGripper == 1) && (ActualVacuum == 0) &&(flagplaceend == 0) && (HAL_GetTick()>= timestampVacuum) ){
		//wait 400 ms
		flagplace = 1;
		a = 5;
		relay[0] = 1; //pull
		relay[1] = 0; //
		flagplaceend = 1;

	}
	if((flagplace == 1) && (ActualGripper == 0)&&(flagplaceend == 1) ){
		GripperFlag = 0;
		registerFrame[0x10].U16 = 4;
		j += 1; //use
		a = 6;
		MotorDriveFlag = 0;
		flagplace = 0;
		flagplaceend = 0;
		flagpickend = 0;

	}
}


uint64_t Micros() {
//	static uint32_t timestamp = 0;
	uint32_t lower = 0;
	uint64_t time = 0;
	lower = __HAL_TIM_GET_COUNTER(&htim5);
	time = (upper << 32) | lower;
	return time;
}

void QEIEncoderPosVel_Update() {
	//CurrentPos = QEIdata.TotalPos-HomePos;
	//collect data
	QEIdata.TimeStamp[NEW] = Micros();
	QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim3);

	//Position 1 turn calculation
	QEIdata.QEIPostion_1turn[NEW] = QEIdata.Position[NEW] % 800;
	QEIdata.Angle = QEIdata.QEIPostion_1turn[NEW] * 360 / 800;
	//calculate dx
	int32_t diffPosition = QEIdata.Position[NEW] - QEIdata.Position[OLD];
	int32_t diff1turn = QEIdata.QEIPostion_1turn[NEW] - QEIdata.QEIPostion_1turn[OLD];

	//Handle Warp around
	if (diffPosition > 32400) {
		diffPosition -= 64800;
	}
	if (diffPosition < -32400) {
		diffPosition += 64800;
	}
	//Calculate Linear Position in mm unit
	if (diff1turn > 400) {
		QEIdata.QEIRound -= 1;
	}
	if (diff1turn < -400) {
		QEIdata.QEIRound += 1;
	}


	QEIdata.TotalPos = (QEIdata.QEIRound * 14) + QEIdata.QEIPostion_1turn[NEW] * 14 / 800; //linear pos in mm uint

			//calculate dt
	float diffTime = (QEIdata.TimeStamp[NEW] - QEIdata.TimeStamp[OLD])
			* 1e-6;

	//calculate angular velocity
	QEIdata.QEIAngularVelocity = diffPosition / diffTime;
	velodegree = QEIdata.QEIAngularVelocity;
	velodegree = (velodegree * 60) / 800;
	linearspeed[NEW] = velodegree * 14 / 60.0;


	float diffVel = linearspeed[NEW] - linearspeed[OLD];
	linearacc = diffVel/diffTime;
	//store value for next loop
	QEIdata.Position[OLD] = QEIdata.Position[NEW];
	QEIdata.TimeStamp[OLD] = QEIdata.TimeStamp[NEW];
	QEIdata.QEIPostion_1turn[OLD] = QEIdata.QEIPostion_1turn[NEW];
	linearspeed[OLD] = linearspeed[NEW];

}

void ReadLogicConv() {
	Lo1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); //Lo1 Pull
	Lo2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); //Lo2 Push
	Lo3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4); //Lo3
	Lo4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0); //Lo4
	if(Lo1 == 0 && Lo2 == 1){ //Push
		ActualGripper = 1;
	}else if(Lo1 == 1 && Lo2 == 0){ //Pull
		ActualGripper = 0;
	}
}
void ReadButton() {
	bt1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8); //BT1
	bt2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9); //BT2
	bt3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8); //BT3
	bt4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9); //BT4
	bt5 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); //BT5
}


void MotorDrive() {
	if (MotorDriveFlag == 0) {
		// Start: This box of code run only one time.
		StartTotalPos = QEIdata.TotalPos;
		MotorDriveTravelDistance = Goal - QEIdata.TotalPos;
		MotorDriveDampDistance = MotorDriveTravelDistance * 0.2;
		// End
		MotorDriveFlag = 1;
	}

	float PosNow = QEIdata.TotalPos - StartTotalPos;

	if((MotorDriveTravelDistance-PosNow) > 0.1 || ((MotorDriveTravelDistance-PosNow) < -0.1)){
		Arrived = 0;
		b_check[8] = PosNow;
		int8_t DriveDirection = 1; // direction is 1 if up, -1 if down.
		if (Goal <= StartTotalPos) {
			DriveDirection = -1;
			b_check[7] = 1;
		}

		// Trajectory generator
		if(DriveDirection == -1){
			if ((PosNow <= MotorDriveDampDistance) && (PosNow >= MotorDriveTravelDistance-MotorDriveDampDistance)) { // Middle
				RealVfeedback = 10;
				b_check[6]= 1;
			} else if (PosNow > MotorDriveDampDistance) { // Start
				//RealVfeedback = 1.5;
				RealVfeedback = (fabs(PosNow)+1)*13 / MotorDriveTravelDistance;
				b_check[6]= 2;
			}  else if (PosNow <= MotorDriveTravelDistance) {  //Hard Stop
				RealVfeedback = 0;
				b_check[6]= 3;
			} else if (PosNow < MotorDriveTravelDistance - MotorDriveDampDistance) {  //Stop
				//RealVfeedback = 1.5;
				RealVfeedback = (MotorDriveTravelDistance-PosNow)*10 / MotorDriveTravelDistance;
				b_check[6]= 4;
			}
		}
		if(DriveDirection == 1){
			if ((PosNow >= MotorDriveDampDistance) && (PosNow <= MotorDriveTravelDistance-MotorDriveDampDistance)) { // Middle
				RealVfeedback = 12;
				b_check[6]= 5;
			} else if (PosNow < MotorDriveDampDistance) { // Start
				//RealVfeedback = 2;
				RealVfeedback = (PosNow+1) * 12/ MotorDriveDampDistance;
				b_check[6]= 6;
			} else if (PosNow > MotorDriveTravelDistance) {  //Hard Stop
				RealVfeedback = 0;
				b_check[6]= 7;
			} else if (PosNow > MotorDriveTravelDistance - MotorDriveDampDistance) {  //Stop
				//RealVfeedback = 1.5;
				RealVfeedback = (MotorDriveTravelDistance-PosNow) * 12 / MotorDriveDampDistance;
				b_check[6]= 8;
			}
		}

		PIDVFeedback = Update_pid(&pid_control, MotorDriveTravelDistance-PosNow, 10, 12);

		if (fabs(PIDVFeedback) < fabs(RealVfeedback)) {
			RealVfeedback = PIDVFeedback;
		}

		RealVfeedback = RealVfeedback * DriveDirection;


		if (DriveDirection == 1) {  //go up
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
			b_check[9]= 4;
		} else {  //go down
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
			RealVfeedback = RealVfeedback * (-1);
			b_check[9]= 5;
		}

		if(fabs(RealVfeedback) < 1.6  && RealVfeedback!=0){
			if (DriveDirection == -1) {
				RealVfeedback = 1.25;
			} else {
				RealVfeedback = 1.6;
			}
		}

		duty_cycle_pid = fabs(RealVfeedback) * 4000 / 12;
		if(RealVfeedback == 0){
			duty_cycle_pid = 0;
		}

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle_pid);
	}else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		Arrived = 1;
		RealVfeedback = 0;
		b_check[6] = 6;
	}
}
void MotorDrivePoint() {
	if (MotorDriveFlag == 0) {
		// Start: This box of code run only one time.
		StartTotalPos = QEIdata.TotalPos;
		MotorDriveTravelDistance = Goal - QEIdata.TotalPos;
		MotorDriveDampDistance = MotorDriveTravelDistance * 0.3;
		// End
		MotorDriveFlag = 1;
	}

	float PosNow = QEIdata.TotalPos - StartTotalPos;

	if((MotorDriveTravelDistance-PosNow) > 0.1 || ((MotorDriveTravelDistance-PosNow) < -0.1)){
		Arrived = 0;
		b_check[8] = PosNow;
		int8_t DriveDirection = 1; // direction is 1 if up, -1 if down.
		if (Goal <= StartTotalPos) {
			DriveDirection = -1;
			b_check[7] = 1;
		}
//		if(MotorDriveTravelDistance<=100){
//			RealVfeedback = 3;
//		}

		// Trajectory generator
		if((DriveDirection == -1)&&(MotorDriveTravelDistance>0)){
			if ((PosNow <= MotorDriveDampDistance) && (PosNow >= MotorDriveTravelDistance-MotorDriveDampDistance)) { // Middle
				RealVfeedback = 7;
				b_check[6]= 1;
			} else if (PosNow > MotorDriveDampDistance) { // Start
				//RealVfeedback = 1.5;
				RealVfeedback = (fabs(PosNow)+1)*7 / MotorDriveTravelDistance;
				b_check[6]= 2;
			}  else if (PosNow <= MotorDriveTravelDistance) {  //Hard Stop
				RealVfeedback = 0;
				b_check[6]= 3;
			} else if (PosNow < MotorDriveTravelDistance - MotorDriveDampDistance) {  //Stop
				//RealVfeedback = 1.5;
				RealVfeedback = (MotorDriveTravelDistance-PosNow)*7 / MotorDriveTravelDistance;
				b_check[6]= 4;
			}
		}
		if((DriveDirection == 1)&&(MotorDriveTravelDistance>0)){
			if ((PosNow >= MotorDriveDampDistance) && (PosNow <= MotorDriveTravelDistance-MotorDriveDampDistance)) { // Middle
				RealVfeedback = 10;
				b_check[6]= 5;
			} else if (PosNow < MotorDriveDampDistance) { // Start
				//RealVfeedback = 2;
				RealVfeedback = (PosNow+1) * 10/ MotorDriveDampDistance;
				b_check[6]= 6;
			} else if (PosNow > MotorDriveTravelDistance) {  //Hard Stop
				RealVfeedback = 0;
				b_check[6]= 7;
			} else if (PosNow > MotorDriveTravelDistance - MotorDriveDampDistance) {  //Stop
				//RealVfeedback = 1.5;
				RealVfeedback = (MotorDriveTravelDistance-PosNow) * 10 / MotorDriveDampDistance;
				b_check[6]= 8;
			}
		}

		PIDVFeedback = Update_pid(&pid_control, MotorDriveTravelDistance-PosNow, 10, 12);

		if (fabs(PIDVFeedback) < fabs(RealVfeedback)) {
			RealVfeedback = PIDVFeedback;
		}

		RealVfeedback = RealVfeedback * DriveDirection;


		if (DriveDirection == 1) {  //go up
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
			b_check[9]= 4;
		} else {  //go down
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
			RealVfeedback = RealVfeedback * (-1);
			b_check[9]= 5;
		}

		if(fabs(RealVfeedback) < 1.6  && RealVfeedback!=0){
			if (DriveDirection == -1) {
				RealVfeedback = 1.2;
			} else {
				RealVfeedback = 1.6;
			}
		}

		duty_cycle_pid = fabs(RealVfeedback) * 4000 / 12;
		if(RealVfeedback == 0){
			duty_cycle_pid = 0;
		}

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle_pid);
	}else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		Arrived = 1;
		RealVfeedback = 0;
		b_check[6] = 6;
	}
}

void RelayDrive() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, relay[0]); // Pull
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, relay[1]); // Push
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, relay[2]); // Vacuum
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, relay[3]); // mode status

}

void ReadLimit(){

	LimitBottom = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);//bottom
	LimitTop = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);//top
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
