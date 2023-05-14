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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../externalSrc/block_4Hz.h"
#include "../../externalSrc/block_10Hz.h"
#include "../../externalSrc/block_100Hz.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*ERROR HANDLER TO-DO*/
#define ERROR_START_CAN		0x10
#define ERROR_DATA_CAN		0x11
#define ERROR_TIMEOUT_CAN	0x12

/*CAN IDs*/
/*NB Low First Byte in received data*/
#define PC_WARNING		0x100

#define BUFFER_BLOCK1	0x101
#define BUFFER_BLOCK2	0x102
#define BUFFER_BLOCK3	0x103

#define GPS_BLOCK1		0x104
#define GPS_BLOCK2		0x105
#define GPS_BLOCK3		0x106

#define GYRO_BLOCK1		0x107
#define GYRO_BLOCK2		0x108

#define FWHEEL_BLOCK1	0x109
#define FWHEEL_BLOCK2	0x110

#define RWHEEL_BLOCK1	0x111
#define RWHEEL_BLOCK2	0x112
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//DATA BLOCKS
block_100Hz block100;

block_100Hz* pointer;

block_10Hz block10;
block_4Hz block4;

//CANBUS VARIABLES
CAN_RxHeaderTypeDef receiveHeader;
CAN_TxHeaderTypeDef transmitHeader;
CAN_FilterTypeDef sFilter;
uint8_t can_received_data[8];
uint8_t can_transmit_data[8];
uint32_t mailbox;
uint32_t canCurrTime;
uint16_t canTimeout = 10;
uint16_t workingIndex;

//TIMING VARIABLES
uint32_t startTime=0;
uint32_t nowTime=0;
uint16_t counter=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void filterSettings(void);
void User_Error_Handler(uint8_t error_id);
void codeData100Hz(block_100Hz *block);
void setDecodedData100Hz(block_100Hz *block);
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  filterSettings();
  if(HAL_CAN_Start(&hcan1) != HAL_OK){
	  User_Error_Handler(ERROR_START_CAN);
  }

  startTime=HAL_GetTick();

  /*Set transmitHeader fields*/
  transmitHeader.DLC=0;
  transmitHeader.IDE=CAN_ID_STD;
  transmitHeader.StdId=PC_WARNING;
  transmitHeader.ExtId=0x00;
  transmitHeader.RTR=CAN_RTR_REMOTE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GetTick()!=nowTime){
		  nowTime=HAL_GetTick();

		  /*100Hz BLOCK*/
		  if((nowTime-startTime)%10==0){
			  workingIndex=BUFFER_BLOCK1;
			  transmitHeader.StdId=workingIndex;
			  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
				  //Error Handler
			  }
			  canCurrTime=HAL_GetTick();
			  /*Wait until receive a message or goes to timeout*/
			  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout){
			  }
			  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==5){
				  block100.rpm=can_received_data[1]<<8 | can_received_data[0];
				  block100.tps=can_received_data[3]<<8 | can_received_data[2];
			  }else{
				  block100.rpm=0;
				  block100.tps=0;
				  if(HAL_GetTick()-canCurrTime>=canTimeout)
					  User_Error_Handler(ERROR_TIMEOUT_CAN);
				  else
					  User_Error_Handler(ERROR_DATA_CAN);
			  }


			  workingIndex=GYRO_BLOCK1;
			  transmitHeader.StdId=workingIndex;
			  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
				  //Error Handler
			  }
			  canCurrTime=HAL_GetTick();
			  /*Wait until receive a message or goes to timeout*/
			  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
			  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==6){
				  block100.accelX=can_received_data[0]<<8 | can_received_data[1];
				  block100.accelY=can_received_data[2]<<8 | can_received_data[3];
				  block100.accelZ=can_received_data[4]<<8 | can_received_data[5];
			  }else{
				  block100.accelX=0;
				  block100.accelY=0;
				  block100.accelZ=0;
				  if(HAL_GetTick()-canCurrTime>=canTimeout)
					  User_Error_Handler(ERROR_TIMEOUT_CAN);
				  else
					  User_Error_Handler(ERROR_DATA_CAN);
			  }


			  workingIndex=GYRO_BLOCK2;
			  transmitHeader.StdId=workingIndex;
			  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
				  //Error Handler
			  }
			  canCurrTime=HAL_GetTick();
			  /*Wait until receive a message or goes to timeout*/
			  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
			  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==6){
				  block100.gyroX=can_received_data[0]<<8 | can_received_data[1];
				  block100.gyroY=can_received_data[2]<<8 | can_received_data[3];
				  block100.gyroZ=can_received_data[4]<<8 | can_received_data[5];
			  }else{
				  block100.gyroX=0;
				  block100.gyroY=0;
				  block100.gyroZ=0;
				  if(HAL_GetTick()-canCurrTime>=canTimeout)
					  User_Error_Handler(ERROR_TIMEOUT_CAN);
				  else
					  User_Error_Handler(ERROR_DATA_CAN);
			  }


			  workingIndex=FWHEEL_BLOCK1;
			  transmitHeader.StdId=workingIndex;
			  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
				  //Error Handler
			  }
			  canCurrTime=HAL_GetTick();
			  /*Wait until receive a message or goes to timeout*/
			  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
			  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==4){
				  block100.potFSx=can_received_data[0];
				  block100.potFDx=can_received_data[1];
				  block100.potFOver=can_received_data[2];
				  block100.potFAccuracy=can_received_data[3];
			  }else{
				  block100.potFSx=0;
				  block100.potFDx=0;
				  block100.potFOver=0;
				  block100.potFAccuracy=0;
				  if(HAL_GetTick()-canCurrTime>=canTimeout)
					  User_Error_Handler(ERROR_TIMEOUT_CAN);
				  else
					  User_Error_Handler(ERROR_DATA_CAN);
			  }


			  workingIndex=FWHEEL_BLOCK2;
			  transmitHeader.StdId=workingIndex;
			  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
				  //Error Handler
			  }
			  canCurrTime=HAL_GetTick();
			  /*Wait until receive a message or goes to timeout*/
			  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
			  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==5){
				  block100.countFSx=can_received_data[0];
				  block100.countFDx=can_received_data[1];
				  block100.countFOver=can_received_data[2];
				  block100.dtF=can_received_data[3];
				  block100.steering_encoder=can_received_data[4];
			  }else{
				  block100.countFSx=0;
				  block100.countFDx=0;
				  block100.countFOver=0;
				  block100.dtF=0;
				  block100.steering_encoder=0;
				  if(HAL_GetTick()-canCurrTime>=canTimeout)
					  User_Error_Handler(ERROR_TIMEOUT_CAN);
				  else
					  User_Error_Handler(ERROR_DATA_CAN);
			  }


			  workingIndex=RWHEEL_BLOCK1;
			  transmitHeader.StdId=workingIndex;
			  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
				  //Error Handler
			  }
			  canCurrTime=HAL_GetTick();
			  /*Wait until receive a message or goes to timeout*/
			  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
			  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==4){
				  block100.potRSx=can_received_data[0];
				  block100.potRDx=can_received_data[1];
				  block100.potROver=can_received_data[2];
				  block100.potRAccuracy=can_received_data[3];
			  }else{
				  block100.potRSx=0;
				  block100.potRDx=0;
				  block100.potROver=0;
				  block100.potRAccuracy=0;
				  if(HAL_GetTick()-canCurrTime>=canTimeout)
					  User_Error_Handler(ERROR_TIMEOUT_CAN);
				  else
					  User_Error_Handler(ERROR_DATA_CAN);
			  }


			  workingIndex=RWHEEL_BLOCK2;
			  transmitHeader.StdId=workingIndex;
			  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
				  //Error Handler
			  }
			  canCurrTime=HAL_GetTick();
			  /*Wait until receive a message or goes to timeout*/
			  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
			  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==5){
				  block100.countRSx=can_received_data[0];
				  block100.countRDx=can_received_data[1];
				  block100.countROver=can_received_data[2];
				  block100.dtR=can_received_data[3];
				  block100.gear=can_received_data[4];
			  }else{
				  block100.countRSx=0;
				  block100.countRDx=0;
				  block100.countROver=0;
				  block100.dtR=0;
				  block100.gear=0;
				  if(HAL_GetTick()-canCurrTime>=canTimeout)
					  User_Error_Handler(ERROR_TIMEOUT_CAN);
				  else
					  User_Error_Handler(ERROR_DATA_CAN);
			  }

			  //BUILD SEND PACKET
			  setDecodedData100Hz(&block100);
			  codeData100Hz(&block100);

			  //SEND PACKET
			  HAL_UART_Transmit_DMA(&huart1, block100.coded_data, sizeof(block100.coded_data));


			  /*10Hz BLOCK*/
			  if(counter%10==0){
				  workingIndex=BUFFER_BLOCK2;
				  transmitHeader.StdId=workingIndex;
				  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
					  //Error Handler
				  }
				  canCurrTime=HAL_GetTick();
				  /*Wait until receive a message or goes to timeout*/
				  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
				  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==4){
					  block10.t_h2o=can_received_data[0];
					  block10.t_air=can_received_data[1];
					  block10.t_oil=can_received_data[2];
					  block10.vbb=can_received_data[3];
				  }else{
					  block10.t_h2o=0;
					  block10.t_air=0;
					  block10.t_oil=0;
					  block10.vbb=0;
					  if(HAL_GetTick()-canCurrTime>=canTimeout)
						  User_Error_Handler(ERROR_TIMEOUT_CAN);
					  else
						  User_Error_Handler(ERROR_DATA_CAN);
				  }


				  workingIndex=BUFFER_BLOCK3;
				  transmitHeader.StdId=workingIndex;
				  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
					  //Error Handler
				  }
				  canCurrTime=HAL_GetTick();
				  /*Wait until receive a message or goes to timeout*/
				  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
				  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==8){
					  block10.lambda1_avg=can_received_data[0];
					  block10.lambda1_raw=can_received_data[1];
					  block10.klambda1=can_received_data[3]<<8 | can_received_data[2];
					  block10.injLow=can_received_data[5]<<8 | can_received_data[4];
					  block10.injHigh=can_received_data[7]<<8 | can_received_data[6];
				  }else{
					  block10.lambda1_avg=0;
					  block10.lambda1_raw=0;
					  block10.klambda1=0;
					  block10.injLow=0;
					  block10.injHigh=0;
					  if(HAL_GetTick()-canCurrTime>=canTimeout)
						  User_Error_Handler(ERROR_TIMEOUT_CAN);
					  else
						  User_Error_Handler(ERROR_DATA_CAN);
				  }

				  //BUILD SEND PACKET
				  setDecodedData10Hz(&block10);
				  codeData10Hz(&block10);

				  //SEND PACKET
				  HAL_UART_Transmit_DMA(&huart1, block10.coded_data, sizeof(block10.coded_data));
			  }/*END BLOCK 10Hz*/

			  /*4Hz BLOCK - GPS (10Hz)*/
			  if(counter%10==0){
				  workingIndex=GPS_BLOCK1;
				  transmitHeader.StdId=workingIndex;
				  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
					//Error Handler
				  }
				  canCurrTime=HAL_GetTick();
				  /*Wait until receive a message or goes to timeout*/
				  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
				  if(HAL_GetTick()-canCurrTime>=canTimeout){		//Modificato per debug timeout
					  HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
					  receiveHeader.StdId = 1;
				  }
				  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && receiveHeader.DLC==8){
					  block4.hour=can_received_data[0];
					  block4.minute=can_received_data[1];
					  block4.second=can_received_data[2];
					  block4.microsecond=can_received_data[4]<<8 | can_received_data[3];
					  block4.info=can_received_data[5];
					  block4.HDOP1=can_received_data[6];
					  block4.HDOP2=can_received_data[7];
				  }else{
					  block4.hour=0;
					  block4.minute=0;
					  block4.second=0;
					  block4.microsecond=0;
					  block4.info=0;
					  block4.HDOP1=0;
					  block4.HDOP2=0;
					  if(HAL_GetTick()-canCurrTime>=canTimeout)
						  User_Error_Handler(ERROR_TIMEOUT_CAN);
					  else
						  User_Error_Handler(ERROR_DATA_CAN);
				  }



				  workingIndex=GPS_BLOCK2;
				  transmitHeader.StdId=workingIndex;
				  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
					  //Error Handler
				  }
				  canCurrTime=HAL_GetTick();
				  /*Wait until receive a message or goes to timeout*/
				  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);
				  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==8){
					  block4.Latitude1=can_received_data[1]<<8 | can_received_data[0];
					  block4.Latitude2=can_received_data[5]<<24 | can_received_data[4]<<16 | can_received_data[3]<<8 | can_received_data[2];
					  block4.Vel1=can_received_data[6];
					  block4.Vel2=can_received_data[7];
				  }else{
					  block4.Latitude1=0;
					  block4.Latitude2=0;
					  block4.Vel1=0;
					  block4.Vel2=0;
					  if(HAL_GetTick()-canCurrTime>=canTimeout)
						  User_Error_Handler(ERROR_TIMEOUT_CAN);
					  else
						  User_Error_Handler(ERROR_DATA_CAN);
				  }


				  workingIndex=GPS_BLOCK3;
				  transmitHeader.StdId=workingIndex;
				  if (HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_transmit_data, &mailbox) != HAL_OK){
					  //Error Handler
				  }
				  canCurrTime=HAL_GetTick();
				  /*Wait until receive a message or goes to timeout*/
				  while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &receiveHeader, can_received_data)!=HAL_OK && HAL_GetTick()-canCurrTime<canTimeout);

				  if(receiveHeader.StdId==workingIndex && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_DATA && HAL_GetTick()-canCurrTime<canTimeout && receiveHeader.DLC==8){
					  block4.Longitude1=can_received_data[3]<<24 | can_received_data[2]<<16 | can_received_data[1]<<8 | can_received_data[0];
					  block4.Longitude2=can_received_data[7]<<24 | can_received_data[6]<<16 | can_received_data[5]<<8 | can_received_data[4];
				  }else{
					  block4.Longitude1=0;
					  block4.Longitude2=0;
					  if(HAL_GetTick()-canCurrTime>=canTimeout)
						  User_Error_Handler(ERROR_TIMEOUT_CAN);
					  else
						  User_Error_Handler(ERROR_DATA_CAN);
				  }


				  //BUILD SEND PACKET
				  setDecodedData4Hz(&block4);
				  codeData4Hz(&block4);

				  //SEND PACKET
				  HAL_UART_Transmit_DMA(&huart1, block4.coded_data, sizeof(block4.coded_data));
			  }/*END BLOCK 4Hz*/

			  /*add 1 to counter variable (1 every 10 ms)*/
			  counter++;
		  }/*END BLOCK 100Hz*/

		  /*Reset once a minute*/
		  if(counter>6000){
			  counter=0;
		  }
		  if(nowTime-startTime>60000){
			  startTime=0;
		  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
void filterSettings(void){
	sFilter.FilterBank = 0;
	sFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilter.FilterIdHigh = 0;
	sFilter.FilterIdLow = 0;
	sFilter.FilterMaskIdHigh = 0;
	sFilter.FilterMaskIdLow = 0;
	sFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilter.FilterActivation = ENABLE;
	sFilter.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilter);
}

/*When DMA finishes call stop*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_DMAStop(&huart1);
}

/*TO-DO*/
void User_Error_Handler(uint8_t error_id){
	switch(error_id){
		case 0x10:
			/*TO-DO*/
			/*USE AN LED AND BLINK N TIMES TO IDENTIFY THE ERROR*/
			break;
		default:
			/*TO-DO*/
			break;
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
