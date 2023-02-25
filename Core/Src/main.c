/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "semphr.h"
#include "queue.h"
#include <string.h>
#include <max7219.h>
#include <max7219_matrix.h>
#include <llist.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void GameLoop( void *pvParameters );
void ButtonHandler( void *pvParameters );
void CollisionCheck( void *pvParameters );
void Task3( void *pvParameters );
void FoodPositionGenerator( void *pvParameters );
void ButtonHandler_A( void *pvParameters );
void ButtonHandler_B( void *pvParameters );
void ButtonHandler_C( void *pvParameters );
void ButtonHandler_D( void *pvParameters );
void ScreenDriver( void *pvParameters );

QueueHandle_t xDirectionQ, xBoardQ, xSnakeQ, xCellStateQ, xFoodPositionQ;

SemaphoreHandle_t xButtonPressedSem, xGameOverSem, xCheckCollison, xCollisionChecked, xGenerateFood, xFoodGenerated, xRepaintScreen;
SemaphoreHandle_t xHandleButtonPressA, xHandleButtonPressB, xHandleButtonPressC, xHandleButtonPressD;
SemaphoreHandle_t xScreenDriverMutex;

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MAX7219_MatrixInit(&hspi1, SPI_CS_GPIO_Port, SPI_CS_Pin);
  MAX7219_MatrixUpdate();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  xButtonPressedSem = xSemaphoreCreateBinary();
  xGameOverSem = xSemaphoreCreateBinary();
  xCheckCollison = xSemaphoreCreateBinary();
  xCollisionChecked = xSemaphoreCreateBinary();
  xGenerateFood = xSemaphoreCreateBinary();
  xFoodGenerated = xSemaphoreCreateBinary();
  xHandleButtonPressA = xSemaphoreCreateBinary();
  xHandleButtonPressB = xSemaphoreCreateBinary();
  xHandleButtonPressC = xSemaphoreCreateBinary();
  xHandleButtonPressD = xSemaphoreCreateBinary();
  xRepaintScreen = xSemaphoreCreateBinary();

  xScreenDriverMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

//  int test[4][8][8];
//  memset(test, 0, sizeof(test));
  xDirectionQ = xQueueCreate( 10, sizeof( char ) );
  xBoardQ = xQueueCreate( 2, sizeof( int ) *4 *8 *8 );
  xSnakeQ = xQueueCreate( 2, sizeof(  llist ) );
  xCellStateQ = xQueueCreate( 5, sizeof( int ));
  xFoodPositionQ = xQueueCreate( 5, sizeof( int ) * 3);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(GameLoop, "GameLoop", configMINIMAL_STACK_SIZE*8, NULL, 1, NULL );
  //xTaskCreate(ButtonHandler, "ButtonHandler", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
  //xTaskCreate(CollisionCheck, "CollisionCheck", configMINIMAL_STACK_SIZE*8, NULL, 2, NULL );
  xTaskCreate(Task3, "Task3", configMINIMAL_STACK_SIZE, NULL, 5, NULL );
  xTaskCreate(FoodPositionGenerator, "FoodPositionGenerator", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(ButtonHandler_A, "ButtonHandler_A", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
  xTaskCreate(ButtonHandler_B, "ButtonHandler_B", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
  xTaskCreate(ButtonHandler_C, "ButtonHandler_C", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
  xTaskCreate(ButtonHandler_D, "ButtonHandler_D", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
  xTaskCreate(ScreenDriver, "ScreenDriver", configMINIMAL_STACK_SIZE*8, NULL, 1, NULL );


  // LOW
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin PA6 */
  GPIO_InitStruct.Pin = SPI_CS_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_A3_Pin PB10 */
  GPIO_InitStruct.Pin = Button_A3_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_B_Pin PB4 PB5 */
  GPIO_InitStruct.Pin = Button_B_Pin|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	static  BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);


	xSemaphoreGiveFromISR( xHandleButtonPressB, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  /* USER CODE END EXTI3_IRQn 0 */

  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	static  BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);


		xSemaphoreGiveFromISR( xHandleButtonPressD, &xHigherPriorityTaskWoken );

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  /* USER CODE END EXTI4_IRQn 0 */
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	static  BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);


		xSemaphoreGiveFromISR( xHandleButtonPressC, &xHigherPriorityTaskWoken );

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  /* USER CODE END EXTI9_5_IRQn 0 */
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	static  BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);


		xSemaphoreGiveFromISR( xHandleButtonPressA, &xHigherPriorityTaskWoken );

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  /* USER CODE END EXTI15_10_IRQn 0 */
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

void ButtonHandler_A( void *pvParameters ){
	char buttonLetter = 'a';

	for(;;)
	    {

			if( xSemaphoreTake( xHandleButtonPressA, ( TickType_t ) portMAX_DELAY) == pdTRUE );
			xQueueSend( xDirectionQ, ( void * ) &buttonLetter,  portMAX_DELAY );
			xSemaphoreGive( xButtonPressedSem );
	    }
};
void ButtonHandler_B( void *pvParameters ){
	char buttonLetter = 'b';
	for(;;)
		    {

				if( xSemaphoreTake( xHandleButtonPressB, ( TickType_t ) portMAX_DELAY) == pdTRUE );
				xQueueSend( xDirectionQ, ( void * ) &buttonLetter,  portMAX_DELAY );
				xSemaphoreGive( xButtonPressedSem );
		    }
};
void ButtonHandler_C( void *pvParameters ){
	char buttonLetter = 'c';
	for(;;)
		    {

				if( xSemaphoreTake( xHandleButtonPressC, ( TickType_t ) portMAX_DELAY) == pdTRUE );
				xQueueSend( xDirectionQ, ( void * ) &buttonLetter,  portMAX_DELAY );
				xSemaphoreGive( xButtonPressedSem );
		    }
};
void ButtonHandler_D( void *pvParameters ){
	char buttonLetter = 'd';
	for(;;)
		    {

				if( xSemaphoreTake( xHandleButtonPressD, ( TickType_t ) portMAX_DELAY) == pdTRUE );
				xQueueSend( xDirectionQ, ( void * ) &buttonLetter,  portMAX_DELAY );
				xSemaphoreGive( xButtonPressedSem );
		    }

};

void ScreenDriver( void * pvParameters ){
	int board[4][8][8];

	for(;;)
			    {
		if( xSemaphoreTake( xRepaintScreen, ( TickType_t ) portMAX_DELAY) == pdTRUE );
		xQueueReceive( xBoardQ, &board, ( TickType_t ) 10 );

		MAX7219_paintPoints(0, board[0]);
		MAX7219_paintPoints(1, board[1]);
		MAX7219_paintPoints(2, board[2]);
		MAX7219_paintPoints(3, board[3]);
		MAX7219_MatrixUpdate();
			    }

};


void GameLoop( void * pvParameters )

{
	int board[4][8][8];
	char direction = 'a';
	char previousDirection = 'a';
	int delay = 350;
	int cellState = 0;
	int foodTimeout = 3;
	int food[3];
	int grow = 0;

	memset(board, 0, sizeof(board));

	llist *snake = llist_create(NULL);
	llist_push(snake, 0,4,1);
	llist_push(snake, 0,4,2);

	llist_printSnake(snake, board);

	if( xSemaphoreTake( xScreenDriverMutex, ( TickType_t ) portMAX_DELAY) == pdTRUE ) ;

	xQueueSend( xBoardQ, ( void * ) &board,  1 );
	xSemaphoreGive( xRepaintScreen );

	xSemaphoreGive(xScreenDriverMutex);

	vTaskDelay(delay);

    for(;;)
    {

    	if( xSemaphoreTake( xButtonPressedSem, ( TickType_t ) 1) == pdTRUE ){
    		previousDirection = direction;
    		xQueueReceive( xDirectionQ, &direction, ( TickType_t ) 10 );
    		switch (direction)
			{
				case 'b':
					if (previousDirection == 'd') direction = previousDirection;

					break;
				case 'd':

					if (previousDirection == 'b') direction = previousDirection;
					break;
				case 'c':

					if (previousDirection == 'a') direction = previousDirection;
					break;
				case 'a':

					if (previousDirection == 'c') direction = previousDirection;

					break;
				default:
					break;
			}

    		xQueueReset(xDirectionQ);
    	}else {


    		switch (direction)
			{
			case 'b':
				cellState = checkOccupiedDown( snake, board);
				if (previousDirection != 'd'){
					if (cellState == 1 || cellState == -1) xSemaphoreGive( xGameOverSem );
					if (cellState == 2 ) {
						foodTimeout = 0;
						grow = 1;
					}
					 moveDown(snake, grow);
				}
				break;
			case 'd':
				cellState = checkOccupiedUp( snake, board);

				if (previousDirection != 'b') {

					if (cellState == 1 || cellState == -1) xSemaphoreGive( xGameOverSem );
					if (cellState == 2 ) {
										foodTimeout = 0;
										grow = 1;
									}
					moveUp(snake, grow);
				}
				break;
			case 'c':
				cellState = checkOccupiedLeft( snake, board);
				if (previousDirection != 'a'){
					if (cellState == 1 || cellState == -1) xSemaphoreGive( xGameOverSem );
					if (cellState == 2 ) {
										foodTimeout = 0;
										grow = 1;
									}
					 moveLeft(snake, grow);
				}
				break;
			case 'a':
				cellState = checkOccupiedRight( snake, board);


				if (previousDirection != 'c'){
					if (cellState == 1 || cellState == -1) xSemaphoreGive( xGameOverSem );
					if (cellState == 2 ) {
						foodTimeout = 0;
						grow = 1;
					}
					moveRight(snake, grow);
				}


				//if (checkOccupiedRight( *snake, board) != 0 ) xSemaphoreGive( xGameOverSem );
//				xQueueSend( xBoardQ, ( void * ) &board,  1 );
//				xQueueSend( xSnakeQ, ( void * ) &snake,  1 );
//				xSemaphoreGive( xCheckCollison );
//				if(xSemaphoreTake( xCollisionChecked, ( TickType_t ) portMAX_DELAY) == pdTRUE);
//				xQueueReceive( xCellStateQ, &cellState, ( TickType_t ) 10 );
				break;
			default:
				break;
			}
    		if (grow == 1) if (delay > 150) delay = delay - 1;
    		grow = 0;

    		memset(board, 0, sizeof(board));

    		if (foodTimeout == 4){
				xQueueSend( xSnakeQ, ( void * ) &snake,  1 );
				xSemaphoreGive( xGenerateFood );

				if(xSemaphoreTake( xFoodGenerated, ( TickType_t ) portMAX_DELAY) == pdTRUE);
				xQueueReceive( xFoodPositionQ, &food, ( TickType_t ) 10 );

			}
    		if (foodTimeout >= 4) board[food[0]][food[2]][food[1]] = 2;

        	llist_printSnake(snake, board);

//        	board[3][0][0] = 1; test code
//        	board[3][0][1] = 1;
//        	board[3][2][0] = 1;

        	if( xSemaphoreTake( xScreenDriverMutex, ( TickType_t ) portMAX_DELAY) == pdTRUE ) ;

			xQueueSend( xBoardQ, ( void * ) &board,  1 );
			xSemaphoreGive( xRepaintScreen );

			xSemaphoreGive(xScreenDriverMutex);


    		vTaskDelay(delay);
    		foodTimeout++;
    	}


    }

}

void FoodPositionGenerator(void *  pvParameters){
	srand((unsigned int)time(NULL));
	llist *snakeBody;
	int display;
	int x;
	int y;
	int valid = 0;
	int food[3];

	for(;;) {
		if( xSemaphoreTake( xGameOverSem, ( TickType_t ) 0) == pdTRUE ) break;
		display = rand() % 4;

		if(xSemaphoreTake( xGenerateFood, ( TickType_t ) portMAX_DELAY) == pdTRUE);
		xQueueReceive( xSnakeQ, &snakeBody, ( TickType_t ) 10 );


		display = rand() % 4;
		while(valid == 0 ){
			struct node *curr = *snakeBody;
			valid = 1;

			x = rand() % 8;
			y = rand() % 8;

			while (curr != NULL) {

				if (curr->x && curr->y == y && curr->display) {
					valid = 0;
				};

				if (valid == 0) break;
				curr = curr->next;
			}
		}

		food[0] = display;
		food[1] = x;
		food[2] = y;

		xQueueSend( xFoodPositionQ, (void *) &food,  1 );
		xSemaphoreGive( xFoodGenerated );
	}
}


void CollisionCheck( void * pvParameters )
{

	int board[4][8][8];
	llist *snakeBody = llist_create(NULL);
	int cellState;
    for(;;)
    {
    	if(xSemaphoreTake( xCheckCollison, ( TickType_t ) portMAX_DELAY) == pdTRUE);

    	xQueueReceive( xBoardQ, &board, ( TickType_t ) 10 );
    	xQueueReceive( xSnakeQ, &snakeBody, ( TickType_t ) 10 );


    	cellState = checkOccupiedRight( snakeBody, board);
    	if (cellState != 0 ) xSemaphoreGive( xGameOverSem );
    	xQueueSend( xCellStateQ, (void *) &cellState,  1 );
    	xSemaphoreGive( xCollisionChecked );

    }
}

void ButtonHandler( void * pvParameters )
{

	char buttonPressed;
    for(;;)
    {

    	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0) {
      		buttonPressed = 'a';
      		xQueueReset(xDirectionQ);
      		xQueueSend( xDirectionQ, ( void * ) &buttonPressed,  1 );
      		xSemaphoreGive( xButtonPressedSem );
      	}
    	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0) {
    	      		buttonPressed = 'b';
    	      		xQueueReset(xDirectionQ);
    	      		xQueueSend( xDirectionQ, ( void * ) &buttonPressed,  1 );
    	      		xSemaphoreGive( xButtonPressedSem );
    	      	}
    	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0) {
    	      		buttonPressed = 'c';
    	      		xQueueReset(xDirectionQ);
    	      		xQueueSend( xDirectionQ, ( void * ) &buttonPressed,  1 );
    	      		xSemaphoreGive( xButtonPressedSem );
    	      	}
    	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0) {
    	      		buttonPressed = 'd';
    	      		xQueueReset(xDirectionQ);
    	      		xQueueSend( xDirectionQ, ( void * ) &buttonPressed,  1 );
    	      		xSemaphoreGive( xButtonPressedSem );
    	      	}
    	vTaskDelay(50);

    }
}

void Task3( void * pvParameters )
{
	const uint64_t IMAGES[] = {
	  0xff00000000000000,
	  0xff18000000000000,
	  0xff24180000000000,
	  0xff5a241800000000,
	  0xff7e5a2418000000,
	  0xff7e7e5a24180000,
	  0xff5a7e7e5a241800,
	  0xff5a5a7e7e5a2418,
	  0xff5a5a7e7e5a2418
	};
	const int IMAGES_LEN = sizeof(IMAGES)/8;

	if(xSemaphoreTake( xGameOverSem, ( TickType_t ) portMAX_DELAY) == pdTRUE);
	if( xSemaphoreTake( xScreenDriverMutex, ( TickType_t ) portMAX_DELAY) == pdTRUE ) ;
	for (int i = 0; i < IMAGES_LEN; ++i) {
		MAX7219_MatrixSetRow64(0, IMAGES[i]);
		MAX7219_MatrixSetRow64(1, IMAGES[i]);
		MAX7219_MatrixSetRow64(2, IMAGES[i]);
		MAX7219_MatrixSetRow64(3, IMAGES[i]);
		MAX7219_MatrixUpdate();
		vTaskDelay(200);

	}
    for(;;)
    {

    	 MAX7219_MatrixSetRow64(0, CHR('D'));
		MAX7219_MatrixSetRow64(1, CHR('E'));
		  MAX7219_MatrixSetRow64(2, CHR('A'));
		MAX7219_MatrixSetRow64(3, CHR('D'));
		MAX7219_MatrixUpdate();
    }
}





/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
