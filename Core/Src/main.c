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
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "fonts.h"
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "mk_dht11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FLASH_EraseInitTypeDef EraseInitStruct;

typedef struct
{
	uint8_t tensec;
	uint8_t tenmin;
	uint8_t tenhour;
	uint8_t tendate;
	uint8_t tenmonth;
	uint16_t realyear;
	uint16_t vibralevel;

	float avertemperature;
	float averwaterlevel;
	float averdistance;
}datameet;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char modem[80] = { 0 };
char final[50] = { 0 };
char hcsr[50] = { 0 };
char humitemp[30] = { 0 };
char send[30] = { 0 };

uint8_t timer = 0;
uint16_t data;
uint32_t tempco = 0;
uint32_t humico = 0;
dht11_t dht;
uint16_t finaltemp = 0;
uint16_t finalhumi = 0;
uint16_t stack = 0;
uint16_t warning = 0;

char buffer[2000] = "";
char yearcom[50] = "";
char hourcom[50] = "";
char mincom[50] = "";
char seccom[50] = "";
char datecom[50] = "";
char monthcom[50] = "";

uint8_t sec = 0;
uint8_t min = 0;
uint8_t hour = 0;
uint8_t date = 0;
uint8_t month = 0;
uint8_t year = 0;
uint8_t tenyear = 0;

datameet timedata;

float distance;
float distco = 0;
float finaldist = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_Delayus(uint16_t);

void ProtectRESET();
void DS1302_RESET();
void DS1302_WriteByte(uint8_t,uint8_t);
void Clk();
uint16_t DS1302_ReadByte(uint16_t);
uint8_t DS1302_tenone(uint8_t);

void HCSR();
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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  init_dht11(&dht,&htim1,GPIOB,GPIO_PIN_9);

  // !!!! Time Setting Code !!!!
  /*DS1302_WriteByte(0x80,0x00);
  DS1302_WriteByte(0x82,0x28);
  DS1302_WriteByte(0x84,0x16);
  DS1302_WriteByte(0x86,0x17);
  DS1302_WriteByte(0x88,0x01);
  DS1302_WriteByte(0x8C,0x25);*/


  ILI9341_FillScreen(WHITE);
  ILI9341_DrawText("Start",FONT4,95,170,BLACK,WHITE);

  HAL_Delay(5000);

  ILI9341_FillScreen(WHITE);

  while(1)
   {
 	  ILI9341_FillScreen(BLACK);

 	  HAL_Delay(1000);

 	  ILI9341_FillScreen(WHITE);

 	  HAL_Delay(1000);

   ILI9341_DrawText("AT",FONT4,100,160,BLACK, WHITE);


   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT\r\n",strlen("AT\r\n"),1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*)modem,"OK") == NULL)
   {
 	  ILI9341_DrawText("AT Fail",FONT4,100,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  ILI9341_FillScreen(WHITE);

 	  continue;
   }
   memset(modem,0,sizeof(modem));

   ILI9341_DrawText("AT OK",FONT4,100,160,BLACK, WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT CFUN",FONT4,80,160,BLACK,WHITE);

   HAL_Delay(1000);

   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT+CFUN=1,1\r\n",strlen("AT+CFUN=1,1\r\n"), 1000);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("Modem reseting..",FONT4,60,160,BLACK,WHITE);

   HAL_Delay(30000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT CREG",FONT4,90,160,BLACK,WHITE);

   HAL_Delay(1000);

   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT+CREG?\r\n",strlen("AT+CREG?\r\n"),1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if (strstr((const char*)modem, "0,1") == NULL && strstr((const char*)modem, "0,5") == NULL)
   {
 	  ILI9341_DrawText("AT CREG Fail",FONT4,70,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  continue;
   }

   ILI9341_DrawText("AT CREG OK",FONT4,70,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT+CGDCONT",FONT4,60,160,BLACK,WHITE);

   HAL_Delay(1000);

   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT+CGDCONT=1,\"IP\",\"lte.sktelecom.com\"\r\n",strlen("AT+CGDCONT=1,\"IP\",\"lte.sktelecom.com\"\r\n"),1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*)modem,"OK") == NULL)
   {
 	  ILI9341_DrawText("AT CGDCONT Fail",FONT4,50,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  continue;
   }

   ILI9341_DrawText("AT CGDCONT OK",FONT4,50,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT SICS TYPE",FONT4,50,160,BLACK,WHITE);

   HAL_Delay(1000);

   memset(modem,0,sizeof(modem));

   HAL_UART_Transmit(&huart1,(uint8_t*)"AT^SICS=0,\"conType\",\"GPRS0\"\r\n",strlen("AT^SICS=0,\"conType\",\"GPRS0\"\r\n"),1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*)modem,"OK") == NULL)
   {
 	  ILI9341_DrawText("AT SICS TYPE Fail",FONT4,50,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  continue;
   }

   ILI9341_DrawText("AT SICS TYPE OK",FONT4,50,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT SICS APN",FONT4,60,160,BLACK,WHITE);

   HAL_Delay(1000);

   memset(modem,0,sizeof(modem));

   HAL_UART_Transmit(&huart1,(uint8_t*)"AT^SICS=0,\"apn\",\"lte.sktelecom.com\"\r\n",strlen("AT^SICS=0,\"apn\",\"lte.sktelecom.com\"\r\n"),1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*)modem, "OK") == NULL)
   {
 	  ILI9341_DrawText("AT SICS APN Fail",FONT4,50,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  continue;
   }

   ILI9341_DrawText("AT SICS APN OK",FONT4,50,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT SISS TYPE",FONT4,60,160,BLACK,WHITE);

   HAL_Delay(1000);
   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT^SISS=0,\"srvType\",\"Socket\"\r\n",strlen("AT^SISS=0,\"srvType\",\"Socket\"\r\n"),1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*) modem, "OK") == NULL)
   {
 	  ILI9341_DrawText("AT SISS TYPE Fail",FONT4,50,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  continue;
   }

   ILI9341_DrawText("AT SISS TYPE OK",FONT4,60,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT SISS ID",FONT4,70,160,BLACK,WHITE);

   HAL_Delay(1000);
   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT^SISS=0,\"conId\",0\r\n",strlen("AT^SISS=0,\"conId\",0\r\n"),1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*)modem,"OK") == NULL)
   {
 	  ILI9341_DrawText("AT SISS ID Fail",FONT4,60,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  continue;
   }

   ILI9341_DrawText("AT SISS ID OK",FONT4,70,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT SISS address",FONT4,60,160,BLACK,WHITE);

   HAL_Delay(1000);

   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT^SISS=0,\"address\",\"socktcp://49.254.169.124:5001\"\r\n",strlen("AT^SISS=0,\"address\",\"socktcp://49.254.169.124:5001\"\r\n"), 1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*)modem, "OK") == NULL)
   {
 	  ILI9341_DrawText("AT SISS address Fail",FONT4,40,160,BLACK,WHITE);
 	  HAL_Delay(1000);

 	  continue;
   }

   ILI9341_DrawText("AT SISS address OK",FONT4,50,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT+CGATT",FONT4,80,160,BLACK,WHITE);

   HAL_Delay(1000);
   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT+CGATT?\r\n",strlen("AT+CGATT?\r\n"), 1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   if(strstr((const char*)modem,"1") == NULL)
   {
 	 ILI9341_DrawText("AT+CGATT Fail",FONT4,70,160,BLACK,WHITE);
 	 HAL_Delay(1000);

 	 continue;
   }

   ILI9341_DrawText("AT CGATT OK",FONT4,70,160,BLACK,WHITE);

   HAL_Delay(1000);

   ILI9341_FillScreen(WHITE);

   ILI9341_DrawText("AT+CGACT",FONT4,70,160,BLACK,WHITE);

   HAL_Delay(1000);
   memset(modem,0,sizeof(modem));
   HAL_UART_Transmit(&huart1,(uint8_t*)"AT+CGACT=1,1\r\n",strlen("AT+CGACT=1,1\r\n"), 1000);
   HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

   HAL_Delay(5000);

   if(strstr((const char*)modem,"1,1") == NULL)
    {
  	  ILI9341_DrawText("AT+CGACT Fail",FONT4,80,160,BLACK,WHITE);
  	  HAL_Delay(1000);

  	  continue;
    }

    ILI9341_DrawText("AT+CGACT OK",FONT4,70,160,BLACK,WHITE);

      HAL_Delay(1000);

      ILI9341_DrawText("Communication OK?",FONT4,60,160,BLACK,WHITE);

      HAL_Delay(1000);

      HAL_UART_Transmit(&huart1,(uint8_t*)"AT^SISO=0\r\n",strlen("AT^SISO=0\r\n"),1000);
      HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

      HAL_Delay(3000);

      if(strstr((const char*)modem, "OK") == NULL)
      {
     	 ILI9341_DrawText("Communication Fail",FONT4,40,160,BLACK,WHITE);
     	 HAL_Delay(1000);

     	 continue;
      }
      ILI9341_FillScreen(WHITE);

      ILI9341_DrawText("Communication OK",FONT4,50,160,BLACK,WHITE);

      HAL_Delay(1000);

      ILI9341_FillScreen(WHITE);

      break;
   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    Clk();

	    sprintf(seccom,"  %d/%02d/%02d    %02d:%02d:%02d\r\n\r\n",timedata.realyear,timedata.tenmonth,timedata.tendate,timedata.tenhour,timedata.tenmin,timedata.tensec);
	    ILI9341_DrawText(seccom,FONT4,1,260,BLACK,WHITE);

	    readDHT11(&dht);
	    HCSR();

	    uint8_t temp = dht.temperature;
	    uint8_t humi = dht.humidty;

	    sprintf(hcsr,"Distance : %.2fcm",distance);
	    sprintf(humitemp,"Temp : %d*C Humi : %drh",temp,humi);

	    ILI9341_DrawText(hcsr,FONT4,15,120,BLACK, WHITE);
	    ILI9341_DrawText(humitemp,FONT4,15,170,BLACK,WHITE);

	    distco += distance;
	    tempco += temp;
	    humico += humi;
	    stack++;

	    if(timedata.tenmin % 10 == 0 && timedata.tensec == 0)
	    {
	    finaldist = distco/stack;
	    finaltemp = tempco/stack;
	    finalhumi = humico/stack;

		sprintf(final,"%d%02d%02d%02d%02d%02d %d %d %.2f",timedata.realyear,timedata.tenmonth,timedata.tendate,timedata.tenhour,timedata.tenmin,timedata.tensec,finaltemp,finalhumi,finaldist);

		HAL_UART_Transmit(&huart1,(uint8_t*)"AT+CGACT?\r\n",strlen("AT+CGACT?\r\n"),1000);
		HAL_UART_Receive(&huart1,(uint8_t*)modem,sizeof(modem),1000);

		if((strstr((const char*)modem,"1,1") != NULL))
		{
		sprintf(send,"AT^SISW=0,%d\r\n",strlen(final));

		HAL_UART_Transmit(&huart1,(uint8_t*)send,strlen(send),1000);

		HAL_Delay(2000);

		HAL_UART_Transmit(&huart1,(uint8_t*)final,strlen(final),1000);
		}

	    distco = 0;
		tempco = 0;
		humico = 0;

		finalhumi = 0;
		finaltemp = 0;
		finaldist = 0;
		stack = 0;

	    }

	    HAL_Delay(800);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, CLK_Pin|DAT_Pin|RST_Pin|RED_Pin
                          |GREEN_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|WarningLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|RESET_Pin|CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT_DATA_GPIO_Port, DHT_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin DAT_Pin RST_Pin LED_Pin
                           RESET_Pin CS_Pin RED_Pin GREEN_Pin
                           TRIG_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|DAT_Pin|RST_Pin|LED_Pin
                          |RESET_Pin|CS_Pin|RED_Pin|GREEN_Pin
                          |TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WarningLED_Pin */
  GPIO_InitStruct.Pin = WarningLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WarningLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin DHT_DATA_Pin */
  GPIO_InitStruct.Pin = DC_Pin|DHT_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_Delayus(uint16_t time)
{
	HAL_TIM_Base_Start(&htim1);

	__HAL_TIM_SET_COUNTER(&htim1,0);

	while(__HAL_TIM_GET_COUNTER(&htim1) < time);

	HAL_TIM_Base_Stop(&htim1);
}

void ProtectRESET()
{
	uint8_t protect = 0x8E;

	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);

	for(int i = 0; i < 8; i++)
	{
	   HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(protect >> i) & 0x01);

	   HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
	   HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
	}

	for(int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,0x00);

		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
	    HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);
}

void DS1302_RESET()
{
	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
}

void DS1302_WriteByte(uint8_t ch,uint8_t data)
{

	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);
	for(int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(ch >> i) & 0x01);

		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
	}

	HAL_Delay(10);

	for(int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(data >> i) & 0x01);

		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);
}

uint16_t DS1302_ReadByte(uint16_t ch)
{
	uint16_t time = 0;

	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);

	for(int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(ch >> i) & 0x01);
		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = DAT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	for(int i = 0; i < 8; i++)
	{
	 if(HAL_GPIO_ReadPin(DAT_GPIO_Port,DAT_Pin) == GPIO_PIN_SET)
	 {
		time |= 1 << i;
	 }

	 HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);

	 GPIO_InitStruct.Pin = DAT_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	return time;
}

uint8_t DS1302_tenone(uint8_t num)
{
	uint8_t ten = (num >> 4) * 10;
	uint8_t one = num & 0x0f;
	uint8_t sum = ten+one;

	return sum;
}

void Clk()
{
	  sec = DS1302_ReadByte(0x81);
	  min = DS1302_ReadByte(0x83);
	  hour = DS1302_ReadByte(0x85);
	  date = DS1302_ReadByte(0x87);
	  month = DS1302_ReadByte(0x89);
	  year = DS1302_ReadByte(0x8D);

	  timedata.tensec = DS1302_tenone(sec);
	  timedata.tenmin = DS1302_tenone(min);
	  timedata.tenhour = DS1302_tenone(hour);
	  timedata.tendate = DS1302_tenone(date);
	  timedata.tenmonth = DS1302_tenone(month);
	  tenyear = DS1302_tenone(year);
	  timedata.realyear = tenyear + 2000;
}

void HCSR()
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);

    HAL_Delayus(10);

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin) != GPIO_PIN_SET);

    HAL_TIM_Base_Start(&htim2);

    __HAL_TIM_SET_COUNTER(&htim2,0);

    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin) == GPIO_PIN_SET);

    HAL_TIM_Base_Stop(&htim2);

    uint32_t sonic = __HAL_TIM_GET_COUNTER(&htim2);

    distance = (sonic * 0.0343) / 2;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3)
	{
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3))
	    {
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
	    }
		else
		{
	    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
		}
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
