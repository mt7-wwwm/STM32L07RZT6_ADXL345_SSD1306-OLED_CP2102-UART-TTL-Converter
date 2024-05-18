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
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int counter2; //cia counter2, nes buvo naudojamas tim2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
#define adxl_address 0x53<<1

#define BUFFER_LEN 10
#define NUM_CHANNELS 3

char data_UART[100];
/* for handling I2C errors */
void MyErrorHandlerI2C()
{  
	
}
//sensoriui
uint8_t data_rec[6];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//ADXL345 kintamieji
int16_t x_raw, y_raw, z_raw;
float x_g, y_g, z_g;

uint8_t chipid=0;

//skaiciavimu kintamieji
float channel_data[BUFFER_LEN][NUM_CHANNELS];
float vidutine_kvadratine_verte[NUM_CHANNELS];

uint8_t buffer_index = 0;


void adxl_write (uint8_t reg, uint8_t value)
{
	uint8_t data [2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit (&hi2c1, adxl_address, data, 2, 100);
}

//nuskaitymas
void adxl_read (uint8_t reg)
{
	HAL_I2C_Mem_Read (&hi2c1, adxl_address, reg, 1, (uint8_t *)data_rec, 6, 100);
}
void adxl_read_address (uint8_t reg) {
	HAL_I2C_Mem_Read (&hi2c1, adxl_address, reg, 1, &chipid, 1, 100);
}
void adxl_init (void)
{
	adxl_read(0x00); //icializavimas:
	adxl_write (0x2d, 0x00); //isvalo visus bitus (cia POWER_CTL registras)
	adxl_write (0x2d, 0x08); //matuoja bit-1, wakeup, 0 8hz dazniu
	adxl_write (0x31, 0x01); //+-4g diapazone(data_format)
}	

//toliau issaugojimas informacijos apdorojimui
void save_data(float x, float y, float z) {
    static uint8_t buffer_index = 0;

    //issaugo i buferius
    channel_data[buffer_index][0] = x; // x asies data
    channel_data[buffer_index][1] = y; // y asies data
    channel_data[buffer_index][2] = z; // z asies data

    // bufferio indekso didinimas
    buffer_index = (buffer_index + 1) % BUFFER_LEN;
}
//nuskaitymas (cia 0x32 - 0x37 registrais)
void read_ADXL_data(void){
	adxl_read(0x32);
	x_raw = ((data_rec[1] << 8) | data_rec[0]);
	y_raw = ((data_rec[3] << 8) | data_rec[2]);
	z_raw = ((data_rec[5] << 8) | data_rec[4]);

	x_g = x_raw * 0.0078;
	y_g = y_raw * 0.0078;
	z_g = z_raw * 0.0078;
	save_data(x_g, y_g, z_g);
}
//kiekvienos asies rms apskaiciavimas
void calculate_rms() {
    for (uint8_t j = 0; j < NUM_CHANNELS; j++) {
        float suma = 0.0;
        for (uint8_t i = 0; i < BUFFER_LEN; i++) {
            suma += pow(channel_data[i][j], 2); //kvadratu sumos
        }
        vidutine_kvadratine_verte[j] = sqrt(suma / BUFFER_LEN); // RMS vertes, kur [0] - x verciu masyvas, [1] - y ir [2] - z
				
		}
		
}

// atvaizdavimo funkcija
void display_rms() {
		char rms_value[20];

    // Display X-axis RMS value
    ssd1306_SetCursor(16,1);
    sprintf(rms_value, "X_rms: %.5f", vidutine_kvadratine_verte[0]);
    ssd1306_WriteString(rms_value, Font_6x8, 0x01);

    // Display Y-axis RMS value
    ssd1306_SetCursor(16,9);
    sprintf(rms_value, "Y_rms: %.5f", vidutine_kvadratine_verte[1]);
    ssd1306_WriteString(rms_value, Font_6x8, 0x01);

    // Display Z-axis RMS value
    ssd1306_SetCursor(16,17);
    sprintf(rms_value, "Z_rms: %.5f", vidutine_kvadratine_verte[2]);
    ssd1306_WriteString(rms_value, Font_6x8, 0x01);
	
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //atnaujinant uzsidega LED2 [green]
	
    ssd1306_UpdateScreen(); // Update OLED display
		
}

void dataUART(void)
{
    sprintf(data_UART, "X_g: %.4f, Y_g: %.4f, Z_g: %.4f \r\n", x_g, y_g, z_g);
		HAL_UART_Transmit_IT(&huart4, (uint8_t *)data_UART, strlen(data_UART));		
	
	if (HAL_UART_Transmit_IT(&huart4, (uint8_t *)data_UART, strlen(data_UART)) != HAL_OK)
    {
        // Handle transmit error
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //esant klaidai ijungia LED2
    }
    else
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // mirksi LED2 [zalias ant maketo]
    }
	
}


//cia pabandymui su rms verciu perdavimu UART, nerodo z_rms verciu, susikuriant nauja buferi, pvz uart2[100], verciu isvis nesiuncia
//void dataUART2(void)
//{
//    sprintf(data_UART, "\n X_rms: %.5f, Y_rms: %.5f, Z_rms: %.5f", vidutine_kvadratine_verte[0], vidutine_kvadratine_verte[1], vidutine_kvadratine_verte[2]);
//		HAL_UART_Transmit_IT(&huart4, (uint8_t *)data_UART, strlen(data_UART));		
//	
//	if (HAL_UART_Transmit_IT(&huart4, (uint8_t *)data_UART, strlen(data_UART)) != HAL_OK)
//    {
//        // Handle transmit error
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //esant klaidai ijungia LED2
//    }
//    else
//    {
//        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // mirksi LED2 [zalias ant maketo]
//    }
//	
//}
	

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART4) {
			printf("UART Error Occurred\r\n");
        // Toggle a different LED or print error to UART
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Assuming another LED is connected to PA6
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART4) {
			printf("UART Transmission Complete\r\n");
    //    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Assuming another LED is connected to PA7
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		
    if (htim == &htim2) //cia kai t = 200 [ms]
    {
			counter2++;
			read_ADXL_data();
			dataUART();
    
			if(counter2>=10){ //cia kai t = 200 * 10 [ms] = 2 [s]
				calculate_rms();
				display_rms();
				
				//dataUART2();				
				//isvalomi visi kaupiami masyvai bei counteris			
				//cia isvalant rms vertes nematomos vertes watch1 lange
//				vidutine_kvadratine_verte[0]=0;
//				vidutine_kvadratine_verte[1]=0;
//				vidutine_kvadratine_verte[2]=0;
//				
				counter2=0;				
				
				}
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
		if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) //run TIM6 timer
   {
    Error_Handler();
   }
	 
	 adxl_init();
	 ssd1306_Init();
	 
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 40000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 160;
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
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_2_GPIO_Port, LD_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD_2_Pin */
  GPIO_InitStruct.Pin = LD_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD_2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
