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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;
typedef struct{
	uint8_t reportID;
	uint8_t modifier;
	uint8_t reserved;
	uint8_t key1;
	uint8_t key2;
	uint8_t key3;
	uint8_t key4;
	uint8_t key5;
	uint8_t key6;

} keyPress;

typedef struct{
	uint8_t reportID;
	uint8_t mediaKey1;
}mediaKeyPress;

keyPress keyPad = {0x01, 0, 0, 0, 0, 0, 0, 0, 0};
mediaKeyPress mediaKeyPad = {0x02, 0};

void pressKey(uint8_t k1){
	keyPad.modifier = 0;
	    keyPad.key1 = k1;
	    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyPad, sizeof(keyPad));
	    HAL_Delay(100);
	    keyPad.modifier = 0;
	        keyPad.key1 = 0;

	        USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyPad, sizeof(keyPad));
	        HAL_Delay(100);
};

void mediaKey(uint16_t m1){
	mediaKeyPad.mediaKey1 = m1;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaKeyPad, sizeof(mediaKeyPad));
		    HAL_Delay(100);
		        mediaKeyPad.mediaKey1 = 0;

		        USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaKeyPad, sizeof(mediaKeyPad));
		        HAL_Delay(100);

};

void releaseAll(){
    keyPad.modifier = 0;
    keyPad.key1 = 0;
    keyPad.key2 = 0;
    keyPad.key3 = 0;
    keyPad.key4 = 0;
    keyPad.key5 = 0;
    keyPad.key6 = 0;
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyPad, sizeof(keyPad));
    HAL_Delay(5);
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  int lastButtonState[16] = {0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int prevSong = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	  int dimdit = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	  int f8 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	  int dimAligned = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	  int nextSong = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	  int f5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	  int undo = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
	  int enter = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	  int isoCircle = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	  int circle = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	  int line= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	  int esc = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	  int mirror = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	  int copy = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	  int trim = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	  int move = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
	  if (isoCircle == 0 && lastButtonState[0] == 1){
	  pressKey(0x08);
	  pressKey(0x0F);
	  pressKey(0x28);
	  pressKey(0x0C);
	  pressKey(0x28);
	  }
	  else if(nextSong == 0 && lastButtonState[1] == 1){
		  mediaKey(0xCD);
	  }
	  else if(prevSong == 0 && lastButtonState[2] == 1){
		  mediaKey(0xCE);
	  	  }
	  else if(dimdit == 0 && lastButtonState[3] == 1){
		pressKey(0x07);
  		pressKey(0x0C);
	   	pressKey(0x10);
	   	pressKey(0x07);
	  	pressKey(0x0C);
	  	pressKey(0X17);
	  	pressKey(0x28);
	  	  	  }
	  else if(f8 == 0 && lastButtonState[4] == 1){
		  pressKey(0x41);
		  pressKey(0x28);
	  	  	  }
	  else if(dimAligned == 0 && lastButtonState[5] == 1){
		  pressKey(0x07);
		  pressKey(0x0C);
		  pressKey(0x10);
		  pressKey(0x04);
		  pressKey(0x0F);
		  pressKey(0x0C);
		  pressKey(0x28);
	  	  	  }
	  else if(f5 == 0 && lastButtonState[6] == 1){
		  pressKey(0x3E);
	  	  	  }
	  else if(undo == 0 && lastButtonState[7] == 1){
		  pressKey(0x18);
		  pressKey(0x11);
		  pressKey(0x07);
		  pressKey(0x28);
		  pressKey(0x1E);
		  pressKey(0x28);
	  	  	  }
	  else if(enter == 0 && lastButtonState[8] == 1){
	  	  pressKey(0x28);
	  	  	  }
	  else if(circle == 0 && lastButtonState[9] == 1){
		  pressKey(0x06);
		  pressKey(0x0C);
		  pressKey(0x15);
		  pressKey(0x28);
	  	  	  }
	  else if(line == 0 && lastButtonState[10] == 1){
		  pressKey(0x0F);
		  pressKey(0x28);
	  	  	  }
	  else if(esc == 0 && lastButtonState[11] == 1){
		  pressKey(0x29);
	  	  	  }
	  else if(mirror == 0 && lastButtonState[12] == 1){
		  pressKey(0x10);
		  pressKey(0x0C);
		  pressKey(0x15);
		  pressKey(0x28);
	  	  	  }
	  else if(copy == 0 && lastButtonState[13] == 1){
		  pressKey(0x06);
		  pressKey(0x12);
		  pressKey(0x13);
		  pressKey(0x28);
	  }
	  else if(trim == 0 && lastButtonState[14] == 1){
		  pressKey(0x17);
		  pressKey(0x15);
		  pressKey(0x28);
	  	  	  }
	  else if(move == 0 && lastButtonState[15] == 1){
		  pressKey(0x10);
		  pressKey(0x12);
		  pressKey(0x19);
		  pressKey(0x08);
		  pressKey(0x28);
	  	  	  }



	  lastButtonState[0] = isoCircle;
	  lastButtonState[1] = nextSong;
	  lastButtonState[2] = prevSong;
	  lastButtonState[3] = dimdit;
	  lastButtonState[4] = f8;
	  lastButtonState[5] = dimAligned;
	  lastButtonState[6] = f5;
	  lastButtonState[7] = undo;
	  lastButtonState[8] = enter;
  	  lastButtonState[9] = circle;
	  lastButtonState[10] = line;
  	  lastButtonState[11] = esc;
  	  lastButtonState[12] = mirror;
  	  lastButtonState[13] = copy;
  	  lastButtonState[14] = trim;
  	  lastButtonState[15] = move;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA1 PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
