/**
  ******************************************************************************
  * @file    BSP/Src/main.c
  * @author  MCD Application Team
  * @brief   This example code shows how to use the STM324xG BSP Drivers
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  * This code was modified for use in ENCM 515 in 2022 and 2023
  * B. Tan
  * Note: DO NOT REGENERATE CODE/MODIFY THE IOC
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE 100
#define NUMBER_OF_TAPS 1000
#define AUDIO_SIZE (0x2BEEC - 44)/4
#define DELAY_NUMBER 1
//#define FUNCTIONAL_TEST
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__IO uint8_t UserPressButton = 0;

/* Wave Player Pause/Resume Status. Defined as external in waveplayer.c file */
__IO uint32_t PauseResumeStatus = IDLE_STATUS;

/* Counter for User button presses */
__IO uint32_t PressCount = 0;

TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef   sConfig;
uint32_t uwPrescalerValue = 0;
uint32_t uwCapturedValue = 0;

volatile int32_t *raw_audio = 0x802002C; // ignore first 44 bytes of header
volatile int new_sample_flag = 0;
static int sample_count = 0;
int16_t newSampleL = 0;
int16_t newSampleR = 0;
int16_t filteredSampleL;
int16_t filteredSampleR;
int16_t delayBuffer[DELAY_NUMBER];

// Probably don't need to modify these, they are for audio output
static volatile int32_t filteredOutBufferA[BUFFER_SIZE];
static volatile int32_t filteredOutBufferB[BUFFER_SIZE];
static volatile int bufchoice = 0;
volatile int bufArdy = 0;
volatile int bufBrdy = 0;
volatile int ready = 0;

extern I2S_HandleTypeDef       hAudioOutI2s;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void GPIOA_Init(void);
static int16_t ProcessSample(int16_t newsample);
static int16_t echoEffect(int16_t newsample);
static int16_t reverbEffect(int16_t newsample);
/* Private functions ---------------------------------------------------------*/


int16_t FixedFilterGet(float, float);
void FloatFilterInitB(void);
void FloatFilterInitA(void);



static int16_t filter_taps_b[3] = {10158, 20283, 10158};
static int16_t filter_taps_a[2] = {-2261, -10060};

/* let's have our history and output arrays defined somewhere */
int16_t history_b[31];
int16_t history_a[31];
int16_t newdata[NUMBER_OF_TAPS];



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);

  /* Configure the system clock to 100 MHz */
  SystemClock_Config();

  /* Configure GPIO so that we can probe PB2 with an Oscilloscope */
  GPIOA_Init();

  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;


  /* Initialize the Audio driver */
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 60, 8000) != 0) {
	  Error_Handler();
  }


  /* Initialize TIM3 peripheral to toggle with a frequency of ~ 8 kHz
   * System clock is 100 MHz and TIM3 is counting at the rate of the system clock
   * so 100 M / 8 k is 12500
   */
  TimHandle.Init.Period = 12499;
  TimHandle.Init.Prescaler = 0;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }

  ITM_Port32(30) = 0;
#ifndef FUNCTIONAL_TEST
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
	  /* Starting Error */
	  Error_Handler();
  }
#endif

  /******************************************************************************
   ******************************************************************************
   ******************************************************************************
   * Init Complete
   * BEGIN LAB 2 CODE HERE
   ******************************************************************************
   ******************************************************************************
   ******************************************************************************
   */



  static int i = 0;
  static int k = 0;
  static int start = 0;

  while (1) {

#ifdef FUNCTIONAL_TEST
	  new_sample_flag = 1;
		if (sample_count < AUDIO_SIZE) {
			  newSampleL = (int16_t)raw_audio[sample_count];
			  newSampleR = (int16_t)(raw_audio[sample_count] >> 16);
			  sample_count++;
		  } else {
			  sample_count = 0;
		  }
#endif

	if (new_sample_flag == 1) {
		filteredSampleL = ProcessSample(newSampleL); // "L"
		new_sample_flag = 0;

		/* Attempt at double buffering here: note that we are duplicating the sample for L and R, but this could be changed*/
		if (bufchoice == 0) {
			filteredOutBufferA[k] = ((int32_t)filteredSampleL << 16) + (int32_t)newSampleR; // copy the filtered output to both channels
		} else {
			filteredOutBufferB[k] = ((int32_t)filteredSampleL << 16) + (int32_t)newSampleR;
		}

		k++;
	}

	// once a buffer is full, we can swap to fill up the other buffer
	if (k == BUFFER_SIZE) {
		k = 0;
		if (bufchoice == 0) {
			bufchoice = 1;
			bufArdy = 1;
		} else {
			bufchoice = 0;
			bufBrdy = 1;
		}
	}
#ifndef FUNCTIONAL_TEST

	/* We'll use double buffering here, so that once one buffer is ready to go, we use
	 * BSP_AUDIO_OUT_ChangeBuffer to tell the DMA to send the audio to the DAC*/
	if(bufBrdy == 1 && ready == 1 && start == 1) {
		bufBrdy = 0;
		BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)(filteredOutBufferB), BUFFER_SIZE*2);
		ready = 0;
	}

	else if(bufArdy == 1 && ready == 1 && start == 1) {
		bufArdy = 0;
		BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)(filteredOutBufferA), BUFFER_SIZE*2);
		ready = 0;
	}

	/* AUDIO_OUT_PLAY is the BSP function essentially tells the audio chip to start working
	 * so every time the audio DAC receives some new data via DMA /I2S, it will play sound*/
	if (bufArdy == 1 && bufBrdy == 1 && start == 0) {
		BSP_AUDIO_OUT_Play((uint16_t*)(filteredOutBufferA), BUFFER_SIZE*2);
		start = 1;
		bufArdy = 0;
	}
#endif

  }


  //IIR FIXED POINT:
   // ITM_Port32(31) = 1;
  FixedFilterInitB(); // shifting samples
  for(int index = 30; index < NUMBER_OF_TAPS; index++)
      {
        newdata[index] = FixedFilterGet(raw_audio[index], new_data[index]);
        FixedFilterInitA();
      }
    //ITM_Port32(31) = 2;

}


void FloatFilterInitB(void)
{
        history_b[30] = raw_audio[0];
        history_b[29] = raw_audio[1];
        history_b[28] = raw_audio[2];
        history_b[27] = raw_audio[3];
        history_b[26] = raw_audio[4];
        history_b[25] = raw_audio[5];
        history_b[24] = raw_audio[6];
        history_b[23] = raw_audio[7];
        history_b[22] = raw_audio[8];
        history_b[21] = raw_audio[9];
        history_b[20] = raw_audio[10];
        history_b[19] = raw_audio[11];
        history_b[18] = raw_audio[12];
        history_b[17] = raw_audio[13];
        history_b[16] = raw_audio[14];
        history_b[15] = raw_audio[15];
        history_b[14] = raw_audio[16];
        history_b[13] = raw_audio[17];
        history_b[12] = raw_audio[18];
        history_b[11] = raw_audio[19];
        history_b[10] = raw_audio[20];
        history_b[9] = raw_audio[21];
        history_b[8] = raw_audio[22];
        history_b[7] = raw_audio[23];
        history_b[6] = raw_audio[24];
        history_b[5] = raw_audio[25];
        history_b[4] = raw_audio[26];
        history_b[3] = raw_audio[27];
        history_b[2] = raw_audio[28];
        history_b[1] = raw_audio[29];
}

void FloatFilterInitA(void)
{
        history_a[30] = new_data[0];
        history_a[29] = new_data[1];
        history_a[28] = mew_data[2];
        history_a[27] = new_data[3];
        history_a[26] = new_data[4];
        history_a[25] = new_data[5];
        history_a[24] = new_data[6];
        history_a[23] = new_data[7];
        history_a[22] = new_data[8];
        history_a[21] = new_data[9];
        history_a[20] = new_data[10];
        history_a[19] = new_data[11];
        history_a[18] = new_data[12];
        history_a[17] = new_data[13];
        history_a[16] = new_data[14];
        history_a[15] = new_data[15];
        history_a[14] = new_data[16];
        history_a[13] = new_data[17];
        history_a[12] = new_data[18];
        history_a[11] = new_data[19];
        history_a[10] = new_data[20];
        history_a[9] = new_data[21];
        history_a[8] = new_data[22];
        history_a[7] = new_data[23];
        history_a[6] = new_data[24];
        history_a[5] = new_data[25];
        history_a[4] = new_data[26];
        history_a[3] = new_data[27];
        history_a[2] = new_data[28];
        history_a[1] = new_data[29];
}






// FIXED:
int16_t FixedFilterGet (float32_t raw_data, float32 new_data) {
    // set the new sample as the head


    history_b[0] = raw_data;
    history_a[0] = new_data;

    // set up and do our convolution

    int32_t accumulator = 0;
    int tap = 0;
    int16_t temp;
    int tap2 = 0;

    for (tap = 0; tap < 2; tap++)
    {
        accumulator += filter_taps_b[tap]*history_b[tap];
        //accumulator += filter_taps_fixed[tap]*history[tap];
        for (tap2 = 0; tap2 < 1; tap2++)
        {
            accumulator -= filter_taps_a[tap2]*history_a[tap2];
        }
    }

    // shuffle the history along for the next one?

    for(tap = 29; tap > -1; tap--)
    {
        history_b[tap+1] = history_b[tap];
    }
    
    for(tap2 = 29; tap2 > -1; tap2--)
    {
        history_a[tap2+1] = history_a[tap2];
    }

    temp = (int16_t)(accumulator >> 15);

    return temp;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (KEY_BUTTON_PIN == GPIO_Pin)
  {
    while (BSP_PB_GetState(BUTTON_KEY) != RESET);
    UserPressButton = 1;
  }
}

/**
  * @brief  Toggle LEDs
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  BSP_LED_Toggle(LED3);
  HAL_Delay(100);
//  BSP_LED_Toggle(LED4);
//  HAL_Delay(100);
  BSP_LED_Toggle(LED5);
  HAL_Delay(100);
  BSP_LED_Toggle(LED6);
  HAL_Delay(100);
}

// This timer callback should trigger every 1/8000 Hz, and it emulates
// the idea of receiving a new sample peridiocally
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

//  BSP_LED_Toggle(LED4);
//  HAL_GPIO_TogglePin(SCOPE_CHECK_GPIO_Port, SCOPE_CHECK_Pin);

	// If we "miss" processing a sample, the new_sample_flag will still be
	// high on the trigger of the interrupt
	if (new_sample_flag == 1) {
		ITM_Port32(30) = 10;
	}

	// Otherwise, go to the raw audio in memory and "retrieve" a new sample every timer period
	// set the new_sample_flag high
	if (sample_count < AUDIO_SIZE) {
		newSampleL = (int16_t)raw_audio[sample_count];
		newSampleR = (int16_t)(raw_audio[sample_count] >> 16);
		sample_count++;

		if (sample_count >= AUDIO_SIZE) sample_count = 0;
		new_sample_flag = 1;
	}
}

int _write(int file, char* ptr, int len) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
}

static void GPIOA_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Configure GPIO pin : SCOPE_CHECK_Pin */
	  GPIO_InitStruct.Pin = SCOPE_CHECK_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(SCOPE_CHECK_GPIO_Port, &GPIO_InitStruct);

}

static int16_t ProcessSample(int16_t newsample) {
	return newsample;
}

static int16_t echoEffect(int16_t newsample) {
	return newsample;
}


static int16_t reverbEffect(int16_t newsample) {
	return newsample;
}

void BSP_AUDIO_OUT_TransferComplete_CallBack() {
	ready = 1;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
