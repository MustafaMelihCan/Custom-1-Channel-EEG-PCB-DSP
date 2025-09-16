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
#include "arm_math.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUM_STAGES   3          // you said: 2 for bandpass + 1 notch
#define BLOCK_SIZE   512        // number of samples you process per call (can be half DMA buffer)
#define FS_HZ       500
//#define FFT_LEN     512
//#define FFT_BINS    (FFT_LEN/2)

////// old code end here
#define SEG_LEN    256
#define HOP        128          // 50% overlap
#define NFFT       SEG_LEN       // main bin length
#define K_TARGET   7			// # of subbins in 2 seconds
#define FFT_BINS   (NFFT/2)     // FFT bins
#define DF_HZ      ((float)FS_HZ/(float)SEG_LEN)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t Buffer_Length = 512;
uint16_t adc_buf[512];   // DMA buffer for ADC samples
uint16_t work_buf1[512];
uint16_t work_buf2[512];// a buffer for me to use to do math dc removal/windowing/filtering etc.
uint16_t out_buf[512];
volatile uint8_t which_half = 0;
volatile uint8_t buffer_ready = 0;
static float dac_scale = 15.0f;   //
/////////////////////////////////////////////////////////////////////////// Filter coefficients
float biquad_coeffs[NUM_STAGES * 5] = {
    0.0366f, 0.0731f, 0.0366f, 1.3909f, -0.5372f,   // LPF coefiicients b0, b1, b2, a1, a2 BE CAREFUL THIS PROGRAM FLIPS THE SIGNS OF A COEFFICIENTS
	1.0000f,-1.6180f, 1.0000f, 1.6019f, -0.9801f,  //Notch coefiieicnts b0, b1, b2, a1, a2
	1.0000f,-2.0000f, 1.0000f, 1.9800f, -0.9801f,  //HPF coefiieicnts b0, b1, b2, a1, a2
};
/////////////////////////////////////////////////////////////////////////// Filter coefficients
/////////////////////////////////////////////////////////////////////////// Filtering Variables
float biquad_state[NUM_STAGES * 2];
arm_biquad_cascade_df2T_instance_f32 biquadS;

static float in_f[BLOCK_SIZE];
static float out_f[BLOCK_SIZE];
/////////////////////////////////////////////////////////////////////////// Filtering Variables

///////////////////////////////////////////////////////////////////////////	FFT variables
//arm_rfft_fast_instance_f32 rfft;
//static float fft_in[FFT_LEN];
//static float fft_out[FFT_LEN];
//float mag[(FFT_LEN/2)+1];
//float hann[FFT_LEN];
//static uint16_t k_max = 55;
float EEG_DATA[6]; // indices: 0 = delta; 1 = theta; 2 = alpha; 3 = beta; 4 = 50hz hum; 5 = power band of 12HZ+24Hz
///// old code end here

// Welch / Hamming state
static float ring[SEG_LEN*2];          // copy-free overlap
static int   wr = 0, newCount = 0, K = 0;

arm_rfft_fast_instance_f32 rfft;
static float ham[SEG_LEN];             // Hamming window
static float U = 0.0f;                 // window power (≈0.3974 for Hamming 256)
static float seg[SEG_LEN];
static float fft_in[NFFT], fft_out[NFFT];

static float psd_accum[NFFT/2 + 1]; // psd means power spectral density = power of frequency spectrum basically
static float psd[NFFT/2 + 1];



///////////////////////////////////////////////////////////////////////////	FFT variables


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static void print_stats(uint16_t *p, size_t n);
static void DMA_Tester(void);
static void Dc_Removal(const uint16_t *in, uint16_t *out, size_t n);
static void ADC_DAC_test (uint16_t *in, uint16_t *out, size_t n);
static void process_block(uint16_t *in, uint16_t *out, size_t n);
static void my_fft_block (const uint16_t *in, float *hann, uint16_t k_max);  // FFT pipeline
static void init_hamming(void);
static void Bandpower(const float *in);
static void EEG_OUT(void);
static void init_hann_window(void); // comment from here to go back to previous version
static void Bandpower_from_psd(const float *Sxx, float df);
static void welch_process_segment(void);
static inline void welch_push_sample(float x);
static void EEG_OUT2(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
int __io_putchar(int ch) {
  extern UART_HandleTypeDef huart2;
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
  return ch;
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
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buf, Buffer_Length);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_TIM_Base_Start(&htim8);
  arm_biquad_cascade_df2T_init_f32(&biquadS, NUM_STAGES,biquad_coeffs,biquad_state);
  arm_rfft_fast_init_f32(&rfft, NFFT);   // Init FFT
  init_hamming();
  memset(psd_accum, 0, sizeof(psd_accum));
  K = 0;
//  init_hann_window();                       // Fill hann[] table /////// to go back to previous version uncomment
  printf("delta,theta,alpha,beta\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (buffer_ready){
		  Dc_Removal(adc_buf, work_buf1, Buffer_Length); // the processing that we do
		  process_block(work_buf1, out_buf,Buffer_Length);

//		  ADC_DAC_test(adc_buf, out_buf, Buffer_Length);
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)out_buf, Buffer_Length, DAC_ALIGN_12B_R);
//		  //// uncomment for the previus fft block
//		  my_fft_block(out_buf,hann,k_max); //FFT function
//		  Bandpower(mag);
//		  EEG_OUT();

		  for (size_t i=0; i<Buffer_Length; i++){
		      welch_push_sample(out_f[i]);   // analyze filtered float stream
		  }



		  buffer_ready  = 0;
	  }




//	  DMA_Tester();
//	  HAL_Delay(20);
//	  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);

//	  printf("Hello UART2!\r\n");
//	  HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 15999;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//  if (hadc->Instance == ADC3) {
//    uint16_t raw = HAL_ADC_GetValue(hadc);           // 0..4095
//    uint32_t mv  = (uint32_t)raw * 3300 / 4095;      // assume VDDA ≈ 3.3 V
//    printf("%u,%lu\r\n", raw, (unsigned long)mv);    // CSV to serial
//  }
//}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC3) {
    	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    	which_half = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC3) {
    	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    	which_half = 2;
    	buffer_ready = 1;
    }
}

static void Dc_Removal(const uint16_t *in, uint16_t *out, size_t n){
	uint32_t sum = 0;
	for (size_t i=0; i<n;i++) {sum += in[i];} 						// here we take the sum of the buffer by looping through each element and accumulating
		uint32_t mean = sum/n; 										// here we take the average

	for (size_t i=0; i<n;i++){
		int32_t v = (int32_t)in[i]-(int32_t)mean;
		int32_t x = (int32_t)in[i];
		int32_t y = v + 2048; 									 // we are shifting it to the mid scale so that DAC can write it
//		int32_t y = in;
//		int32_t y = v;
		if (y < 0) y = 0;										 // failsafe ifs to make our output bounded
		if (y > 4095) y = 4095;
		out[i] = (uint16_t)y;
	}
}
static void ADC_DAC_test (uint16_t *in, uint16_t *out, size_t n){
	for (size_t i = 0; i < n; i++) {
	            out[i] = in[i];
	        }
}

static void print_stats(uint16_t *p, size_t n){
	uint32_t sum = 0;											//initilize the variables that we are going to use
	for (size_t i=0; i<n;i++) {uint16_t v = p[i]; sum += v;} 	// here we take the sum of the buffer by looping through each element and accumulating
	uint32_t mean = sum/n;										 // here we take the average
	uint32_t mean_mV = mean * (3000/4095);
	printf("mean=%u (%lumV)\r\n", mean,(unsigned long)mean_mV);
}

static void DMA_Tester(void){
	if (which_half == 1){
			  print_stats(&adc_buf[0], 256);
			  which_half = 0;
		  }

		  else if (which_half == 2){
			  print_stats(&adc_buf[256], 256);
			  which_half = 0;
		  }
}

static inline uint16_t sat12f(float v){ // making sure that our function is bounded
	if (v < 0.f) v = 0.f;
	if (v > 4095.f) v = 4095.f;
	return (uint16_t)(v+0.5f);
}

static void process_block(uint16_t *in, uint16_t *out, size_t n){
	for (size_t i = 0; i<n; i++) {
		in_f[i] = ((float)in[i]-2048.0f)/2048; // i didn't understand why is this the case
	}
	arm_biquad_cascade_df2T_f32(&biquadS, in_f, out_f, n);
	for (size_t i=0; i<n; i++){
		float v = (out_f[i]*dac_scale) * 2048.0f + 2048.0f;
		out[i] = sat12f(v);
	}
}

static void welch_process_segment(void){
	int start = (wr - SEG_LEN + SEG_LEN) % SEG_LEN;

	for (int i = 0; i<SEG_LEN; i++) seg[i] = ring[start + i] * ham[i]; // apply the hammin window to our subsegment

	for (int i = 0; i<SEG_LEN; i++) fft_in[i] = seg[i]; // i fill my fft buffer with windowed subsegment

	for (int i = SEG_LEN; i<NFFT; i++) fft_in[i] = 0.0f; // if the rest of the buffer is not needed fill with zeros

	arm_rfft_fast_f32(&rfft, fft_in, fft_out, 0); // we take the fft of our subwindow/subsegment

	static float power[NFFT/2 + 1]; //this will be the power of buffer fft removes half the samples and makes from 1hz to 500 hz

	power[0] =  fft_out[0]*fft_out[0];   // DC
	power[NFFT/2] = fft_out[1]*fft_out[1];   // Nyquist
	arm_cmplx_mag_squared_f32(&fft_out[2], &power[1], (NFFT/2 - 1)); // we here square and store

	for (int k = 0; k <= NFFT/2; k++) psd_accum[k] += power[k];

	if (++K >= K_TARGET){ // check if we reached 7 subsamples and if so:
		for (int k=0; k<=NFFT/2;k++){
			float two = (k==0 || k==NFFT/2) ? 1.0f : 2.0f; // if the frequencies are not nyquist or zero then w_i = w_i*2 because they represent both the -w_i and +w_i energies
			psd[k] = (two * psd_accum[k]) / ((float)K * (float)FS_HZ * (float)SEG_LEN * U);
		}
		memset(psd_accum, 0, sizeof(psd_accum));
		K = 0;

		Bandpower_from_psd(psd, DF_HZ); // calculate the band power
		EEG_OUT2();
	}
}

static inline void welch_push_sample(float x){
	ring [wr] = x; ring[wr + SEG_LEN] = x; // mirror
	wr = (wr + 1) % SEG_LEN;
	if(++newCount >= HOP){
		newCount = 0;
		welch_process_segment();
	}
}

// also uncomment to go back to previous versi
//static void my_fft_block (const uint16_t *in, float *hann, uint16_t k_max){
//	const float LSB_V = 3.3f/ 4095.0f; // this is the voltage of one bit
//	for (int i = 0; i < FFT_LEN; i++){
//		float x = ((float)in[i] - 2048.0f) * LSB_V; // -2048 because removing 1.65Dc offset, LSB_V is voltage of one count so multiply to get volts
//		fft_in[i] = x * hann[i];
//	}
//	arm_rfft_fast_f32(&rfft, fft_in, fft_out, 0);
//
//	for (int k = 0; k < FFT_BINS; k++){
//		float re = fft_out[2*k + 0];
//		float im = fft_out[2*k + 1]; // because it is a complex array we store a complex number in two array points
//		mag[k] = (re*re + im*im) / (FFT_LEN * FFT_LEN); // here we get the magnitude array of our
//	}
//	for (int k = 0; k <= k_max; k++){
//		float freq = (FS_HZ *(float)k) / (float)FFT_LEN;
////		printf("%2d, %6.2f Hz, %f\r\n", k, freq, mag[k]);
//	}
////	printf("---\r\n");
//}

// uncomment to go back to previous versino of the code
//static void init_hann_window(void) {
//    for (int i = 0; i < FFT_LEN; i++) {
//        hann[i] = 0.5f * (1.0f - cosf((2.0f * M_PI * i) / (FFT_LEN - 1)));
//    }
//}

static void init_hamming(void){
	U = 0.0f;
	for (int n=0; n<SEG_LEN; n++){
		ham[n] = 0.54f - 0.46f * cosf(2.0f * M_PI * n / (SEG_LEN - 1));
		U += ham[n]*ham[n];
	}
	U /= (float)SEG_LEN;
}

static void Bandpower_from_psd(const float *Sxx, float df){
    // 0..3: classic bands
    const float lo[4] = {1.0f, 4.0f,  8.0f, 13.0f};
    const float hi[4] = {4.0f, 8.0f, 13.0f, 30.0f};
    for (int i=0;i<6;i++) EEG_DATA[i]=0.0f;

    for (int b=0; b<4; b++){
        int k0 = (int)lroundf(lo[b]/df); if (k0<1) k0=1;
        int k1 = (int)lroundf(hi[b]/df); if (k1>(NFFT/2)) k1=(NFFT/2);
        float sum=0.0f; for (int k=k0;k<=k1;k++) sum+=Sxx[k];
        EEG_DATA[b] = sum * df;
    }
    // 4: 50 Hz line (single-bin approx)
    int k50 = (int)lroundf(50.0f/df); if (k50>(NFFT/2)) k50=(NFFT/2);
    EEG_DATA[4] = Sxx[k50]*df;

    // 5: 12 Hz + 24 Hz narrow bands (±0.5 Hz)
    int k12a=(int)floorf((12.0f-0.5f)/df), k12b=(int)ceilf((12.0f+0.5f)/df);
    int k24a=(int)floorf((24.0f-0.5f)/df), k24b=(int)ceilf((24.0f+0.5f)/df);
    float s=0.0f;
    for(int k=k12a;k<=k12b;k++) if(k>=1 && k<=NFFT/2) s+=Sxx[k];
    for(int k=k24a;k<=k24b;k++) if(k>=1 && k<=NFFT/2) s+=Sxx[k];
    EEG_DATA[5]=s*df;
}

static void Bandpower(const float *in){
	for(int i=0; i<4; i++) EEG_DATA[i] = 0;   // reset

	for (int i = 1; i < 5; i++){
		EEG_DATA[0] += in[i]; // this is delta band
	}
	for (int i = 5; i < 8; i++){
		EEG_DATA[1] += in[i]; // this is theta
	}
	for (int i = 8; i < 13; i++){
		EEG_DATA[2] += in[i];
	}
	for (int i = 13; i<22; i++){
		EEG_DATA[3] += in[i];
	}
}

static void EEG_OUT(void){

	printf("delta:%f,theta:%f,alpha:%f,beta:%f\n",
	           EEG_DATA[0], EEG_DATA[1], EEG_DATA[2], EEG_DATA[3]);
}

static void EEG_OUT2(void){

	printf("delta:%f,theta:%f,alpha:%f,beta:%f\n,50hz:%f\n,12+24hz:%f\n",
	           EEG_DATA[0], EEG_DATA[1], EEG_DATA[2], EEG_DATA[3], EEG_DATA[4], EEG_DATA[5]);
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
