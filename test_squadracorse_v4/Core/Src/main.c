/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"


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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum Stato { danger_state, waiting_state, running_state } ;// definizione degli stati
volatile enum Stato stato_corrente = running_state;

// costanti per il calcolo della temperatura

#define V25  760 // valore della tensione dell adc1 a 25 °C
#define avg_slope  2.5 // dato da datasheet
#define Vdd  3.6 // tensione di alimentazione adc da datasheet

// variabili contenenti i valori di uscita dall'adc

int valore_tensione;
float valore_temperatura;

// valori calcolati di tensione e temperatura

float tensione;
float temperatura;

// valori delle soglie dell'output compare del timer 3

uint16_t current_ccr1 = 38230;
uint16_t current_ccr2 = 21846;
uint16_t current_ccr3 = 54615;

// stringhe per l'invio dei valorii tramite UART

char stringa_tensione[20];
char stringa_temperatura[20];
char stringa_PA5[30];
char stringa_PC5[30];

// flag di fine scansione adc

uint8_t scansione_tensione = 0;
uint8_t scansione_temperatura = 0;

// condizione del corrispettivo pin gpio
int stato_PA5 = 0;
int stato_PC5 = 0;



//  funzione chiamata alla fine della scansione dell adc

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){


	if (hadc == &hadc1){                                     // controllo se l adc1 ha chiamato la funzione

//		scansione tensione

		if (scansione_tensione == 1){ 						 // se flag di fine scansione tensione = 1

			scansione_tensione = 0;                          // reset flag scansione tensione
			valore_tensione = HAL_ADC_GetValue(&hadc1);	     // salvataggio valore di uscita adc
			tensione = (3.3*100/4096)*valore_tensione;       // calcolo della tensione [(Val*Vadc)/2^12], moltiplico per 100
															 //	per non usare il float nella sprintf siccome non è supportato
			sprintf(stringa_tensione, "tensione = %d,%d\n", (int)tensione/100, (int)tensione%100); // definisco la stringa da inviare dividendo
																								   // per 100 ottenendo il valore reale
			HAL_UART_Transmit(&huart2, stringa_tensione,strlen(stringa_tensione),  1);             // trasmissione della tensione

			if (tensione/100 < 1.8){ // tensione/100 è il valore reale
				stato_corrente = danger_state;              // imposto danger state
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);    // setto PA5 a 1 (LED verde sulla board)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);    // setto PC5 a 0 (pilota un LED esterno rosso montato su breadboard)

			}

			else if (tensione/100 > 2.7){
				stato_corrente = danger_state;              // imposto danger state
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);    // setto PC5 a 1 (pilota un LED esterno rosso montato su breadboard)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);    // setto PA5 a 0 (LED verde sulla board)
			}

			else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);    // setto PA5 a 0 (LED verde sulla board)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);    // setto PC5 a 0 (pilota un LED esterno rosso montato su breadboard)
			}

		}





// 		scansione temperatura

		else if (scansione_temperatura == 1){ 						      // se flag di fine scansione temperatura = 1

			scansione_temperatura = 0;									  // reset flag fine scansione temperatura
			valore_temperatura = HAL_ADC_GetValue(&hadc1);				  // salvo in una variabile il valore in uscita dall adc
			temperatura = (valore_temperatura/4096)* (3.3 *1000);         // calcolo valore in tensione dell uscita dell adc moltiplicato per 1000 per non usare float
																		  // nella sprintf, siccome non è supportato
			temperatura = ((temperatura - V25)*100/avg_slope) + 2500 ;    // calcolo valore della temperatura con la formula fornita sul datasheet

			if((temperatura/100) > -40 && (temperatura/100) < 125)	{     // se la temperatura rientra nel range fornito sul datasheet per il sensore
				sprintf(stringa_temperatura,"temperatura = %d,%d\n", (int)temperatura/100, (int)temperatura%100 ); // creo una stringa col valore della temperatura
				HAL_UART_Transmit(&huart2, stringa_temperatura, sizeof(stringa_temperatura), 1);                   // invio temperatura via UART
			}

			else {														  // se la temperatura non rientra nel range segnalo l'errore
				HAL_UART_Transmit(&huart2, "Errore : temperatura out of range\n", sizeof("Errore : temperatura out of range\n"), 1);   // trasmetto l'errore

			}
	    }
	}

}


// funzione chiamata quando viene raggiunta una soglia impostata nei canali dei timer per l output compare

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim){

//   configuro canale 0 per la misura di tensione
   ADC_ChannelConfTypeDef sConfig_0 = {0};
   sConfig_0.Channel = ADC_CHANNEL_0;
   sConfig_0.Rank = 1;
   sConfig_0.SamplingTime = ADC_SAMPLETIME_3CYCLES;

//   configuro canale per lettura del sensore di temperatura
   ADC_ChannelConfTypeDef sConfig_18 = {0};
   sConfig_18.Channel = ADC_CHANNEL_TEMPSENSOR;
   sConfig_18.Rank = 1;
   sConfig_18.SamplingTime = ADC_SAMPLETIME_3CYCLES;

   if (htim == &htim3){                                                                // controllo se il timer 3 ha chiamato la funzione

		if (stato_corrente == running_state || stato_corrente == danger_state){

			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){						       // controllo se il canale 1 ha chiamato la funzione
				HAL_ADC_ConfigChannel(&hadc1, &sConfig_0); 							   // attivo il canale 0
				current_ccr1  += 38230;												   // incremento il valore della soglia per il canale 1
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_ccr1);            // effettuo il cambiamento della soglia
				scansione_tensione = 1;												   // attivo flag per identificare quale canale ha attivato la scansione
				HAL_ADC_Start_IT(&hadc1);                                              // start scansione adc


		    }

			else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && scansione_tensione != 1){   // controllo se il canale 2 ha chiamato la funzione
				HAL_ADC_ConfigChannel(&hadc1, &sConfig_18);                                   // attivo il canale del sensore di temperatura
				current_ccr2  += 21846;                                                       // incremento il valore della soglia per il canale 2
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, current_ccr2);					  // effettuo il cambiamento della soglia
				scansione_temperatura = 1;													  // attivo flag per identificare quale canale ha attivato la scansione
				HAL_ADC_Start_IT(&hadc1);													  // start scansione adc
			}
		}

		if ( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){// controllo se il canale 3 ha chiamato la funzione e se sono in waiting state

			// trasmetto stato del pin PA5

			stato_PA5 = (int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
			sprintf(stringa_PA5, "stato del pin PA5 = %d\n", stato_PA5);
			HAL_UART_Transmit(&huart2,stringa_PA5, sizeof(stringa_PA5),1);

			// trasmetto stato del pin PC5

			stato_PC5 = (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
			sprintf(stringa_PC5, "stato del pin PC5 = %d\n", stato_PC5);
			HAL_UART_Transmit(&huart2,stringa_PC5, sizeof(stringa_PC5),1);


			if(stato_corrente == waiting_state){
				current_ccr3  += 54615; 														  // incremento il valore della soglia per il canale 3
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, current_ccr3);						  // effettuo il cambiamento della soglia
				scansione_tensione = 1;															  // attivo flag per identificare quale canale ha attivato la scansione
				HAL_UART_Transmit(&huart2, "Board in waiting state - please press the emergency button\n", sizeof("Board in waiting state - please press the emergency button\n"), 1);
            // trasmetto errore tramite UART
			}
		}


	}
}


// funzione chiamata quando avviene l evento che scatena l input capture (la pressione del tasto utente presente sulla board)

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim){

	if (htim == &htim4){							// controllo se il tim 4 ha chiamato la funzione

		if (stato_corrente != waiting_state){		// se lo stato è diverso dal waiting lo imposto
			stato_corrente = waiting_state;
		}

		else if (stato_corrente == waiting_state){  // se lo stato è di wait lo pongo di run
			stato_corrente = running_state;
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // ferma tim3 durante stop del debugger
  __HAL_DBGMCU_FREEZE_TIM3();


//  start del timer 3 con i primi 3 canali in output compare
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);

//  start del timer 4 con canale 4 in input capture
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

//  imposto i valori delle soglie  dell'output compare
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_ccr1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, current_ccr2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, current_ccr3);



  //HAL_ADC_Start(&hadc1); // hadc1 è la struttura dell'ADC
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 768;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
