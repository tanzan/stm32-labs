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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	Init, Step1, Step2, Step3, Step4, Step5, Step6
} State;

typedef struct {
	uint32_t HalfPeriod;
	uint32_t TogglesNum;
	uint32_t MaxTogglesNum;

} StateContext;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBOUNCE_DELAY_MS 20

#define SHORT_FQ 3
#define MEDIUM_FQ 5
#define LONG_FQ 10

#define SHORT_DELAY_SEC 1
#define MEDIUM_DELAY_SEC 1
#define LONG_DELAY_SEC 1

#define HZ_TO_MILIS_HALF_PERIOD(hz) (1000 / 2 * 1 / (hz))

#define TOGGLES_NUM(fq, delay_sec) (2 * (fq) * (delay_sec))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
State CurrentState = Init;

StateContext StateContexts[] = { { 0, 0, 0 },
		{ HZ_TO_MILIS_HALF_PERIOD(SHORT_FQ), 0, TOGGLES_NUM(SHORT_FQ, SHORT_DELAY_SEC) },
		{ HZ_TO_MILIS_HALF_PERIOD(MEDIUM_FQ), 0, TOGGLES_NUM(MEDIUM_FQ, MEDIUM_DELAY_SEC) },
		{ HZ_TO_MILIS_HALF_PERIOD(LONG_FQ), 0, TOGGLES_NUM(LONG_FQ, LONG_DELAY_SEC) },
		{ HZ_TO_MILIS_HALF_PERIOD(LONG_FQ), 0, TOGGLES_NUM(LONG_FQ, LONG_DELAY_SEC) },
		{ HZ_TO_MILIS_HALF_PERIOD(SHORT_FQ), 0, TOGGLES_NUM(MEDIUM_FQ, MEDIUM_DELAY_SEC) },
		{ HZ_TO_MILIS_HALF_PERIOD(SHORT_FQ), 0, TOGGLES_NUM(SHORT_FQ, SHORT_DELAY_SEC) } };

uint32_t LastDelayTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ToggleLED() {
	HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
	LastDelayTime = HAL_GetTick();
	StateContexts[CurrentState].TogglesNum++;
}

void ResetLED() {
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	StateContexts[CurrentState].TogglesNum = 0;
}

uint32_t GetHalfPeriod() {
	return StateContexts[CurrentState].HalfPeriod;
}

uint32_t IsMaxTogglesNum() {
	StateContext Ctx = StateContexts[CurrentState];
	return Ctx.MaxTogglesNum == Ctx.TogglesNum;
}

void GotoState(State state) {
	CurrentState = state;
	StateContexts[CurrentState].TogglesNum = 0;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	/* USER CODE BEGIN 2 */
	uint32_t LastDebounceTime = 0;
	uint8_t BtnState = GPIO_PIN_RESET;
	uint8_t LastBtnState = GPIO_PIN_RESET;
	uint8_t BtnReading;

	ResetLED();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	for (;;) {
		BtnReading = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
		if (BtnReading != LastBtnState) {
			LastDebounceTime = HAL_GetTick();
		}
		if ((HAL_GetTick() - LastDebounceTime) > DEBOUNCE_DELAY_MS) {
			if (BtnReading != BtnState) {
				BtnState = BtnReading;
				if (BtnState == 1) {
					LastDelayTime = HAL_GetTick();
					if (CurrentState == Init) {
						ResetLED();
						GotoState(Step1);
						ToggleLED();
					} else if (CurrentState > Init && CurrentState < Step4) {
						ResetLED();
						GotoState(Step4);
						ToggleLED();
					}
				}
			}
		}
		LastBtnState = BtnReading;
		if (IsMaxTogglesNum()) {
			switch (CurrentState) {
			case Step1:
				GotoState(Step2);
				break;
			case Step2:
				GotoState(Step3);
				break;
			case Step3:
				GotoState(Step1);
				break;
			case Step4:
				GotoState(Step5);
				break;
			case Step5:
				GotoState(Step6);
				break;
			case Step6:
				GotoState(Init);
				ResetLED();
				break;
			default:
			}
		}

		if (CurrentState > Init
				&& ((HAL_GetTick() - LastDelayTime) >= GetHalfPeriod())) {
			ToggleLED();
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 120;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
