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

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// GPIO_A
#define VR1 GPIO_PIN_0
#define SW3 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)
// GPIO_B
#define LED_GPIO GPIOB
#define SET_LED_STATE(mask, status) \
  (HAL_GPIO_WritePin(LED_GPIO, (mask), !(status)))
#define LED2_LED GPIO_PIN_10  // green
#define LED3_LED GPIO_PIN_9   // green
#define LED4_LED GPIO_PIN_8   // yellow
// GPIO_C
#define DS1_COM GPIO_PIN_8
#define DS2_COM GPIO_PIN_9
#define DS3_1_COM GPIO_PIN_10
#define DS3_2_COM GPIO_PIN_11
#define DS3_3_COM GPIO_PIN_12

#define DIGITAL_DISPLAY_MAP_LEN 14
// PC0~PC7 -> dp~A

// PA6 & PA7
#define SW1_IDR ((GPIOA->IDR >> 6) & 0x03)
#define SW1_MODE ((SW1_IDR >> 1) | ((SW1_IDR << 1) & 0x02))
#define SW2 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)
#define SW3 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum { LOW, HIGH };
enum { MIDDLE, UP, DOWN = 3 };

uint8_t direct, gear;
uint8_t old_mode = 0;  // store old mode

uint8_t display_dp_flag = 0, display_disable_flag = 0, show_index = 0;
uint8_t show_value[5] = {};

uint8_t display_count = 0, acc_count = 0;
uint16_t buz_count = 0;

float now_speed;
uint16_t adc[2];

uint8_t emergency_braking;
uint8_t sw3_flag, buz_flag;
uint32_t sw3_last_status_time;
// A0, A1
float vol0, vol1;

// [0bxxx].map(x => x.toString(16).padStart(2, 0)).join("\n")
const uint8_t digital_display_map[DIGITAL_DISPLAY_MAP_LEN] = {
    // a, b, c, d, e, f, g; 1 -> LOW
    0x01, 0x4f, 0x12, 0x06, 0x4c, 0x24, 0x20,  // 0, 1, 2, 3, 4, 5, 6
    0x0f, 0x00, 0x04, 0x60, 0x30, 0x6a, 0x18,  // 7, 8, 9, b, e, n, p
};

const uint8_t speed_pwm_duty_map[15] = {0,  6,  12, 18, 24, 30, 36, 42,
                                        48, 55, 64, 73, 82, 91, 100};

#define D_B 10
#define D_E 11
#define D_N 12
#define D_P 13
#define SET_DISPLAY(a, b, c, d, e) \
  do {                             \
    show_value[0] = (a);           \
    show_value[1] = (b);           \
    show_value[2] = (c);           \
    show_value[3] = (d);           \
    show_value[4] = (e);           \
  } while (0)

#define abs(value) ((value) > 0 ? (value) : -(value))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC_Init();
  MX_TIM21_Init();
  MX_TIM6_Init();
  MX_TIM22_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);

  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint8_t mode = SW1_MODE;
    vol0 = adc[0] * 3300 / 4096 % 10000;
    vol1 = adc[1] * 3300 / 4096 % 10000;
    gear = abs(26 - (int)(vol0 / 100)) % 16;
    gear -= gear > 9 ? 1 : 0;
    direct = vol1 / 1000;  // 1 is UP, 0 is MIDDLE, 3 is DOWN

    if (mode != old_mode) {
      old_mode = mode;
      TIM21->CCR1 = 0;
      TIM22->CCR1 = TIM22->CCR2 = 0;
      SET_LED_STATE(LED2_LED | LED3_LED | LED4_LED, LOW);
      SET_DISPLAY(0, 0, 0, 0, 0);
      sw3_last_status_time = HAL_GetTick();
      display_disable_flag = display_dp_flag = 0;
      now_speed = acc_count = emergency_braking = sw3_flag = buz_flag = 0;

      switch (mode) {
        case 0: {
          SET_LED_STATE(LED2_LED | LED3_LED | LED4_LED, HIGH);
          display_dp_flag = 1 << 3;
        } break;
        case 1: {
          // SET_LED_STATE(LED2_LED, LOW); // no change
        } break;
      }
    }

    switch (mode) {
      case 0: {
        // 32 * 1e6 / 32 / 2000 => 500Hz
        // >> 500Hz, 50%, 2ms
        TIM21->CCR1 = SW3 ? 1000 : 0;

        int value = vol0;
        SET_DISPLAY(0, value / 1000, (value / 100) % 10, (value / 10) % 10,
                    value % 10);
      } break;

      case 1: {
        SET_LED_STATE(LED3_LED, direct == UP);
        SET_LED_STATE(LED4_LED, direct == DOWN);

        uint8_t speed = speed_pwm_duty_map[gear % 15];
        SET_DISPLAY(gear / 10, gear % 10, speed / 100, (speed / 10) % 10,
                    speed % 10);

        // 32 * 1e6 / 32 / 1000 => 1kHz
        TIM22->CCR1 = direct == UP ? speed * 10 : 0;
        TIM22->CCR2 = direct == DOWN ? speed * 10 : 0;
      } break;

      case 2: {
        SET_LED_STATE(LED3_LED, direct == UP);
        SET_LED_STATE(LED4_LED, direct == DOWN);

        if (gear == 0 && now_speed == 0) {
          // jump out emergency braking
          emergency_braking = 0;
          sw3_last_status_time = HAL_GetTick();
        } else if (emergency_braking) gear = 0;

        if (!acc_count) {
          float acc_speed = 0;

          if (gear >= 14) {
            if (now_speed < 100) acc_speed = 5;
          } else if (gear > 9) {
            if (now_speed < (gear - 9) * 20) acc_speed = gear - 9;
            else acc_speed = 0.5;  // 0.5%
          } else if (now_speed > 0) {
            if (gear == 0) acc_speed = -10;
            else acc_speed = gear == 0 ? -10 : -(11 - gear) * 0.5;
          }

          acc_count = 1;
          now_speed += acc_speed * 0.001;
          if (now_speed > 100) now_speed = 100;
          else if (now_speed < 0) now_speed = 0;
        }

        // direct == MIDDLE ? 0 : now_speed
        int _speed = now_speed;
        display_disable_flag = 0;
        if (gear == 0) {
          SET_DISPLAY(D_E, D_B, _speed / 100, (_speed / 10) % 10, _speed % 10);
        } else if (gear > 0 && gear < 9) {
          SET_DISPLAY(D_B, 9 - gear, _speed / 100, (_speed / 10) % 10,
                      _speed % 10);
        } else if (gear == 9) {
          display_disable_flag = 1 << 0;
          SET_DISPLAY(0, D_N, _speed / 100, (_speed / 10) % 10, _speed % 10);
        } else {
          SET_DISPLAY(D_P, gear - 9, _speed / 100, (_speed / 10) % 10,
                      _speed % 10);
        }

        if (SW2 && now_speed != 0 && !emergency_braking) {  // use mode
          uint8_t button_status = SW3;
          if (button_status != sw3_flag) sw3_last_status_time = HAL_GetTick();
          uint32_t diff = HAL_GetTick() - sw3_last_status_time;

          if (diff > (button_status ? 15000 + 6000 : 9000)) {
            buz_flag = 0;
            emergency_braking = 1;
          } else if (diff > (button_status ? 15000 + 3000 : 6000)) {
            if (!buz_count) {
              buz_count = 250;
              buz_flag = !buz_flag;
            }
          } else if (diff > (button_status ? 15000 : 3000)) {
            if (!buz_count) {
              buz_count = 500;
              buz_flag = !buz_flag;
            }
          } else buz_flag = 0;

          sw3_flag = button_status;
        } else {
          buz_flag = 0;
          sw3_last_status_time = HAL_GetTick();
        }

        // 32 * 1e6 / 32 / 2000 => 500Hz
        // >> 500Hz, 50%, 2ms
        TIM21->CCR1 = buz_flag ? 1000 : 0;

        // 32 * 1e6 / 32 / 1000 => 1kHz
        TIM22->CCR1 = direct == UP ? _speed * 10 : 0;
        TIM22->CCR2 = direct == DOWN ? _speed * 10 : 0;
        SET_LED_STATE(LED2_LED, SW2);
      } break;
      default:
        break;
    }

    if (!display_count) {
      show_index = (show_index + 1) % 5;

      uint8_t mask = 1 << show_index;
      if (!(display_disable_flag & mask)) {
        uint16_t out = !(display_dp_flag & (1 << (4 - show_index)));
        out |= (uint16_t)digital_display_map[show_value[show_index]] << 1;
        out |= (0x1f & ~mask) << 8;

        GPIOC->ODR = out;
      }
      display_count = 3;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim6) {
  if (htim6->Instance == TIM6) {
    if (acc_count > 0) acc_count--;
    if (buz_count > 0) buz_count--;
    if (display_count > 0) display_count--;
  }
}
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
