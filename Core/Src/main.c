/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LHA7878.h"
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
static lha7878_dev_t   g_adc_dev;
static lha7878_frame_t g_frame;
static int64_t         g_uv[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting\n\t");
__asm(".global __ARM_use_no_argv\n\t");

FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
  (void)f;
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1, 1000);
  return ch;
}

void _sys_exit(int return_code)
{
  (void)return_code;
  while (1);
}
#else
int fputc(int ch, FILE *f)
{
  (void)f;
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1, 1000);
  return ch;
}
#endif

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1, 1000);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LHA7878_Reset_GPIO_Init();
  printf("\r\n========================================\r\n");
  printf("  STM32H750 LHA7878 8-Channel ADC Demo\r\n");
  printf("========================================\r\n");

  lha7878_config_t cfg;
  lha7878_config_default(&cfg);
  cfg.data_rate = LHA7878_DR_8K;          /* 8kSPS */
  cfg.mode      = LHA7878_MODE_RDATAC;    /* 连续读数 */
  cfg.vref      = LHA7878_VREF_INT_2V4;   /* 内部 2.4V */
  cfg.avdd_mv   = 5000;                   /* 板上 AVDD 是 5V */
  cfg.dvdd_mv   = 3300;                   /* DVDD 3.3V */
  cfg.clk_hz    = 2048000;                /* TODO: 待确认外部 2.048MHz 还是内部振荡器 */
  cfg.sclk_hz   = 4687500;                /* 实际 SCLK = PLL1Q 75MHz / 16 */
  for (uint8_t i = 0; i < 8; i++) {
      cfg.gain[i] = LHA7878_GAIN_1;        /* 全通道增益 1 */
      cfg.mux[i]  = LHA7878_MUX_NORMAL;    /* 全通道正常输入 */
  }

  lha7878_dev_default(&g_adc_dev, &cfg);

  printf("[INIT] Starting LHA7878 initialization...\r\n");
  lha7878_status_t status = lha7878_init(&g_adc_dev);
  if (status != LHA7878_OK) {
      printf("[ERROR] LHA7878 init failed! Error code: %d\r\n", (int)status);
      switch (status) {
          case LHA7878_ERR_ID: {
              printf("        Reason: Chip ID mismatch (Check SPI connection & MISO pin)\r\n");
              /* 诊断: START 拉高让芯片开始转换, 再观察 DRDY / 原始数据 / ID 重复读 */
              lha7878_start_pin(&g_adc_dev, true);
              HAL_Delay(5);
              uint32_t drdy_lows = 0;
              uint32_t t0 = HAL_GetTick();
              while (HAL_GetTick() - t0 < 20u) {
                  if (lha7878_drdy_is_low(&g_adc_dev)) { drdy_lows++; }
              }
              printf("        Debug: DRDY low samples in 20ms = %lu (0 = no conversion/CLK)\r\n",
                     (unsigned long)drdy_lows);
              {
                  uint8_t tx[27] = {0}, rx[27] = {0};
                  HAL_GPIO_WritePin(LHA7878_SPI2_CS_GPIO_Port, LHA7878_SPI2_CS_Pin, GPIO_PIN_RESET);
                  HAL_StatusTypeDef hs = HAL_SPI_TransmitReceive(g_adc_dev.hspi, tx, rx, 27u, 100u);
                  HAL_GPIO_WritePin(LHA7878_SPI2_CS_GPIO_Port, LHA7878_SPI2_CS_Pin, GPIO_PIN_SET);
                  printf("        Debug: raw27 (st=%d):", (int)hs);
                  for (uint8_t i = 0; i < 27u; i++) { printf(" %02X", (unsigned)rx[i]); }
                  printf("\r\n");
              }
              for (uint8_t i = 0; i < 4u; i++) {
                  uint8_t v = 0;
                  lha7878_status_t r = lha7878_read_reg(&g_adc_dev, LHA7878_REG_ID, &v);
                  printf("        Debug: ID retry[%u]=0x%02X (st=%d)\r\n",
                         (unsigned)i, (unsigned)v, (int)r);
              }
              lha7878_start_pin(&g_adc_dev, false);
              break;
          }
          case LHA7878_ERR_VERIFY:
              printf("        Reason: Register write-verify failed (Check SPI Mode 1 / CPHA=2Edge)\r\n");
              break;
          case LHA7878_ERR_TIMEOUT:
              printf("        Reason: DRDY timeout (Check CLK, START pin, or DRDY pin)\r\n");
              break;
          case LHA7878_ERR_RATE:
              printf("        Reason: SCLK rate or data rate configuration conflict\r\n");
              break;
          case LHA7878_ERR_VREF:
              printf("        Reason: VREF parameter invalid (e.g. 4V needs AVDD>=4.5V)\r\n");
              break;
          default:
              printf("        Reason: Generic HAL or Argument error\r\n");
              break;
      }
      Error_Handler();
  }

  printf("[INIT] LHA7878 initialized successfully! (REV_ID OK)\r\n");
  printf("[INIT] Entering continuous 8-channel acquisition...\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_tick = 0;
  uint32_t sample_count = 0;
  uint32_t row_count = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    lha7878_status_t st = lha7878_read_frame(&g_adc_dev, &g_frame, 10);
    if (st == LHA7878_OK || st == LHA7878_ERR_FAULT) {
        lha7878_frame_to_uv(&g_adc_dev, &g_frame, g_uv);
        sample_count++;

        uint32_t now = HAL_GetTick();
        if (now - last_print_tick >= 200) {
            last_print_tick = now;
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            /* 每 20 行重印表头, 便于滚动时对照列; 每帧一行, 纵向对比同一通道. */
            if (row_count % 20u == 0u) {
                printf("\r\n  Approx Rate: %lu sps\r\n", (unsigned long)(sample_count * 5));
                printf("  Frame |    CH1    |    CH2    |    CH3    |    CH4    |    CH5    |    CH6    |    CH7    |    CH8    |  mV\r\n");
            }
            sample_count = 0;
            row_count++;

            printf("%7lu |", (unsigned long)g_frame.frame_index);
            for (uint8_t i = 0; i < 8u; i++) {
                printf(" %+9.3f |", (double)g_uv[i] / 1000.0);
            }
            if (g_frame.fault_p != 0u || g_frame.fault_n != 0u) {
                printf("  FAULT P=0x%02X N=0x%02X", g_frame.fault_p, g_frame.fault_n);
            }
            printf("\r\n");
        }
    } else if (st == LHA7878_ERR_TIMEOUT) {
        uint32_t now = HAL_GetTick();
        if (now - last_print_tick >= 1000) {
            last_print_tick = now;
            printf("[WARN] Waiting for DRDY timeout...\r\n");
        }
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
