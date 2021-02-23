/* USER CODE BEGIN Header */
/**
  **************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  **************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  **************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern int adc_value;
extern int sicak_urun;
extern int soguk_urun;
extern int genel_urun;
int sayac2=0;
extern int sayac3;
extern  int run;
extern char Grup1[];
int sayac =0;
uint8_t veri[]="Grp1:xxx,Grp2:yyy,Toplam:zzz\r\n";
uint8_t tData[20];
uint8_t tData[]="Limit Asimi!\r\n"; 
uint8_t tDataUrunYok[]="Urun Yok!\r\n";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI0_1_IRQn 0 */
    if (run % 2 == 1)
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
        {
            sayac = 0;
            if (adc_value <= 32)
            {
                soguk_urun++;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            }
            else
            {
                sicak_urun++;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            }
            genel_urun++;
        }
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
    {
        run++;
        if (run % 2 == 1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        }
    }
    /* USER CODE END EXTI0_1_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    /* USER CODE BEGIN EXTI0_1_IRQn 1 */

    /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles ADC global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */
    if (run % 2 == 1)
    {
        sayac++;
        sayac2++;
    }

    /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
    HAL_TIM_IRQHandler(&htim1);
    /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */
    if (sayac > 2 && run % 2 == 1&&(genel_urun<200))
    {
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)&tDataUrunYok, 11);
    }
    if (genel_urun > 199 && run % 2 == 1)
    {

        HAL_UART_Transmit_IT(&huart2, (uint8_t *)&tData, 14);
    }

    if (run % 2 == 1)
    {
        sayac2 = 0;
        int temp_soguk = soguk_urun;
        int temp_sicak = sicak_urun;
        int temp_genel = genel_urun;

        veri[7] = temp_soguk % 10 + '0';
        temp_soguk = temp_soguk / 10;
        veri[6] = temp_soguk % 10 + '0';
        temp_soguk = temp_soguk / 10;
        veri[5] = temp_soguk % 10 + '0';

        veri[16] = temp_sicak % 10 + '0';
        temp_sicak = temp_sicak / 10;
        veri[15] = temp_sicak % 10 + '0';
        temp_sicak = temp_sicak / 10;
        veri[14] = temp_sicak % 10 + '0';

        veri[27] = temp_genel % 10 + '0';
        temp_genel = temp_genel / 10;
        veri[26] = temp_genel % 10 + '0';
        temp_genel = temp_genel / 10;
        veri[25] = temp_genel % 10 + '0';
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)&veri, 30);
    }

    HAL_UART_IRQHandler(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */

    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */

    if (run % 2 == 1)
    {
        adc_value = HAL_ADC_GetValue(&hadc);
        adc_value = adc_value + 26;
    }

    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */

    if (run % 2 == 1)
    {
        HAL_ADC_Start_IT(&hadc);
    }

    /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
