/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32_lpm_if.c
  * @author  MCD Application Team
  * @brief   Low layer function to enter/exit low power modes (stop, sleep)
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
#include "stm32_lpm.h"
#include "stm32_lpm_if.h"

/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "stm32wlxx_hal.h"
#include "communication.h"
#include "fonctions.h"

void SystemClock_Config(void);

uint32_t nb_entrees_mode_stop;
uint32_t nb_entrees_mode_sleep;

extern LPTIM_HandleTypeDef hlptim1;

/* Fréquence LSE = 32768 Hz */
#define LSE_FREQUENCY 32768UL

/* Compensation empirique (à ajuster si besoin) */
#define LPTIM_STOP_COMPENSATION 2UL

#define MIN_TICKS_FOR_STOP 50  // correspond ~50ms

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief Power driver callbacks handler
  */
const struct UTIL_LPM_Driver_s UTIL_PowerDriver =
{
  PWR_EnterSleepMode,
  PWR_ExitSleepMode,

  PWR_EnterStopMode,
  PWR_ExitStopMode,

  PWR_EnterOffMode,
  PWR_ExitOffMode,
};

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
/* USER CODE BEGIN PFP */
void EnterStopWithLPTIM(TickType_t xExpectedIdleTime);

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
	if(xExpectedIdleTime < MIN_TICKS_FOR_STOP)
	{
		nb_entrees_mode_sleep++;
		// sleep léger
		configPRE_SLEEP_PROCESSING(&xExpectedIdleTime);
		if(xExpectedIdleTime > 0) __asm volatile("wfi");
		configPOST_SLEEP_PROCESSING(&xExpectedIdleTime);
		return;
	}
	else
	// Sinon : STOP classique avec LPTIM1
		EnterStopWithLPTIM(xExpectedIdleTime);
}

void EnterStopWithLPTIM(TickType_t  xExpectedIdleTime)
{
	nb_entrees_mode_stop++;

    uint32_t reloadValue;
    TickType_t completeTickPeriods;

	if (test_index < test_MAX)
		test_tab[test_index++] = xExpectedIdleTime;

    /* Limite max LPTIM (16 bits) */
    if( xExpectedIdleTime == 0 )
        return;

    if( xExpectedIdleTime > 0xFFFF )
        xExpectedIdleTime = 0xFFFF;

    /* Stop SysTick */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /* Conversion ticks FreeRTOS -> ticks LPTIM */
    uint16_t lptim_freq = LSE_FREQUENCY / 16;
    reloadValue = ( lptim_freq * xExpectedIdleTime) / configTICK_RATE_HZ;

    if (reloadValue == 0)  reloadValue = 1;
    if( reloadValue > 0xFFFF )  reloadValue = 0xFFFF;

    /* Section critique légère */
    __disable_irq();

    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        /* Reprendre SysTick */
        SysTick->LOAD = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1;
        SysTick->VAL  = 0;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

        __enable_irq();
        return;
    }

    /* Configuration LPTIM1 */
    __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_ARRM);
    __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);

    HAL_LPTIM_TimeOut_Start_IT( &hlptim1, 0xFFFF, reloadValue);

    __enable_irq();

    /* Hook utilisateur éventuel */
    configPRE_SLEEP_PROCESSING( &xExpectedIdleTime );

    /* Entrée en STOP1 */
    HAL_PWREx_EnterSTOP2Mode( PWR_STOPENTRY_WFI );

    /* -------------  STOP ---------------- Réveil ici */

    configPOST_SLEEP_PROCESSING( &xExpectedIdleTime );

    /* Reconfigurer l’horloge système */
    SystemClock_Config();

    completeTickPeriods = xExpectedIdleTime/2;

    if (__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_CMPM) != RESET)
    { // Reveil par LPTIM2
    	__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);
    	completeTickPeriods = xExpectedIdleTime;
    }
    else
    {  // Reveil par autre cause
        TickType_t elapsedRtosTicks;

        // Lire le compteur LPTIM
        uint32_t elapsedLptimTicks = hlptim1.Instance->CNT;

        // Convertir LPTIM ticks -> FreeRTOS ticks
        elapsedRtosTicks = (elapsedLptimTicks * configTICK_RATE_HZ) / lptim_freq;

        // Sécurité
        if (elapsedRtosTicks > xExpectedIdleTime)
            elapsedRtosTicks = xExpectedIdleTime;

        completeTickPeriods = elapsedRtosTicks;
    }
    HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);

    // Mise à jour du tick RTOS
    if (completeTickPeriods > LPTIM_STOP_COMPENSATION)
    {
        completeTickPeriods =  completeTickPeriods - LPTIM_STOP_COMPENSATION;
    }
    else
    {
        completeTickPeriods = LPTIM_STOP_COMPENSATION;
    }

    vTaskStepTick( completeTickPeriods );

    // Redémarrage SysTick
    SysTick->LOAD = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    //UART_SEND("Reveil 2\n\r");
}


/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

void PWR_EnterOffMode(void)
{
  /* USER CODE BEGIN EnterOffMode_1 */

  /* USER CODE END EnterOffMode_1 */
}

void PWR_ExitOffMode(void)
{
  /* USER CODE BEGIN ExitOffMode_1 */

  /* USER CODE END ExitOffMode_1 */
}

void PWR_EnterStopMode(void)
{
  /* USER CODE BEGIN EnterStopMode_1 */

  /* USER CODE END EnterStopMode_1 */
}

void PWR_ExitStopMode(void)
{
  /* USER CODE BEGIN ExitStopMode_1 */

  /* USER CODE END ExitStopMode_1 */
}

void PWR_EnterSleepMode(void)
{
  /* USER CODE BEGIN EnterSleepMode_1 */

  /* USER CODE END EnterSleepMode_1 */
}

void PWR_ExitSleepMode(void)
{
  /* USER CODE BEGIN ExitSleepMode_1 */

  /* USER CODE END ExitSleepMode_1 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
