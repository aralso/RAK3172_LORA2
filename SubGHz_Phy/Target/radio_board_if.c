/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    radio_board_if.c
  * @author  MCD Application Team
  * @brief   This file provides an interface layer between MW and Radio Board
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "radio_board_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
int32_t RBI_Init(void)
{
  /* USER CODE BEGIN RBI_Init_1 */

  /* USER CODE END RBI_Init_1 */
#if defined(USE_BSP_DRIVER)
  /* Important note: BSP code is board dependent
   * STM32WL_Nucleo code can be found
   *       either in STM32CubeWL package under Drivers/BSP/STM32WLxx_Nucleo/
   *       or at https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Drivers/BSP/STM32WLxx_Nucleo/
   * 1/ For User boards, the BSP/STM32WLxx_Nucleo/ directory can be copied and replaced in the project. The copy must then be updated depending:
   *       on board RF switch configuration (pin control, number of port etc)
   *       on TCXO configuration
   *       on DC/DC configuration
   *       on maximum output power that the board can deliver*/
  return BSP_RADIO_Init();
#else
  int32_t retcode = 0;
  /* USER CODE BEGIN RBI_Init_2 */
  GPIO_InitTypeDef  gpio_init_structure = {0};

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA8 */
  gpio_init_structure.Pin = GPIO_PIN_8;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOA, &gpio_init_structure);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  /* USER CODE END RBI_Init_2 */
  return retcode;
#endif  /* USE_BSP_DRIVER  */
}

int32_t RBI_DeInit(void)
{
  /* USER CODE BEGIN RBI_DeInit_1 */

  /* USER CODE END RBI_DeInit_1 */
#if defined(USE_BSP_DRIVER)
  /* Important note: BSP code is board dependent
   * STM32WL_Nucleo code can be found
   *       either in STM32CubeWL package under Drivers/BSP/STM32WLxx_Nucleo/
   *       or at https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Drivers/BSP/STM32WLxx_Nucleo/
   * 1/ For User boards, the BSP/STM32WLxx_Nucleo/ directory can be copied and replaced in the project. The copy must then be updated depending:
   *       on board RF switch configuration (pin control, number of port etc)
   *       on TCXO configuration
   *       on DC/DC configuration
   *       on maximum output power that the board can deliver*/
  return BSP_RADIO_DeInit();
#else
  int32_t retcode = 0;
  /* USER CODE BEGIN RBI_DeInit_2 */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
  /* USER CODE END RBI_DeInit_2 */
  return retcode;
#endif  /* USE_BSP_DRIVER */
}

int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config)
{
  /* USER CODE BEGIN RBI_ConfigRFSwitch_1 */

  /* USER CODE END RBI_ConfigRFSwitch_1 */
#if defined(USE_BSP_DRIVER)

  /* Important note: BSP code is board dependent
   * STM32WL_Nucleo code can be found
   *       either in STM32CubeWL package under Drivers/BSP/STM32WLxx_Nucleo/
   *       or at https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Drivers/BSP/STM32WLxx_Nucleo/
   * 1/ For User boards, the BSP/STM32WLxx_Nucleo/ directory can be copied and replaced in the project. The copy must then be updated depending:
   *       on board RF switch configuration (pin control, number of port etc)
   *       on TCXO configuration
   *       on DC/DC configuration
   *       on maximum output power that the board can deliver*/
  return BSP_RADIO_ConfigRFSwitch((BSP_RADIO_Switch_TypeDef) Config);
#else
  int32_t retcode = 0;
  /* USER CODE BEGIN RBI_ConfigRFSwitch_2 */
  switch (Config)
  {
    case RBI_SWITCH_OFF:
    case RBI_SWITCH_RX:
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
      break;
    }
    case RBI_SWITCH_RFO_LP:
    case RBI_SWITCH_RFO_HP:
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
      break;
    }
    default:
      break;
  }
  /* USER CODE END RBI_ConfigRFSwitch_2 */
  return retcode;
#endif  /* USE_BSP_DRIVER */
}

int32_t RBI_GetTxConfig(void)
{
  /* USER CODE BEGIN RBI_GetTxConfig_1 */

  /* USER CODE END RBI_GetTxConfig_1 */
#if defined(USE_BSP_DRIVER)
  /* Important note: BSP code is board dependent
   * STM32WL_Nucleo code can be found
   *       either in STM32CubeWL package under Drivers/BSP/STM32WLxx_Nucleo/
   *       or at https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Drivers/BSP/STM32WLxx_Nucleo/
   * 1/ For User boards, the BSP/STM32WLxx_Nucleo/ directory can be copied and replaced in the project. The copy must then be updated depending:
   *       on board RF switch configuration (pin control, number of port etc)
   *       on TCXO configuration
   *       on DC/DC configuration
   *       on maximum output power that the board can deliver*/
  return BSP_RADIO_GetTxConfig();
#else
  int32_t retcode = RBI_CONF_RFO;
  /* USER CODE BEGIN RBI_GetTxConfig_2 */
  /* RAK3172 can use RFO_LP or RFO_HP but let's default to RFO LP for now if unknown */
  /* USER CODE END RBI_GetTxConfig_2 */
  return retcode;
#endif  /* USE_BSP_DRIVER */
}

int32_t RBI_IsTCXO(void)
{
  /* USER CODE BEGIN RBI_IsTCXO_1 */

  /* USER CODE END RBI_IsTCXO_1 */
#if defined(USE_BSP_DRIVER)
  /* Important note: BSP code is board dependent
   * STM32WL_Nucleo code can be found
   *       either in STM32CubeWL package under Drivers/BSP/STM32WLxx_Nucleo/
   *       or at https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Drivers/BSP/STM32WLxx_Nucleo/
   * 1/ For User boards, the BSP/STM32WLxx_Nucleo/ directory can be copied and replaced in the project. The copy must then be updated depending:
   *       on board RF switch configuration (pin control, number of port etc)
   *       on TCXO configuration
   *       on DC/DC configuration
   *       on maximum output power that the board can deliver*/
  return BSP_RADIO_IsTCXO();
#else
  int32_t retcode = 1; // TCXO is supported on RAK3172
  /* USER CODE BEGIN RBI_IsTCXO_2 */

  /* USER CODE END RBI_IsTCXO_2 */
  return retcode;
#endif  /* USE_BSP_DRIVER  */
}

int32_t RBI_IsDCDC(void)
{
  /* USER CODE BEGIN RBI_IsDCDC_1 */

  /* USER CODE END RBI_IsDCDC_1 */
#if defined(USE_BSP_DRIVER)
  /* Important note: BSP code is board dependent
   * STM32WL_Nucleo code can be found
   *       either in STM32CubeWL package under Drivers/BSP/STM32WLxx_Nucleo/
   *       or at https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Drivers/BSP/STM32WLxx_Nucleo/
   * 1/ For User boards, the BSP/STM32WLxx_Nucleo/ directory can be copied and replaced in the project. The copy must then be updated depending:
   *       on board RF switch configuration (pin control, number of port etc)
   *       on TCXO configuration
   *       on DC/DC configuration
   *       on maximum output power that the board can deliver*/
  return BSP_RADIO_IsDCDC();
#else
  int32_t retcode = 1; // DCDC is supported on RAK3172
  /* USER CODE BEGIN RBI_IsDCDC_2 */

  /* USER CODE END RBI_IsDCDC_2 */
  return retcode;
#endif  /* USE_BSP_DRIVER  */
}

int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef Config)
{
  /* USER CODE BEGIN RBI_GetRFOMaxPowerConfig_1 */

  /* USER CODE END RBI_GetRFOMaxPowerConfig_1 */
#if defined(USE_BSP_DRIVER)
  /* Important note: BSP code is board dependent
   * STM32WL_Nucleo code can be found
   *       either in STM32CubeWL package under Drivers/BSP/STM32WLxx_Nucleo/
   *       or at https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Drivers/BSP/STM32WLxx_Nucleo/
   * 1/ For User boards, the BSP/STM32WLxx_Nucleo/ directory can be copied and replaced in the project. The copy must then be updated depending:
   *       on board RF switch configuration (pin control, number of port etc)
   *       on TCXO configuration
   *       on DC/DC configuration
   *       on maximum output power that the board can deliver*/
  return BSP_RADIO_GetRFOMaxPowerConfig((BSP_RADIO_RFOMaxPowerConfig_TypeDef) Config);
#else
  int32_t ret = 0;
  /* USER CODE BEGIN RBI_GetRFOMaxPowerConfig_2 */
  if (Config == RBI_RFO_LP_MAXPOWER)
  {
    ret = 15; /*dBm*/
  }
  else
  {
    ret = 22; /*dBm*/
  }
  /* USER CODE END RBI_GetRFOMaxPowerConfig_2 */
  return ret;
#endif  /* USE_BSP_DRIVER  */
}
/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
