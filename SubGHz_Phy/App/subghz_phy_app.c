/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "fonctions.h"

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
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */

static void OnCadDone(bool channelActivityDetected);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

	// Activ pin output : A8 et A11
	GPIO_InitTypeDef gpio_init_structure = {0};

	/* Enable the GPIO Clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure the GPIO pins */
	gpio_init_structure.Pin = GPIO_PIN_8;
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(GPIOA, &gpio_init_structure);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	gpio_init_structure.Pin = GPIO_PIN_11;

	HAL_GPIO_Init(GPIOA, &gpio_init_structure);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);


	RadioEvents.CadDone = OnCadDone;
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  configure_radio_parameters();

  uint8_t radio_status;
  if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
	  LOG_INFO("Radio status avant Rx(0): 0x%02X", radio_status);

	  if (radio_status == 0x00) {
		  LOG_ERROR("❌ Radio inactif - Réveil nécessaire");

		  // Réveiller le radio
		  uint8_t wakeup_cmd = 0x80;
		  HAL_SUBGHZ_ExecSetCmd(&hsubghz, wakeup_cmd, NULL, 0);
		  HAL_Delay(100);

		  // Vérifier après réveil
		  if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
			  LOG_INFO("Radio status après réveil: 0x%02X", radio_status);
		  }
	  }
  }

      // Démarrer la réception continue
  Radio.Rx(0); // Timeout infini

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
	LOG_INFO("LoRa TX: Message envoyé");
	    // Redémarrer la réception après transmission
	    Radio.Rx(0);

  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
	LOG_INFO("LoRa RX: %d bytes, RSSI: %d dBm, SNR: %d", size, rssi, LoraSnr_FskCfo);

	    // Envoyer un événement à votre application
	    // send_event(EVENT_LORA_RX, SOURCE_LORA, size);

	    // Redémarrer la réception
	    Radio.Rx(0);

  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
	 LOG_ERROR("LoRa TX: Timeout");
	    // Redémarrer la réception
	 Radio.Rx(0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
	LOG_DEBUG("LoRa RX: Timeout");
	    // Redémarrer la réception
	Radio.Rx(0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
	LOG_ERROR("LoRa RX: Erreur");
	// Redémarrer la réception
	Radio.Rx(0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

static void OnCadDone(bool channelActivityDetected)
{
	event_t evt = { EVENT_CAD_DONE, SOURCE_LORA, channelActivityDetected ? 1 : 0 };

	if (xQueueSendFromISR(Event_QueueHandle, &evt, 0) != pdPASS)
	{
		code_erreur = ISR_callback;
		err_donnee1 = 2;
	}
}

/* USER CODE END PrFD */
