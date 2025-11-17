/*
 * Appli.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tocqueville


 TODO :
, adresses LORA/Uart
 clignot sorties, pwm,  antirebond 2 boutons, 2e uart
TODO BUG : timer apres uart_rx, HLH

 v1.9 11/2025 : process LORA RX-TX
 v1.8 10/2025 : envoi subghz ok
 v1.7 10/2025 : ok:hlpuart1, lptimer1, stop mode, bouton IR   en cours:subGhz
 v1.6 10/2025 : en cours : hlpuart1, lpTimer, boutton_IT, SubGhz_init, lowPower
 v1.5 10/2025 : eeprom, log_flash, rtc, messages binaires, attente dans en_queue
 v1.4 09/2025 : fct : refonte reception uart, watchdog contextuel, traitement_rx, opti stack
 v1.3 09/2025 : fct : augmentation stack taches, erreur_freertos
 v1.2 09/2025 : pile envoi uart, timer, code_erreur
 v1.1 09/2025 : STM32CubeMX + freertos+ subGhz+ Uart2+ RTC+ print_log+ event_queue


Conso en mode veille :
Sleep 1,4mA  Stop:0,4uA(réveil uart/RTC)  Standby 0,1uA(pas de réveil uart)

Conso en MSI_range8 et HSI : 1,33mA
*/

#include <appli.h>
#include "main.h"
#include "cmsis_os.h"
#include <communication.h>
#include <fonctions.h>
#include <lora.h>
#include <eeprom_emul.h>
#include <log_flash.h>
#include "app_subghz_phy.h"
#include "timers.h"
#include "queue.h"
#include <stdio.h>    // Pour sprintf
#include <string.h>   // Pour strlen
#include <stdarg.h>   // Pour va_list (si vous utilisez print_log)


extern TimerHandle_t HTimer_24h;
extern TimerHandle_t HTimer_20min;
extern osThreadId_t Uart_RX_TaskHandle;
extern osThreadId_t Uart_TX_TaskHandle;

extern uint8_t uart_rx_char;
extern osThreadId_t defaultTaskHandle;
extern IWDG_HandleTypeDef hiwdg;

HAL_StatusTypeDef configure_lse_oscillator(void);
HAL_StatusTypeDef configure_lsi_oscillator(void);

QueueHandle_t Event_QueueHandle;
uint8_t test_val=0;
uint32_t last_status_time = 0;
uint32_t last_save_time = 0;

uint8_t cpt_timer20s;
uint8_t cpt_message;
uint8_t mess_pay[100];

uint16_t temp;
uint8_t hygro;

/* Definitions for LORA_RX_Task */
osThreadId_t LORA_RX_TaskHandle;
const osThreadAttr_t LORA_RX_Task_attributes = {
  .name = "LORA_RX_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for LORA_TX_Task */
osThreadId_t LORA_TX_TaskHandle;
const osThreadAttr_t LORA_TX_Task_attributes = {
  .name = "LORA_TX_Task",
  .priority = (osPriority_t) osPriorityLow4,
  .stack_size = 256 * 4
};
/* Definitions for Appli_Task */
osThreadId_t Appli_TaskHandle;
const osThreadAttr_t Appli_Task_attributes = {
  .name = "Appli_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 600 * 4
};

void LORA_RXTsk(void *argument);
void LORA_TXTsk(void *argument);
void Appli_Tsk(void *argument);
uint8_t mesure_temp(uint16_t* temp, uint8_t* hygro);


void init1()  // avant KernelInitialize
{

	  /* make sure that no LPUART transfer is on-going */
	  while (__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_BUSY) == SET);


	  HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);  // Demarrage réception Uart1


	  char init_msg[] = "-- RAK3172 Init. Log level:x\r";
	  init_msg[0] = dest_log;
	  init_msg[1] = My_Address;
	  init_msg[27] = get_log_level()+'0';
	  uint16_t len = strlen(init_msg);
	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)init_msg, len, 3000);
	  HAL_Delay(500);

      init_functions1();

      if (HAL_LPTIM_Counter_Start_IT(&hlptim1, 20000) != HAL_OK)
      {
        Error_Handler();
      }


	  /*if (HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 8000,0) != HAL_OK)  // 4IT:ARROK, ARRM, REPOK, UPDATE
	  {
	    Error_Handler();
	  }*/
      /* Disable autoreload write complete interrupt */
      __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARROK);

}

void init2()  // création queue, timer, semaphore
{
	  Event_QueueHandle = xQueueCreate(32, sizeof(event_t));

	  /*size_t freeHeap = xPortGetFreeHeapSize();
	    char msgL[50];
	    sprintf(msgL, "Free heap before tasks: %i bytes\r", freeHeap);
	    //HAL_Delay(500);
	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)msgL, strlen(msgL), 3000);
	    //HAL_Delay(500);*/

	 init_functions2();  // timer freertos
	 init_communication(); // tache uart


}

void init3()   // taches
{
    /* creation of LORA_RX_Task */
    //LORA_RX_TaskHandle = osThreadNew(LORA_RXTsk, NULL, &LORA_RX_Task_attributes);

    /* creation of LORA_TX_Task */
    //LORA_TX_TaskHandle = osThreadNew(LORA_TXTsk, NULL, &LORA_TX_Task_attributes);

    /* creation of Appli_Task */
    Appli_TaskHandle = osThreadNew(Appli_Tsk, NULL, &Appli_Task_attributes);

    /*char msgL[50];

	  uint8_t code_err_tache=0;
      if (defaultTaskHandle == NULL) code_err_tache=1;
      if (LORA_RX_TaskHandle == NULL) code_err_tache|=2;
      if (LORA_TX_TaskHandle == NULL) code_err_tache|=4;
      if (Appli_TaskHandle == NULL) code_err_tache|=8;
      if (Uart_RX_TaskHandle == NULL) code_err_tache|=16;
      if (Uart_TX_TaskHandle == NULL) code_err_tache|=32;

      if (code_err_tache) {
    	   //HAL_Delay(500);
    	   sprintf(msgL, "erreur tache: %02X ", code_err_tache);
    	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msgL, strlen(msgL), 3000);
    	   //HAL_Delay(500);
          //LOG_ERROR("Failed to create Appli_Task");
          // La tâche n'a pas pu être créée
      }*/

      // Après la création :
      /*size_t freeHeap = xPortGetFreeHeapSize();
      sprintf(msgL, "Free heap after tasks: %i bytes\r", freeHeap);
      //HAL_Delay(500);
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)msgL, strlen(msgL), 3000);
      //HAL_Delay(500); */


}

// Apres KernelStart, dans Appli_task
void init4(void)
{
    init_functions4();  // watchdog, Log, eeprom

    MX_SubGHz_Phy_Init();

    //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"InitB", 5, 3000);
    //HAL_Delay(500);

      //osDelay(100);

      // Vérifier la configuration flash
      //check_flash_config();
      //check_flash_permissions();
      //osDelay(1000);

      //osDelay(1000);


      /*MX_SubGHz_Phy_Init();

  	#ifdef mode_sleep
  		uint8_t sleep_cmd = RADIO_SET_SLEEP;  // 0x84
  		HAL_SUBGHZ_ExecSetCmd(&hsubghz, sleep_cmd, NULL, 0);

  		LOG_INFO("SUBGHZ: Put to sleep at startup");
  	#endif

  		osDelay(1000);*/

  		//LOG_INFO("SUBGHZ State: %d", hsubghz.State);
  		//LOG_INFO("SUBGHZ DeepSleep: %d", hsubghz.DeepSleep);

  		/*if (hsubghz.DeepSleep != SUBGHZ_DEEP_SLEEP_ENABLE) {
  			LOG_INFO("SUBGHZ: pb sleep");
  		    // SUBGHZ consomme 500-600µA !
  		}*/
      /*if (EEPROM_Init() == HAL_OK) {
          LOG_INFO("eeprom initialisee");
          osDelay(1000);
          // Test simple
          test_eeprom_simple();
      } else {
          LOG_ERROR("Erreur eeprom");
      }*/

    /*Initialize timer and RTC*/
    //UTIL_TIMER_Init();

    // 1ms / tick
	/*  if (HAL_LPTIM_Counter_Start_IT(&hlptim1, 32000) != HAL_OK)  // 4IT:ARROK, ARRM, REPOK, UPDATE
	  {
	    Error_Handler();
	  }*/

	  // IT importantes :
	  // HAL_LPTIM_TimeOut_Start_IT(period, timeout).démarre à 0, jusqu'à period, ARRM, puis redémarre à 0
	  // s'arette au bout de timeout ticks.
	  // HAL_LPTIM_Counter_Start_IT(ARRM) : démarre à start, IT CMPM et ARRM
	  // HAL_LPTIM_OnePulse_Start_IT : 1 fois

	  /* Disable autoreload write complete interrupt */
	  __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARROK);
	  //__HAL_LPTIM_ENABLE_IT(&hlptim1, LPTIM_IT_CMPOK);

}


/**
 * @brief Configuration de LSE (TCXO externe)
 * @retval HAL_StatusTypeDef: Statut de la configuration
 */
HAL_StatusTypeDef configure_lse_oscillator(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    HAL_StatusTypeDef status;

    LOG_DEBUG("Attempting to configure LSE (TCXO)");

    // Configuration LSE
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    status = HAL_RCC_OscConfig(&RCC_OscInitStruct);

    if (status == HAL_OK) {
        // Vérifier que LSE est vraiment actif
        uint32_t timeout = 1000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) && timeout > 0) {
            osDelay(1);
            timeout--;
        }

        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) {
            LOG_INFO("LSE (TCXO) configured successfully");
            return HAL_OK;
        } else {
            LOG_WARNING("LSE startup timeout");
            return HAL_TIMEOUT;
        }
    } else {
        LOG_WARNING("LSE configuration failed: %d", status);
        return status;
    }
}

/**
 * @brief Configuration de LSI (oscillateur interne)
 * @retval HAL_StatusTypeDef: Statut de la configuration
 */
HAL_StatusTypeDef configure_lsi_oscillator(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    HAL_StatusTypeDef status;

    LOG_DEBUG("Attempting to configure LSI (internal oscillator)");

    // Configuration LSI
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    status = HAL_RCC_OscConfig(&RCC_OscInitStruct);

    if (status == HAL_OK) {
        // Vérifier que LSI est actif
        uint32_t timeout = 1000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) && timeout > 0) {
            osDelay(1);
            timeout--;
        }

        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)) {
            LOG_INFO("LSI (internal oscillator) configured successfully");
            return HAL_OK;
        } else {
            LOG_WARNING("LSI startup timeout");
            return HAL_TIMEOUT;
        }
    } else {
        LOG_WARNING("LSI configuration failed: %d", status);
        return status;
    }
}


/**
 * @brief Envoyer un événement dans la queue
 * @param type: Type d'événement
 * @param source: Source de l'événement
 * @param data: Données de l'événement
 * @retval osStatus_t: Statut de l'envoi
 */
osStatus_t send_event(uint8_t type, uint8_t source, uint16_t data)
{
    event_t evt;
    evt.type = type;
    evt.source = source;
    evt.data = data;
    if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
    {
        LOG_ERROR("Event queue full - message lost");
        return osErrorResource;
    }
    return osOK;
}

/**
 * @brief Envoyer un événement simple
 * @param type: Type d'événement
 * @retval osStatus_t: Statut de l'envoi
 */
osStatus_t send_simple_event(uint8_t type)
{
    return send_event(type, SOURCE_SYSTEM, 0);
}

void startDefTsk()
{
    // Enregistrer un heartbeat pour le watchdog
    watchdog_task_heartbeat(WATCHDOG_TASK_DEFAULT);

    uint32_t current_time = HAL_GetTick();

    // Afficher le statut du watchdog toutes les 30 secondes
    if (current_time - last_status_time > 10000) {
        //watchdog_print_status();
        last_status_time = current_time;
        //osDelay(3000); // Attendre 3 seconde
		//check_stack_usage();

        uint8_t value;
        if (test_val ==1)
        	LOG_INFO("toto");
        if (test_val == 2)
            LOG_INFO("test:FLASH_PAGE_SIZE: %i octets", 1233330);
        if (test_val == 3)
        {
        	char messa[10];
        	messa[0] = '1';
            messa[1] = 'S';  // Accusï¿½ reception ok : S1
            messa[2] = '1';
            messa[3] = car_fin_trame;
            envoie_mess_ASC(param_def, (const char*)messa);
        }
        if (test_val == 4)
        {
        	uint8_t messa = 'c';
            envoie_mess_ASC(param_def, "1te%cVAnal %i", messa, 12);
        }
        if (test_val == 5)
        	EEPROM_Read8(1, &value);
        if (test_val == 6)
        	EEPROM_Write8(1, 12);
        if (test_val == 7)
        	EEPROM_Read8(1, &value);
        if (test_val == 8)
        	log_read(1, 1, '1', 0);
        if (test_val ==9)
        	log_write('T', 1, 0x02, 0x03, "testRxBl");    }
		if (test_val ==10)
			log_read(1, 4, '1', 0);
   	    if (test_val ==11)
		    log_read(1, 1, '1', 1);

    // Sauvegarder les données de diagnostic toutes les 60 secondes
    if (current_time - last_save_time > 30000) {
        //save_diagnostic_data();
    	debug_uart_complete();
    	//check_memory();
    	//LOG_INFO("rx uart:%i",cpt_rx_uart);

    	last_save_time = current_time;
    }
    HAL_IWDG_Refresh(&hiwdg);


    osDelay(10); // Attendre 1 seconde entre chaque heartbeat
}

void LORA_RXTsk(void *argument)
{
  /* USER CODE BEGIN LORA_RXTsk */
  // Démarrer la surveillance watchdog pour cette tâche
  watchdog_task_start(WATCHDOG_TASK_LORA_RX);
  //LOG_INFO("LoRa RX Task started with watchdog protection");

  /* Infinite loop */
//	uint8_t rx_buffer[64];

    osDelay(100);

	for(;;)
	{
		// Enregistrer un heartbeat pour le watchdog
		watchdog_task_heartbeat(WATCHDOG_TASK_LORA_RX);

		// Vérifier l'état du radio
		/*if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {

			// Vérifier si un message est disponible
			if (radio_status & 0x02) { // Bit RX_DONE
				LOG_DEBUG("LoRa message received");

				// Lire le message depuis le buffer radio
				if (HAL_SUBGHZ_ReadBuffer(&hsubghz, 0x00, rx_buffer, 64) == HAL_OK) {
					LOG_INFO("Received LoRa message: %s", rx_buffer);
		            send_event(EVENT_LORA_RX, SOURCE_LORA, strlen((char*)rx_buffer));
				} else {
					LOG_ERROR("Failed to read LoRa buffer");
				}

				// Effacer le flag RX_DONE
				HAL_SUBGHZ_WriteRegister(&hsubghz, 0x01, 0x02);
			} else {
				LOG_VERBOSE("No LoRa message");
			}
		} else {
			LOG_WARNING("Failed to read radio status");
		}*/

		osDelay(1000);
	}
  /* USER CODE END LORA_RXTsk */
}

/* USER CODE BEGIN Header_LORA_TXTsk */
/**
* @brief Function implementing the LORA_TX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LORA_TXTsk */
void LORA_TXTsk(void *argument)
{
  /* USER CODE BEGIN LORA_TXTsk */
  // Démarrer la surveillance watchdog pour cette tâche
  watchdog_task_start(WATCHDOG_TASK_LORA_TX);
  //LOG_INFO("LoRa TX Task started with watchdog protection");

  /* Infinite loop */

  //uint32_t message_count = 0;
  //uint8_t tx_buffer[64];
  //uint8_t radio_status;

  osDelay(100);

  for(;;)
  {
	  // Enregistrer un heartbeat pour le watchdog
	  watchdog_task_heartbeat(WATCHDOG_TASK_LORA_TX);

	  // Attendre un délai
	  osDelay(12000);
	  //LOG_INFO("a");

	  // Créer un message avec timestamp
	  //uint32_t timestamp = HAL_GetTick() / 1000; // secondes
	  //sprintf((char *)tx_buffer, "LoRa message #%lu at %lu s", message_count++, timestamp);

       //LOG_DEBUG("Sending LoRa message: %s", tx_buffer);

       // Vérifier que le radio est libre
       /*if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
           if (!(radio_status & 0x01)) { // Pas en transmission

               // Écrire le message dans le buffer radio
               if (HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, tx_buffer, strlen((char*)tx_buffer)) == HAL_OK) {

                   // Démarrer la transmission
                   uint8_t tx_cmd = 0x83; // Commande TX
                   if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, tx_cmd, NULL, 0) == HAL_OK)
                   {
                       LOG_INFO("LoRa transmission started");

                       // Attendre la fin de transmission
                       osDelay(100);

                       // Vérifier le statut
                       if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
                           if (radio_status & 0x08) { // TX_DONE
                               LOG_INFO("LoRa message sent successfully");
                               send_event(EVENT_LORA_TX, SOURCE_LORA, message_count);
                           } else {
                               LOG_ERROR("LoRa transmission failed");
                               send_event(EVENT_ERROR, SOURCE_LORA, 1);
                           }
                       }
                   } else {
                       LOG_ERROR("Failed to start LoRa transmission");
                   }
               } else {
                   LOG_ERROR("Failed to write LoRa buffer");
               }
           } else {
               LOG_WARNING("LoRa radio busy");
           }
       } else {
           LOG_ERROR("Failed to read radio status");
       }*/
  }
  /* USER CODE END LORA_TXTsk */
}

/* USER CODE BEGIN Header_Appli_Tsk */
/**
* @brief Function implementing the Appli_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Appli_Tsk */
void Appli_Tsk(void *argument)
{
    event_t evt;
    osStatus_t status;

  /* USER CODE BEGIN Appli_Tsk */

    init4(); // watchdog, Log, eeprom, demarrage LPTIM1

    // Démarrer la surveillance watchdog pour cette tâche
  	   //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"InitA", 5, 3000);
  	   //HAL_Delay(500);
	watchdog_task_start(WATCHDOG_TASK_APPLI);
    //LOG_INFO("Appli_Task started with watchdog protection");
	//uint16_t event_count;

	for(;;)
    {
        // Enregistrer un heartbeat pour le watchdog
        watchdog_task_heartbeat(WATCHDOG_TASK_APPLI);

        watchdog_set_context(WATCHDOG_TASK_APPLI, WATCHDOG_CONTEXT_WAITING);

        // Attendre un événement (bloque tant qu'il n'y a rien)
        status = osMessageQueueGet(Event_QueueHandle, &evt, NULL, osWaitForever);

        watchdog_set_context(WATCHDOG_TASK_APPLI, WATCHDOG_CONTEXT_ACTIVE);

        if (status == osOK)
        //{
			// Traiter l'événement reçu
			//LOG_DEBUG("Processing event: type=%d, source=%d, data=%d",
			//		  evt.type, evt.source, evt.data);
			//osDelay(30);

        //while (xQueueReceive(Event_QueueHandle, &evt, 0) == pdPASS)
        {
        	//event_count++;
            //LOG_INFO("Événement #%i: type=%d", event_count, evt.type);
        	switch (evt.type) {

				case EVENT_BUTTON: {
					LOG_INFO("Button pressed event");
					// Actions pour bouton pressé
					//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // Toggle LED

					// Envoyer message LoRa
					//char messa[] = "Button pressed!";
					//send_lora_message((const char*)messa, 16, 'Q');
					break;
				}

				case EVENT_CAD_DONE: {
					LOG_INFO("Cad Done : %i", evt.data);
					lora_tx_on_cad_result(evt.data != 0);

					break;
				}
				case EVENT_LORA_TX: {
					LOG_INFO("Debut de transmission LORA");
					lora_handle_event_tx(evt.source);
					break;
				}
				case EVENT_LORA_TX_STEP: {
					LOG_INFO("TX_step");
					lora_tx_state_step();
					break;
				}
				case EVENT_LORA_RX: {
					LOG_INFO("message LORA recu len:%i rssi:%i snr:%i param:%02X mess:%s", evt.data, message_recu.rssi, \
							message_recu.snr, message_recu.param, message_recu.data);
					if (evt.data) evt.data--;
					traitement_rx(message_recu.data, evt.data);
					//lora_handle_event_rx();
					break;
				}
				case EVENT_LORA_RX_TEST: {
					for (int j = 0; j < evt.data; j++) {
						LOG_DEBUG("%02X ", mess_pay[j]);
						// H23U 11 07 HUTTT10
					}
					break;
				}



				case EVENT_LORA_ACK_TIMEOUT: {
					// Timeout ACK → retry ou passage à l’état suivant
					lora_tx_state_step();
					break;
				}

				case EVENT_LORA_TX_DONE: {
					LOG_INFO("LoRa message sent event");
					// Actions pour message LoRa envoyé
					// TODO
					break;
				}

				case EVENT_LORA_REVEIL_BALISE : {
					LOG_INFO("classe B : Fenetre ecoute balise Radio");
					lora_handle_classb_beacon_event();
					break;
				}

				case EVENT_RELANCE_RX: {
					relance_radio_rx((uint8_t)evt.data);
					if (evt.source == 6)
						LOG_ERROR("Lora error RX");
					break;
				}

				case EVENT_ERROR: {
					relance_radio_rx((uint8_t)evt.data);
					LOG_ERROR("Error event - source:%i data: %d", evt.source, evt.data);
					// Actions pour erreur
					break;
				}

				case EVENT_WAKE_UP: {
					LOG_INFO("Wake up event");
					// Actions pour réveil
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
					osDelay(100);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					break;
				}

				case EVENT_SLEEP: {
					LOG_INFO("Sleep event");
					// Actions avant sleep
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
					osDelay(100);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					break;
				}

				case EVENT_UART_RX: {
					//LOG_INFO("UART message received event");
					in_message_t message_in;
			        if (xQueueReceive(in_message_queue, &message_in, portMAX_DELAY) == pdPASS)
			        {
			            // Traiter le message selon son type
						reception_message_Uart2(&message_in);
			        }
					break;
				}

				case EVENT_SYSTEM_RESET: {
					LOG_INFO("System reset event");
					// Actions avant reset
					for (int i = 0; i < 10; i++) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						osDelay(50);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
						osDelay(50);
					}
					osDelay(1000);
					HAL_NVIC_SystemReset();
					break;
				}
				case EVENT_WATCHDOG_CHECK: {
					HAL_IWDG_Refresh(&hiwdg);  // refresh watchdog hardware
				    watchdog_check_all_tasks(); // test watdchdog logiciel
				    break;
				}
				case EVENT_TIMER_24h: {
					envoie_mess_ASC(param_def, "1Message periodique 24h\r\n");

					// Debug : afficher le temps restant avant la prochaine expiration
					TickType_t expiry = xTimerGetExpiryTime(HTimer_24h);
					TickType_t now = xTaskGetTickCount();
					LOG_INFO("Il reste %lu ticks avant la prochaine expiration\n",
						   (expiry > now) ? (expiry - now) : 0);
					break;
				}

				case EVENT_TIMER_1min: { // moteur
					LOG_INFO("timer 1min");
					break;
				}
				case EVENT_TIMER_5min: {  // Thermometre
					LOG_INFO("timer 5min");
					uint8_t ret = mesure_temp(&temp, &hygro);
					if (ret)
					{
						LOG_INFO("erreur_temp:%i", ret);
						log_write('E', log_w_err_temp, ret, 0, "Err_Temp");
					}
					else
					{
						param_def = 0x10; // 2 avec ack, 2 renvois
						envoie_mess_ASC(param_def, "HT%i-%i", temp, hygro);     // CTxxyy
						envoie_mess_ASC(param_def, "JT%i-%i", temp, hygro);     // CTxxyy
						message.param = param_def;
					    message.data[0] = 'J';
					    message.data[1] = 4;
					    message.data[2] = 'C';
					    message.data[3] = 'H';
					    message.data[4] = (temp>>8);
					    message.data[5] = (temp & 0xFF);
					    message.data[6] = hygro;
					    envoie_mess_bin(&message);
					}
					break;
				}
				case EVENT_TIMER_20min: {

					//LOG_INFO("a");
					LOG_INFO("timer 20s");

					cpt_timer20s++;  // 1(20s), 3(1min), 6(2min)
					if ((cpt_timer20s==1) || ((cpt_timer20s%3) == 0))
					{
						cpt_message++;
						param_def=0x30; // 0x30 sans Ack, RX apres   0x10 sans ack
						if (cpt_message==2) param_def = 0x30;  // 0x20:avec ack, RX apres
						if (cpt_message==3) param_def = 0x30;  // 2:avec ack, 2 renvois
						if (cpt_message==4) param_def = 0x20;  // 0x30:sans ack, RX apres
						envoie_mess_ASC(param_def, "HTTT%i", cpt_message);
					}
					//LOG_INFO("Entrée en mode Stop avec HSI...");
					/*char init_msg[] = "mode stop\n\r";
					  uint16_t len = strlen(init_msg);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t*)init_msg, len, 500);

					    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

					  SystemClock_Config();
						char init_msg2[] = "Reveil\n\r";
						  len = strlen(init_msg2);
						  HAL_UART_Transmit(&hlpuart1, (uint8_t*)init_msg2, len, 500);*/

					    //LOG_INFO("Réveil du mode Stop - HSI fonctionne !");

					//osDelay(300);
					//check_all_clocks();

					/*vTaskSuspendAll();  // Suspendre FreeRTOS
					  // enter STOP mode
					  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

					  SystemClock_Config_fromSTOP();

					  xTaskResumeAll();  // Reprendre FreeRTOS
					LOG_INFO("Reveil");
					osDelay(300);*/

					  /* Ensure that MSI is wake-up system clock */
					  //__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

					 // diagnose_uart_wakeup();
					//test_stop_mode();
					//osDelay(1000);
					//check_stack_usage();
					//uint8_t statut=envoie_mess_ASC("Message periodique 30s");
					//if (statut) { code_erreur= code_erreur_envoi; err_donnee1=statut;}
					//uint8_t statut=envoie_mess_ASC("PRS");
					//osDelay(100);
					//LOG_INFO("statut:%d", statut);
					//osDelay(1000);

					// Debug : afficher le temps restant avant la prochaine expiration
					//TickType_t expiry = xTimerGetExpiryTime(HTimer_20min);
					//TickType_t now = xTaskGetTickCount();
					//osDelay(100);
					//LOG_INFO("Il reste %lu ticks avant la prochaine expir\n",
					//	   (expiry > now) ? (expiry - now) : 0);
					//osDelay(100);
					break;
				}
				case EVENT_TIMER_LPTIM: {  // chaque 10 secondes
					/*if (evt.data==1)
						envoie_mess_ASC("1LPTIM1\r\n");
					else
						envoie_mess_ASC("1LPTIM1 %i\r\n", evt.data);*/
					break;
				}
				case EVENT_UART_RAZ: {
					LOG_WARNING("RX Timeout for UART %i", evt.source);
					code_erreur=timeout_RX;   //timeout apres 1 car Recu
				    err_donnee1= evt.source+'0';
				    raz_Uart(evt.data);
				    break;
				}

				default: {
					LOG_WARNING("Unknown event type: %d", evt.type);
					break;
				}
			}
	        if (code_erreur)
	            envoi_code_erreur();
        }
        /*else {
            LOG_ERROR("Failed to receive event: %d", status);
        }*/
        osDelay(10);
    }

  /* USER CODE END Appli_Tsk */
}

void assert_failed(const char *file, int line)
{
    char msg[100];
    int len = snprintf(msg, sizeof(msg), "-- ASSERT failed at %s:%d\r\n", file, line);
    msg[0] = dest_log;
    msg[1] = My_Address;
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, len, HAL_MAX_DELAY);

    HAL_Delay(2000);

    // Reset du système
    NVIC_SystemReset();
    // Bloquer ici
    //taskDISABLE_INTERRUPTS();
    //for(;;);
/* USER CODE END Callback 1 */
}

uint8_t mesure_temp(uint16_t* temp, uint8_t* hygro)
{
	*temp=10;
	*hygro = 52;
	return 0;
}
