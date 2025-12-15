/*
 * Appli.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tocqueville


 TODO :
, bug get_battery_level
 clignot sorties, pwm,  antirebond 2 boutons, 2e uart
TODO BUG : timer apres uart_rx, HLH

 v1.11 12/2025 : modif STOP freertos par timer LPTIM2
 v1.10 11/2025 : divers bugs lora, vrefInt
 v1.9 11/2025 : process LORA RX-TX, hdc1080, VRefInt, i2c(temp)
 v1.8 10/2025 : envoi subghz ok
 v1.7 10/2025 : ok:hlpuart1, lptimer1, stop mode, bouton IR   en cours:subGhz
 v1.6 10/2025 : en cours : hlpuart1, lpTimer, boutton_IT, SubGhz_init, lowPower
 v1.5 10/2025 : eeprom, log_flash, rtc, messages binaires, attente dans en_queue
 v1.4 09/2025 : fct : refonte reception uart, watchdog contextuel, traitement_rx, opti stack
 v1.3 09/2025 : fct : augmentation stack taches, erreur_freertos
 v1.2 09/2025 : pile envoi uart, timer, code_erreur
 v1.1 09/2025 : STM32CubeMX + freertos+ subGhz+ Uart2+ RTC+ print_log+ event_queue

LPTIM1 : interrup toutes les 10 secondes pour action watchdog, etc...
LPTIM2 : timer freertos en mode stop
LPTIM3 : timers expirés de la radio
Alarm_RTC : interrupt toutes les 24 heures

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
#include <math.h>
#include "hdc1080.h"

extern TimerHandle_t HTimer_24h;
extern TimerHandle_t HTimer_20min;
extern TimerHandle_t HTimer_M3voies;

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
uint8_t Keepalive=1;

#if CODE_TYPE == 'B'  // Thermometre chaudiere
	uint16_t temp;  // /10 + 100
	uint8_t hygro;
	uint8_t nb_samples=1;
	uint8_t num_val_temp;
	uint16_t tempe[MAX_SENS];
	uint8_t humid[MAX_SENS];
	uint16_t temp_period;
#endif

#if CODE_TYPE == 'C'  // Vanne motorisee chaudiere
	uint8_t ch_debut[NB_MAX_PGM];   // debut de chauffe :heure par pas de 10 minutes
	uint8_t ch_fin[NB_MAX_PGM];   // fin de chauffe
	uint8_t ch_type[NB_MAX_PGM];     // 0:tous les jours, 1:semaine, 2:week-end (2 bits)
	uint8_t ch_consigne[NB_MAX_PGM];  // 5° à 23°C, par pas de 0,1°C
	uint8_t ch_cons_apres[NB_MAX_PGM];  // 3° à 23°C, par pas de 0,5°C (6 bits)
	uint32_t forcage_duree;    // par pas de 10 min 1/1/2020=0 (sur 23bits)
	uint8_t forcage_consigne;  // 0 à 23°C
	uint8_t consigne_normale;
	uint8_t consigne_apres;
	uint8_t ch_arret; // 1 bit
	float pos_prec;
	uint8_t init_attente;
	uint8_t cpt_circulateur;
	uint16_t Tint;
	uint8_t ch_circulateur;
	uint8_t consigne_regulation;
	uint16_t heure_der_temp;
	uint16_t cpt_temp_recu;
	uint16_t nb_mes_temp;  // 144 mesures de temp par jour
	uint16_t nb_mes_temp24;  // précédentes mesures de temp par jour
	uint16_t puis_chaud;
	uint8_t puis_chaud24;
	uint8_t batt_thermo_av;
	uint8_t batt_thermo_ap;
#endif


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


void LORA_RXTsk(void *argument);
void LORA_TXTsk(void *argument);
void envoi_data (uint8_t nb_valeur);
void calcul_pid_vanne(void);
void chgt_consigne(void);
void pid_forcage_init(void);
void lecture_temp_i2c(uint8_t);

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

      /*if (HAL_LPTIM_Counter_Start_IT(&hlptim1, 20000) != HAL_OK)
      {
        Error_Handler();
      }*/


	  /*if (HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 8000,0) != HAL_OK)  // 4IT:ARROK, ARRM, REPOK, UPDATE
	  {
	    Error_Handler();
	  }*/
      /* Disable autoreload write complete interrupt */
      //__HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARROK);

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

    // Configurer le RTC (sans calculer le jour de la semaine)
    RTC_DateTypeDef sDate = {0};
    sDate.WeekDay = 1;  // Lundi par défaut (ou jour fixe)
    sDate.Month = 11;
    sDate.Date = 24;
    sDate.Year = 25;  // RTC stocke l'année - 2000

    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);



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

    init_functions4();  // watchdog, Log_flash, eeprom
    log_write('R', 0, 0x00, 0x00, "Init");

	#ifndef SANS_RADIO
		MX_SubGHz_Phy_Init();  // init radio, mutex lora
	#endif

    // lecture parametres en EEPROM
    // periode de lecture de temperature
	#if CODE_TYPE == 'B'
		uint8_t status = EEPROM_Read16(0, &temp_period);  // 0 ou 30 à 15000
		if ((status==0) && ((temp_period ==0) || (temp_period < 15001)))
			 LOG_INFO("periode temp: %isec", temp_period);
		else
		{
			temp_period = TEMP_PERIOD;
			EEPROM_Write16(0, temp_period);
			LOG_INFO("Raz periode temp: val par defaut %is", temp_period);
		}
		if (temp_period)
		{
			if (HTimer_temp_period != NULL)
			{
				// change la periode et le démarre
				xTimerChangePeriod( HTimer_temp_period, pdMS_TO_TICKS(temp_period*1000), 0 );
			}
		}
	#endif

  #if CODE_TYPE == 'C'
	consigne_normale = 19*10;
	consigne_apres = 15*10;
	consigne_regulation = 190;  // consigne par défaut : 19°C
	Tint = 190;

	/*ch_debut[0] = 6*6;
	ch_fin[0] = 22*6;
	ch_consigne[0] = 20*10;
	ch_cons_apres[0] = 16*10;
	test_var = 190;
	ch_arret = 1;*/

	uint8_t err;
	// Forcage et arret chauffage :  Arret : bit 31  forcage_duree: 8à30 bits forcage_consigne:0à7
	uint32_t val32;
	err = EEPROM_Read32( 1 , &val32 );
	if (!err)
	{
		ch_arret = val32>>31;
		forcage_duree = (val32>>8) & 0x007FFFFF;
		forcage_consigne = (val32 & 0xFF);
		if ((forcage_consigne<50) || (forcage_consigne>230))
			err=1;
		else
			LOG_INFO("Chauffage Arret:%i Forcage:%i consigne forcage:%i", ch_arret, forcage_duree, forcage_consigne);
	}
	if (err)
	{
		ch_arret = 1;
		forcage_duree = 0;
		forcage_consigne = 180;
		EEPROM_Write32(1, (ch_arret<<31) | (forcage_duree<<8) | (forcage_consigne));
		LOG_INFO("Raz Init forcage");
	}

	// Lecture programmes 26-31:cons_ap 24-25:type 16-23:cons 8-15:fin 0-7:debut
	for (uint8_t i=0; i<NB_MAX_PGM; i++)
	{
		err = EEPROM_Read32( 2+i , &val32 );
		if (!err)
		{
			ch_debut[i] = val32 & 0xFF;
			ch_fin[i] = (val32>>8) & 0xFF;
			ch_consigne[i] = (val32>>16) & 0xFF;
			ch_cons_apres[i] = (val32>>26) & 0xFF;
			ch_type[i] = (val32>>24) & 3;
			if (((ch_debut[i]<145) && (ch_fin[i]<145) && (ch_type[i]<3) && (ch_consigne[i]>=50) \
					&& (ch_consigne[i]<=230) && (ch_cons_apres[i]>=6) && (ch_cons_apres[i]<=64)))
				LOG_INFO("Chauffage Programme %i: debut:%i fin:%i type:%i consigne:%i cons_apres:%i", \
						i, ch_debut[i], ch_fin[i], ch_type[i], ch_consigne[i], ch_cons_apres[i]);
			else
				err=1;
		}
		if (err)
		{
			if (i)
			{
				ch_debut[i] = 0;
				ch_fin[i] = 0;
				ch_consigne[i] = 19*10;
				ch_cons_apres[i] = 16*2;
				ch_type[i] = 0;
			}
			else
			{
				ch_debut[i] = 7*6;
				ch_fin[i] = 22*6;
				ch_consigne[i] = 19*10;
				ch_cons_apres[i] = 16*2;
				ch_type[i] = 0;
			}
			EEPROM_Write32(2+i, (ch_cons_apres[i]<<26) | (ch_type[i]<<24) | (ch_consigne[i]<<16) | (ch_fin[i]<<8) | ch_debut[i]);
			LOG_INFO("Raz Programme %i", i);
		}
	}

	// init circulateur
	ch_circulateur = 1 - ch_arret;
	if (ch_circulateur)  // marche
		HAL_GPIO_WritePin(CIRCULATEUR_GPIO, CIRCULATEUR_PIN, GPIO_PIN_SET);
	else  // marche
		HAL_GPIO_WritePin(CIRCULATEUR_GPIO, CIRCULATEUR_PIN, GPIO_PIN_RESET);

	init_attente = 2;
	#ifndef MODE_DEBUG
		// ferme la vanne : 140 secondes en fermeture
		pos_prec = 0.0;
		HAL_GPIO_WritePin(VALVE_OPEN_GPIO, VALVE_OPEN_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VALVE_CLOSE_GPIO, VALVE_CLOSE_PIN, GPIO_PIN_SET);

		TickType_t ticks = pdMS_TO_TICKS((uint32_t)(FULL_TRAVEL_TIME * 1000.0f));
		xTimerChangePeriod(HTimer_M3voies, ticks, 0); // Change la durée
		xTimerStart(HTimer_M3voies, 0);               // Lance le countdown

		init_attente=3;
	#endif

	PIDd_Init(&myPID,
	              2.0f,       // Kp
	              60.0f,      // Ti (minutes)
	              0.0f,       // Td (minutes)
	              1.0f,      // dt = 1 minute
	              0.0f, 100.0f);
  #endif

	// Demande de mise à l'heure par le node
	#ifdef END_NODE
		//envoie_mess_ASC(0x10, "HHLS");  // 22:Ack avec 2 renvois, RX apres  24:5 renvois
	#endif

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
	  //__HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARROK);
	  //__HAL_LPTIM_ENABLE_IT(&hlptim1, LPTIM_IT_CMPOK);

}

void test_i2c()
{
	 uint8_t init=0;

		  UART_SEND("Init0\n\r");
	      osDelay(500);
		  UART_SEND("Init00\n\r");
		  HAL_Delay(500);
		  char messa[20];

		  if (init==0)
		  {
			  init=1;
			  UART_SEND("Init2\n\r");


			  messa[0] = 't';
			  messa[1] = 'm';
			  messa[2] = '1';
			  messa[3] = '\n';
			  messa[4] = '\r';
			  messa[5] = 0;
			  HAL_UART_Transmit(&hlpuart1, (uint8_t*)messa, strlen(messa), 3000);

		      osDelay(100);
			  //HDC1080_init();
		  }

		  //uint32_t primask = __get_PRIMASK();
		  uint16_t conf_reg;
		  conf_reg = HDC1080_read_configuration_register();
			messa[0] = 'C';
			messa[1] = '0';
			messa[2] = ':';
		      messa[3] = ((conf_reg >> 12) & 0x0F) + '0';
		      messa[4] = ((conf_reg >> 8) & 0x0F) + '0';
		      messa[5] = ((conf_reg >> 4) & 0x0F) + '0';
		      messa[6] = (conf_reg & 0x0F) + '0';
			messa[7] = '\n';
			messa[8] = '\r';
			messa[9] = 0;
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)messa, strlen(messa), 3000);
			//HAL_Delay(500);
		      osDelay(1000);


		  if (HAL_I2C_IsDeviceReady(&hi2c2, HDC1080_I2C_Address, 3, 100) != HAL_OK)
		  {
		      osDelay(500);
			  UART_SEND("HDC1080 NOT READY !\n\r");
		  }
		  else
		  {
		      osDelay(500);
			  UART_SEND("HDC1080 READY !\n\r");
		  }
	      osDelay(500);
		  HAL_Delay(500);


		  uint16_t tempL;
		  uint8_t hygroL=0;
		  uint8_t ret = 0;
		  ret = HDC1080_read_tempe_humid(&tempL, &hygroL);
		  //HDC1080_start_read_configuration_registerIT();

		  //osDelay(1000);
		  //tempL = HDC1080_config_reg;

	  	  messa[0] = ret+'0';
	  	  messa[1] = 'T';
	      messa[2] = ((tempL >> 12) & 0x0F) + '0';
	      messa[3] = ((tempL >> 8) & 0x0F) + '0';
	      messa[4] = ((tempL >> 4) & 0x0F) + '0';
	      messa[5] = (tempL & 0x0F) + '0';
	  	  messa[6] = ' ';
	      messa[7] = ((hygroL >>4) & 0x0F) + '0';
	      messa[8] = (hygroL & 0x0F) + '0';
	      messa[9] = '\n';
	      messa[10] = '\r';
	      messa[11] = 0;
	      HAL_UART_Transmit(&hlpuart1, (uint8_t*)messa, strlen(messa), 3000);

	      osDelay(500);
		  HAL_Delay(500);

		  tempL=0;
		  ret = HDC1080_read_tempe_humid(&tempL, &hygroL);
		  //tempL = HDC1080_read_temperature_humidity();

	  	  messa[0] = ret+'0';
	  	  messa[1] = 'T';
	      messa[2] = ((tempL >> 12) & 0x0F) + '0';
	      messa[3] = ((tempL >> 8) & 0x0F) + '0';
	      messa[4] = ((tempL >> 4) & 0x0F) + '0';
	      messa[5] = (tempL & 0x0F) + '0';
	  	  messa[6] = ' ';
	      messa[7] = ((hygroL >>4) & 0x0F) + '0';
	      messa[8] = (hygroL & 0x0F) + '0';
	      messa[9] = '\n';
	      messa[10] = '\r';
	      messa[11] = 0;
	      HAL_UART_Transmit(&hlpuart1, (uint8_t*)messa, strlen(messa), 3000);

		  HAL_Delay(500);


	/*char messa[20];
	uint32_t a[7];

	a[0] = HAL_GetTick();
	osDelay(500);  // 500 ticks = 500ms
	a[1] = HAL_GetTick();
	a[2] = HAL_GetTick();
	HAL_Delay(500);
	a[3] = HAL_GetTick();
	a[4] = HAL_GetTick();

	for (uint8_t i=0; i<6; i++)
	{
		messa[0] = 't';
		messa[1] = i + '0';
		messa[2] = ':';
	      messa[3] = ((a[i] >> 12) & 0x0F) + '0';
	      messa[4] = ((a[i] >> 8) & 0x0F) + '0';
	      messa[5] = ((a[i] >> 4) & 0x0F) + '0';
	      messa[6] = (a[i] & 0x0F) + '0';
		messa[7] = '\n';
		messa[8] = '\r';
		messa[9] = 0;
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)messa, strlen(messa), 3000);
		osDelay(30);
	}*/

}

void lecture_temp_i2c(uint8_t nb)
{
	  if (HAL_I2C_IsDeviceReady(&hi2c2, HDC1080_I2C_Address, 3, 100) != HAL_OK)
	  {
	      osDelay(500);
		  UART_SEND("HDC1080 NOT READY !\n\r");
	  }
	  else
	  {
	      osDelay(500);
		  UART_SEND("HDC1080 READY !\n\r");
	  }
    osDelay(500);
	  //HAL_Delay(500);


      uint16_t tempL=0;
	  uint8_t hygroL=0;
	  uint8_t ret = 0;
	  ret = HDC1080_read_tempe_humid(&tempL, &hygroL);
	  //HDC1080_start_read_configuration_registerIT();

	  //osDelay(1000);
	  //tempL = HDC1080_config_reg;
	  char messa[20];

	  messa[0] = ret+'0';
	  messa[1] = nb+'0';
    messa[2] = ((tempL >> 12) & 0x0F) + '0';
    messa[3] = ((tempL >> 8) & 0x0F) + '0';
    messa[4] = ((tempL >> 4) & 0x0F) + '0';
    messa[5] = (tempL & 0x0F) + '0';
	  messa[6] = ' ';
    messa[7] = ((hygroL >>4) & 0x0F) + '0';
    messa[8] = (hygroL & 0x0F) + '0';
    messa[9] = '\n';
    messa[10] = '\r';
    messa[11] = 0;
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)messa, strlen(messa), 3000);

    osDelay(500);
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

    //test_i2c();
    //lecture_temp_i2c(1);

    init4(); // Radio, watchdog, Log, eeprom


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
					LOG_INFO("TX_step reveil ");
					//lora_tx_state_step();
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
					#ifndef Sans_Watchdog
						HAL_IWDG_Refresh(&hiwdg);  // refresh watchdog hardware
						watchdog_check_all_tasks(); // test watdchdog logiciel
					#endif
					LOG_INFO("Refresh watchdog");
				    break;
				}
				case EVENT_AlarmA: {
					LOG_INFO("Alarme A : 19h");
					RAZ_nb_max_log_write();
					mesure_batt_ok=0;  // permet de lire le niveau batterie apres transmission
					batt_avant = GetBatteryLevel();

					if (Keepalive < KeepAlive)
						Keepalive++;
					else
					{
						Keepalive=1;

						#if CODE_TYPE == 'B'  // thermometre
							log_write('K', batt_avant, batt_apres, 0, "Keep");
							// envoi message Temp vers vanne motorisée chaudiere  CHETxx
							message.dest = 'J';
							message.param = 0x10; //param_def;
							message.type = 1;  // binaire
							message.data[0] = message.dest | 0x80;
							uint8_t i=2;
							message.data[i++] = 'C';
							message.data[i++] = 'H';
							message.data[i++] = 'E';
							message.data[i++] = 'K';
							message.data[i++] = batt_avant;
							message.data[i++] = batt_apres;
							message.data[1] = i-3;
							envoie_mess_bin(&message);
						#endif

						#if CODE_TYPE == 'C'  // vanne motoriséee
							// enregistrement du nb de mesures de temp recues et puissance
							nb_mes_temp24 = nb_mes_temp;
							nb_mes_temp = 0;
							puis_chaud24 = (uint8_t)(puis_chaud/144);

							log_write('K', nb_mes_temp24, puis_chaud24, batt_thermo_ap, "Keep");
						#endif

					break;
				}
				case EVENT_TIMER_24h: {
					}
					// Debug : afficher le temps restant avant la prochaine expiration
					TickType_t expiry = xTimerGetExpiryTime(HTimer_24h);
					TickType_t now = xTaskGetTickCount();
					LOG_INFO("Il reste %lu ticks avant la prochaine expiration\n",
						   (expiry > now) ? (expiry - now) : 0);
					break;
				}

				#if CODE_TYPE == 'C'  // Chaudiere chauffage
					case EVENT_TIMER_1min: { // calcul PID -> pilotage vanne 3 voies
						LOG_INFO("timer 1min");
						HAL_Delay(1000);
						if (init_attente) init_attente--;          // attent 3 minutes que la vanne se ferme
						if (init_attente==1) envoie_mess_ASC(0x10, "HHLS");  // 22:Ack avec 2 renvois, RX apres  24:5 renvois
						//if (init_attente==1)  pid_forcage_init();  // initialise la consigne de forcage
						if (init_attente==0)  calcul_pid_vanne();  // régule chaque minute le pid
						break;
					}

					case EVENT_TIMER_3Voies: {  // fin du timer de modif vanne 3 voie
						LOG_INFO("fin mouvement 3 voies");
						break;
					}

					case EVENT_TIMER_10min: { // chaque 10 min : chgt de consigne
						LOG_INFO("timer 10min");
						HAL_Delay(1000);
						if (!ch_arret)  chgt_consigne();
						break;
					}
				#endif

				case EVENT_TIMER_Tempe: {  // Thermometre-hygro
					LOG_INFO("timer mesure temp-hygro 5min");
					#if CODE_TYPE == 'B'

				      /*osDelay(1000);
					  if (HAL_I2C_IsDeviceReady(&hi2c2, HDC1080_I2C_Address, 3, 100) != HAL_OK)
					  {
					      osDelay(1000);
						  UART_SEND("HDC1080_2 NOT READY !\n\r");
					  }
					  else
					  {
					      osDelay(1000);
						  UART_SEND("HDC1080_2 READY !\n\r");
					  }
				      osDelay(1500);*/

						uint8_t ret = HDC1080_read_tempe_humid(&temp, &hygro);
						if (ret)
						{
							LOG_INFO("erreur_temp:%i", ret);
							log_write('E', log_w_err_temp, ret, 0, "Err_Temp");
						}
						else
						{
							// envoi message Temp vers vanne motorisée chaudiere  CTxx (8 car)
							message.dest = 'H';
							message.type = 1;  // binaire
							message.param = param_def; //10:Pas d'ack, pas RX apres.  30:pas d'ack, RX apres
							message.data[0] = message.dest | 0x80;
							uint8_t i=2;
							message.data[i++] = 'C';
							message.data[i++] = 'T';
							message.data[i++] = temp >> 8;
							message.data[i++] = temp & 0xFF;
							message.data[1] = i-3;
							envoie_mess_bin(&message);

							int8_t temp_int = temp/100- 100;
							uint8_t temp_vir = temp%100;
							LOG_INFO("Temp:%i.%i Hygro:%i", temp_int, temp_vir, hygro);
							/*tempe[num_val_temp] = temp;
							humid[num_val_temp] = hygro;
							num_val_temp++;
							// envoi des données, quand la trame est complète
							if (num_val_temp == nb_samples)
							{
								//envoi_data(num_val_temp);
								num_val_temp = 0;  // nouvelle trame
							}*/
						}
					#endif
					break;
				}
				case EVENT_TIMER_20min: {

					//LOG_INFO("a");
					//uint8_t Vbat = GetBatteryLevel();
					//uint16_t mvolt = Vbat*4+2600;


					//LOG_INFO("timer 20s batt:%i %i", Vbat, mvolt);

					/*GetInternalTemp();

					#ifdef END_NODE
						cpt_timer20s++;  // 1(20s), 3(1min), 6(2min)
						if ((cpt_timer20s==1) || ((cpt_timer20s%3) == 0))
						{
							cpt_message++;
							param_def=0x30; // 0x30 sans Ack, RX apres   0x10 sans ack
							if (cpt_message==2) param_def = 0x30;  // 0x20:avec ack, RX apres
							if (cpt_message==3) param_def = 0x30;  // 2:avec ack, 2 renvois
							if (cpt_message==4) param_def = 0x20;  // 0x30:sans ack, RX apres
							//envoie_mess_ASC(param_def, "HTTT%i", cpt_message);
						}
					#endif*/
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


#if CODE_TYPE == 'B'
void envoi_data (uint8_t nb_valeur)
// 14 Octets pour 1 valeur
{
    uint8_t batteryLevel = GetBatteryLevel();
   	LOG_INFO("Batt VDDA: %d\r\n", batteryLevel);

   	message.dest = '1'; // uart
   	message.type = 1;  // binaire

   	message.data[0] = message.dest | 0x80;
   	uint8_t i=2;
	message.data[i++] = 'C';  // Capteur
	message.data[i++] = 'V';  // Valeur
	message.data[i++] = CODE_TYPE;  // 2:capteur temperature STM32
	message.data[i++] = 0x41;  // code fonction : Hist, Temp-humid sans timestamp
	message.data[i++] = nb_valeur*3+4;  // longueur des données
	/*message.data[i++] = (timestamp_start >> 24) &0xFF;
	message.data[i++] = (timestamp_start >> 16) &0xFF;
	message.data[i++] = (timestamp_start >> 8) &0xFF;
	message.data[i++] = (timestamp_start) &0xFF;*/

	message.data[i++] = batteryLevel;
	message.data[i++] = ((temp_period)>>8) & 0xFF;
	message.data[i++] = (temp_period) & 0xFF;

	for (int8_t j=0; j < nb_valeur; j++)
	{
	  message.data[i++] = (tempe[j]>>8);
	  message.data[i++] = (tempe[j] & 0xFF);
	  message.data[i++] =	humid[j];
	  }
	message.data[1] = i-3; // longueur
	envoie_mess_bin(&message);
}
#endif

#ifdef END_NODE
#if CODE_TYPE == 'B'
/*void  lect()
{
   buf[0] = REG_TEMP;

   ret = HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
   if ( ret != HAL_OK ) {
     strcpy((char*)buf, "Error Tx\r\n");
   } else {

     // Read 2 bytes from the temperature register
     ret = HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, buf, 2, HAL_MAX_DELAY);
     if ( ret != HAL_OK ) {
       strcpy((char*)buf, "Error Rx\r\n");
     } else {

       //Combine the bytes
       val = ((int16_t)buf[0] << 4) | (buf[1] >> 4);

       // Convert to 2's complement, since temperature can be negative
       if ( val > 0x7FF )      val |= 0xF000;

       // Convert to float temperature value (Celsius)
       temp_c = val * 0.0625;

       // Convert temperature to decimal format
       temp_c *= 100;
       sprintf((char*)buf, "%u.%u C\r\n",((unsigned int)temp_c / 100),((unsigned int)temp_c % 100));
     }
   }
   // Send out buffer (temperature or error message)
   HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
}*/
#endif
#endif


#if CODE_TYPE == 'C'  // Chaudiere - chauffage

void chgt_consigne(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	uint8_t tps_actuel = sTime.Hours*6 + sTime.Minutes/10;
	uint32_t date_actuel = ((sDate.Year-20)*372 + sDate.Month*31 + sDate.Date)*144  + sTime.Hours*6 + sTime.Minutes/10;
	//LOG_INFO("forcage:actuel%i fin:%i", date_actuel, forcage_duree);
	if (forcage_duree <= date_actuel)
	{
		forcage_duree = 0;
		uint8_t valid = 0;
		for (uint8_t i=0; i<NB_MAX_PGM; i++)
		{
			if (ch_debut[i] != ch_fin[i])
			{
				if ((tps_actuel >= ch_debut[i]) && (tps_actuel <= ch_fin[i]))
				{
					valid=1;
					consigne_normale = ch_consigne[i];
					consigne_regulation = consigne_normale;
					consigne_apres = ch_cons_apres[i] * 5;  // pas de 0,5°C => pas de 0,1°C
					LOG_INFO("pgm valide %i", i);
				}
			}
		}
		if (!valid)
		{
			consigne_regulation = consigne_apres;
			LOG_INFO("fin de prog. cons:%i", consigne_apres);
		}
	}
}


void calcul_pid_vanne(void)
{
	// calcul PID
	float temp_int = ((float)Tint)/10.0;     // mesure
	float fl_consigne = ((float) consigne_regulation)/10.0;     // référence

	float position = PIDd_Compute(&myPID, fl_consigne, temp_int);

	LOG_INFO("PID:cons:%i temp:%i prec:%i futur:%i", consigne_regulation, Tint, (uint16_t)(pos_prec*10), (uint16_t)(position*10));

	if ((!position) && (!pos_prec))
	{
		LOG_INFO("vanne fermee %i", cpt_circulateur);
		if (cpt_circulateur < 3)
		{
			cpt_circulateur++;
			if (cpt_circulateur == 3)  // arret circulateur
			{
				ch_circulateur = 0;
				HAL_GPIO_WritePin(CIRCULATEUR_GPIO, CIRCULATEUR_PIN, GPIO_PIN_RESET);
				LOG_INFO("Arret circulateur %i", cpt_circulateur);
			}
		}
	}
	else if (!ch_circulateur)
	{
		ch_circulateur = 1;
		cpt_circulateur = 0;
		HAL_GPIO_WritePin(CIRCULATEUR_GPIO, CIRCULATEUR_PIN, GPIO_PIN_SET);
	}

	// si écart significatif => modif vanne 3 voies
	if (fabsf(position - pos_prec) < DEADBAND_PERCENT) {
		LOG_INFO("Pas de mouvement");
        return;
	}
	uint8_t action_time = (uint8_t) (fabsf(position - pos_prec) * FULL_TRAVEL_TIME / 100.0f);

	if (action_time > WINDOW_TIME)
	{
		action_time = WINDOW_TIME-3;
		LOG_INFO("limit action time %i", action_time);
	}
	else
		LOG_INFO("ok action time %i %i", action_time, WINDOW_TIME);

	if (position > pos_prec)
	{   // ouverture vanne

			LOG_INFO("Etat:%i Ouverture pendant %isec etat:%i", (uint16_t)(pos_prec*10), action_time, (uint16_t)(position*10));

			// Active moteur ouverture
			HAL_GPIO_WritePin(VALVE_CLOSE_GPIO, VALVE_CLOSE_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(VALVE_OPEN_GPIO, VALVE_OPEN_PIN, GPIO_PIN_SET);

			TickType_t ticks = pdMS_TO_TICKS((uint32_t)(action_time * 1000.0f));

			xTimerChangePeriod(HTimer_M3voies, ticks, 0); // Change la durée
			xTimerStart(HTimer_M3voies, 0);               // Lance le countdown
	}
	else
	{   // fermeture vanne

			LOG_INFO("Etat:%i Fermeture pendant %isec etat:%i", (uint16_t)(pos_prec*10), action_time, (uint16_t)(position*10));

			// Active moteur fermeture
			HAL_GPIO_WritePin(VALVE_OPEN_GPIO, VALVE_OPEN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(VALVE_CLOSE_GPIO, VALVE_CLOSE_PIN, GPIO_PIN_SET);

			TickType_t ticks = pdMS_TO_TICKS((uint32_t)(action_time * 1000.0f));

			xTimerChangePeriod(HTimer_M3voies, ticks, 0); // Change la durée
			xTimerStart(HTimer_M3voies, 0);               // Lance le countdown
	}
	pos_prec = position;
	puis_chaud += (uint8_t)(position);
}

// initialise la consigne de forcage
void pid_forcage_init(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	uint32_t date_actuel = ((sDate.Year-20)*372 + sDate.Month*31 + sDate.Date)*144  + sTime.Hours*6 + sTime.Minutes/10;
	//LOG_INFO("forcage:actuel%i fin:%i", date_actuel, forcage_duree);
	if (forcage_duree <= date_actuel)  // pas de forcage
	{
		forcage_duree = 0;
		LOG_INFO("Init : pas de forcage");
	}
	else
	{
		consigne_regulation = forcage_consigne;
		LOG_INFO("Init : forcage cons:%i", consigne_regulation);
	}
}

#endif
