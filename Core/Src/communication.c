/*
 * functions.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#include <main.h>
#include <communication.h>
#include <fonctions.h>
#include <eeprom_emul.h>
#include <log_flash.h>
#include "lora.h"
#include "cmsis_os.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef END_NODE
	#define nb_ligne_routage 2  //  0->Loop  1->Uart   Add+1->Uart
	uint8_t table_routage[nb_ligne_routage][6] = {
		{'1', '1', 3, 0, 0, 0},
		{'2', 'z', 7, 'H', 0, 0}
	};
#else
	#define nb_ligne_routage 4  // 1-> liaison série du a
	uint8_t table_routage[nb_ligne_routage][6] = {  // Uart:1, AàG  RF:Iàz
			{'1', '1', 3, 0, 0, 0}, {'a','z',6,0,0},
			{'I','Z',6,0}, {'A','G',3,0,0,0} };  // Passerelle RF (Q)
#endif

uint8_t param_def = 0x10; // bit0:dernier  bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)
              // bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
			  // 0x22 : Ack - 2 envois , rx apres
/* gestion des erreurs
Code_erreur => utile dans les ISR, peut masquer les premieres erreurs simultanées, gestion des répétitions
LOG_WARN(..) => envoie un message sur la sortie prédéfinie. message complet, limite 4/10min, pas ISR
LOG en Flash => pb répétitions. Limiter à 4 / 10 minutes*/

#define UART_RX_BUFFER_SIZE    50
#define UART_RX_QUEUE_SIZE     64

QueueHandle_t in_message_queue;  // queue pour les messages entrants

// Timers
//TimerHandle_t timer_handles[2];
uint32_t uart_timeout_rx;
uint8_t uart_timeout_on;

uint16_t cpt_rx_uart;

static out_message_t log_buffer;
out_message_t message;
static uint16_t len_ASC;
static out_message_t form_buf; // Buffer pour le formatage de message ASCII
static SemaphoreHandle_t log_mutex = NULL;

extern SUBGHZ_HandleTypeDef hsubghz;

extern UART_HandleTypeDef hlpuart1;

static uint8_t rx_char;
static uint8_t expected_length = 0;
static uint8_t message_type = 0;
static in_message_t mess_rx_uart;
static uint8_t car_valid;
static uint8_t buffer_index = 0;

//static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
//static uint16_t uart_rx_head = 0;
//static uint16_t uart_rx_tail = 0;
static QueueHandle_t uart_rx_queue;
uint8_t uart_rx_char;

UartStruct UartSt[NB_UART];


extern QueueHandle_t Event_QueueHandle;


void Uart_RX_Tsk(void *argument);
void Uart_TX_Tsk(void *argument);
uint8_t Uart2_receive (uint8_t* data, uint8_t type);
void debug_uart_interrupt_init();

/* Definitions for Uart2_RX_Task */
osThreadId_t Uart_RX_TaskHandle;
const osThreadAttr_t Uart_RX_Task_attributes = {
  .name = "Uart_RX_Task",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 512 * 4    // 190 utilisé
};

/* Definitions for Uart2_TX_Task */
osThreadId_t Uart_TX_TaskHandle;
const osThreadAttr_t Uart_TX_Task_attributes = {
  .name = "Uart_TXTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4    // 89 utilisé (+121 pour LOG)
};


// Variable globale pour le niveau de verbosité
static uint8_t current_log_level = CURRENT_LOG_LEVEL;

in_message_t message_recu;

static uint8_t mess_buffer[MESS_BUFFER_SIZE];
static uint16_t uart_head = 0;
static uint16_t uart_tail = 0;

static osMutexId_t bufferMutex;


uint8_t mess_enqueue(out_message_t* mess);
uint8_t mess_dequeue(out_message_t* mess);


/*void TIMEOUT_RX_Callback(TimerHandle_t xTimer)
{
	uint8_t num_uart = (uint8_t)(uint32_t)pvTimerGetTimerID(xTimer);  // timer_id

	event_t evt = { EVENT_UART_RAZ, num_uart, 0 };
	if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	{
	    LOG_ERROR("Event uart timeout - message lost");
	}
}*/

// verification si des caractères uart_rx sont dans le buffer
void verif_timout_uart_rx(void)
{
		//get_rtc_timestamp()
	    uint32_t rtc_actuel = get_rtc_seconds_since_midnight();
		//LOG_INFO("act:%i debut:%i", rtc_actuel, uart_timeout_rx);
	    if (rtc_actuel - uart_timeout_rx > 10)
	    {
			code_erreur=timeout_RX;
			raz_Uart(0);
	    	/*event_t evt = { EVENT_UART_RAZ, 1, 0 };
	    	if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	    	{
	    		code_erreur = erreur_queue_appli;
	    		err_donnee1 = 2;
	    	}*/
	    }
}

uint8_t init_communication(void)
{
    // Créer la queue pour les caractères reçus
    uart_rx_queue = xQueueCreate(UART_RX_QUEUE_SIZE, sizeof(uint8_t));

    if (uart_rx_queue == NULL) {
        //LOG_ERROR("Failed to create UART RX queue");
        return 1;
    }


    //LOG_INFO("UART2 interrupt reception initialized");

    // Créer la queue de messages
 	in_message_queue = xQueueCreate(10, sizeof(in_message_t));
 	/*if (in_message_queue == NULL)
 	{
 	  LOG_ERROR("Failed to create in_message queue");
 	  return;
 	}*/

	// mutex pour envoyer les log
    log_mutex = xSemaphoreCreateMutex();

	// Initialisation du mutex pour mettre enqueue
	bufferMutex = osMutexNew(NULL);
	/*if (bufferMutex == NULL) {
		LOG_ERROR("Failed to create bufferMutex");
		return;
	}*/
	//LOG_INFO("bufferMutex created: %p", bufferMutex);

	//UartSt[0].h_timeout_RX = xTimerCreate("TimeoutRX2", pdMS_TO_TICKS(5000), pdFALSE, ( void * ) 0, TIMEOUT_RX_Callback);  // name,period-tick, autoreload,id, callback
	//UartSt[0].h_timeout_TX = xTimerCreate("TimeoutTX2", pdMS_TO_TICKS(10000), pdFALSE, ( void * ) 0, TIMEOUT_TX_Callback);  // name,period-tick, autoreload,id, callback

	/* creation of Uart_TX_Task */
	Uart_TX_TaskHandle = osThreadNew(Uart_TX_Tsk, NULL, &Uart_TX_Task_attributes);


   /* creation of Uart_RX_Task */
   Uart_RX_TaskHandle = osThreadNew(Uart_RX_Tsk, NULL, &Uart_RX_Task_attributes);


   raz_Uart(0);

  /* HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
   HAL_NVIC_EnableIRQ(USART2_IRQn);
   SET_BIT(hlpuart1.Instance->CR1, USART_CR1_RXFFIE); // Interrupt avec Fifo
   SET_BIT(hlpuart1.Instance->CR3, USART_CR3_RXFTIE); // Interrupt thresold avec Fifo*/

   //SET_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);  // RXNE interrupt

   // Démarrer la réception par interruption
   //HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);

   //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Init2", 5, 3000);
   //HAL_Delay(500);
   return 0;
}


/**
* @brief Function implementing the Uart1_Task thread.
* @param argument: Not used
ACSII : longueur n'inclut pas le car_fin_trame (inclus emetteur) : 5 pour RLSLO (en fait 6)
1SLO : 49 83 76 79 13
Binaire : 12SLO => lg=2 (en fait 6)  B1 02 53 4C 4F
* @retval None
*/

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // Traitement du caractère reçu (sans transmission bloquante)
        // Le caractère est disponible dans uart_rx_char
        
        // Écho du caractère reçu
        uint8_t uart_tx_char = 0;
        uart_tx_char = uart_rx_char+1;
        HAL_UART_Transmit(&hlpuart1, &uart_tx_char, 1, 1000);

        // Redémarrer la réception - TOUJOURS À LA FIN
        HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
    }
}*/

	/*cpt_rx_uart++;

    // ✅ Vérifications de sécurité
    if (huart == NULL) {
        // Plantage évité
        return;
    }
    cpt_rx_uart++;

    if (huart->Instance != USART2) {
        // Mauvais UART
        return;
    }

    cpt_rx_uart++;

    // ✅ Pas de LOG_INFO dans l'ISR !
    // ✅ Pas de traitement complexe

    // Juste relancer la réception
    //HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
}*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == LPUART1) {
        // Traitement du caractère reçu (sans transmission bloquante)
        // Le caractère est disponible dans uart_rx_char

        // Écho du caractère reçu
        /*uint8_t uart_tx_char = 0;
        uart_tx_char = uart_rx_char+1;
        HAL_UART_Transmit(&hlpuart1, &uart_tx_char, 1, 1000);*/


        //if (!xQueueSendFromISR(uart_rx_queue, &uart_rx_char, &xHigherPriorityTaskWoken))
        if (!xQueueSendFromISR(uart_rx_queue, &uart_rx_char, NULL))
        	code_erreur = erreur_RX_queue;

        // Forcer le changement de contexte si nécessaire
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        // Redémarrer la réception - TOUJOURS À LA FIN
        HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
    }
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1) {
        //LOG_ERROR("UART2 error: 0x%08lX", huart->ErrorCode);
    	code_erreur = ISR_fifo_full;
    	err_donnee1 = huart->ErrorCode;

        // Réinitialiser la réception
        HAL_UART_AbortReceive_IT(&hlpuart1);
        HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
    }
}

void Uart_RX_Tsk(void *argument)
{

    // Démarrer la surveillance watchdog
    watchdog_task_start(WATCHDOG_TASK_UART_RX);
    //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Init4", 5, 3000);
    //HAL_Delay(500);

    //LOG_INFO("Uart_RX_Task started with watchdog protection");
    //uint8_t k=0;
    expected_length=0;

    for(;;)
    {
        // Heartbeat watchdog
        watchdog_task_heartbeat(WATCHDOG_TASK_UART_RX);

        watchdog_set_context(WATCHDOG_TASK_UART_RX, WATCHDOG_CONTEXT_WAITING);

        if (xQueueReceive(uart_rx_queue, &rx_char, osWaitForever) == pdPASS)
        {
            watchdog_set_context(WATCHDOG_TASK_UART_RX, WATCHDOG_CONTEXT_ACTIVE);
			/*k+=1;
			if (!(k%250))
			HAL_UART_Transmit(&hlpuart1, &rx_char, 1, 3000);*/

    		//k+=1;
    		//if (!(k%3))
    			//HAL_UART_Transmit(&hlpuart1, &rx_char, 1, 3000);

        	car_valid=1;
   		  //LOG_INFO("Received: 0x%02X ('%c')", rx_char, (rx_char >= 32 && rx_char <= 126) ? rx_char : '.');

            // Premier caractère : déterminer le type
            if (buffer_index == 0)
            {
                // Vérifier le bit de poids fort
                if (rx_char & 0x80)
                {
                	if ((rx_char==13) || (rx_char==10) || (!rx_char)) car_valid=0;  // pas CR ou LF en premier car
                    message_type = 1;  // Message binaire
                    //LOG_DEBUG("Binary message detected");
                }
                else
                {
                    message_type = 0;  // Message ASCII
                    //LOG_DEBUG("ASCII message detected");
                }
            }

            //LOG_INFO("Received: %i 0x%02X", buffer_index, rx_char);

            // Ajouter au buffer
            if ((buffer_index < sizeof(mess_rx_uart.data) - 2) && (car_valid))
            {
   			    if ((buffer_index==0) && ((rx_char)=='1'))  rx_char=My_Address;  // remplacement de 1 par mon adresse
   			    if ((buffer_index==0) && ((rx_char)==0xB1))  rx_char=(My_Address|0x80);  // remplacement de 1 par mon adresse

            	mess_rx_uart.data[buffer_index++] = rx_char;
   			    //xTimerReset(UartSt[0].h_timeout_RX, 0); // timeout au bout de x secondes si on ne recoit pas la fin du message
   			    uart_timeout_rx = get_rtc_seconds_since_midnight();
   			    uart_timeout_on = 1;

				#ifdef UART_AJOUT_EMETTEUR  // 1yyy->X1yyy   Xyyy->X1yyy   Qyyy->QXyyy
			    if (buffer_index==1)
				{
		           	if (rx_char == My_Address)  // dest local
		           		mess_rx_uart.data[buffer_index++] = '1';
		           	else
		           		mess_rx_uart.data[buffer_index++] = My_Address;
				}
				#endif

                // Traitement selon le type
                if (message_type == 0)
                {
	  	  	  	  	// Message ASCII : fin par '\0'
                    if ((rx_char == '\0') || (rx_char == 13))
                    {
                        // Message ASCII terminé
                    	uart_timeout_on=0;
   					    //xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout
   					    mess_rx_uart.length = buffer_index;
   					    mess_rx_uart.type = 0; // ASCII
   					    mess_rx_uart.source = 2; // UART2
                        //mess_rx_uart.timestamp = HAL_GetTick();

   					    if (rx_char==13)  // remplace par 0
   					    {
   					    	mess_rx_uart.data[buffer_index-1] = 0;

   					    }
                        //LOG_INFO("%s lg:%d",mess_rx_uart.data, mess_rx_uart.length-1 );
                        // Envoyer à la queue
                        if (buffer_index>4)  // minimum xxSL
                        {
                        	mess_rx_uart.length--;
							if (xQueueSend(in_message_queue, &mess_rx_uart, 0) != pdPASS)
							{
								code_erreur = erreur_RX_queue;
								LOG_ERROR("UART message queue full");
							}
							else
							{  // envoi de l'evenement a la tache appli
								event_t evt = {EVENT_UART_RX, SOURCE_UART, 0};
								if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
									{ code_erreur = erreur_queue_appli; err_donnee1=3;}
							}
                        }
                        // Réinitialiser
                        buffer_index = 0;
                        expected_length = 0;
                    }
                }
                else
                {
                    // Message binaire : longueur dans le 3ème octet
                    if (buffer_index == 3)
                    {
                        expected_length = mess_rx_uart.data[2]; // 3ème octet = longueur, min1
                        //LOG_DEBUG("Binary message length: %d", expected_length);
                        if ((!expected_length) || (expected_length>(MESS_LG_MAX-3)))
                        {
                        	code_erreur=erreur_rx_uart_bin;
                        	err_donnee1 = 1;
                        	err_donnee2 = expected_length;
                            buffer_index = 0;
                            expected_length = 0;
                        }
                    }

                    // Vérifier si on a reçu toute la longueur
                    if ((buffer_index >= 3) && (buffer_index >= expected_length + 4))
                    {
                    	//LOG_INFO("ok fin bin");
                    	//osDelay(100);
                        // Message binaire terminé
                    	uart_timeout_on=0;
   					    //xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout
   					    mess_rx_uart.length = buffer_index-1;
   					    mess_rx_uart.type = 1; // Binaire
   					    mess_rx_uart.source = 2; // UART2
                        //message.timestamp = HAL_GetTick();

                        // Envoyer à la queue
                        if (xQueueSend(in_message_queue, &mess_rx_uart, 0) != pdPASS)
                        {
                            LOG_ERROR("uartin_message queue full");
                        }
                        else
                        {  // envoi de l'evenement a la tache appli
			                event_t evt = {EVENT_UART_RX, SOURCE_UART, 0};
			                if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
   								{ code_erreur = erreur_queue_appli; err_donnee1=4;}
                        }

                        // Réinitialiser
                        buffer_index = 0;
                        expected_length = 0;
                    }
                }
            }
            else
            {
                // Buffer plein, réinitialiser
            	code_erreur = erreur_RX_full;
            	LOG_WARNING("UART buffer overflow, resetting");
                buffer_index = 0;
                expected_length = 0;
            }
		}

		//osDelay(10); // 1 car à 115200bauds = 0,1ms
		//LOG_INFO(".");
	}
  /* USER CODE END Uart1_Tsk */
}

// Tache appli : TODO attendre que message_in soit libre (CC:30us) ou mettre en queue
void reception_message_Uart2(in_message_t *msg)
{
	if (msg->type == 0)
	{
		// Message ASCII
		//LOG_INFO("Received ASCII message: %.*s", msg->length, msg->data);
		//LOG_INFO("Received ASCII message: %s lg:%d", msg->data, msg->length);
	}
	else
	{

		// Message binaire
		/*LOG_INFO("Received binary message, length: %d", msg->length);
		LOG_DEBUG("Binary data: ");
		for (int i = 0; i < msg->length; i++)
		{
			LOG_DEBUG("%02X ", msg->data[i]);
		}*/
	}

	// Appeler votre fonction de traitement existante
	traitement_rx(msg->data, msg->length);
}




void raz_Uart(uint8_t num_uart)  // raz car en reception (suite timeout)
{
   expected_length = 0;
   buffer_index = 0;
   //uart_rx_head = 0;
   //uart_rx_tail = 0;
   uart_timeout_on=0;
   //xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout RX
}


// PRS0 -> PURS0 (lg=5)
uint8_t envoie_mess_ASC(uint8_t param, const char* format, ...)
{

	if (format == NULL) {
		return 1; // Erreur : buffer nul
	}

	// Formatage des arguments variables
	va_list args;
	va_start(args, format);
	len_ASC = vsnprintf((char*)form_buf.data, sizeof(form_buf.data) - 1, format, args);
	va_end(args);

	// Vérifier si le formatage a réussi
	if (len_ASC < 0) {
		return 3; // Erreur : formatage échoué
	}


	// Vérifier si len<3 ou la longueur dépasse la taille maximale
	if ((len_ASC < 3) || ( (len_ASC+3) >= MESS_LG_MAX)) {
		return 2; // Erreur : dépassement de buffer ou trop court
	}

    /*char hex_str[40];  // 2 chars par octet + 1 pour \0, ajustez selon tx.len
    char *p = hex_str;

    for (uint8_t i = 0; i < len_ASC+2; i++) {
        p += sprintf(p, "%02X ", form_buf.data[i]);  // Espace entre chaque octet
    }
    LOG_INFO("envoie:lg:%i %s", len_ASC, hex_str);*/

	// décaler form_buf
	memmove(&form_buf.data[1], &form_buf.data[0], len_ASC+1);
	//memcpy(mess.data+1, form_buf, len_ASC + 1); // decalage & +1 pour inclure le '\0'
	len_ASC += 2;
	form_buf.data[0] = form_buf.data[1];
	form_buf.data[1] = My_Address;

    /*char hex_str2[40];  // 2 chars par octet + 1 pour \0, ajustez selon tx.len
    char *p2 = hex_str2;
    for (uint8_t i = 0; i < len_ASC+2; i++) {
        p2 += sprintf(p2, "%02X ", form_buf.data[i]);  // Espace entre chaque octet
    }
    LOG_INFO("envoie:l2:%i %s", len_ASC, hex_str2);*/

	// Envoyer le message
	form_buf.length = len_ASC;
	form_buf.dest = form_buf.data[0];
	form_buf.type = 0;  // ascii
	form_buf.param = param;
	uint8_t res = envoie_routage(&form_buf);

	if (res)
		LOG_INFO("err envoi mess ASCII: code:%i lg:%i mess:%s", res, len_ASC, form_buf.data);
	//osDelay(100);
	//LOG_INFO("enqueue:%i %s", len, mess);
    //osDelay(100);
	return res;
}

// 11OK :  31 01 4F 4B => B1 55 01 4F 4B (lg=1, en fait:5)
// P1RS => PU1RS (lg=5)
uint8_t envoie_mess_bin(out_message_t* mess)
{
	uint8_t len;
	//uint8_t mess[MESS_LG_MAX]; // Variable locale pour travailler

	len = mess->data[1]+3;

	// Vérifier si len<3 ou la longueur dépasse la taille maximale
	if ((len < 3) || ( (len+2) >= MESS_LG_MAX)) {
		return 2; // Erreur : dépassement de buffer
	}

	// Copier buf dans mess
	//memcpy(mess.data+1, buf, len); // decalage de 11TE
	memmove(&mess->data[1], &mess->data[0], len);
	len++; // pour emetteur  : 5
	mess->data[0] = mess->data[0] | 0x80;
	mess->data[1] = My_Address;

    /*for (int j = 0; j < len; j++) {
        LOG_DEBUG("0x%02X ", mess[j]);
    }*/
	mess->length = len;
	mess->type = 1;  // binaire
	mess->dest = mess->data[0] & 0x7F;


	// Envoyer le message
	uint8_t res = envoie_routage(mess);
	return res;

}

// utilise la table de routage pour envoyer le message
uint8_t envoie_routage( out_message_t* mess)  // envoi du message
{
	uint8_t destinataire, i, j, retc;
	retc=1;

	//LOG_INFO("envoi routage dest:%c lg:%i %s", mess->dest, mess->length, mess->data);

	destinataire = mess->dest;

	if (((destinataire == My_Address) && (My_Address!='1')) || (destinataire == '0'))   // envoi sur soi-meme -loop
	{
	    if (mess->length && (mess->length < MESS_LG_MAX -1))
	    {
	    	mess->length--;
	        //message_in[0] = '0' + message[0] & 0x80;  // emetteur=loop
	        traitement_rx (mess->data, mess->length);
	        retc=0;
	    }
	}
	else
	{


	  j = 0;
	  for (i = 0; i < nb_ligne_routage; i++) // recherche de la liaison a utiliser dans la table de routage
	  {
		  if ((destinataire >= table_routage[i][0])
			  && (destinataire <= table_routage[i][1]))
			{
			  j = table_routage[i][2]; // type liaison
			  break;
			}
	  }
	  if (j)
	  {
		  //LOG_INFO("AD j:%i", j);

		   if (j==3)
			   retc = mess_enqueue(mess); // Vers UART
		   if (j==6)
		   {
			    //HAL_Delay(10);
			    //char uart_msg[50];
			    //snprintf(uart_msg, sizeof(uart_msg), "Lora1: %s \r\n", mess);
			    //HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), 3000);
			    //HAL_Delay(10);
			    //UART_SEND("Send1\n\r");
			    retc = mess_LORA_enqueue(mess);
		   }
  		   if (j==7) {
			    retc = mess_LORA_enqueue(mess);
  			    mess->dest = table_routage[i][3];
			    /*char uart_msg[50];
			    snprintf(uart_msg, sizeof(uart_msg), "Lora2: %s \r\n", mess);
			    HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), 3000);
			    HAL_Delay(10);*/
			    //LOG_INFO("enqueue:%i", retc);
			    //HAL_Delay(100);
			  //retc = send_lora_message((const char*)mess, len,  table_routage[i][3]);
  		   }
 	  }
  }
  return retc;

}


// Ajout d’un message Uart_TX
uint8_t mess_enqueue(out_message_t* mess)
{

    if ((mess->length < 5) || (mess->length > MESS_LG_MAX))
    {
    	return 1;
    }

    uint16_t total_size = mess->length+4;

    if (uart_head >= MESS_BUFFER_SIZE) {
            uart_head = 0; // Reset si corruption
            uart_tail = 0;
    }
    uint32_t start_time = HAL_GetTick();

    uint16_t free_space;

    if (uart_head >= uart_tail)
        free_space = MESS_BUFFER_SIZE - (uart_head - uart_tail) - 1;
    else
        free_space = (uart_tail - uart_head) - 1;

	 while (free_space < (uint16_t)(total_size + 10))
	 {
        osDelay(100);  // Attendre 100ms
		if ((HAL_GetTick() - start_time) > 2000)
		{
		    log_write('E', log_w_err_uart_bloque, 0x02, 0x03, "uartRxBl");
		    return 2;  // Timeout
		}
	    if (uart_head >= uart_tail)
	        free_space = MESS_BUFFER_SIZE - (uart_head - uart_tail) - 1;
	    else
	        free_space = (uart_tail - uart_head) - 1;
	 }

	osStatus_t status = osMutexAcquire(bufferMutex, 5000);
	if (status != osOK) return 3;

    uint16_t head_prov = uart_head;

    uint8_t* mess_ptr = (uint8_t*)mess;
        for (uint16_t i = 0; i < total_size; i++) {
            mess_buffer[head_prov] = mess_ptr[i];
            head_prov = (head_prov + 1) % MESS_BUFFER_SIZE;
        }

        uart_head = head_prov;

    osMutexRelease(bufferMutex);
    xTaskNotifyGive(Uart_TX_TaskHandle); // Notifier la tache UArt_TX
    return 0;
}

// Extraction d’un message
uint8_t mess_dequeue(out_message_t* mess)
{
	osStatus_t status = osMutexAcquire(bufferMutex, 10000);
	if (status != osOK) return 4;

	uint16_t tail_prov=uart_tail;

    if (uart_head == uart_tail) {
        osMutexRelease(bufferMutex);
        return 1; // FIFO vide
    }


    uint16_t size = mess_buffer[uart_tail];

	//osDelay(300);
	//LOG_INFO("dequeue1:head:%d tail:%d", head, tail);
    //osDelay(300);

    // Vérif longueur valide
	if ((size < 5) || (size > MESS_LG_MAX) || tail_prov >= MESS_BUFFER_SIZE)
	{
		uart_head=0;
		uart_tail=0;
		osMutexRelease(bufferMutex);
		return 2; // corruption détectée
	}

	// Vérif que les données tiennent dans la FIFO actuelle
	uint16_t available = (uart_head >= tail_prov) ?
						 (uart_head - tail_prov) :
						 (MESS_BUFFER_SIZE - (tail_prov - uart_head));

	if (available < size) {
		uart_head=0;
		uart_tail=0;
		osMutexRelease(bufferMutex);
		return 3; // corruption : message incomplet
	}

    uint8_t* mess_ptr = (uint8_t*)mess;
        for (uint16_t i = 0; i < size+4; i++) {
            mess_ptr[i] = mess_buffer[tail_prov];
            tail_prov = (tail_prov + 1) % MESS_BUFFER_SIZE;
        }
    uart_tail = tail_prov;
	//osDelay(300);
	//LOG_INFO("dequeue2:head:%d tail:%d lg:%d", head, tail, *len);
    //osDelay(300);

    osMutexRelease(bufferMutex);
    return 0;
}

void Uart_TX_Tsk(void *argument)
{
    out_message_t mess;
    //uint32_t last_status_time = 0;

    //osDelay(100);

    // Démarrer la surveillance watchdog pour cette tâche
    watchdog_task_start(WATCHDOG_TASK_UART_TX);
    //LOG_INFO("Uart_TX_Task started with watchdog protection");

    for (;;)
    {
        // Enregistrer un heartbeat pour le watchdog
        watchdog_task_heartbeat(WATCHDOG_TASK_UART_TX);
        
        watchdog_set_context(WATCHDOG_TASK_UART_TX, WATCHDOG_CONTEXT_WAITING);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        watchdog_set_context(WATCHDOG_TASK_UART_TX, WATCHDOG_CONTEXT_ACTIVE);

        while (1)
        {
			uint8_t stat = mess_dequeue(&mess);

			if (stat == 0) {
				// ⭐ MESSAGE DISPONIBLE - Envoyer
				HAL_StatusTypeDef status = HAL_UART_Transmit(&hlpuart1, mess.data, mess.length, 10000);
				if (status != HAL_OK) {
					code_erreur = code_erreur_envoi;
					err_donnee1 = status;
					err_donnee2 = mess.length;
				}
			} else if (stat == 1) {
				// ⭐ PAS DE MESSAGE (NORMAL) - Sortir de la boucle
				break;
			} else {
				// ⭐ ERREUR DE mess_dequeue - Gérer l'erreur
				code_erreur = code_erreur_dequeue;
				err_donnee1 = stat;
				err_donnee2 = mess.length;
				break;  // Sortir en cas d'erreur
			}
		}
        //osDelay(1000); // rien à envoyer → on laisse tourner le CPU
        //LOG_INFO(".");
        //osDelay(30); // rien à envoyer → on laisse tourner le CPU
    }
}

/**
 * @brief Envoyer un message LoRa : mise en pile d'anvoi
 * @param message: Message à envoyer (string)
 * @retval HAL_StatusTypeDef: Statut de l'opération
 */
/*HAL_StatusTypeDef send_lora_message(const char* message, uint8_t message_length, uint8_t dest)
{
	return 0;
}*/

/*HAL_StatusTypeDef send_lora_message(const char* message, uint8_t message_length, uint8_t dest)
{
    uint8_t radio_status;
    //uint16_t message_length = strlen(message);

    // Vérifier que le radio est libre
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) != HAL_OK) {
        LOG_ERROR("Failed to read radio status");
        return HAL_ERROR;
    }

    if (radio_status & 0x01) { // En transmission
        LOG_WARNING("LoRa radio busy");
        return HAL_BUSY;
    }

    // Écrire le message dans le buffer radio
    if (HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, (uint8_t*)message, message_length) != HAL_OK) {
        LOG_ERROR("Failed to write LoRa buffer");
        return HAL_ERROR;
    }

    // Démarrer la transmission
    uint8_t tx_cmd = 0x83; // Commande TX
    HAL_StatusTypeDef status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, tx_cmd, NULL, 0);

    if (status == HAL_OK) {
        LOG_INFO("LoRa transmission started: %s", message);
    } else {
        LOG_ERROR("Failed to start LoRa transmission: %d", status);
    }

    return status;
}*/

/**
 * @brief Fonction principale de logging avec niveau de verbosité
 * @param level: Niveau de verbosité (1-10)
 * @param format: Format string (comme printf)
 * @param ...: Arguments variables
 */
void print_log(uint8_t level, const char* format, ...)
{

    // Vérifier si le niveau est suffisant pour afficher
    if (level > current_log_level) {
        return;
    }

    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return; // Échec d'acquisition du mutex
        }
    else
    {
		// Préfixe selon le niveau
		const char* prefix;
		switch (level) {
			case LOG_LEVEL_ERROR:
				prefix = "[ERROR] ";
				break;
			case LOG_LEVEL_WARNING:
				prefix = "[WARN]  ";
				break;
			case LOG_LEVEL_INFO:
				prefix = "[INFO]  ";
				break;
			case LOG_LEVEL_DEBUG:
				prefix = "[DEBUG] ";
				break;
			case LOG_LEVEL_VERBOSE:
				prefix = "[VERB]  ";
				break;
			default:
				prefix = "[LOG]   ";
				break;
		}

		// Ajouter le préfixe
		log_buffer.data[0] = dest_log;
		log_buffer.data[1] = My_Address;
		strcpy((char*)log_buffer.data+2, prefix);

		// Formatage des arguments
		va_list args;
		va_start(args, format);
		vsnprintf((char*)log_buffer.data + strlen(prefix)+2, sizeof(log_buffer.data) - strlen(prefix) - 4, format, args);
		va_end(args);

		// Ajouter un retour à la ligne
		strcat((char*)log_buffer.data, "\r\n");
		//len_ASC+=2;

		// Envoyer via UART
		//HAL_UART_Transmit(&hlpuart1, (uint8_t*)log_buffer, strlen(log_buffer), 3000);
		//envoie_mess_ASC("%s", log_buffer);
		log_buffer.length = strlen((char*)log_buffer.data);
		log_buffer.dest = dest_log;
		log_buffer.type = 0; // ascii
		log_buffer.param = 0;
		envoie_routage(&log_buffer);
	    xSemaphoreGive(log_mutex);
    }
}

/**
 * @brief Changer le niveau de verbosité à l'exécution
 * @param level: Nouveau niveau (1-10)
 */
void set_log_level(uint8_t level)
{
    if (level >= 1 && level <= 10) {
        current_log_level = level;
        LOG_INFO("Log level changed to %d", level);
    }
}

/**
 * @brief Obtenir le niveau de verbosité actuel
 * @retval Niveau actuel (1-10)
 */
uint8_t get_log_level(void)
{
    return current_log_level;
}

void traitement_rx (uint8_t* message_in, uint8_t longueur_m) // var :longueur n'inclut pas le car_fin_trame (inclus emetteur) : 4 pour RLT1
{       // def :longueur n'inclut pas la longueur (inclus l'emetteur) : 4 pour RL1T1
  uint8_t a,  crc, type;//, emet_m;
  //uint8_t tempo1, tempo2, tempo3, tempo4;
  //unsigned char volatile   * pregistre_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)
  //unsigned int volatile    * pregistre_int_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)
  //uint32_t i32;

  type = message_in[0] & 0x80;

  //for (a=0; a<longueur_m+1; a++)
  //    LOG_DEBUG("%i:%02X ", a, message_in[a]);

  //raz_timer_sleep ();
  message.data[0] = message_in[1]; // emetteur devient le destinataire du futur message
  message.dest = message_in[1];
  message.type =0;
  message.param = 0;

  //emet_m = message[0];

  if (((message_in[0] & 0x7F) != My_Address) && ((message_in[0] & 0x7F) != '0'))  // Transfert ailleurs
  {
      if (longueur_m  < MESS_LG_MAX-1)
      {
        #ifdef CC13xx
          // Message long
          if ((message_in[0]==('L'+0x80)) && (message_in[3]=='Y'))    // MESSAGE LONG : L-Em-Lg-Y-type-num*2
          {
              uint8_t code;
              send_long_buffer_tx=0;
              if (port_dest(message_in[1])==3)  // Node et STM32 : si reception de l'uart
              {
                  message_test[1] = 1;  // TODO
                  recep_mess_long_uart();
                  code = 1;
              }
              else
              {
                  code = recep_long( message_in);   // Concentrateur : reception de RF
                  recep_mess_long_rf(code);   // Concentrateur : reception de RF
              }

              if ((send_long_actif) && (code < 4))
              {
                  if (message_in[4] == 2)   // derniere trame
                  {
                      send_long_actif = 0;
                      CLOCK_STOP( xTimer_envoi);
                  }
                  memcpy (message, message_in, longueur_m + 1);
                  flag_comm_transfert=1;
                  if ((port_dest ('L') & 0b110) == 6)  // vers RF : 6 ou 7
                  {
                       uint16_t num_trame;
                       num_trame = ((uint16_t)message_in[5]<<8) + message_in[6];
                       Mess_statut = 0x10; // pas d'Ack  // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)  bit3:diffï¿½rï¿½   bit4:pas d'ack  bit5:RX apres
                       if (!(num_trame%10)) Mess_statut = 0;  // Ack pour 1 message sur 10  TODO
                  }

                  transf_buff (longueur_m);  // transfert du message long
              }
          }
          else  // message normal a transferer
          #endif
          {
              memcpy (message.data, message_in, longueur_m + 1);
              message.length = longueur_m+1;
              message.param = param_def;
              message.dest = message_in[0] & 0x7F;

              if (message_in[0] & 80) message.type =1;
              envoie_routage (&message);  // envoi du message
          }
      }
      else
      {
          code_erreur = erreur_mess;  // et '2'
          err_donnee1 = '2';
          err_donnee2 = message_in[0];
      }
  }
  else if ( (message_in[0] == My_Address) && (message_in[2] == '1'))  // transfert vers Uart : XL1HLH->1LHLH - message texte
	  // XL31HLH->1L2HLH - message longueur definie
  {
	  if (longueur_m+1 < MESS_LG_MAX)
	  {
		message.dest = '1';
		message.length = longueur_m;
		message.type = 0;
		if (message_in[0] & 0x80) message_type=1;
		memmove(message.data, message_in+1, longueur_m);
		message.data[0] = '1';
		message.data[1] = message_in[1];
		if (message_type)  // binaire
		{
			message.data[2] --;  // diminution longueur
		}
		else  // acsii
		{
		}

	    for (int j = 0; j < longueur_m+1; j++) {
	        LOG_DEBUG("0x%02X ", message.data[j]);
	    }
	    mess_enqueue(&message);
	  }
  }
  else if ((message_in[0] == (My_Address|0x80)) && (message_in[3] == '1'))  // transfert vers Uart : XL31HLH->1L2HLH - message longueur definie
  {
	  if (longueur_m+1 < MESS_LG_MAX)
	  {
	  }
  }
  else
  {
      if (longueur_m >= MESS_LG_MAX)
      {
          longueur_m = MESS_LG_MAX;
          code_erreur =erreur_mess;
      }

      if (type)         // message binaire - longueur definie dans le message
      {
          for (a = 2; a < longueur_m; a++) // 2 pour long=3, RL0T - suppression de la longueur
            message_in[a] = message_in[a + 1];
      }

      //for (a=0; a<longueur_m+1; a++)
      //    LOG_DEBUG("bis %i:%02X ", a, message_in[a]);

      crc = 1;

      if (message_in[2] & 0x80)   // si CRC avec 1er car de la payload
      {
          crc = 0;
          for (a = 2; a < (longueur_m - 1); a++)   // long=5 si RLT1cf
            crc = crc + message_in[a];
          message_in[2] = message_in[2] & 0x7F;
          if (message_in[longueur_m - 1] != crc)
            crc = 0;
          else
            {
              longueur_m--;
            }
          message.data[1] = 'S';  // Accuse reception ok : S1
          message.data[2] = '1';
          message.data[3] = car_fin_trame;
          envoie_mess_ASC(param_def, (const char*)message.data);
          if (!crc)
              message.data[2] = '0'; // si crc errone => envoi de S0
      }

      if (crc)
      {
          #ifdef CC13xx
          Mess_statut = 0;  // statut par defaut pour tout envoi de message
          if (rxrf_numero_device > lastAddedSensorNode)   // 0:pas de device  1:device 0 etc...
              rxrf_numero_device=0;
          #endif

          // ******************************** AAAAAAAAAAAAABAAAAAAA  ********************

          if ((message_in[2] == 'A') && (message_in[3] == 'L')) // AL Lecture entree analogique
          {
              if ( (message_in[4] >='0') &&  (message_in[4] <= '9') && (longueur_m==5)) // Read ALx  : Lecture entree analog x
              {
                  uint8_t ret;
                  uint16_t ent_anal=13;
                  //ret = lecture_analog((message_in[4]-'0'), &ent_anal);  // 0:30(solaire)   1:27(batterie)
                  ret=0;
                  if (!ret)
                  {
                      envoie_mess_ASC(param_def, "%cVAnal %i", message.data[0], ent_anal);
                  }
              }
          }


		// ******************************** CCCCCCCCCCCCCCCCCCC  ********************

		#if CODE_TYPE == 'C'  // Chaudiere chauffage
		if ((message_in[2] == 'C') && (message_in[3] == 'H'))  // Chaudiere
		{
			if (message_in[4] == 'L')  // Lecture
			{
				if ((message_in[5] == 'S') && (longueur_m == 6))   // CHLS  Lecture Statut
				{
					RTC_TimeTypeDef sTime;
					RTC_DateTypeDef sDate;
					HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
					uint16_t der_temp = sDate.Date*1440 + sTime.Hours*60 + sTime.Minutes - heure_der_temp;

                   envoie_mess_ASC(param_def, "%cCHLS:Cons:%i Tint:%i(%imin) Cons_norm:%i Cons_ap:%i 3V:%i", message_in[1], \
                		   consigne_regulation, Tint, der_temp, consigne_normale, consigne_apres, (uint16_t)(pos_prec) );
				}
				if ((message_in[5] == 'F') && (longueur_m == 6))   // CHLF  Forcage chauffage
				{
                   envoie_mess_ASC(param_def, "%cCHLF%04X%02X", message_in[1], forcage_duree, forcage_consigne);
				}
				if ((message_in[5] == 'A') && (longueur_m == 6))   // CHLA  Arret chauffage
				{
                   envoie_mess_ASC(param_def, "%cCHLA%i", message_in[1], ch_arret);
				}
				if ((message_in[5] == 'P') && (longueur_m == 6))   // CHLP  Planning chauffage
				{
					for (uint8_t i=0; i<NB_MAX_PGM; i++)
					{
						if (ch_debut[i] != ch_fin[i])
							envoie_mess_ASC(param_def, "%cCHLP%i%02X%02X%i%02X%02X", message_in[1], i, \
									ch_debut[i], ch_fin[i], ch_type[i], ch_consigne[i], ch_cons_apres[i]);
					}
				}

			}
			if (message_in[4] == 'E')  // Ecriture
			{
				// temperature interieure
				if ((message_in[5] == 'T') && (longueur_m == 8))   // CHETxx  Temperature interieure
				{
					uint16_t TempI = decod_asc8(message_in+6); // 5° à 25°
					if ((TempI>=50) && (TempI<=250))
					{
						Tint = TempI;
						RTC_TimeTypeDef sTime;
						RTC_DateTypeDef sDate;
						HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
						heure_der_temp = sDate.Date*1440 + sTime.Hours*60 + sTime.Minutes;
					}
				}
				// forcage
				if ((message_in[5] == 'F') && (longueur_m == 16))   // CHEFddddddddcc  Forcage chauffage
				{
					uint8_t cons = decod_asc8(message_in+14);
					uint32_t dur = decod_asc32(message_in+6);
					LOG_INFO("CHEF:lg:%i cons:%i duree:%i", longueur_m, cons, dur);
					if ((cons>50) && (cons<230) && (dur<(1<<23)))
					{
						RTC_TimeTypeDef sTime;
						RTC_DateTypeDef sDate;
						HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
						uint32_t date_actuel = ((sDate.Year-20)*372 + sDate.Month*31 + sDate.Date)*144  + sTime.Hours*6 + sTime.Minutes/10;
						LOG_INFO("actuel:%i", date_actuel);
						forcage_duree = dur + date_actuel;
						forcage_consigne = cons;
						consigne_regulation = cons;
						// Arret : bit 31  forcage_duree: 23 bits
						EEPROM_Write32(1, (ch_arret<<31) | (forcage_duree<<8) | (forcage_consigne));
					}
				}
				if ((message_in[5] == 'A') && (longueur_m == 7))   // CHEAx  Arret chauffage
				{
					if (message_in[6]=='0' || message_in[6]=='1')
					{
						ch_arret = message_in[6] - '0';
						// Arret : bit 31  forcage_duree: 23 bits
						EEPROM_Write32(1, (ch_arret<<31) | (forcage_duree<<8) | (forcage_consigne));
						ch_circulateur = 1 - ch_arret;
						if (ch_circulateur)  // marche
							HAL_GPIO_WritePin(CIRCULATEUR_GPIO, CIRCULATEUR_PIN, GPIO_PIN_SET);
						else  // marche
							HAL_GPIO_WritePin(CIRCULATEUR_GPIO, CIRCULATEUR_PIN, GPIO_PIN_RESET);
					}
				}
				if ((message_in[5] == 'P') && (longueur_m == 16))   // CHEPxddfftccaa  Planning chauffage
				{  //
					uint8_t num = message_in[6] -'0';
					uint8_t debut = decod_asc8(message_in+7);
					uint8_t fin = decod_asc8(message_in+9);
					uint8_t type = message_in[11]-'0';
					uint8_t cons = decod_asc8(message_in+12);
					uint8_t cons_ap = decod_asc8(message_in+14);

					LOG_INFO("CHEP:lg:%i num:%i deb:%i fin:%i t:%i cons:%i cons_ap:%i", longueur_m, num, debut, fin, type, cons, cons_ap);

					if ((num<NB_MAX_PGM) && (debut<145) && (fin<145) && (type<3) && (cons>=50) \
							&& (cons<=230) && (cons_ap>=6) && (cons_ap<=64))
					{
						ch_debut[num] = debut;
						ch_fin[num] = fin;
						ch_type[num] = type;
						ch_consigne[num] = cons;
						ch_cons_apres[num] = cons_ap;

						// 26-31:cons_ap 24-25:type 16-23:cons 8-15:fin 0-7:debut
						EEPROM_Write32(2+num, (cons_ap<<26) | (type<<24) | (cons<<16) | (fin<<8) | debut);
					}
				}
			}
		}
		#endif

		// ******************************** HHHHHHHHHHHHHHHHHHH  ********************

		if ((message_in[2] == 'H'))
		{
		  if (message_in[3] == 'E')  // HEHhhmmss : Ecriture heure
		  {
				if (message_in[4] == 'H')  // HEHhhmmss : Ecriture heure
				{
					set_rtc_time_from_string((const char*)message_in+5);
				}
				if (message_in[4] == 'D')  // HEDjjmmaa : Ecriture date
				{
					set_rtc_date_from_string((const char*)message_in+5);
				}
				if (message_in[4] == 'S')  // HESxxxx : Ecriture date-heure avec timestamp 32bits
					// 0xB1 6 72 69 83 104 229 1 0
				{
					uint32_t timestamp;
					memcpy(&timestamp, message_in + 5, 4);
					LOG_INFO("HES:Timestamp: %08X", timestamp);
					set_rtc_from_timestamp(timestamp);
				}
		  }
		  if (message_in[3] == 'L')
		  {
			  if (message_in[4] == 'H')  // HLH : Lecture Heure
			  {
				display_current_time();
			  }
			  if (message_in[4] == 'S')  // HLS : Lecture Heure en binaire
			  {
				    uint32_t time = get_rtc_timestamp();
					message.dest = message_in[1];
					message.type = 1;  // binaire
					message.data[0] = message.dest | 0x80;
					uint8_t i=2;
					message.data[i++] = 'H';
					message.data[i++] = 'E';
					message.data[i++] = 'S';
					memcpy (&message.data[i], &time, 4);
					i += 4;
					message.data[1] = i-3;
					//envoie_mess_bin(&message);

	                char hex_str[40];  // 2 chars par octet + 1 pour \0, ajustez selon tx.len
	                char *p = hex_str;
	                for (uint8_t j = 0; j < i; j++) {
	                    p += sprintf(p, "%02X ", message.data[j]);  // Espace entre chaque octet
	                }
	                LOG_INFO("HES:time:%i lg:%i %s", time, i, hex_str);

			  }
		  }
		}

		// ******************************** LLLLLLLLLLLLLLLLLLLL  ********************

		if ((message_in[2] == 'L') && (message_in[3] == 'O'))  // log_flash
		{
			if ( (message_in[4] =='R')  && (message_in[5] =='a') &&  (message_in[6] =='z')
				  && (longueur_m==7))  // Force effacement page Log 1LORaz
			{
			  LOG_Format();
			}
			if ( (message_in[4] =='S')   && (longueur_m==5))  // Log:stats 1LOS
			{
			  uint32_t total_entries, free_space;
			  log_get_stats(&total_entries, &free_space);
			}
			if ( (message_in[4] =='E')  && (message_in[5] =='E') &&  (message_in[6] =='Z')
				  && (longueur_m==7))                   // Force effacement page eeprom 1LOEEZ
			{
			  EEPROM_Format();
			}
			if ( (message_in[4] =='E') && (message_in[5] =='E') &&(longueur_m==6))  // EEprom:stats 1LOEE
			{
			  uint32_t total_entries, free_space;
			  EEPROM_GetStats(&total_entries, &free_space);
			}
		}

		// ******************************** OOOOOOOOOOOOOOOOOO  ********************

		if ((message_in[2] == 'O'))            // OK
		{
			  //LOG_INFO("AAA%i", longueur_m);
			  if ((message_in[3] == 'K') && (message_in[4] == 'K')) //OKK....
				  LOG_INFO("recu:%s", message_in);
		}

		// ******************************** RRRRRRRRRRRRRRRRRRR  ********************

          if ((message_in[2] == 'R') && (message_in[3] == 'L'))   // RL : Lecture Radio
          {
              if ((message_in[4] == 'S') && (message_in[5] == 'I'))   // RLSI : Lecture Radio RSSI (recu du concent)
              {
            	  // envoi à l'uart du end_node
        		  LOG_INFO("RSSI:%s node:%i", message_in, nodes[0].latestRssi);

              }
          }

          // ******************************** SSSSSSSSSSSSSSSSSS  ********************

          if ((message_in[2] == 'S') && (message_in[3] == 'L'))   // SL : Lecture Statut
          {
              if ( (message_in[4] =='O')  && (longueur_m==5))  // 1SLO
              {
            	  uint8_t node_id = Node_id(message_in[1]);
            	  if (node_id) node_id--;
            	  int8_t rssi = nodes[node_id].latestRssi;
                  envoie_mess_ASC(param_def, "%cRLSI %i", message_in[1], rssi);
              }
              if ( (message_in[4] =='V')  && (longueur_m==5))  // 1SLV : version-type
              {
                  envoie_mess_ASC(param_def, "%cVer:%s Type:%c", message.data[0], CODE_VERSION, CODE_TYPE);
              }
              if ( (message_in[4] =='N') && (message_in[5] =='L') && (longueur_m==6))  // SLNL  Nodes Liste
  				      lecture_Nodes();
              if ( (message_in[4] =='N') && (message_in[5] =='I') && (longueur_m==8))  // SLNIxy  Info MEssages Node x
  				      info_Node(message_in[6]-'0', message_in[7]-'0');
              if ( (message_in[4] =='N') && (message_in[5] =='S') && (longueur_m==7))  // SLNSx  suppression Node x (lettre)
  				      suppression_node(message_in[6]);

              if ( (message_in[4] =='T') && (message_in[5] =='a') && (longueur_m==6))  // SLTa  Stack des taches
  				 check_stack_usage();
              // free : defaut:358 Appli:358 Lora_TX:358 LORA_RX:358 UartTX:163 UartRx:471

              if ( (message_in[4] =='W') && (message_in[5] =='a') && (longueur_m==6))  // SLWa  Watchdog etat
            	  watchdog_print_status();

              if ( (message_in[4] =='R') && (message_in[5] =='e') && (longueur_m==6))  // SLRe  Reset cause et diagnostic
            	  display_reset_cause();

              if ( (message_in[4] =='R') && (message_in[5] =='E') && (longueur_m==6))  // SLRE  Radio Etat
              {
          		LOG_INFO("Radio : Etat:%d Sleep:%d", hsubghz.State, hsubghz.DeepSleep);
              }
              if ( (message_in[4] =='R') && (message_in[5] =='X') && (longueur_m==8))  // SLRXxy Radio TX paramx = y
              {
            	  SetRadioTxParam(message_in[6]-'0', message_in[7]-'0');
              }
              if ( (message_in[4] =='R') && (message_in[5] =='L') && (longueur_m==6))  // SLRL  Radio Lecture etat
              {
            	  PrintRadioTxParam();
              }
              if ( (message_in[4] =='R') && (message_in[5] =='T') && (longueur_m==6))  // SLRT  Etat Radio transmission
              {
                  LOG_INFO("etat LORA Tx: ok:%i renvoi:%i supp:%i timeout:%i busy:%i trop long:%i", \
                		  lora_etat.mess_envoy_ok, lora_etat.mess_renvoyes, lora_etat.mess_envoy_supp, lora_etat.lora_tx_timeout, \
						  lora_etat.channel_busy, lora_etat.tx_trop_long);
              }
              if ( (message_in[4] =='R') && (message_in[5] =='R') && (longueur_m==6))  // SLRR  Etat Radio reception
              {
                  LOG_INFO("etat LORA Rx: ok:%i radio:%i error:%i", \
                		  lora_etat.mess_recu_ok, lora_etat.radio_rx, lora_etat.lora_rx_error);
              }
              if ( (message_in[4] =='R') && (message_in[5] =='N') && (longueur_m==6))  // SLRN  Etat Nodes
              {
            	  for (uint8_t i=0; i<nb_nodes; i++)
            	  {
                  LOG_INFO("Node %i: val:%i class:%i recu:%i envoi:%i, err:%i rssi:%i add:%c", \
                		  i, nodes[i].valid, nodes[i].class, nodes[i].nb_recus, \
						  nodes[i].nb_envoyes, nodes[i].nb_err, nodes[i].latestRssi, nodes[i].adresse);
            	  }
              }
          }

          // ********************************   TTTTTTTTTTTTTTTTTT  ********************

          if ((message_in[2] == 'T') && (message_in[3] == 'L'))  //  TL  test
          {
              if ((message_in[4] == '0') && (longueur_m==5))  // TL0 renvoie OK
              {
				  //param_def = 0x10; // 10:pas d'ack, pas de rx apres
				  LOG_INFO("recep TL0");
                  envoie_mess_ASC(param_def, "%cOK", message_in[1]);
              }
              if ((message_in[3] == 'T') && (longueur_m==4))  // 1TT => print test_tab
              {
            	  LOG_INFO("index:%i  val2:%i", test_index, test_var);
            	  for (uint8_t i=0; i<test_index; i++)
            	  {
            		  LOG_INFO("ind:%i val:%u", i, test_tab[i]);
            	  }
              }
              if ((message_in[3] == 'R') && (longueur_m==5))  // 1TR1 => envoi test radio
              {
            	  test_var=1;
				// 1. Démarrer CAD
				Radio.StartCad();

				// 2. Attendre le résultat dans OnCadDone
				// 3. Si canal libre, transmettre
              }
          }

          if ((message_in[2] == 'T') && (message_in[3] == 'T'))  // Tests
          {
			  if ( (message_in[4] =='T')  && (longueur_m==6))  // 1TTTx : mess recu
			  {
				  param_def = 0x10; // 10:pas d'ack, pas de rx apres
				  LOG_INFO("envoi UTEQ");
                  envoie_mess_ASC(param_def, "UTEQ");
			  }
          }
          if ((message_in[2] == 'T') && (message_in[3] == 'E'))  // Tests
          {
		      if ( (message_in[4] =='0')  && (longueur_m==7))  // LEcture Log 1TE001
		      {
			     uint16_t logs_read = log_read(message_in[5]-'0', message_in[6]-'0', '1', 0);
			     LOG_INFO("Logs lus: %i", logs_read);
		      }
			  if ( (message_in[4] =='A')  && (longueur_m==7))  // 1TEAxx : envoi par lora vers node
			  {
				  param_def = (message_in[5]-'0')*16 + message_in[6]-'0';
                  envoie_mess_ASC(param_def, "UTL1");
			  }
			  if ( (message_in[4] =='B')  && (longueur_m==7))  // 1TEBxx : envoi 2 mess par lora
			  {
				  param_def = (message_in[5]-'0')*16 + message_in[6]-'0';
                  envoie_mess_ASC(param_def, "UOKK");
                  envoie_mess_ASC(param_def, "UBBB");
			  }
			  if ( (message_in[4] =='C')  && (longueur_m==7))  // 1TECxx : envoi 1 mess par lora
			  {
				  param_def = (message_in[5]-'0')*16 + message_in[6]-'0';
                  envoie_mess_ASC(param_def, "UTES");
			  }
			  if ( (message_in[4] =='D')  && (longueur_m==9))  // 1TEDxzyy : envoi 1 mess par lora
			  {
				  uint8_t dest = message_in[5];
				  uint8_t nb = message_in[6]-'0';
				  param_def = (message_in[7]-'0')*16 + message_in[8]-'0';
				  for (uint8_t i=0; i<nb; i++)
				  {
					  envoie_mess_ASC(param_def, "%cTES%i", dest, i);
				  }
			  }
			  if ( (message_in[4] =='E')  && (longueur_m==7))  // Ecriture eeprom  1TEE12
			  {
			     if (EEPROM_Write8(message_in[5]-'0', message_in[6]-'0') == HAL_OK) {
				    LOG_INFO("EEPROM écrit");
			     } else {
				    LOG_ERROR("Erreur écriture EEPROM ");
			     }
			  }
			  if ( (message_in[4] =='L')  && (longueur_m==7))  // Ecriture LOG  1TEL12
			  {
				char short_message[8];
				strncpy(short_message, (char*)message_in, 7);
				short_message[7] = '\0';
			    if (log_write('5', 0x01, 0x02, 0x03, short_message) == 0) {
				  LOG_INFO("LOG écrit");
			    }   else {
				  LOG_ERROR("Erreur écriture LOG ");
			    }
		      }
			  if ( (message_in[4] =='Q')  && (longueur_m==5))  // TEQ : envoi vers port serie
			  {
                  envoie_mess_ASC(param_def, "1AAA");
			  }
			  if ( (message_in[4] =='R')  && (longueur_m==6))  // TERx Lecture eeprom
			  {
				uint8_t value;
				if (EEPROM_Read8(message_in[5]-'0', &value) == HAL_OK) {
					LOG_INFO("EEPROM lu: adresse %d = 0x%02X", message_in[5]-'0', value);
				} else {
					LOG_ERROR("Erreur lecture EEPROM ");
				}
			  }
			  if ( (message_in[4] =='S')  && (longueur_m==5))  // 1TES : envoi par lora vers concen
			  {
                  envoie_mess_ASC(param_def, "HTL1");
			  }
			  if ( (message_in[4] =='T')  && (longueur_m==7))  // 1TETxy : param_def xy
			  {
				  param_def = (message_in[5]-'0')*16 + message_in[6]-'0';
				 LOG_INFO("param : %02X", param_def);
			  }
			  if ( (message_in[4] =='U')  && (longueur_m==5))  // 1TEU : envoi 2 mess par lora
			  {
                  envoie_mess_ASC(param_def, "AOKK");
                  envoie_mess_ASC(param_def, "ABBB");
			  }
			  if ( (message_in[4] =='U')  && (longueur_m==7))  // 1TEU :
			  {
				  uint8_t ret = mess_LORA_suppression_milieu(message_in[5]-'0', message_in[6]-'0');
				 LOG_INFO("supp %i", ret);
			  }
			  if ( (message_in[4] =='Z')  && (longueur_m==7))  // TEZxx : valeur test_var
			  {
				  test_var = decod_asc8(message_in+5);
				  LOG_INFO("val:%i", test_var);
			  }

          }
          if ((message_in[2] == 'T') && (message_in[3] == 'L'))  // Tests
          {
		      if ( (message_in[4] =='1'))  // LEcture TL1
		      {
			     LOG_INFO("Mess recu lora: %s lg:%i", message_in, longueur_m);
		      }
          }

      }
  }
}

/**
 * @brief Vérifie la configuration flash
 */
void check_flash_config(void)
{
    LOG_INFO("=== CONFIGURATION FLASH ===");
    LOG_INFO("FLASH_BASE: 0x%08lX", FLASH_BASE);
    LOG_INFO("FLASH_SIZE: %d Ko", FLASH_SIZE);
    LOG_INFO("FLASH_PAGE_SIZE: %d octets", FLASH_PAGE_SIZE);
    
    // Vérifier que les adresses EEPROM sont dans la plage flash
    if (EEPROM_PAGE_0_ADDR >= FLASH_BASE && EEPROM_PAGE_0_ADDR < (FLASH_BASE + FLASH_SIZE * 1024)) {
        LOG_INFO("Adresses EEPROM dans la plage flash: OK");
    } else {
        LOG_ERROR("Adresses EEPROM hors plage flash!");
    }
    
    // Vérifier que les pages ne sont pas dans la zone du programme
    if (EEPROM_PAGE_0_ADDR < (FLASH_BASE + 50 * 1024)) {  // 50KB de programme
        LOG_ERROR("Pages EEPROM dans la zone du programme!");
    } else {
        LOG_INFO("Pages EEPROM hors zone programme: OK");
    }
    
    LOG_INFO("EEPROM_PAGE_0_ADDR: 0x%08lX", EEPROM_PAGE_0_ADDR);
}

/**
 * @brief Vérifie les permissions flash
 */
void check_flash_permissions(void)
{
    LOG_INFO("=== PERMISSIONS FLASH ===");
    
    // Vérifier les erreurs ECC
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCR_ERRORS)) {
        LOG_ERROR("Erreurs ECC detectees!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCR_ERRORS);
    }
    
    // Vérifier les erreurs d'opération
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR)) {
        LOG_ERROR("Erreur d'operation flash!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    }
    
    // Vérifier les erreurs de protection
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR)) {
        LOG_ERROR("Erreur de protection d'ecriture!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    }
    
    // Vérifier les erreurs d'alignement
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR)) {
        LOG_ERROR("Erreur d'alignement!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    }
    
    LOG_INFO("Flash accessible: OK");
}

void debug_uart_config(void)
{
    LOG_INFO("=== UART CONFIG DEBUG ===");

    // Vérifier la configuration UART
    LOG_INFO("UART BaudRate: %lu", hlpuart1.Init.BaudRate);
    LOG_INFO("UART WordLength: %d", hlpuart1.Init.WordLength);
    LOG_INFO("UART StopBits: %d", hlpuart1.Init.StopBits);
    LOG_INFO("UART Parity: %d", hlpuart1.Init.Parity);
    LOG_INFO("UART Mode: %d", hlpuart1.Init.Mode);

    // Vérifier les registres UART
    LOG_INFO("UART CR1: 0x%08lX", hlpuart1.Instance->CR1);
    LOG_INFO("UART CR2: 0x%08lX", hlpuart1.Instance->CR2);
    LOG_INFO("UART CR3: 0x%08lX", hlpuart1.Instance->CR3);
}

void debug_uart_registers(void)
{
    LOG_INFO("=== UART REGISTERS DEBUG ===");

    // Registres de contrôle
    LOG_INFO("CR1: 0x%08lX", hlpuart1.Instance->CR1);
    LOG_INFO("CR2: 0x%08lX", hlpuart1.Instance->CR2);
    LOG_INFO("CR3: 0x%08lX", hlpuart1.Instance->CR3);

    // Registres de statut (STM32WL)
    LOG_INFO("ISR: 0x%08lX", hlpuart1.Instance->ISR);
    LOG_INFO("RDR: 0x%02X", (uint8_t)hlpuart1.Instance->RDR);
    LOG_INFO("TDR: 0x%02X", (uint8_t)hlpuart1.Instance->TDR);

    // Vérifier les bits d'interruption (STM32WL)
    if (hlpuart1.Instance->CR1 & USART_CR1_RXFFIE) {
        LOG_INFO("RXFF FIFO interrupt enabled");
    } else {
        LOG_ERROR("RXFF FIFO interrupt NOT enabled");
    }

    if (hlpuart1.Instance->CR1 & USART_CR1_TXFEIE) {
        LOG_INFO("TXFE FIFO interrupt enabled");
    } else {
        LOG_INFO("TXFE FIFO interrupt disabled");
    }

    // Vérifier les flags de statut (STM32WL)
    if (hlpuart1.Instance->ISR & USART_ISR_RXNE_RXFNE) {  // ✅ Correction
        LOG_INFO("RXNE_RXFNE flag set - data available");
    } else {
        LOG_INFO("RXNE_RXFNE flag clear - no data");
    }

    if (hlpuart1.Instance->ISR & USART_ISR_TXE_TXFNF) {  // ✅ Correction
        LOG_INFO("TXE_TXFNF flag set - ready to transmit");
    } else {
        LOG_INFO("TXE_TXFNF flag clear - not ready");
    }

    // Vérifier les erreurs
    if (hlpuart1.Instance->ISR & USART_ISR_ORE) {
        LOG_ERROR("Overrun error flag set");
    }

    if (hlpuart1.Instance->ISR & USART_ISR_FE) {
        LOG_ERROR("Framing error flag set");
    }

    if (hlpuart1.Instance->ISR & USART_ISR_NE) {
        LOG_ERROR("Noise error flag set");
    }

    if (hlpuart1.Instance->ISR & USART_ISR_PE) {
        LOG_ERROR("Parity error flag set");
    }
}

void check_global_interrupts(void)
{
    LOG_INFO("=== GLOBAL INTERRUPTS CHECK ===");

    // 1. Vérifier PRIMASK (interruptions principales)
    if (__get_PRIMASK() & 1) {
        LOG_ERROR("PRIMASK: INTERRUPTS DISABLED ❌");
    } else {
        LOG_INFO("PRIMASK: INTERRUPTS ENABLED ✅");
    }

    // 2. Vérifier FAULTMASK (interruptions de faute)
    if (__get_FAULTMASK() & 1) {
        LOG_ERROR("FAULTMASK: FAULT INTERRUPTS DISABLED ❌");
    } else {
        LOG_INFO("FAULTMASK: FAULT INTERRUPTS ENABLED ✅");
    }

    // 3. Vérifier BASEPRI (priorité de base)
    uint32_t basepri = __get_BASEPRI();
    LOG_INFO("BASEPRI: 0x%08lX", basepri);

    // 4. Vérifier CONTROL (mode du processeur)
    uint32_t control = __get_CONTROL();
    LOG_INFO("CONTROL: 0x%08lX", control);
}

void debug_uart_interrupt_init(void)
{
    LOG_INFO("=== UART INTERRUPT DEBUG ===");

    // 1. Vérifier que l'interruption est activée
    if (__HAL_UART_GET_IT_SOURCE(&hlpuart1, UART_IT_RXNE)) {
        LOG_INFO("UART RX interrupt enabled");
    } else {
        LOG_ERROR("UART RX interrupt NOT enabled");
    }

    // 2. Vérifier l'état de l'UART
    LOG_INFO("UART state: %d", hlpuart1.gState);
    LOG_INFO("UART error: 0x%08lX", hlpuart1.ErrorCode);

    // 3. Vérifier la queue
    if (uart_rx_queue != NULL) {
        LOG_INFO("UART RX queue created");
    } else {
        LOG_ERROR("UART RX queue NOT created");
    }

    // 4. Vérifier la priorité d'interruption - VERSION SIMPLE
    LOG_INFO("USART2 IRQ priority: %lu", NVIC_GetPriority(LPUART1_IRQn));

    // 5. Vérifier si l'interruption est activée
    LOG_INFO("USART2 IRQ enabled: %s", NVIC_GetEnableIRQ(LPUART1_IRQn) ? "YES" : "NO");
}

void test_uart_reception(void)
{
    LOG_INFO("=== UART RECEPTION TEST ===");

    // Vérifier l'état de l'UART
    LOG_INFO("UART state before test: %d", hlpuart1.gState);

    // Démarrer la réception
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
    LOG_INFO("HAL_UART_Receive_IT status: %d", status);

    if (status == HAL_OK) {
        LOG_INFO("UART RX started successfully");
    } else {
        LOG_ERROR("Failed to start UART RX: %d", status);
    }

    for(;;)
    {
		// Vérifier l'état après
		LOG_INFO("UART state after start: %d", hlpuart1.gState);

		// Vérifier les flags
		if (hlpuart1.Instance->ISR & USART_ISR_RXNE_RXFNE) {
			LOG_INFO("RXNE_RXFNE flag set - data available");
		} else {
			LOG_INFO("RXNE_RXFNE flag clear - no data");
		}
       osDelay(5000);
    }
}


void check_memory(void)
{
    LOG_INFO("=== MEMORY CHECK ===");

    // Vérifier la heap libre
    size_t free_heap = xPortGetFreeHeapSize();
    LOG_INFO("Free heap: %d bytes", free_heap);

    if (free_heap < 1000) {
        LOG_ERROR("Low heap memory: %d bytes", free_heap);
    }

    // Vérifier la pile minimale
    UBaseType_t min_stack = uxTaskGetStackHighWaterMark(NULL);
    LOG_INFO("Stack high water mark: %d", min_stack);

    if (min_stack < 100) {
        LOG_ERROR("Low stack memory: %d", min_stack);
    }
}

void debug_uart_complete(void)
{
    LOG_INFO("=== COMPLETE UART DEBUG ===");

    // 1. Configuration
    debug_uart_config();

    check_global_interrupts();

    // 2. Initialisation
    debug_uart_interrupt_init();

    // 3. Registres
    debug_uart_registers();

    // 4. Test de réception
    //test_uart_reception();

    // 5. Test manuel
    //test_uart_interrupt_manual();

    LOG_INFO("=== END UART DEBUG ===");
}

// Fonction de diagnostic - VERSION CORRIGÉE
void diagnose_uart_wakeup(void)
{
    LOG_INFO("=== UART WAKEUP DIAGNOSIS ===");

    // 1. Vérifier si l'interruption LPUART1 est activée
    if (NVIC_GetEnableIRQ(LPUART1_IRQn)) {
        LOG_INFO("LPUART1 IRQ: ENABLED");
    } else {
        LOG_ERROR("LPUART1 IRQ: DISABLED - CANNOT WAKE UP!");
    }

    // 2. Vérifier la priorité
    uint32_t priority = NVIC_GetPriority(LPUART1_IRQn);
    LOG_INFO("LPUART1 Priority: %lu", priority);

    // 3. Vérifier l'état de l'UART - VERSION CORRIGÉE
    LOG_INFO("LPUART1 gState: %d", hlpuart1.gState);
    LOG_INFO("LPUART1 RxState: %d", hlpuart1.RxState);

    // 4. Vérifier l'erreur
    LOG_INFO("LPUART1 ErrorCode: %lu", hlpuart1.ErrorCode);

    LOG_INFO("=============================");
}
