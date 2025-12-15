/*
 * functions.h
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_


#include "main.h"
#include <appli.h>
#include <stdio.h>
#include <stdarg.h>
#include "cmsis_os.h"
#include "timers.h"
#include "queue.h"


#define dest_erreur	'1'
#define dest_log '1'
#define UART_AJOUT_EMETTEUR


#define MESS_BUFFER_SIZE 500
#define MESS_LG_MAX	100
#define MESS_LG_MAX_LOG	60


// Niveaux de verbosité
#define LOG_LEVEL_ERROR    1
#define LOG_LEVEL_WARNING  2
#define LOG_LEVEL_INFO     3
#define LOG_LEVEL_DEBUG    4
#define LOG_LEVEL_VERBOSE  5


// Niveau de verbosité global (modifiable)
#define CURRENT_LOG_LEVEL  LOG_LEVEL_DEBUG
#define WRITE_LOG_LEVEL	 LOG_LEVEL_WARNING


// Code erreur

     // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
     // Erreurs 20 a 7F : 4 fois        (Appli >0x70)
     // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

#define code_erreur_envoi 0x50
#define code_erreur_dequeue 0x51

#define erreur_RX_full         0x20
#define erreur_rx_uart_bin		0x21
#define ISR_uart_full		   0x22
#define erreur_RX_queue        0x23
#define timeout_RX             0x24
#define erreur_queue_appli     0x25
#define ISR_fifo_full		   0x26
#define ISR_callback		   0x27
#define Timer_callback		   0x28
#define erreur_mess            0x2F
#define erreur_LORA_TX		   0x30
#define erreur_TO_LORA_TX	   0x31
#define erreur_nb_nodes_max	   0x32
#define depass_q_id 		   0x33
#define LORA_tx_bloque		   0x34


// Code erreur pour log_write :
#define log_w_err_uart_bloque	0x01
#define log_w_reseau_occupe		0x02
#define tx_lora_node_non_identifie	0x03
#define log_w_err_temp	0x04

#define UART_SEND(msg) do { \
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, sizeof(msg) - 1, 3000); \
    osDelay(100); \
} while(0)


#define NB_UART 1
#define car_fin_trame 13

typedef struct
{
  uint8_t           num_Uart;
  TimerHandle_t     h_timeout_RX;
  TimerHandle_t     h_timeout_TX;
} UartStruct;

typedef struct {
    uint8_t length;          // Longueur du message
    uint8_t type;            // Type : 0=ASCII, 1=Binaire
    uint8_t source;          // Source (UART1, UART2, etc.)
    int16_t rssi;
    int8_t snr;
    uint8_t param;
    uint8_t data[MESS_LG_MAX];        // Données du message
} in_message_t;

typedef struct {
    uint8_t length;          // Longueur du message (champs data)
    uint8_t dest;          // dest
    uint8_t type;            // Type : 0=ASCII, 1=Binaire
    uint8_t param; // bit0:dernier  bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6-7:classe
    uint8_t data[MESS_LG_MAX];        // Données du message
} out_message_t;


extern in_message_t message_recu;
extern UartStruct UartSt[NB_UART];
void reception_message_Uart2(in_message_t *msg);
uint8_t envoie_routage(out_message_t* message);
void verif_timout_uart_rx(void);
void traitement_rx (uint8_t*, uint8_t lg); // var :longueur n'inclut pas le car_fin_trame (inclus emetteur) : 4 pour RLT1


// Fonction principale de logging
void print_log(uint8_t level, const char* format, ...);
uint8_t init_communication(void);
void raz_Uart(uint8_t num_uart);  // raz car en reception (suite timeout)
void debug_uart_complete(void);

// Macros pour faciliter l'utilisation
#define LOG_ERROR(...)   print_log(LOG_LEVEL_ERROR,   __VA_ARGS__)
#define LOG_WARNING(...) print_log(LOG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_INFO(...)    print_log(LOG_LEVEL_INFO,    __VA_ARGS__)
#define LOG_DEBUG(...)   print_log(LOG_LEVEL_DEBUG,   __VA_ARGS__)
#define LOG_VERBOSE(...) print_log(LOG_LEVEL_VERBOSE, __VA_ARGS__)

extern uint8_t code_erreur, comptage_erreur;
extern uint8_t err_donnee1, err_donnee2;
extern uint8_t param_def;
extern out_message_t message;

extern osThreadId_t Uart_TX_TaskHandle;
extern UART_HandleTypeDef hlpuart1;
extern SUBGHZ_HandleTypeDef hsubghz;
extern HAL_StatusTypeDef send_lora_message(const char* message, uint8_t message_length, uint8_t dest);
extern QueueHandle_t in_message_queue;  // queue pour les messages entrants
extern uint16_t cpt_rx_uart;


// Fonction pour changer le niveau de verbosité à l'exécution
void set_log_level(uint8_t level);

// Fonction pour obtenir le niveau actuel
uint8_t get_log_level(void);
uint8_t envoie_mess_ASC(uint8_t param, const char* format, ...);
uint8_t envoie_mess_bin(out_message_t* mess);
uint8_t deci (uint8_t val);
void envoi_code_erreur(void);

// Fonctions de vérification flash
void check_flash_config(void);
void check_flash_permissions(void);
void check_memory(void);
void diagnose_uart_wakeup(void);

#endif /* INC_COMMUNICATION_H_ */
