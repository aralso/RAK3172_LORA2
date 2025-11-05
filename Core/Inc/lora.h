/*
 * lora.h
 *
 *  Created on: Oct 12, 2025
 *      Author: Tocqueville
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include <main.h>
#include <communication.h>
#include <fonctions.h>
#include <eeprom_emul.h>
#include <log_flash.h>
#include "cmsis_os.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "radio.h"

#include "stm32wlxx_hal_subghz.h"  // ← AJOUTER CETTE LIGNE
#include "fonctions.h"  // pour lptim_program_compare_advance_ms

#define ReseauAddr		0x23

// Classes LoRaWAN simplifiées
#define LORA_CLASS_A                0
#define LORA_CLASS_B                1
#define LORA_CLASS_C                2

#define LORA_BROADCAST_ADDR         0x7D


// Machine d’états non bloquante pour la phase TX/ACK/RX
typedef enum {
    TX_IDLE = 0,
    TX_WAIT_CAD,
    TX_SENDING,
	TX_SENT,
	TX_ACK_RECU,
    TX_WAIT_ACK,
    RX_RESPONSES
} lora_tx_state_t;

// Machine d’états non bloquante pour la phase RX
typedef enum {
    RX_IDLE = 0,
	RX_ATTENTE,
    RX_MESS_RECU,
    RX_WAIT_ACK_SENT
} lora_rx_state_t;


typedef struct lora_etat_s
{
	uint8_t mess_envoy_ok;
	uint8_t mess_renvoyes;
	uint8_t mess_envoy_supp;
	uint8_t radio_rx;
	uint8_t mess_recu_ok;
	uint8_t lora_tx_timeout;
	uint8_t lora_rx_error;
	uint8_t channel_busy;
	uint8_t tx_trop_long;
} lora_etat_t;

typedef struct radio_TxParam_s
{
	uint32_t freq;
	uint8_t channel;
	uint8_t DR;
	RadioModems_t modem;
	int8_t power;
	uint32_t fdev;
	uint32_t bandwidth;
	uint32_t SF;
	uint8_t coderate;
	uint16_t preambleLen;
	bool fixLen;
	bool crcOn;
	bool freqHopOn;
	uint8_t hopPeriod;
	bool iqInverted;
	uint32_t timeout;
} radio_TxParam_t;

extern  lora_tx_state_t g_tx_state;
extern struct lora_etat_s lora_etat;
extern uint8_t att_cad;

// API de contrôle haute-niveau
void lora_radio_init(void);
void lora_set_class(uint8_t lora_class);
void lora_handle_event_tx(void);
void lora_handle_event_rx(void);
void lora_tx_state_step(void);
void lora_schedule_ack_timeout(uint32_t ms);
void lora_tx_on_cad_result(bool channelBusy);
void lora_handle_classb_beacon_event(void);
void relance_radio_rx(uint8_t actif);
void relance_rx(uint8_t actif);  // envoie vers l'evenement appli

// Callbacks Radio → LoRa layer
void lora_on_tx_done(void);
void lora_on_rx_done(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void lora_on_tx_timeout(void);
void lora_on_rx_timeout(void);
void lora_on_rx_error(void);
void lora_on_cad_done(bool channelActivityDetected);

typedef struct
{
        uint8_t destAddr;              //!< Dst Address of RX'ed packet
        uint8_t reseauAddr;				// Adresse du reseau
        int8_t rssi;                     //!< rssi of RX'ed packet
        uint8_t len;                     //!< length of RX'ed packet
        uint8_t param;			// bit0:dernier message reçu, bit4:ack non requis
        uint8_t payload[MESS_LG_MAX]; //!< payload of RX'ed packet
} lora_RxPacket;

typedef struct
{
        uint8_t destAddr;              //!<  Destination address
        uint8_t reseauAddr;				// Adresse du reseau
        uint8_t len;                     //!< Payload Length
        uint8_t param;					//bit0:dernier message transmis, bit4:ack non requis
        uint8_t payload[MESS_LG_MAX];       //!< Payload
} lora_TxPacket;

void subghz_enter_sleep_mode(void);
void subghz_wake_up(void);
void configure_radio_parameters(void);
void sendRadio();
void test_radio_progressive(void);
uint8_t SendFrameModif( uint8_t channel );
void check_radio_power_supply(void);
void test_radio_configuration_detailed(void);
void test_radio_wakeup_methods(void);
void test_radio_direct_transmission(void);
void check_radio_hardware_configuration(void);
void try_radio_wakeup_all_commands(void);
void diagnose_radio_deep_sleep(void);
void test_radio_write_register(void);
void SetRadioTxParam (uint8_t param, uint8_t val);
void PrintRadioTxParam(void);
uint8_t mess_LORA_enqueue(out_message_t* mess);
uint8_t mess_LORA_dequeue(out_message_t* mess);
uint8_t mess_LORA_dequeue_fictif(void);

// Comptage LPTIM1 et calculs balise
void lora_on_lptim1_10s_tick(void);
void lora_get_time_since_last_and_to_next(uint32_t* elapsed_ms_since_last,
                                          uint32_t* remaining_ms_to_next);

#endif /* INC_LORA_H_ */
