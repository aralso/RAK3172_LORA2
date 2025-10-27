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

#include "stm32wlxx_hal_subghz.h"  // ← AJOUTER CETTE LIGNE

#define ReseauAddr		0x23

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

#endif /* INC_LORA_H_ */
