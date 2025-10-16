/*
 * Lora.c
 *
 *  Created on: Oct 12, 2025
 *      Author: Tocqueville
 */

#include <lora.h>
#include "radio.h"                    // API radio
#include "radio_def.h"                // Définitions radio
#include "radio_ex.h"                 // Extensions radio
#include "radio_conf.h"               // Configuration radio


void configure_radio_parameters(void)
{
    // 1. Vérifier que la fréquence est supportée
    if (!Radio.CheckRfFrequency(868100000UL)) {
        LOG_ERROR("Fréquence 868.1 MHz non supportée");
        return;
    }

    // 2. Configurer le modem LoRa
    Radio.SetModem(MODEM_LORA);

    // 3. Définir la fréquence (✅ CORRECTION ICI)
    Radio.SetChannel(868100000UL);  // 868.1 MHz

    // 4. Configuration réception LoRa
    Radio.SetRxConfig(
        MODEM_LORA,           // Modem LoRa
        0,                    // Bandwidth: 125 kHz
        7,                    // Datarate: 7:128
        1,                    // Coderate: 4/5
        0,                    // AFC bandwidth (N/A pour LoRa)
        8,                    // Preamble length
        0,                    // Symbol timeout
        false,                // Variable length
        0,                    // Payload length (variable)
        true,                 // CRC enabled
        false,                // Frequency hopping OFF
        0,                    // Hop period
        false,                // IQ not inverted
        true                  // Continuous RX
    );


    // 5. Configuration transmission LoRa
    Radio.SetTxConfig(
        MODEM_LORA,           // Modem LoRa
        14,                   // Power: 14 dBm
        0,                    // Frequency deviation (N/A pour LoRa)
        0,                    // Bandwidth (N/A pour LoRa)
        7,                    // Datarate: 128 chips
        1,                    // Coderate: 4/5
        8,                    // Preamble length
        false,                // Variable length
        true,                 // CRC enabled
        false,                // Frequency hopping 0:OFF
        0,                    // Hop period
        false,                // IQ not inverted
        3000                  // Timeout: 3 secondes
    );

    LOG_INFO("Radio LoRa configurée: 868.1 MHz, SF7, 14 dBm");
}

// Fonction pour mettre le radio en veille
/*void subghz_enter_sleep_mode(void)
{
    // ⭐ METTRE LE RADIO EN MODE SLEEP
    uint8_t sleep_cmd = RADIO_SET_SLEEP;  // 0x84
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, sleep_cmd, NULL, 0);

    //LOG_INFO("SUBGHZ: Entered sleep mode");
}

// Fonction pour réveiller le radio
void subghz_wake_up(void)
{
    // ⭐ RÉVEILLER LE RADIO
    uint8_t wakeup_cmd = RADIO_SET_STANDBY;  // 0x80
    //HAL_SUBGHZ_ExecSetCmd(&hsubghz, wakeup_cmd, NULL, 0);

    //LOG_INFO("SUBGHZ: Woke up from sleep");
}*/
