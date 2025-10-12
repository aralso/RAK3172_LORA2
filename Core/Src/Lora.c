/*
 * Lora.c
 *
 *  Created on: Oct 12, 2025
 *      Author: Tocqueville
 */

#include <lora.h>

// Fonction pour mettre le radio en veille
void subghz_enter_sleep_mode(void)
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
}
