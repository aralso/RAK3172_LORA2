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

void subghz_enter_sleep_mode(void);
void subghz_wake_up(void);
void configure_radio_parameters(void);

#endif /* INC_LORA_H_ */
