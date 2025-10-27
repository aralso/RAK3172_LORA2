/*
 * Appli.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tocqueville
 */

#ifndef INC_APPLI_H_
#define INC_APPLI_H_

#include "stm32wlxx_hal.h"



#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOA
#define bouton_Pin GPIO_PIN_12
#define bouton_GPIO_Port GPIOA

#define CODE_VERSION  "1.8"
#define CODE_TYPE 'A'  // A:End_node Radar
#define CLASS 'B'  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps

extern uint8_t test_val;
extern RTC_HandleTypeDef hrtc;


#endif /* INC_APPLI_H_ */
