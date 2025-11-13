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

#define CODE_VERSION  "1.9"
#define CODE_TYPE 'A'  // A:End_node Radar

//#define END_NODE   // sinon #define CONCENTRATOR

#ifdef END_NODE
	#define My_Address 'U'
	#define CLASS LORA_CLASS_A  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps
#else
	#define My_Address 'H'
	#define CLASS LORA_CLASS_C
#endif


extern uint8_t test_val;
extern RTC_HandleTypeDef hrtc;
extern uint8_t mess_pay[100];


#endif /* INC_APPLI_H_ */
