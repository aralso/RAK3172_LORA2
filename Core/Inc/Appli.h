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
#define ID_CONCENTRATOR	'H'

#define CODE_VERSION  "1.9"
#define CODE_TYPE 'C'  // A:End_node Radar  B-C:régul chaudiere garches(B:Thermo C:moteur)

#define END_NODE   // sinon #define CONCENTRATOR

#ifdef END_NODE
	#if CODE_TYPE == 'A'
		#define My_Address 'U'
		#define CLASS LORA_CLASS_C  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps
		//#define mode_sleep
	#elif CODE_TYPE == 'B' // Garches chaudiere Thermometre
		#define My_Address 'I'
		#define CLASS LORA_CLASS_A  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps
		#define MAX_SENS 20
		#define TEMP_PERIOD	30  // 30 secondes
		#define mode_sleep

	#elif CODE_TYPE == 'C' // Garches chaudiere Moteur
		#define My_Address 'J'
		#define CLASS LORA_CLASS_C  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps
	#endif
#else
	#define My_Address ID_CONCENTRATOR
	#define CLASS LORA_CLASS_C
#endif



#if CODE_TYPE == 'C'   // Moteur chaudière
	#define NB_MAX_PGM 3
	extern uint8_t ch_debut[];   // debut de chauffe :heure par pas de 10 minutes
	extern uint8_t ch_fin[];   // fin de chauffe
	extern uint8_t ch_type[];     // 0:tous les jours, 1:semaine, 2:week-end (2 bits)
	extern uint8_t ch_consigne[];  // 5° à 23°C, par pas de 0,1°C
	extern uint8_t ch_cons_apres[];  // 3° à 23°C, par pas de 0,5°C (6 bits)
	extern uint32_t forcage_duree;    // par pas de 10 min 1/1/2020=0 (sur 23bits)
	extern uint8_t forcage_consigne;  // 0 à 23°C
	extern uint8_t consigne_actuelle;
	extern uint8_t consigne_apres;
	extern uint8_t ch_arret; // 1 bit
	extern uint16_t Tint;
	extern 	float pos_prec;


	#define FULL_TRAVEL_TIME   140      // temps total 0→100% en secondes
	#define WINDOW_TIME        60       // une fenêtre de 60 secondes
	#define DEADBAND_PERCENT   1        // zone morte (1 %)

	#define VALVE_OPEN_GPIO   GPIOA
	#define VALVE_OPEN_PIN    GPIO_PIN_6   // A6:ouverture vanne
	#define VALVE_CLOSE_GPIO  GPIOA
	#define VALVE_CLOSE_PIN   GPIO_PIN_7   // A7:fermeture vanne
	#define CIRCULATEUR_GPIO  GPIOA
	#define CIRCULATEUR_PIN   GPIO_PIN_13  // A13 : marche circulateur

#endif

extern uint8_t test_val;
extern RTC_HandleTypeDef hrtc;
extern uint8_t mess_pay[100];
extern 	uint16_t temp_period;


#endif /* INC_APPLI_H_ */
