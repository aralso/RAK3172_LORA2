/*
 * Appli.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tocqueville
 */

#ifndef INC_APPLI_H_
#define INC_APPLI_H_

#include "stm32wlxx_hal.h"

#define MODE_DEBUG  // permet de ne pas fermer la vanne 3 minutes à l'init

#ifdef MODE_DEBUG
	#define SANS_RADIO
	//#define Sans_Watchdog
	#define SANS_I2C
#endif


#define LED_Pin GPIO_PIN_11   // LED : A11
#define LED_GPIO_Port GPIOA
#define bouton_Pin GPIO_PIN_12 // bouton : A12
#define bouton_GPIO_Port GPIOA
#define ID_CONCENTRATOR	'H'

#define CODE_VERSION  "1.10"


#define END_NODE   // sinon  Define CONCENTRATOR

#define MAX_SENS 3

#ifdef END_NODE
	#define CODE_TYPE 'A'  // A:End_node Radar  B-C:régul chaudiere garches(B:Thermo C:moteur)
	#if CODE_TYPE == 'A'
		#define My_Address 'U'
		#define CLASS LORA_CLASS_A  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps
		#define mode_sleep
		#define KeepAlive 1   // Nb de jours entre chaque keepalive
	#elif CODE_TYPE == 'B' // Garches chaudiere Thermometre
		#define My_Address 'I'
		#define CLASS LORA_CLASS_A  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps
		#define TEMP_PERIOD	30  // 30 secondes
		#define mode_sleep
		#define KeepAlive 1

	#elif CODE_TYPE == 'C' // Garches chaudiere Moteur
		#define My_Address 'J'
		#define CLASS LORA_CLASS_C  // A:sleep,  B:ecoute chaque 30 seconde,  C:rx tout le temps
		#define KeepAlive 1
	#endif
#else
	#define My_Address ID_CONCENTRATOR
	#define CLASS LORA_CLASS_C
#endif

#ifndef KeepAlive
	#define KeepAlive  3
#endif

#if CODE_TYPE == 'C'   // Moteur chaudière
	#define NB_MAX_PGM 3
	extern uint8_t ch_debut[];   // debut de chauffe :heure par pas de 10 minutes
	extern uint8_t ch_fin[];   // fin de chauffe
	extern uint8_t ch_type[];     // 0:tous les jours, 1:semaine, 2:week-end (2 bits)
	extern uint8_t ch_consigne[];  // 5° à 23°C, par pas de 0,1°C
	extern uint8_t ch_cons_apres[];  // 3° à 23°C, par pas de 0,5°C (6 bits)
	extern uint32_t forcage_duree;    // par pas de 10 min 1/1/2020=0 (sur 23bits)
	extern uint8_t forcage_consigne;  // 0 à 23°C, par pas de 0,1°C
	extern uint8_t consigne_normale;
	extern uint8_t consigne_regulation;
	extern uint8_t consigne_apres;
	extern uint8_t ch_arret; // 1 bit
	extern uint16_t Tint;
	extern 	float pos_prec;
	extern uint8_t ch_circulateur;
	extern uint16_t heure_der_temp;
	extern uint16_t nb_mes_temp;  // 144 mesures de temp par jour
	extern uint16_t nb_mes_temp24;  // 144 mesures de temp par jour
	extern uint16_t puis_chaud;
	extern uint8_t puis_chaud24;
	extern uint8_t batt_thermo_av;
	extern uint8_t batt_thermo_ap;


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
