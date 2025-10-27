/*
 * fonctions.h
 *
 *  Created on: Oct 1, 2025
 *      Author: Tocqueville
 */

#ifndef INC_FONCTIONS_H_
#define INC_FONCTIONS_H_

#include "main.h"
#include <appli.h>
#include <lora.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "radio.h"
#include "stm32wlxx_hal_exti.h"
#include "queue.h"

#define WATCHDOG
//#define mode_sleep

// Structure pour les événements
typedef struct {
    uint8_t type;           // Type d'événement
    uint8_t source;         // Source de l'événement
    uint16_t data;          // Données de l'événement
} event_t;


typedef enum  {
    EVENT_BUTTON = 0,
    EVENT_LORA_TX,
	EVENT_LORA_TX_DONE,
	EVENT_LORA_RX,
	EVENT_LORA_REVEIL_BALISE,
    EVENT_UART_RX,
    EVENT_ERROR,
    EVENT_WAKE_UP,
    EVENT_SLEEP,
    EVENT_SYSTEM_RESET,
	EVENT_WATCHDOG_CHECK,
    EVENT_TIMER_24h,
    EVENT_TIMER_20min,
	EVENT_TIMER_LPTIM,
	EVENT_UART_RAZ,
	EVENT_CAD_DONE
} EventId_t;

#define TIMER_PERIOD_MS  2000   // 50s

// Sources d'événements
#define SOURCE_BUTTON           0x01
#define SOURCE_LORA             0x02
#define SOURCE_UART             0x03
#define SOURCE_TIMER            0x04
#define SOURCE_SYSTEM           0x05

// === SYSTÈME DE WATCHDOG ===
// Identifiants des tâches pour le watchdog
typedef enum {
    WATCHDOG_TASK_DEFAULT = 0,
    WATCHDOG_TASK_LORA_RX,
    WATCHDOG_TASK_LORA_TX,
    WATCHDOG_TASK_APPLI,
    WATCHDOG_TASK_UART_RX,
    WATCHDOG_TASK_UART_TX,
    WATCHDOG_TASK_COUNT  // Nombre total de tâches surveillées
} watchdog_task_id_t;


typedef enum {
    WATCHDOG_CONTEXT_ACTIVE = 0,    // Tâche active (surveillance normale)
    WATCHDOG_CONTEXT_WAITING,       // Tâche en attente (pas de surveillance)
    WATCHDOG_CONTEXT_SLEEPING,      // Tâche endormie (pas de surveillance)
    WATCHDOG_CONTEXT_BLOCKED,       // Tâche bloquée (surveillance critique)
    WATCHDOG_CONTEXT_CRITICAL,      // Tâche critique (surveillance renforcée)
    WATCHDOG_CONTEXT_UART_RX_WAIT,  // UART RX en attente (pas de surveillance)
    WATCHDOG_CONTEXT_UART_RX_ACTIVE // UART RX en traitement (surveillance active)
} watchdog_context_t;

// Structure pour le suivi des tâches
typedef struct {
    uint32_t last_heartbeat;    // Timestamp du dernier heartbeat
    uint32_t timeout_ms;        // Timeout en millisecondes
    uint8_t is_active;          // Tâche active ou non
    uint8_t error_count;        // Nombre d'erreurs consécutives
    watchdog_context_t context; // Contexte de la tâche
} watchdog_task_info_t;

// Configuration du watchdog
#define WATCHDOG_TIMEOUT_MS        22   // 60 secondes par défaut
#define WATCHDOG_ERROR_THRESHOLD   3       // Nombre d'erreurs avant reset
#define WATCHDOG_CHECK_INTERVAL    10000    // Vérification toutes les 5 secondes

extern LPTIM_HandleTypeDef hlptim1;
extern QueueHandle_t Event_QueueHandle;

#define test_MAX 20
extern uint8_t test_index;
extern uint8_t test_var;
extern uint32_t test_tab[test_MAX];

// Fonctions du système watchdog
void watchdog_init(void);
void watchdog_task_heartbeat(watchdog_task_id_t task_id);
void watchdog_task_start(watchdog_task_id_t task_id);
void watchdog_task_stop(watchdog_task_id_t task_id);
void watchdog_check_all_tasks(void);
void watchdog_reset_system(void);
uint8_t watchdog_is_task_alive(watchdog_task_id_t task_id);
void watchdog_print_status(void);
void watchdog_test_task_block(watchdog_task_id_t task_id, uint32_t duration_ms);
void watchdog_set_context(watchdog_task_id_t task_id, watchdog_context_t context);
uint32_t get_rtc_seconds_since_midnight(void);

// Fonctions de diagnostic du reset
void display_reset_cause(void);
const char* get_reset_cause_string(uint32_t reset_flags);
void save_diagnostic_data(void);
void load_diagnostic_data(void);
void check_stack_usage(void);
uint32_t get_rtc_timestamp(void);
void display_current_time(void);
void set_rtc_time_date(void);
HAL_StatusTypeDef set_rtc_from_timestamp(uint32_t timestamp);
HAL_StatusTypeDef set_rtc_date_from_string(const char* date_str);
HAL_StatusTypeDef set_rtc_time_from_string(const char* time_str);
void init_functions1(void);
void init_functions2(void);
void init_functions4(void);
void check_all_clocks(void);
void test_stop_mode(void);



#endif /* INC_FONCTIONS_H_ */
