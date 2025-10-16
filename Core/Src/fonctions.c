/*
 * fonctions.c
 *
 *  Created on: Oct 1, 2025
 *      Author: Tocqueville
 */

#include <communication.h>
#include <fonctions.h>
#include <eeprom_emul.h>
#include <log_flash.h>
#include <main.h>
#include "cmsis_os.h"
#include "timers.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>        // Pour struct tm et mktime

// === SYSTÈME DE WATCHDOG ===
// Tableau de suivi des tâches
static watchdog_task_info_t watchdog_tasks[WATCHDOG_TASK_COUNT];

#define nb_erreurs_enregistrees  20
#define nb_erreurs_envoyees      30
#define nb_erreurs_unique        0x20
#define nb_erreurs_4_fois        0x60

uint8_t code_erreur, comptage_erreur;
uint8_t err_donnee1, err_donnee2;
uint8_t enr_erreur[nb_erreurs_enregistrees];          // Enr_erreur enregistre les 30 premiÃ¨res erreurs
uint8_t erreurs_unique[nb_erreurs_unique/8];    // 1:erreur dÃ©ja envoyÃ©e, plus d'envoi
uint8_t erreurs_4_fois[nb_erreurs_4_fois/4];       // Nb d'erreurs dÃ©ja envoyÃ©es, max 4

uint8_t nb_reset=0;

uint8_t test_index;
uint8_t test_var;
uint32_t test_tab[test_MAX];

extern UART_HandleTypeDef hlpuart1;
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t Appli_TaskHandle;
extern osThreadId_t LORA_TX_TaskHandle;
extern osThreadId_t LORA_RX_TaskHandle;
extern osThreadId_t Uart_RX_TaskHandle;
extern osThreadId_t Uart_TX_TaskHandle;


// Fonctions pour gérer le contexte
void watchdog_set_timeout(watchdog_task_id_t task_id, uint32_t timeout_s);

static const char* watchdog_task_names[WATCHDOG_TASK_COUNT] = {
    "Default_Tsk",      // WATCHDOG_TASK_DEFAULT = 0
    "LORA_RX_Tsk",      // WATCHDOG_TASK_LORA_RX = 1
    "LORA_TX_Tsk",      // WATCHDOG_TASK_LORA_TX = 2
    "Appli___Tsk",      // WATCHDOG_TASK_APPLI = 3
    "Uart_RX_Tsk",      // WATCHDOG_TASK_UART_RX = 4
    "Uart_TX_Tsk"       // WATCHDOG_TASK_UART_TX = 5
};

// Timers
TimerHandle_t HTimer_24h;
TimerHandle_t HTimer_20min;
TimerHandle_t HTimer_Watchdog;  // Timer pour la vérification périodique du watchdog

void SystemClock_Config(void);

static void WatchdogTimerCallback(TimerHandle_t xTimer);
static void Timer24hCallback(TimerHandle_t xTimer);
static void Timer20minCallback(TimerHandle_t xTimer);

void configure_uart_wakeup(void)
{
    // Configuration du réveil UART pour le mode Stop
    UART_WakeUpTypeDef wakeup_config;
    wakeup_config.WakeUpEvent = UART_WAKEUP_ON_READDATA_NONEMPTY;

    // Configurer le réveil UART
    HAL_UARTEx_StopModeWakeUpSourceConfig(&hlpuart1, wakeup_config);

    // Activer le mode Stop pour l'UART
    HAL_UARTEx_EnableStopMode(&hlpuart1);

}

void init_functions(void)
{
	log_init();
    /*if (log_init() == HAL_OK)
        LOG_INFO("LOG initialisee");
     else
        LOG_ERROR("Erreur LOG");*/



	// Afficher la cause du reset au démarrage
	display_reset_cause();


	// creation timers
	 HTimer_24h = xTimerCreate(
	        "Timer24h",                          // Nom
	        24*3600*1000,     // Période en ticks
	        pdTRUE,                             // Auto-reload
	        (void*)0,                           // ID optionnel
	        Timer24hCallback                     // Callback
	    );
	    if (HTimer_24h != NULL) xTimerStart(HTimer_24h, 0);


		 HTimer_20min = xTimerCreate(
		        "Timer20min",                          // Nom
		        pdMS_TO_TICKS(TIMER_PERIOD_MS),     // Période en ticks
		        pdTRUE,                             // Auto-reload
		        (void*)0,                           // ID optionnel
		        Timer20minCallback                     // Callback
		    );
		    if (HTimer_20min != NULL) xTimerStart(HTimer_20min, 0);

	// Initialisation du système de watchdog logiciel
	watchdog_init();

#ifdef mode_sleep
	configure_uart_wakeup();
#endif


}

// Mode Stop1(4uA) : horloges ok : LSE, LSI et APB    mode stop2 (3uA):APB arreté
// LPTIM1 si prescaler=1, 33us/ticks => max : 2 sec
// Dans PreSleepProcessing
void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	if (test_index < test_MAX)
	{
		test_tab[test_index] = *ulExpectedIdleTime;
		test_index++;
	}

	/*
	    (*ulExpectedIdleTime) is set to 0 to indicate that PreSleepProcessing contains
	    its own wait for interrupt or wait for event instruction and so the kernel vPortSuppressTicksAndSleep
	    function does not need to execute the wfi instruction
	  */
	 *ulExpectedIdleTime = 0;

	  /*Enter to sleep Mode using the HAL function HAL_PWR_EnterSLEEPMode with WFI instruction*/
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	if (*ulExpectedIdleTime >= 2)
    {

    	//HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);


    	//HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);


        // ⭐ METTRE LE RADIO EN VEILLE AVANT LE STOP
        //subghz_enter_sleep_mode();

        // Entrer en mode Stop
        //PWR_EnterStopMode();
    }
}

// Dans PostSleepProcessing
void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
    // ⭐ RÉVEILLER LE RADIO APRÈS LE STOP
    //subghz_wake_up();

	  SystemClock_Config();
	  (void) ulExpectedIdleTime;

	test_var++;

	//HAL_PWR_ExitSTOPMode();

    //PWR_ExitStopMode();
     //*ulExpectedIdleTime = 0;
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	if (Event_QueueHandle == NULL)  return; // Queue pas encore créée

	if (hlptim->Instance == LPTIM1)
    {
		event_t evt = { EVENT_TIMER_LPTIM, 1, 1 }; // timer 1, evt1
		if (xQueueSendFromISR(Event_QueueHandle, &evt, 0) != pdPASS)
		{
			code_erreur = ISR_callback;
			err_donnee1 = 1;
		}
    }
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	if (Event_QueueHandle == NULL)  return; // Queue pas encore créée

    if (hlptim->Instance == LPTIM1) {

        //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        event_t evt = { EVENT_TIMER_LPTIM, 1, 2 }; // timer 1, evt2

        if (xQueueSendFromISR(Event_QueueHandle, &evt, 0) != pdPASS)
        {
            code_erreur = ISR_callback;
            err_donnee1 = 4;
        }

        //portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void Timer24hCallback(TimerHandle_t xTimer)
{
    LOG_INFO("timer24h declenche");
	event_t evt = { EVENT_TIMER_24h, 0, 0 };
	if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	{
	    LOG_ERROR("Event queue full - message lost");
	}
}

static void Timer20minCallback(TimerHandle_t xTimer)
{
	event_t evt = { EVENT_TIMER_20min, 0, 0 };
	if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	{
	    LOG_ERROR("Event queue full - message lost");
	}
}

// ISR pour l'appui sur le bouton PA12
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12) {
        // ⭐ VOTRE CODE ICI - Contexte d'interruption !

        // ⚠️ ATTENTION : Contexte d'interruption - Code minimal !

        // ✅ AUTORISÉ : Variables volatiles
        //static volatile bool button_pressed = true;

        // ✅ AUTORISÉ : Envoyer un event (ISR safe)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        event_t evt = { EVENT_BUTTON, 0, 1 };

        if (xQueueSendFromISR(Event_QueueHandle, &evt, &xHigherPriorityTaskWoken) != pdPASS)
        {
            code_erreur = ISR_callback;
            err_donnee1 = 3;
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        // ❌ INTERDIT : LOG_INFO(), osDelay(), etc.
    }
}

/**
 * @brief Retourne le temps restant avant expiration du timer, en millisecondes.
 * @param t Handle du timer FreeRTOS
 * @return Temps restant en ms (0 si expiré ou inactif)
 */
uint32_t TimerGetRemainingMs(TimerHandle_t t)
{
    if (t == NULL || xTimerIsTimerActive(t) == pdFALSE)
    {
        return 0; // Timer non actif ou invalide
    }

    TickType_t expiry = xTimerGetExpiryTime(t);
    TickType_t now = xTaskGetTickCount();

    if (expiry <= now)
    {
        return 0; // Déjà expiré ou en cours de callback
    }

    TickType_t remainingTicks = expiry - now;
    return (uint32_t) ((remainingTicks * 1000) / configTICK_RATE_HZ);
}

void check_stack_usage(void)
{
    UBaseType_t stack_high_water_mark;

    // Vérifier chaque tâche
    //LOG_INFO("=== STACK USAGE REPORT ===");

    // default_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(defaultTaskHandle);
    LOG_INFO("Default_Task: %i free", (uint16_t) stack_high_water_mark);

    // Appli_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Appli_TaskHandle);
    LOG_INFO("Appli_Task: %i free", stack_high_water_mark);

    // LORA_TX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(LORA_TX_TaskHandle);
    LOG_INFO("LORA_TX_Task: %i free", stack_high_water_mark);

    // LORA_RX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(LORA_RX_TaskHandle);
    LOG_INFO("LORA_RX_Task: %i free", stack_high_water_mark);

    // Uart_TX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Uart_TX_TaskHandle);
    LOG_INFO("Uart_TX_Task: %i free", stack_high_water_mark);

    // Uart1_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Uart_RX_TaskHandle);
    LOG_INFO("Uart_RX_Task: %i free", stack_high_water_mark);

    //LOG_INFO("=== END STACK REPORT ===");
}


void envoi_code_erreur (void)        // envoie l'erreur a dest_erreur_reset
{
    uint8_t i3;

    // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
    // Erreurs 20 a 7F : 3 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
    // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

    LOG_INFO("erreur %02X %i", code_erreur, err_donnee1);

    if (comptage_erreur < nb_erreurs_envoyees)  // 30 envoyees, 20 enregistrees
    {
       i3 = 0;
       if (comptage_erreur < nb_erreurs_enregistrees)  enr_erreur[comptage_erreur]=code_erreur;
        comptage_erreur++;
        if (code_erreur < (nb_erreurs_4_fois+nb_erreurs_unique))
        {
            if (code_erreur < nb_erreurs_unique) // 0x20  envoi_une seule fois
            {
                if (code_erreur < nb_erreurs_unique)
                {
                    uint8_t index, bit;
                    index = code_erreur/8;  // si code=0xA =>index=1, bit=2;
                    bit = code_erreur % 8;
                    if (! (erreurs_unique[index] & (1<<bit)))
                    {
                        i3=1;
                        erreurs_unique[index] |= (1<<bit);
                    }
                }
            }
            else      // envoi 4 fois max
            {
                uint8_t index, bit, cpt;
                index = (code_erreur-nb_erreurs_unique)/4;  // si code=0xA =>index=3, bit=4;
                bit = (code_erreur % 4)*2;
                cpt = (erreurs_4_fois[index] >> bit) & 3 ;
                if  (cpt < 3)
                {
                    i3=1;
                    erreurs_4_fois[index] += (1 << bit);
                }
            }
        }
        else i3=1;  //  0x80 Ã  0xFF

        if (i3)
          {
          #ifdef dest_erreur
           uint8_t message[30];
           message[0] = dest_erreur;
           message[1] ='E';
           message[2] ='L';
           message[3] = nb_reset + 48;
           message[4] = comptage_erreur + 48;
           message[5] = ':';
           message[6] =deci(code_erreur >> 4 );
           message[7] =deci(code_erreur & 15);
           message[8] = '-';
           message[9] = err_donnee1+48;
           message[10] = err_donnee2+48;
           i3=9;
           if (err_donnee1) i3=10;
           if (err_donnee2) i3=11;
           message[i3] = 0;
           envoie_mess_ASC((const char*)message);
          #endif
          }
    }
    code_erreur=0;
    err_donnee1=0;
    err_donnee2=0;
}

uint8_t deci (uint8_t val) //transforme un char hexa en son charactere ASCII   0->48 9->57 A->65 F->70 G->71
{
  uint8_t resul;
  if (val > 36)
    resul = 91;  // caractere apres Z pour indiquer une erreur
  else if (val < 10)
    resul = val + 48;
  else
    resul = val + 55;
  return resul;
}

// === IMPLÉMENTATION DU SYSTÈME DE WATCHDOG ===

/**
 * @brief Initialise le système de watchdog
 */
void watchdog_init(void)
{
    //LOG_INFO("Initializing watchdog system...");

    // Initialiser toutes les tâches comme inactives
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        watchdog_tasks[i].last_heartbeat = 0;
        watchdog_tasks[i].timeout_ms = WATCHDOG_TIMEOUT_MS;
        watchdog_tasks[i].is_active = 0;
        watchdog_tasks[i].error_count = 0;
        watchdog_tasks[i].context = WATCHDOG_CONTEXT_ACTIVE;  // Contexte par défaut
    }

    // Créer le timer de vérification du watchdog
    HTimer_Watchdog = xTimerCreate(
        "WatchdogTimer",                    // Nom
        pdMS_TO_TICKS(WATCHDOG_CHECK_INTERVAL), // Période
        pdTRUE,                            // Auto-reload
        (void*)0,                          // ID
        WatchdogTimerCallback              // Callback
    );

	#ifdef WATCHDOG
		if (HTimer_Watchdog != NULL) {
			xTimerStart(HTimer_Watchdog, 0);
			//LOG_INFO("Watchdog timer started");
		} else {
			LOG_ERROR("Failed to create watchdog timer");
		}
	#endif
}

void watchdog_set_context(watchdog_task_id_t task_id, watchdog_context_t context)
{
    if (task_id < WATCHDOG_TASK_COUNT) {
        watchdog_tasks[task_id].context = context;

        // Ajuster le timeout selon le contexte
        switch (context) {
            case WATCHDOG_CONTEXT_ACTIVE:
                watchdog_tasks[task_id].timeout_ms = WATCHDOG_TIMEOUT_MS;  // 30s
                watchdog_tasks[task_id].last_heartbeat = xTaskGetTickCount()/1000;
                break;
            case WATCHDOG_CONTEXT_WAITING:
            case WATCHDOG_CONTEXT_SLEEPING:
            case WATCHDOG_CONTEXT_UART_RX_WAIT:
                watchdog_tasks[task_id].timeout_ms = 0;  // Pas de surveillance
                break;
            case WATCHDOG_CONTEXT_BLOCKED:
                watchdog_tasks[task_id].timeout_ms = 0;  // 5s
                break;
            case WATCHDOG_CONTEXT_CRITICAL:
                watchdog_tasks[task_id].timeout_ms = 10;  // 10s
                watchdog_tasks[task_id].last_heartbeat = xTaskGetTickCount()/1000;
                break;
            case WATCHDOG_CONTEXT_UART_RX_ACTIVE:
                watchdog_tasks[task_id].timeout_ms = 3;  // 2s
                watchdog_tasks[task_id].last_heartbeat = xTaskGetTickCount()/1000;
				break;
        }


        //LOG_DEBUG("%s cont %d, timer: %us",
        //         watchdog_task_names[task_id], watchdog_tasks[task_id].context,
        //         (unsigned int)watchdog_tasks[task_id].timeout_ms);
    }
}

// Fonction pour définir un timeout personnalisé
void watchdog_set_timeout(watchdog_task_id_t task_id, uint32_t timeout_s)
{
    if (task_id < WATCHDOG_TASK_COUNT) {
        watchdog_tasks[task_id].timeout_ms = timeout_s;
        LOG_DEBUG("Task %s timeout set to %us",
                 watchdog_task_names[task_id], (unsigned int)timeout_s);
    }
}

/**
 * @brief Callback du timer de watchdog
 */
static void WatchdogTimerCallback(TimerHandle_t xTimer)
{
    //LOG_INFO("timer24h declenche");
	event_t evt = { EVENT_WATCHDOG_CHECK, 0, 0 };
	if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	{
	    LOG_ERROR("Event watch queue full - message lost");
	}
}

/**
 * @brief Démarre la surveillance d'une tâche
 */
void watchdog_task_start(watchdog_task_id_t task_id)
{
    if (task_id < WATCHDOG_TASK_COUNT) {
        watchdog_tasks[task_id].is_active = 1;
        watchdog_tasks[task_id].last_heartbeat = xTaskGetTickCount()/1000;
        watchdog_tasks[task_id].error_count = 0;
        //LOG_DEBUG("Watchdog started for task %d", task_id);
    }
}

/**
 * @brief Arrête la surveillance d'une tâche
 */
void watchdog_task_stop(watchdog_task_id_t task_id)
{
    if (task_id < WATCHDOG_TASK_COUNT) {
        watchdog_tasks[task_id].is_active = 0;
        LOG_DEBUG("Watchdog stopped for task %d", task_id);
    }
}

/**
 * @brief Enregistre un heartbeat d'une tâche
 */
void watchdog_task_heartbeat(watchdog_task_id_t task_id)
{
    if (task_id < WATCHDOG_TASK_COUNT && watchdog_tasks[task_id].is_active) {
        watchdog_tasks[task_id].last_heartbeat = xTaskGetTickCount()/1000;
        watchdog_tasks[task_id].error_count = 0; // Reset du compteur d'erreurs
    }
}

/**
 * @brief Vérifie si une tâche est vivante
 */
uint8_t watchdog_is_task_alive(watchdog_task_id_t task_id)
{
    if (task_id >= WATCHDOG_TASK_COUNT || !watchdog_tasks[task_id].is_active) {
        return 1; // Tâche non surveillée ou inactive = considérée comme vivante
    }

    uint32_t current_time = xTaskGetTickCount()/1000;
    uint32_t elapsed_s = (current_time - watchdog_tasks[task_id].last_heartbeat);// * 1000 / configTICK_RATE_HZ;

    return (elapsed_s < watchdog_tasks[task_id].timeout_ms);
}

/**
 * @brief Vérifie toutes les tâches surveillées
 */
void watchdog_check_all_tasks(void)
{

    uint32_t current_time = xTaskGetTickCount()/1000;
	//LOG_INFO("Watchdog : check all - Time:%i", current_time);

    uint8_t critical_errors = 0;

    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        if (!watchdog_tasks[i].is_active)
		{
            continue; // Tâche non surveillée
        }

        /*if ((watchdog_tasks[i].context != WATCHDOG_CONTEXT_ACTIVE)
		&&  (watchdog_tasks[i].context != WATCHDOG_CONTEXT_CRITICAL))
		{
            continue; // Tâche non surveillée temporairement
        }*/

        // Si timeout = 0, pas de surveillance
        if (watchdog_tasks[i].timeout_ms == 0) {
            continue; // Tâche en attente/sommeil
        }

        //LOG_INFO("Watchdog : check %i last:%i", i, watchdog_tasks[i].last_heartbeat);

        uint32_t elapsed_s = (current_time - watchdog_tasks[i].last_heartbeat); // * 1000 / configTICK_RATE_HZ;

        if (elapsed_s >= watchdog_tasks[i].timeout_ms)
        {

            watchdog_tasks[i].error_count++;

            const char* context_name = "Unknown";
            switch (watchdog_tasks[i].context) {
                case WATCHDOG_CONTEXT_ACTIVE:   context_name = "Active"; break;
                case WATCHDOG_CONTEXT_WAITING:   context_name = "Waiting"; break;
                case WATCHDOG_CONTEXT_SLEEPING:  context_name = "Sleeping"; break;
                case WATCHDOG_CONTEXT_BLOCKED:  context_name = "Blocked"; break;
                case WATCHDOG_CONTEXT_CRITICAL:  context_name = "Critical"; break;
                case WATCHDOG_CONTEXT_UART_RX_WAIT:  context_name = "UART_RX_Wait"; break;
                case WATCHDOG_CONTEXT_UART_RX_ACTIVE: context_name = "UART_RX_Active"; break;
            }

            LOG_ERROR("%s timeout %s %is, Er:%u tm:%i",
                     watchdog_task_names[i], context_name,
                     (unsigned int)elapsed_s, watchdog_tasks[i].error_count,
					 watchdog_tasks[i].timeout_ms);

            if (watchdog_tasks[i].error_count >= WATCHDOG_ERROR_THRESHOLD) {
                LOG_ERROR("Task %s exceeded threshold, system reset!",
                         watchdog_task_names[i]);
                critical_errors++;
            }
        }
    }
    // Si trop d'erreurs critiques, redémarrer le système
    if (critical_errors > 0) {
        watchdog_reset_system();
    }
}

/**
 * @brief Affiche le statut de toutes les tâches surveillées
 */
void watchdog_print_status(void)
{
    LOG_INFO("=== WATCHDOG STATUS ===");

    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        const char* task_name = watchdog_task_names[i];

        if (watchdog_tasks[i].is_active) {
            uint32_t current_time = xTaskGetTickCount()/1000;
            uint32_t elapsed_s = (current_time - watchdog_tasks[i].last_heartbeat); // * 1000 / configTICK_RATE_HZ;

            const char* context_name = "Unknown";
            switch (watchdog_tasks[i].context) {
                case WATCHDOG_CONTEXT_ACTIVE:   context_name = "Active"; break;
                case WATCHDOG_CONTEXT_WAITING:   context_name = "Waiting"; break;
                case WATCHDOG_CONTEXT_SLEEPING:  context_name = "Sleeping"; break;
                case WATCHDOG_CONTEXT_BLOCKED:  context_name = "Blocked"; break;
                case WATCHDOG_CONTEXT_CRITICAL:  context_name = "Critical"; break;
                case WATCHDOG_CONTEXT_UART_RX_WAIT:  context_name = "UART_RX_Wait"; break;
                case WATCHDOG_CONTEXT_UART_RX_ACTIVE: context_name = "UART_RX_Active"; break;
            }

            LOG_INFO("%s: %s, Last beat: %u s, Err: %u",
                    task_name, context_name, (unsigned int)elapsed_s, watchdog_tasks[i].error_count);
        } else {
            LOG_INFO("%s: Inactive", task_name);
        }
    }

    //LOG_INFO("=== END WATCHDOG STATUS ===");
}
/**
 * @brief Redémarre le système en cas d'erreur critique
 */
void watchdog_reset_system(void)
{
    LOG_ERROR("WATCHDOG: Critic failure detect, reset...");

    // Envoyer un message d'erreur avant le reset
    //envoie_mess_ASC("WATCHDOG: System reset due to task failures");

    // Attendre un peu pour que le message soit envoyé
    osDelay(1000);

    // Forcer un reset système
    NVIC_SystemReset();
}

/**
 * @brief Fonction de test pour simuler une tâche bloquée
 * @param task_id: ID de la tâche à bloquer
 * @param duration_ms: Durée du blocage en millisecondes
 */
void watchdog_test_task_block(watchdog_task_id_t task_id, uint32_t duration_ms)
{
    LOG_WARNING("WATCHDOG TEST: Block task %d for %lu ms", task_id, duration_ms);

    // Arrêter temporairement la surveillance de cette tâche
    watchdog_task_stop(task_id);

    // Simuler un blocage
    osDelay(duration_ms);

    // Redémarrer la surveillance
    watchdog_task_start(task_id);

    LOG_INFO("WATCHDOG TEST: Task %d unblocked", task_id);
}

// === DIAGNOSTIC DE LA CAUSE DU RESET ===

/**
 * @brief Affiche la cause du reset au démarrage
 */
void display_reset_cause(void)
{
    // Charger les données de diagnostic de la session précédente
    //load_diagnostic_data();

    // Lire les flags de reset depuis RCC
    uint32_t reset_flags = RCC->CSR;

    LOG_INFO("=== SYSTEM RESET DIAGNOSTIC ===");
    LOG_INFO("Reset flags: 0x%08lX", reset_flags);

    // Analyser chaque cause de reset
    if (reset_flags & RCC_CSR_LPWRRSTF) {
        LOG_INFO("Reset cause: Low Power (LPWR)");
    }
    if (reset_flags & RCC_CSR_WWDGRSTF) {
        LOG_INFO("Reset cause: Window Watchdog (WWDG)");
    }
    if (reset_flags & RCC_CSR_IWDGRSTF) {
        LOG_INFO("Reset cause: Independent Watchdog (IWDG)");
    }
    if (reset_flags & RCC_CSR_SFTRSTF) {
        LOG_INFO("Reset cause: Soft Reset (NVIC_SystemReset)");
    }
    if (reset_flags & RCC_CSR_BORRSTF) {
        LOG_INFO("Reset cause: Brown On Reset (BOR)");
    }
    if (reset_flags & RCC_CSR_PINRSTF) {
        LOG_INFO("Reset cause: External Pin Reset (NRST)");
    }
    if (reset_flags & RCC_CSR_OBLRSTF) {
        LOG_INFO("Reset cause: Option Byte Loader Reset");
    }

    // Si aucun flag n'est défini, c'est un reset inconnu
    if (!(reset_flags & (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF |
                         RCC_CSR_SFTRSTF | RCC_CSR_BORRSTF | RCC_CSR_PINRSTF |
                         RCC_CSR_OBLRSTF ))) {
        LOG_WARNING("Reset : Unknown/mult causes");
    }

    // Afficher le nombre de resets
    static uint32_t reset_count = 0;
    reset_count++;
    LOG_INFO("Nb_Reset since last power: %lu", reset_count);

    // Afficher le temps d'uptime si disponible
    uint32_t uptime_ms = HAL_GetTick();
    LOG_INFO("System uptime: %lu ms", uptime_ms);

    //LOG_INFO("=== END RESET DIAGNOSTIC ===");

    // Effacer les flags de reset pour le prochain démarrage
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/**
 * @brief Retourne une chaîne décrivant la cause du reset
 * @param reset_flags: Flags de reset du registre RCC->CSR
 * @return Chaîne descriptive de la cause
 */
const char* get_reset_cause_string(uint32_t reset_flags)
{
    if (reset_flags & RCC_CSR_LPWRRSTF) {
        return "Low Power Reset";
    }
    if (reset_flags & RCC_CSR_WWDGRSTF) {
        return "Window Watchdog Reset";
    }
    if (reset_flags & RCC_CSR_IWDGRSTF) {
        return "Independent Watchdog Reset";
    }
    if (reset_flags & RCC_CSR_SFTRSTF) {
        return "Software Reset";
    }
    if (reset_flags & RCC_CSR_BORRSTF) {
        return "Brown On Reset";
    }
    if (reset_flags & RCC_CSR_PINRSTF) {
        return "External Pin Reset";
    }
    if (reset_flags & RCC_CSR_OBLRSTF) {
        return "Option Byte Loader Reset";
    }

    return "Unknown Reset Cause";
}

// === SAUVEGARDE DE DONNÉES DE DIAGNOSTIC ===

// Structure pour sauvegarder des informations de diagnostic
typedef struct {
    uint32_t magic_number;        // Nombre magique pour valider la structure
    uint32_t reset_count;         // Compteur de resets
    uint32_t last_uptime_ms;      // Temps de fonctionnement avant le reset
    uint32_t last_reset_cause;    // Cause du dernier reset
    uint32_t watchdog_errors;     // Nombre d'erreurs watchdog
    uint32_t timestamp;           // Timestamp du dernier enregistrement
} diagnostic_data_t;

#define DIAGNOSTIC_MAGIC_NUMBER   0xDEADBEEF
#define DIAGNOSTIC_DATA_ADDR      (0x20000000 + 0x2000)  // Adresse en RAM backup



// Hook appelé si une tâche dépasse sa pile
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    char msg[100];
    int len = snprintf(msg, sizeof(msg), "-- Stack overflow in task: %s\r\n", pcTaskName);
    msg[0] = dest_log;
    msg[1] = My_Address;
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, len, HAL_MAX_DELAY);

    HAL_Delay(2000);
    NVIC_SystemReset();
    //taskDISABLE_INTERRUPTS();
    //for(;;);
}

// Hook appelé si malloc échoue
void vApplicationMallocFailedHook(void)
{
    char msg[] = "-- Malloc failed!\r\n";
    msg[0] = dest_log;
    msg[1] = My_Address;
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);

    HAL_Delay(2000);
    NVIC_SystemReset();
    //taskDISABLE_INTERRUPTS();
    //for(;;);
}

void set_rtc_time_date(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // Configuration de l'heure (exemple : 14:30:25)
    sTime.Hours = 22;
    sTime.Minutes = 30;
    sTime.Seconds = 25;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        LOG_ERROR("RTC set time failed");
        return;
    }

    // Configuration de la date (exemple : Dimanche 15/12/2024)
    sDate.WeekDay = RTC_WEEKDAY_SUNDAY;  // Dimanche
    sDate.Month = RTC_MONTH_OCTOBER;    // Décembre
    sDate.Date = 5;                     // 15
    sDate.Year = 25;                     // 2024 (année - 2000)

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        LOG_ERROR("RTC set date failed");
        return;
    }

    LOG_INFO("RTC configured: %02d:%02d:%02d %02d/%02d/20%02d",
             sTime.Hours, sTime.Minutes, sTime.Seconds,
             sDate.Date, sDate.Month, sDate.Year);
}

uint32_t get_rtc_timestamp(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    // Lire l'heure et la date actuelles
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Convertir en timestamp Unix (secondes depuis 1er janvier 1970)
    struct tm timeinfo = {
        .tm_sec = sTime.Seconds,
        .tm_min = sTime.Minutes,
        .tm_hour = sTime.Hours,
        .tm_mday = sDate.Date,
        .tm_mon = sDate.Month - 1,      // Janvier = 0
        .tm_year = sDate.Year + 100     // Année depuis 1900
    };

    return (uint32_t)mktime(&timeinfo);
}

void display_current_time(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    LOG_INFO("HL: %02d/%02d/20%02d %02d:%02d:%02d", sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds);
    LOG_INFO("Timestamp: %08X", get_rtc_timestamp());
    //LOG_INFO("Timestamp: %lu", get_rtc_timestamp());
}

HAL_StatusTypeDef set_rtc_time_from_string(const char* time_str)
{
    if (time_str == NULL || strlen(time_str) != 6) {
        LOG_ERROR("Invalid time format. Expected HHMMSS");
        return HAL_ERROR;
    }

    // Extraire les composants
    //uint8_t hours, minutes, seconds;
    /*if (sscanf(time_str, "%2hhu%2hhu%2hhu", &hours, &minutes, &seconds) != 3) {
        LOG_ERROR("Failed to parse time string: %s", time_str);
        return HAL_ERROR;
    }*/
    uint8_t hours = (time_str[0] - '0') * 10 + (time_str[1] - '0');
    uint8_t minutes = (time_str[2] - '0') * 10 + (time_str[3] - '0');
    uint8_t seconds = (time_str[4] - '0') * 10 + (time_str[5] - '0');

    // Vérifier la validité des valeurs
    if (hours > 23 || minutes > 59 || seconds > 59) {
        LOG_ERROR("Invalid time values: %02d:%02d:%02d", hours, minutes, seconds);
        return HAL_ERROR;
    }

    // Configurer le RTC
    RTC_TimeTypeDef sTime = {0};
    sTime.Hours = hours;
    sTime.Minutes = minutes;
    sTime.Seconds = seconds;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    HAL_StatusTypeDef status = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    if (status == HAL_OK) {
        LOG_INFO("RTC time set to: %02d:%02d:%02d", hours, minutes, seconds);
    } else {
        LOG_ERROR("Failed to set RTC time: %d", status);
    }

    return status;
}

HAL_StatusTypeDef set_rtc_date_from_string(const char* date_str)
{
    if (date_str == NULL || strlen(date_str) != 6) {
        LOG_ERROR("Invalid date format. Expected DDMMYY");
        return HAL_ERROR;
    }

    // Extraire les composants
    uint8_t day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
    uint8_t month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
    uint8_t year = (date_str[4] - '0') * 10 + (date_str[5] - '0');

    // Vérifier la validité des valeurs
    if (day < 1 || day > 31) {
        LOG_ERROR("Invalid day: %d (must be 1-31)", day);
        return HAL_ERROR;
    }

    if (month < 1 || month > 12) {
        LOG_ERROR("Invalid month: %d (must be 1-12)", month);
        return HAL_ERROR;
    }

    if (year > 99) {
        LOG_ERROR("Invalid year: %d (must be 0-99)", year);
        return HAL_ERROR;
    }

    // Configurer le RTC (sans calculer le jour de la semaine)
    RTC_DateTypeDef sDate = {0};
    sDate.WeekDay = 1;  // Lundi par défaut (ou jour fixe)
    sDate.Month = month;
    sDate.Date = day;
    sDate.Year = year;  // RTC stocke l'année - 2000

    HAL_StatusTypeDef status = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    if (status == HAL_OK) {
        LOG_INFO("RTC date set to: %02d/%02d/20%02d", day, month, year);
    } else {
        LOG_ERROR("Failed to set RTC date: %d", status);
    }

    return status;
}


HAL_StatusTypeDef set_rtc_from_timestamp(uint32_t timestamp)
{
    // Convertir le timestamp Unix en date/heure
    time_t rawtime = (time_t)timestamp;
    struct tm *timeinfo = localtime(&rawtime);

    if (timeinfo == NULL) {
        LOG_ERROR("Invalid timestamp: %lu", timestamp);
        return HAL_ERROR;
    }

    // Configurer l'heure
    RTC_TimeTypeDef sTime = {0};
    sTime.Hours = timeinfo->tm_hour;
    sTime.Minutes = timeinfo->tm_min;
    sTime.Seconds = timeinfo->tm_sec;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    HAL_StatusTypeDef time_status = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    if (time_status != HAL_OK) {
        LOG_ERROR("Failed to set RTC time: %d", time_status);
        return time_status;
    }

    // Configurer la date
    RTC_DateTypeDef sDate = {0};
    sDate.WeekDay = timeinfo->tm_wday + 1;  // Convertir 0-6 en 1-7
    sDate.Month = timeinfo->tm_mon + 1;     // Convertir 0-11 en 1-12
    sDate.Date = timeinfo->tm_mday;
    sDate.Year = timeinfo->tm_year - 100;    // Convertir depuis 1900

    HAL_StatusTypeDef date_status = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    if (date_status != HAL_OK) {
        LOG_ERROR("Failed to set RTC date: %d", date_status);
        return date_status;
    }

    LOG_INFO("RTC set from timestamp %lu: %02d/%02d/%04d %02d:%02d:%02d",
             timestamp, sDate.Date, sDate.Month, 2000 + sDate.Year,
             sTime.Hours, sTime.Minutes, sTime.Seconds);

    return HAL_OK;
}

void check_all_clocks(void)
{
    LOG_INFO("=== CLOCK DIAGNOSTIC ===");

    // Horloges principales (pas de macro IS_CLK_ENABLED pour LSI/MSI)
    LOG_INFO("MSI Clock: ENABLED (4MHz) - ~1mA");
    LOG_INFO("LSI Clock: ENABLED (32kHz) - ~0.1µA");

    // Horloges périphériques - MACROS QUI EXISTENT
    LOG_INFO("GPIOA Clock: %s", __HAL_RCC_GPIOA_IS_CLK_ENABLED() ? "ENABLED" : "DISABLED");
    LOG_INFO("LPUART1 Clock: %s", __HAL_RCC_LPUART1_IS_CLK_ENABLED() ? "ENABLED" : "DISABLED");
    LOG_INFO("RTC Clock: %s", __HAL_RCC_RTCAPB_IS_CLK_ENABLED() ? "ENABLED" : "DISABLED");

    // IWDG et RF n'ont pas de macro IS_CLK_ENABLED - on les affiche comme toujours actifs
    LOG_INFO("IWDG Clock: ENABLED (always active)");
    LOG_INFO("RF Clock: ENABLED (always active)");

    // Tickless Idle
    LOG_INFO("Tickless Idle: %s", configUSE_TICKLESS_IDLE ? "ENABLED" : "DISABLED");

    // Estimation consommation
    uint32_t consumption = 0;

    // MSI toujours actif (horloge système)
    consumption += 1000; // MSI 4MHz = ~1mA
    LOG_INFO("MSI active: +1000µA (system clock)");

    // IWDG et RF toujours actifs
    consumption += 100; // RF clock
    consumption += 10;  // IWDG clock
    LOG_INFO("RF + IWDG: +110µA (always active)");

    if (__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
        consumption += 10; // GPIO clock
        LOG_INFO("GPIOA clock: +10µA");
    }

    if (__HAL_RCC_LPUART1_IS_CLK_ENABLED()) {
        consumption += 5; // LPUART clock
        LOG_INFO("LPUART1 clock: +5µA");
    }

    LOG_INFO("Estimated consumption: %lu µA", consumption);
    LOG_INFO("Your measurement: 1100 µA");

    if (consumption > 1000) {
        LOG_ERROR("HIGH CONSUMPTION DETECTED!");
        LOG_ERROR("Main cause: MSI 4MHz active (~1mA)");
    }

    LOG_INFO("========================");
}

// Fonction de test du mode Stop
void test_stop_mode(void)
{
    LOG_INFO("=== TEST MODE STOP ===");

    uint32_t start_time = HAL_GetTick();

    // Entrer en mode Stop manuellement
    LOG_INFO("Entering Stop mode...");
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    uint32_t end_time = HAL_GetTick();
    uint32_t elapsed = end_time - start_time;

    LOG_INFO("Exited Stop mode after %lu ms", elapsed);

    if (elapsed < 10) {
        LOG_ERROR("Stop mode NOT working - CPU stayed active!");
    } else {
        LOG_INFO("Stop mode working - CPU was in low power");
    }
}
