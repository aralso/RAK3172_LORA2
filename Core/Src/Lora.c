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
#include "Region.H"
#include "RegionEU868.h"

#define DR_0	0
#define DR_1	1
#define DR_2	2
#define DR_4	3
#define DR_5	5
#define DR_6	6
#define DR_7	7

double floor (double);

uint32_t RegionCommonGetBandwidth( uint32_t drIndex, const uint32_t* bandwidths )
{
    switch( bandwidths[drIndex] )
    {
        default:
        case 125000:
            return 0;
        case 250000:
            return 1;
        case 500000:
            return 2;
    }
}

int8_t RegionCommonComputeTxPower( int8_t txPowerIndex, float maxEirp, float antennaGain )
{
    int8_t phyTxPower = 0;

    phyTxPower = ( int8_t )floor( ( maxEirp - ( txPowerIndex * 2U ) ) - antennaGain );

    return phyTxPower;
}

bool RegionEU868TxConfigM( TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir )
{
    RadioModems_t modem;
    int8_t phyDr = DataratesEU868[txConfig->Datarate];
    int8_t txPowerLimited = 0; // RegionCommonLimitTxPower( txConfig->TxPower, RegionBands[RegionNvmGroup2->Channels[txConfig->Channel].Band].TxMaxPower );
    uint32_t bandwidth = RegionCommonGetBandwidth( txConfig->Datarate, BandwidthsEU868 );
    int8_t phyTxPower = 0;
    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, txConfig->MaxEirp, txConfig->AntennaGain );

    // Setup the radio frequency
    Radio.SetChannel( 868250000); //RegionNvmGroup2->Channels[txConfig->Channel].Frequency );

    if( txConfig->Datarate == DR_7 )
    { // High Speed FSK channel
        modem = MODEM_FSK;
        Radio.SetTxConfig( modem, phyTxPower, 25000, bandwidth, phyDr * 1000, 0, 5, false, true, 0, 0, false, 4000 );
    }
    else
    {
        modem = MODEM_LORA;
        Radio.SetTxConfig( modem, phyTxPower, 0, bandwidth, phyDr, 1, 8, false, true, 0, 0, false, 4000 );
    }
    //MW_LOG(TS_ON, VLEVEL_M,  "PhyTxPower:%i len:%i\r\n", phyTxPower, txConfig->PktLen  );
    //RegionCommonTxConfigPrint(txConfig->Channel, txConfig->Datarate);

    // Update time-on-air
    //*txTimeOnAir = GetTimeOnAir( txConfig->Datarate, txConfig->PktLen );

    // Setup maximum payload length of the radio driver
    Radio.SetMaxPayloadLength( modem, txConfig->PktLen );

    *txPower = txPowerLimited;
    return true;

}


uint8_t SendFrameModif( uint8_t channel )
{
    uint8_t status = 1;
    TxConfigParams_t txConfig;
    int8_t txPower = 0;

    txConfig.Channel = 1; //channel;  // 0, 1 ou 2
    txConfig.Datarate = 0; //Nvm.MacGroup1.ChannelsDatarate;  // 0
    txConfig.TxPower = 0; //Nvm.MacGroup1.ChannelsTxPower;    // 0
    txConfig.MaxEirp = 16; //Nvm.MacGroup2.MacParams.MaxEirp;
    txConfig.AntennaGain = 2.15; //Nvm.MacGroup2.MacParams.AntennaGain;
    txConfig.PktLen = 23; //MacCtx.PktBufferLen;
#if (defined( LORAMAC_VERSION ) && (( LORAMAC_VERSION == 0x01000400 ) || ( LORAMAC_VERSION == 0x01010100 )))
    txConfig.NetworkActivation = Nvm.MacGroup2.NetworkActivation;  // 0
#endif /* LORAMAC_VERSION */

    //RegionTxConfig( Nvm.MacGroup2.Region, &txConfig, &txPower, &MacCtx.TxTimeOnAir );
    TimerTime_t TxTimeOnAir;
    RegionEU868TxConfigM( &txConfig, &txPower, &TxTimeOnAir );

    //int16_t Maxeirp = (int16_t) (txConfig.MaxEirp * 100);
    //int16_t gain = (int16_t) (txConfig.AntennaGain * 100);
    //MW_LOG(TS_ON, VLEVEL_M,  "chan:%i, DR:%i, Txpow:%i  Eirp:%i Gain:%i Len:%i Activ:%i\r\n", channel,
    //		txConfig.Datarate, txConfig.TxPower	, Maxeirp, gain, txConfig.PktLen, txConfig.NetworkActivation);

        // Send now
    //Radio.Send( MacCtx.PktBuffer, MacCtx.PktBufferLen );
	  uint8_t tx_buffer[64];
	  strncpy((char*)tx_buffer, "M", sizeof(tx_buffer) - 1);
	  tx_buffer[sizeof(tx_buffer) - 1] = '\0';

	  // ✅ UTILISER LE MIDDLEWARE
	  Radio.Send(tx_buffer, strlen((char*)tx_buffer));

    return status;
}


void configure_radio_parameters(void)
{
    // 1. Vérifier que la fréquence est supportée
    /*if (!Radio.CheckRfFrequency(868100000UL)) {
        LOG_ERROR("Fréquence 868.1 MHz non supportée");
        return;
    }*/

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
        0,                   // Power: 14 dBm
        0,                    // Frequency deviation (N/A pour LoRa)
        0,                    // Bandwidth (N/A pour LoRa)
        3,                    // Datarate: 128 chips
        1,                    // Coderate: 4/5
        8,                    // Preamble length
        false,                // Variable length
        true,                 // CRC enabled
        false,                // Frequency hopping 0:OFF
        0,                    // Hop period
        false,                // IQ not inverted
        3000                  // Timeout: 3 secondes
    );

    Radio.Sleep();

    LOG_INFO("Radio LoRa configuree: 868.1 MHz, SF7, 14 dBm");
}

void sendRadio()
{
    uint8_t tx_buffer[64];
    strncpy((char*)tx_buffer, "MESS_RADIO", sizeof(tx_buffer) - 1);
    tx_buffer[sizeof(tx_buffer) - 1] = '\0';

    LOG_INFO("Envoi LoRa via middleware: %s", tx_buffer);

    // ✅ UTILISER LE MIDDLEWARE
    Radio.Send(tx_buffer, strlen((char*)tx_buffer));

    // Le callback OnTxDone() sera appelé automatiquement
}


void sendRadio_HAL()
{
	uint8_t radio_status;
    uint8_t tx_buffer[64];
    strcpy((char*)tx_buffer, "MESS RADIO");  // 25ms en SF7

	// Vérifier que le radio est libre
	if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK)
	{
		if (!(radio_status & 0x01))
		{ // Pas en transmission

			// Écrire le message dans le buffer radio
			if (HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, tx_buffer, strlen((char*)tx_buffer)) == HAL_OK)
			{
				// Démarrer la transmission
				uint8_t tx_cmd = 0x83; // Commande TX
				if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, tx_cmd, NULL, 0) == HAL_OK)
				{
					LOG_INFO("Commande TX envoyée, attente...");

					// 4. Attendre avec polling
					uint32_t start_wait = HAL_GetTick();
					bool transmission_done = false;

					while ((HAL_GetTick() - start_wait) < 5000) // Max 5 secondes
					{
						if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK)
						{
							LOG_INFO("Statut pendant attente: 0x%02X", radio_status);

							if (radio_status & 0x08) // TX_DONE
							{
								LOG_INFO("✅ Transmission terminée avec succès !");
								transmission_done = true;
								break;
							}
							else if (radio_status & 0x10) // TX_TIMEOUT
							{
								LOG_ERROR("❌ Timeout transmission");
								break;
							}
						}

						osDelay(50); // Attendre 50ms
					}

					if (!transmission_done) {
						LOG_ERROR("❌ Transmission non terminée après 5s");
						//diagnose_radio_status(radio_status);
					}
				}
			    else {
					LOG_ERROR("Failed to start LoRa transmission");
				}
			} else {
				LOG_ERROR("Failed to write LoRa buffer");
			}
		} else {
			LOG_WARNING("LoRa radio busy");
		}
	} else {
		LOG_ERROR("Failed to read radio status");
	}
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

void debug_radio_configuration(void)
{
    LOG_INFO("=== DIAGNOSTIC CONFIGURATION RADIO ===");

    // Vérifier que configure_radio_parameters() est appelée
    LOG_INFO("Vérification configuration radio...");

    // Vérifier les paramètres critiques
    if (!Radio.CheckRfFrequency(868100000UL)) {
        LOG_ERROR("❌ Fréquence 868.1 MHz non supportée");
        return;
    }

    LOG_INFO("✅ Fréquence supportée");

    // Vérifier l'état du radio
    uint8_t radio_status;
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
        LOG_INFO("Radio status: 0x%02X", radio_status);
    }
}

void verify_radio_initialization(void)
{
    LOG_INFO("=== VÉRIFICATION INITIALISATION ===");

    // Vérifier que MX_SUBGHZ_Init() a été appelé
    LOG_INFO("Vérification MX_SUBGHZ_Init...");

    // Vérifier que MX_SubGHz_Phy_Init() a été appelé
    LOG_INFO("Vérification MX_SubGHz_Phy_Init...");

    // Vérifier l'état du radio
    uint8_t radio_status;
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
        LOG_INFO("Radio status: 0x%02X", radio_status);

        if (radio_status == 0) {
            LOG_ERROR("❌ Radio non initialisé (status = 0)");
        }
    } else {
        LOG_ERROR("❌ Impossible de lire le statut radio");
    }
}

void test_radio_progressive(void)
{
    LOG_INFO("=== TEST PROGRESSIF RADIO ===");

    // Étape 1 : Vérifier l'initialisation
    verify_radio_initialization();

    // Étape 2 : Vérifier la configuration
    debug_radio_configuration();

    // Étape 3 : Test simple (sans envoi)
    LOG_INFO("Test lecture registres...");
    uint8_t reg_value;
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x00, &reg_value) == HAL_OK) {
        LOG_INFO("Registre 0x00: 0x%02X", reg_value);
    }

    // Étape 4 : Test envoi sécurisé
    LOG_INFO("Test envoi sécurisé...");
    sendRadio();

    LOG_INFO("=== FIN TEST ===");
}

void diagnose_radio_deep_sleep(void)
{
    LOG_INFO("=== DIAGNOSTIC RADIO SLEEP PROFOND ===");

    // Vérifier tous les registres
    uint8_t reg_value;
    for (int reg = 0x00; reg <= 0x0F; reg++) {
        if (HAL_SUBGHZ_ReadRegister(&hsubghz, reg, &reg_value) == HAL_OK) {
            LOG_INFO("Registre 0x%02X: 0x%02X", reg, reg_value);
        } else {
            LOG_ERROR("❌ Impossible de lire registre 0x%02X", reg);
        }
    }

    // Essayer différentes commandes de réveil
    uint8_t wakeup_commands[] = {0x80, 0x81, 0x82, 0x83};
    for (int i = 0; i < 4; i++) {
        LOG_INFO("Tentative réveil avec commande 0x%02X", wakeup_commands[i]);
        HAL_SUBGHZ_ExecSetCmd(&hsubghz, wakeup_commands[i], NULL, 0);
        HAL_Delay(100);

        if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &reg_value) == HAL_OK) {
            LOG_INFO("Status après commande 0x%02X: 0x%02X", wakeup_commands[i], reg_value);
        }
    }
}

void test_radio_communication_simple(void)
{
    LOG_INFO("=== TEST COMMUNICATION RADIO SIMPLE ===");

    uint8_t reg_value;

    // ✅ TEST 1 : Lire le registre 0x01 (status)
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &reg_value) == HAL_OK) {
        LOG_INFO("✅ Communication OK - Registre 0x01: 0x%02X", reg_value);
    } else {
        LOG_ERROR("❌ Communication échouée - Registre 0x01");
    }

    // ✅ TEST 2 : Lire le registre 0x08 (configuration)
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x08, &reg_value) == HAL_OK) {
        LOG_INFO("✅ Communication OK - Registre 0x08: 0x%02X", reg_value);
    } else {
        LOG_ERROR("❌ Communication échouée - Registre 0x08");
    }
}

void test_radio_write_register(void)
{
    LOG_INFO("=== TEST ÉCRITURE REGISTRE ===");

    // ✅ TEST : Écrire dans un registre de configuration
    uint8_t test_value = 0x55;

    if (HAL_SUBGHZ_WriteRegister(&hsubghz, 0x02, test_value) == HAL_OK) {
        LOG_INFO("✅ Écriture OK - Valeur 0x%02X écrite", test_value);

        // Vérifier en relisant
        uint8_t read_value;
        if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x02, &read_value) == HAL_OK) {
            LOG_INFO("✅ Lecture OK - Valeur lue: 0x%02X", read_value);
            if (read_value == test_value) {
                LOG_INFO("✅ Communication radio fonctionnelle");
            } else {
                LOG_ERROR("❌ Valeur lue différente de celle écrite");
            }
        }
    } else {
        LOG_ERROR("❌ Écriture échouée");
    }
}

void check_radio_power_supply(void)
{
    LOG_INFO("=== DIAGNOSTIC ALIMENTATION RADIO ===");

    // Vérifier les registres de configuration d'alimentation
    uint8_t reg_value;

    // Registre 0x08 (configuration générale)
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x08, &reg_value) == HAL_OK) {
        LOG_INFO("Registre 0x08 (config): 0x%02X", reg_value);

        // Analyser les bits d'alimentation
        if (reg_value & 0x01) {
            LOG_INFO("✅ Bit 0: Alimentation active");
        } else {
            LOG_ERROR("❌ Bit 0: Alimentation inactive");
        }

        if (reg_value & 0x02) {
            LOG_INFO("✅ Bit 1: SMPS activé");
        } else {
            LOG_ERROR("❌ Bit 1: SMPS désactivé");
        }
    }

    // Registre 0x09 (configuration radio)
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x09, &reg_value) == HAL_OK) {
        LOG_INFO("Registre 0x09 (radio): 0x%02X", reg_value);

        if (reg_value == 0x01) {
            LOG_INFO("✅ Mode radio configuré");
        } else {
            LOG_ERROR("❌ Mode radio incorrect");
        }
    }
}

void test_radio_configuration_detailed(void)
{
    LOG_INFO("=== TEST CONFIGURATION RADIO DÉTAILLÉE ===");

    // Tester la fréquence
    if (Radio.CheckRfFrequency(868100000UL)) {
        LOG_INFO("✅ Fréquence 868.1 MHz supportée");
    } else {
        LOG_ERROR("❌ Fréquence 868.1 MHz non supportée");
    }

    // Tester la configuration LoRa
    LOG_INFO("Test configuration LoRa...");
    Radio.SetModem(MODEM_LORA);
    Radio.SetChannel(868100000UL);

    // Vérifier l'état après configuration
    uint8_t status;
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &status) == HAL_OK) {
        LOG_INFO("Status après config LoRa: 0x%02X", status);
    }
}

void test_radio_wakeup_methods(void)
{
    LOG_INFO("=== TEST MÉTHODES DE RÉVEIL ===");

    // Méthode 1 : Commande STANDBY
    LOG_INFO("Méthode 1: STANDBY (0x80)");
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, 0x80, NULL, 0);
    HAL_Delay(100);

    uint8_t status;
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &status) == HAL_OK) {
        LOG_INFO("Status après STANDBY: 0x%02X", status);
    }

    // Méthode 2 : Commande SET_RF_FREQUENCY
    LOG_INFO("Méthode 2: SET_RF_FREQUENCY (0x86)");
    uint32_t frequency = 868100000;
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, 0x86, (uint8_t*)&frequency, 4);
    HAL_Delay(100);

    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &status) == HAL_OK) {
        LOG_INFO("Status après SET_RF_FREQUENCY: 0x%02X", status);
    }

    // Méthode 3 : Commande SET_MODEM
    LOG_INFO("Méthode 3: SET_MODEM (0x8A)");
    uint8_t modem_config[] = {0x01, 0x00, 0x00, 0x00}; // LoRa
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, 0x8A, modem_config, 4);
    HAL_Delay(100);

    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &status) == HAL_OK) {
        LOG_INFO("Status après SET_MODEM: 0x%02X", status);
    }
}

void test_radio_direct_transmission(void)
{
    LOG_INFO("=== TEST TRANSMISSION DIRECTE ===");

    // 1. Réveiller le radio
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, 0x80, NULL, 0);
    HAL_Delay(100);

    // 2. Configurer la fréquence
    uint32_t frequency = 868100000;
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, 0x86, (uint8_t*)&frequency, 4);
    HAL_Delay(100);

    // 3. Configurer le mode LoRa
    uint8_t modem_config[] = {0x01, 0x00, 0x00, 0x00};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, 0x8A, modem_config, 4);
    HAL_Delay(100);

    // 4. Vérifier l'état
    uint8_t status;
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &status) == HAL_OK) {
        LOG_INFO("Status après configuration: 0x%02X", status);

        if (status != 0x00) {
            LOG_INFO("✅ Radio actif - Test transmission...");

            // 5. Essayer de transmettre
            uint8_t tx_buffer[64];
            strncpy((char*)tx_buffer, "TEST", 5);

            if (HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, tx_buffer, 4) == HAL_OK) {
                LOG_INFO("✅ Buffer écrit");

                if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, 0x83, NULL, 0) == HAL_OK) {
                    LOG_INFO("✅ Transmission démarrée");
                } else {
                    LOG_ERROR("❌ Échec démarrage transmission");
                }
            } else {
                LOG_ERROR("❌ Échec écriture buffer");
            }
        } else {
            LOG_ERROR("❌ Radio toujours inactif après configuration");
        }
    }
}

void check_radio_hardware_configuration(void)
{
    LOG_INFO("=== VÉRIFICATION CONFIGURATION HARDWARE ===");

    // Vérifier que SUBGHZ est bien configuré
    /*if (hsubghz.Instance == NULL) {
        LOG_ERROR("❌ Instance SUBGHZ non définie");
        return;
    }

    LOG_INFO("✅ Instance SUBGHZ: 0x%08X", (uint32_t)hsubghz.Instance);*/

    // Vérifier la configuration SPI
    LOG_INFO("BaudratePrescaler: %d", hsubghz.Init.BaudratePrescaler);

    // Vérifier les registres de configuration
    uint8_t reg_value;
    for (int reg = 0x08; reg <= 0x0C; reg++) {
        if (HAL_SUBGHZ_ReadRegister(&hsubghz, reg, &reg_value) == HAL_OK) {
            LOG_INFO("Registre 0x%02X: 0x%02X", reg, reg_value);
        }
    }
}
void reset_radio_completely(void)
{
}
void try_radio_wakeup_all_commands(void)
{
    LOG_INFO("=== ESSAI TOUTES LES COMMANDES DE RÉVEIL ===");

    // Essayer toutes les commandes possibles
    uint8_t wakeup_commands[] = {
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
        0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F
    };

    for (int i = 0; i < 16; i++) {
        LOG_INFO("Tentative réveil avec 0x%02X...", wakeup_commands[i]);

        if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, wakeup_commands[i], NULL, 0) == HAL_OK) {
            LOG_INFO("✅ Commande 0x%02X envoyée", wakeup_commands[i]);

            // Attendre un peu
            HAL_Delay(200);

            // Vérifier l'état
            uint8_t status;
            if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &status) == HAL_OK) {
                LOG_INFO("Status après 0x%02X: 0x%02X", wakeup_commands[i], status);

                if (status != 0x00) {
                    LOG_INFO("✅ Radio réveillé avec 0x%02X", wakeup_commands[i]);
                    return;
                }
            }
        } else {
            LOG_INFO("❌ Commande 0x%02X échouée", wakeup_commands[i]);
        }
    }

    LOG_ERROR("❌ Aucune commande de réveil ne fonctionne");
}
