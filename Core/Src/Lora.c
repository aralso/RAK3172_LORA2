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

#define CONCENTRATOR_ADDR           'H'

#define RX_delai	1000

double floor (double);
struct radio_TxParam_s radio_TxParam;
uint8_t frame_tx[3 + 2 + MESS_LG_MAX];
uint8_t cpt_process_lora_tx = 0;
struct lora_etat_s lora_etat;

lora_tx_state_t g_tx_state = TX_IDLE;
static out_message_t g_tx_msg;
static uint8_t g_cad_fail_count = 0; // essais CAD infructueux successifs
static uint8_t cpt_renvoi_message = 0;
static uint8_t nb_messages_envoyes=0;

static uint8_t lora_buff[MESS_BUFFER_SIZE];
static uint16_t lora_head = 0;
static uint16_t lora_tail = 0;

static osMutexId_t lora_bufferMutex;
static uint8_t g_lora_class = LORA_CLASS_B;
static uint32_t g_next_beacon_at_ms = 0;             // prochaine balise estimée
static volatile uint32_t g_lptim1_10s_since_beacon = 0; // nb déclenchements LPTIM1 depuis dernière balise

extern QueueHandle_t Event_QueueHandle;
extern osThreadId_t Uart_TX_TaskHandle;

// Configuration réseau
#ifndef My_Address
#include "communication.h"
#endif


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

static TimerTime_t GetTimeOnAir( int8_t datarate, uint16_t pktLen )
{
    int8_t phyDr = DataratesEU868[datarate];
    uint32_t bandwidth = RegionCommonGetBandwidth( datarate, BandwidthsEU868 );
    TimerTime_t timeOnAir = 0;

    if( datarate == DR_7 )
    { // High Speed FSK channel
        timeOnAir = Radio.TimeOnAir( MODEM_FSK, bandwidth, phyDr * 1000, 0, 5, false, pktLen, true );
    }
    else
    {
        timeOnAir = Radio.TimeOnAir( MODEM_LORA, bandwidth, phyDr, 1, 8, false, pktLen, true );
    }
    return timeOnAir;
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

void SetRadioTxParam (uint8_t param, uint8_t val)
{

	if (param==1)  // power
		radio_TxParam.power = val;
	if (param==2)  // bandwith
		radio_TxParam.bandwidth = val;
	if (param==3)  // Spread factor
		radio_TxParam.SF = val;
	if (param==4)  // coderate
		radio_TxParam.coderate = val;
	if (param==5)  // preamble length
		radio_TxParam.preambleLen = val;
	if (param==6)  // timeout
		radio_TxParam.timeout = val;
	if (param==7)  // DR    0 à 7
	{
		radio_TxParam.DR = val;
		radio_TxParam.SF = DataratesEU868[val];
		radio_TxParam.bandwidth  = RegionCommonGetBandwidth( val, BandwidthsEU868 );
	}
	if (param==8)  // freq
	{
		radio_TxParam.freq = (val+860)*1000000;
		Radio.SetChannel (radio_TxParam.freq + radio_TxParam.channel*100000);
	}
	if (param==9)  // channel
	{
		radio_TxParam.channel = val;
		Radio.SetChannel (radio_TxParam.freq + radio_TxParam.channel*100000);
	}
	if (param < 8)
	{
		Radio.SetTxConfig(radio_TxParam.modem, radio_TxParam.power, radio_TxParam.fdev, radio_TxParam.bandwidth, \
				radio_TxParam.SF, radio_TxParam.coderate, radio_TxParam.preambleLen, radio_TxParam.fixLen,\
				radio_TxParam.crcOn, radio_TxParam.freqHopOn, radio_TxParam.hopPeriod, radio_TxParam.iqInverted, \
				radio_TxParam.timeout);
	}

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
        0,                    // Bandwidth 0:125k 1:250k 2:500k
        12,                    // SF Datarate: 6=64chips, 7=128 chips 12=4096chips
        1,                    // Coderate: 1:4/5
        8,                    // Preamble length
        false,                // Variable length : 0:variable
        true,                 // CRC enabled : 0;off 1:CRC
        false,                // Frequency hopping 0:OFF
        0,                    // Hop period
        false,                // IQ not inverted
        3000                  // Timeout: 3 secondes
    );
	radio_TxParam.modem = MODEM_LORA;   // 0:FSK  1:LORA
	radio_TxParam.power = 0;			// en dBm
	radio_TxParam.fdev = 0;				// 25000 pour FSK
	radio_TxParam.bandwidth = 0;		// Bandwidth 0:125k 1:250k 2:500k
	radio_TxParam.SF = 12;			    // SFDatarate: 6=64chips, 7=128 chips 12=4096chips
	radio_TxParam.coderate = 1;			// Coderate 1:4/5  2:4/6
	radio_TxParam.preambleLen = 8;		// En octets (rajouter 4)
	radio_TxParam.fixLen = 0;			// Fixed length : 0:variable
	radio_TxParam.crcOn = true;			// CRC enabled : 0;off 1:CRC
	radio_TxParam.freqHopOn = false;	// Frequency hopping 0:OFF
	radio_TxParam.hopPeriod = 0;
	radio_TxParam.iqInverted = 0;
	radio_TxParam.timeout = 4000;
	radio_TxParam.freq = 868000000;
	radio_TxParam.channel = 1;			// channel de 100kHz
	radio_TxParam.DR = 6	;			// DR 0(SF12) à 7(FSK)

	// DR0:SF12 BW125   DR1:SF11 BW125   DR5:SF7 BW125   DR6:SF7 BW250    DR7:FSK
    Radio.Sleep();

    LOG_INFO("Radio LoRa configuree: 868 MHz, SF7, 0 dBm");
}

void PrintRadioTxParam(void)
{
	//LOG_INFO ()
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

// =====================
//  Gestion haute-niveau
// =====================




static void lora_build_frame(const out_message_t* src, lora_TxPacket* tx)
{
    tx->destAddr = CONCENTRATOR_ADDR;
    tx->reseauAddr = ReseauAddr;
    tx->len = src->length;
    tx->param = src->param;
    if (tx->len > MESS_LG_MAX) tx->len = MESS_LG_MAX;
    memcpy(tx->payload, src->data, tx->len);
}

static void lora_send_packet(const lora_TxPacket* tx)
{
    // Construction sur le fil: [dest][reseau][My_Address][param][len][payload]

    uint16_t idx = 0;
    frame_tx[idx++] = tx->destAddr;
    frame_tx[idx++] = tx->reseauAddr;
    frame_tx[idx++] = My_Address;
    frame_tx[idx++] = tx->param;
    frame_tx[idx++] = tx->len;
    memcpy(&frame_tx[idx], tx->payload, tx->len);
    idx += tx->len;

    Radio.Send(frame_tx, idx);
}


void lora_radio_init(void)
{
    configure_radio_parameters();

    // Déterminer la classe à partir du define CLASS
#ifdef CLASS
#if CLASS == LORA_CLASS_A
    g_lora_class = LORA_CLASS_A;
    Radio.Sleep();
#elif CLASS == LORA_CLASS_B
    g_lora_class = LORA_CLASS_B;
    Radio.Sleep();
#elif CLASS == LORA_CLASS_C
    g_lora_class = LORA_CLASS_C;
    Radio.Rx(0);
#else
    g_lora_class = LORA_CLASS_B;
    Radio.Sleep();
#endif
#else
    g_lora_class = LORA_CLASS_B;
    Radio.Sleep();
#endif
}

void lora_set_class(uint8_t lora_class)
{
    g_lora_class = lora_class;
}



void lora_tx_state_step(void)
{
    switch (g_tx_state) {
    case TX_IDLE: {
        bool msg_loaded = false;
        cpt_renvoi_message=0;
        if (!msg_loaded) {
            if (mess_LORA_dequeue(&g_tx_msg) == 0) {
                msg_loaded = true;
                nb_messages_envoyes ++;
                g_tx_state = TX_WAIT_CAD;
            }
            else // pas/plus de message dans pile d'envoi
            {
                if (g_lora_class == LORA_CLASS_C)
                	Radio.Rx(0);
                else
                	if (nb_messages_envoyes) // si on a deja envoye au moins 1 message => RX
                    	Radio.Rx(RX_delai);
                	else
                		Radio.Sleep();  // rien à envoyer => fin
                break;
            }
        }  // continue directement sur TX_WAIT_CAD
    }
    case TX_WAIT_CAD:
        // attend EVENT_CAD_DONE dans Appli
        // verifier si fenetre d'envoi ok
        if (g_lora_class == LORA_CLASS_B) {
            uint32_t elapsed_ms = 0, remaining_ms = 0;
            lora_get_time_since_last_and_to_next(&elapsed_ms, &remaining_ms);
            TimerTime_t need = 0;
            need += GetTimeOnAir(radio_TxParam.DR, (uint16_t)(g_tx_msg.length + 5+3+10));  // avec ack et reponse
            need += 100; // marge
            if (remaining_ms > 0 && (TimerTime_t)remaining_ms <= need) {
                uint32_t delay_ms = remaining_ms + 1000; // 1000ms après balise
                lptim2_schedule_ms(delay_ms);
                break;
            }
        }
        // vérifier si Canal libre
        Radio.StartCad();
        break;
    case TX_SENDING: {
        lora_TxPacket tx;
        lora_build_frame(&g_tx_msg, &tx);
        lora_send_packet(&tx);
        break;
    }
    case TX_SENT: {
        if (g_tx_msg.param & 0x10)  // Ack non requis
        {  // succes d'envoi
        	lora_etat.mess_envoy_ok++;
            if (g_tx_msg.param & 0x1)  // dernier message
            {
            	// fin phase transmission , début phase réception
            	g_tx_state = RX_RESPONSES;
                event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
                xQueueSend(Event_QueueHandle, &evt, 0);
            }
            else  // encore des messages a envoyer
            {
            	// recharger un nouveau message a envoyer
            	g_tx_state = TX_IDLE;
                event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
                xQueueSend(Event_QueueHandle, &evt, 0);
            }
        }
        else // attendre ack requis
        {
        	Radio.Rx(RX_delai);
			g_tx_state = TX_WAIT_ACK;
			// Programmer le timeout ACK via LPTIM2 (non bloquant et compatible STOP)
			//lptim2_schedule_ms(2000);
        }
        break; }
    case TX_ACK_RECU: { // success avec ack
    	lora_etat.mess_envoy_ok++;
        if (g_tx_msg.param & 0x1)  // dernier message
        {
        	// fin phase transmission , début phase réception
        	g_tx_state = RX_RESPONSES;
            event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
            xQueueSend(Event_QueueHandle, &evt, 0);
        }
        else  // encore des messages a envoyer
        {
        	// recharger un nouveau message a envoyer
        	g_tx_state = TX_IDLE;
            event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
            xQueueSend(Event_QueueHandle, &evt, 0);
        }
    	break;
    }
    case TX_WAIT_ACK:  // reçu un timeout => ack non reçu => on renvoie le message
        // attend EVENT_LORA_ACK_TIMEOUT ou un RX OnRxDone traitant l'ACK
    	//verif compteur_envoi avec param
    	cpt_renvoi_message++;
    	//si ok on renvoie le message
    	uint8_t val = (g_tx_msg.param & 6) > 1;  // bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)
    	if ( ((val==1) && (cpt_renvoi_message<2)) || ((val==2) && (cpt_renvoi_message<5)) )  // bits 1 et 2
		{
    		lora_etat.mess_renvoyes++;
			g_tx_state = TX_WAIT_CAD;
			event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
			xQueueSend(Event_QueueHandle, &evt, 0);
		}
    	else //fin phase transmission , début phase réception
    	{
    		// supression message
    		lora_etat.mess_envoy_supp++;
    		// puis passage en RX
        	g_tx_state = RX_RESPONSES;
            event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
            xQueueSend(Event_QueueHandle, &evt, 0);
    	}
        break;

    case RX_RESPONSES:  // fin phase transmission , début phase réception
    	nb_messages_envoyes = 0;
        if (g_lora_class == LORA_CLASS_C) Radio.Rx(0); else Radio.Rx(RX_delai);
        //g_tx_state = TX_IDLE;
        break;
    }
}

void lora_handle_event_tx(void)
{
    // Déclenche la machine d’états à partir d’un EVENT_LORA_TX
    lora_tx_state_step();
}

void lora_tx_on_cad_result(bool channelBusy)
{
    if (g_tx_state == TX_WAIT_CAD) {
        if (channelBusy) {
            // réessayer plus tard: revenir à IDLE pour redéclencher TX_STEP par l’appli
            if (++g_cad_fail_count >= 5) {
                // journaliser réseau occupé après 5 échecs consécutifs
                log_write('E', log_w_reseau_occupe, 0x00, 0x00, "RadOccup");
                g_cad_fail_count = 0;
            }
            //g_tx_state = TX_IDLE;  // reste en TX_WAIT_CAD
        } else {
            g_cad_fail_count = 0; // reset au succès CAD
            g_tx_state = TX_SENDING;
        }
        lora_tx_state_step();
    }
    else
    	LOG_ERROR("cad done inconnu");
}

// Point d’entrée depuis EVENT_LORA_RX
void lora_handle_event_rx(void)
{
    // Ici, on suppose que la couche IRQ a copié le paquet reçu en tampon
    // et déclenché EVENT_LORA_RX. On appelle ensuite votre traitement existant.
    // À adapter avec votre callback de réception réel.
    // Exemple nominal (à remplacer par buffer réel RX):
    uint8_t rx_frame[3 + 2 + MESS_LG_MAX];
    uint16_t rx_len = 0; // TODO: longueur réelle
    if (rx_len < 5) return;

    // Décodage minimal et filtrage destinataire
    lora_RxPacket rxp;
    rxp.destAddr = rx_frame[0];
    rxp.reseauAddr = rx_frame[1];
    rxp.rssi = 0;
    rxp.param = rx_frame[3];
    rxp.len = rx_frame[4];
    if (rxp.len > MESS_LG_MAX) rxp.len = MESS_LG_MAX;
    memcpy(rxp.payload, &rx_frame[5], rxp.len);

    uint8_t dst = rxp.destAddr & 0x7F;
    if ((dst != My_Address) && (dst != LORA_BROADCAST_ADDR))
        return; // pas destiné

    // Appeler traitement applicatif existant
    traitement_rx(rxp.payload, rxp.len);

    // Répondre ACK si requis
    if ((rxp.param & 0x10) == 0) { // bit4: ack non requis (0 => ACK requis)
        uint8_t ack[3] = { My_Address, ReseauAddr, 1 };
        Radio.Send(ack, 3);
    }

    // Continuer en RX si classe C, sinon dormir
    if (g_lora_class == LORA_CLASS_C) {
        Radio.Rx(0); // continu
    } else {
        Radio.Sleep();
    }
}

void lora_handle_classb_beacon_event(void)
{
    // Petite fenêtre pour détecter rapidement la balise
    Radio.Rx(200);
}

// =======
// Bridges
// =======
void lora_on_tx_done(void)
{
    event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
    g_tx_state = TX_SENT;
    xQueueSend(Event_QueueHandle, &evt, 0);
}

void lora_on_rx_done(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    (void)rssi; (void)snr;

    lora_etat.radio_rx++;

    // Décodage minimal: [dest][reseau][emetteur][param][len][payload]
    if (size < 5) {
        relance_rx(1);
        return;
    }

    uint8_t dest = payload[0] & 0x7F;
    if (((dest != My_Address) && (dest != LORA_BROADCAST_ADDR)) || (payload[1] != ReseauAddr)) {
        relance_rx(1);
        return;
    }

    // message reçu pour ce node
    uint8_t len = payload[4];
    message_recu.length = payload[4];
    message_recu.rssi = rssi;
    message_recu.snr = snr;
    message_recu.param = payload[3];
    if (len > MESS_LG_MAX) len = MESS_LG_MAX;
    uint8_t param = payload[3];

    // Contrainte IRQ: si on est en contexte IRQ, éviter traitement lourd
    // Heuristique: utiliser l’API FromISR uniquement dans IRQ, sinon traitement direct
    //BaseType_t inIsr = xPortIsInsideInterrupt();
    // Détection balise: "BB" au début du payload
    bool is_beacon = (dest == LORA_BROADCAST_ADDR && len >= 2 && len <= 3 && payload[5] == 'B' && payload[6] == 'B');
    //if (!inIsr) {
	if (is_beacon) {
		// Prochaine balise attendue dans 3 minutes (basé sur LPTIM epoch si dispo)
		uint32_t now_s = lptim_get_seconds();
		g_next_beacon_at_ms = (now_s * 1000) + 180000;
		g_lptim1_10s_since_beacon = 0;
		// Recalage fin: viser un réveil très proche de la prochaine balise
		// Marge initiale 10 ms, pourra être ajustée dynamiquement
		lptim_program_compare_advance_ms(10);

		// Si présence d’une adresse de destinataire immédiat (payload[7])
		if (len >= 3) {
			// Si c’est notre adresse, rester en RX continu pour recevoir
			if (payload[7] == My_Address) {
				relance_rx(1);
			}
		}
	}
	else
	{
		// Répondre ACK si requis
		if ((dest != LORA_BROADCAST_ADDR) && ((param & 0x10) == 0)) { // bit4: ack non requis (0 => ACK requis)
			uint8_t ack[6] = { CONCENTRATOR_ADDR, ReseauAddr, My_Address, 0,1,'A' };
			Radio.Send(ack, 6);
		}

		// Traitement applicatif standard
		memcpy(message_recu.data, &payload[5], len);
		event_t evt = { EVENT_LORA_RX, SOURCE_LORA, len };
		xQueueSendFromISR(Event_QueueHandle, &evt, 0);

		uint8_t vide = mess_LORA_dequeue_fictif();  // 0:mess
		if (param & 1)  // dernier RX
		{
			g_tx_state = TX_IDLE;
			if (!vide)
			{
		        event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
		        xQueueSend(Event_QueueHandle, &evt, 0);
			}
		}
		else // continue écoute
			relance_rx(1);
	}


    // Continuer en RX en classe C
    if (g_lora_class == LORA_CLASS_C) {
        Radio.Rx(0);
    } else {
        Radio.Sleep();
    }
}

void lora_on_tx_timeout(void)
{
    // Relancer RX et notifier
	lora_etat.lora_tx_timeout++;
	g_tx_state = TX_IDLE;

	if (nb_messages_envoyes) nb_messages_envoyes--;
    event_t evt = { EVENT_ERROR, SOURCE_LORA, nb_messages_envoyes };
	nb_messages_envoyes = 0;
    xQueueSend(Event_QueueHandle, &evt, 0);
}

void lora_on_rx_timeout(void)
{   // soit on attendait un Ack non reçu, soit on attendait un message non reçu
	if (g_tx_state == TX_WAIT_ACK)  // Ack non reçu
	{
        event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
        xQueueSend(Event_QueueHandle, &evt, 0);
	}
	else  // pas de message a recevoir => passage en transmission
	{
		g_tx_state = TX_IDLE;
        event_t evt = { EVENT_LORA_TX_STEP, SOURCE_LORA, 0 };
        xQueueSend(Event_QueueHandle, &evt, 0);
	}
}

void lora_on_rx_error(void)
{
	lora_etat.lora_rx_error++;

    event_t evt = { EVENT_RELANCE_RX, 6, nb_messages_envoyes };
	nb_messages_envoyes = 0;
    xQueueSend(Event_QueueHandle, &evt, 0);
}

void lora_on_cad_done(bool channelActivityDetected)
{
    event_t evt = { EVENT_CAD_DONE, SOURCE_LORA, channelActivityDetected ? 1 : 0 };
    xQueueSend(Event_QueueHandle, &evt, 0);
}

void relance_rx(uint8_t actif)
{
    event_t evt = { EVENT_RELANCE_RX, 1, actif };
    xQueueSend(Event_QueueHandle, &evt, 0);
}

void relance_radio_rx(uint8_t actif)
{
    if (g_lora_class == LORA_CLASS_C)
        Radio.Rx(0);
    else
    {
    	if (actif)
            Radio.Rx(RX_delai);
    	else
    		Radio.Sleep();
    }
	nb_messages_envoyes = 0;

}


// Ajout d’un message LORA dans la queue
uint8_t mess_LORA_enqueue(out_message_t* mess)
{

    if ((mess->length < 5) || (mess->length > MESS_LG_MAX))
    {
    	return 1;
    }

    uint16_t total_size = mess->length+4;

    if (lora_head >= MESS_BUFFER_SIZE) {
            lora_head = 0; // Reset si corruption
            lora_tail = 0;
    }
    uint32_t start_time = HAL_GetTick();

    uint16_t free_space;

    if (lora_head >= lora_tail)
        free_space = MESS_BUFFER_SIZE - (lora_head - lora_tail) - 1;
    else
        free_space = (lora_tail - lora_head) - 1;

    /*HAL_Delay(10);
    char uart_msg[50];
    snprintf(uart_msg, sizeof(uart_msg), "Uart send1: %i \r\n", free_space);
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), 3000); \
    HAL_Delay(10);*/
    //UART_SEND("Send1\n\r");


	 while (free_space < (uint16_t)(total_size + 10))
	 {
		    /*HAL_Delay(10);
		    char uart_msg[50];
		    snprintf(uart_msg, sizeof(uart_msg), "Uart Att: %i \r\n", free_space);
		    HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), 3000); \
		    HAL_Delay(10);*/
	        //UART_SEND("SendAtt\n\r");
        osDelay(100);  // Attendre 100ms
		if ((HAL_GetTick() - start_time) > 2000)
		{
			//LOG_ERROR("Queue full timeout after %lu ms", 2000);
		    osMutexRelease(lora_bufferMutex);
		    log_write('E', log_w_err_uart_bloque, 0x02, 0x03, "uartRxBl");
		    return 2;  // Timeout
		}
	    if (lora_head >= lora_tail)
	        free_space = MESS_BUFFER_SIZE - (lora_head - lora_tail) - 1;
	    else
	        free_space = (lora_tail - lora_head) - 1;
	 }
    //UART_SEND("Send2\n\r");

	osStatus_t status = osMutexAcquire(lora_bufferMutex, 5000);
	if (status != osOK) return 3;

    uint16_t head_prov = lora_head;

    uint8_t* mess_ptr = (uint8_t*)mess;
        for (uint16_t i = 0; i < total_size; i++) {
            lora_buff[head_prov] = mess_ptr[i];
            head_prov = (head_prov + 1) % MESS_BUFFER_SIZE;
        }

    lora_head = head_prov;

	//osDelay(100);
	LOG_INFO("enqueuelora:head:%d tail:%d mess:%s len:%i", lora_head, lora_tail, mess->data, mess->length);
    //osDelay(100);
    osMutexRelease(lora_bufferMutex);

    if (g_tx_state == TX_IDLE)  // notification d'envoi si pas d'envoi en cours
    {
		event_t evt = { EVENT_LORA_TX, 0, 0 };
		if (xQueueSendFromISR(Event_QueueHandle, &evt, 0) != pdPASS)
			{ code_erreur = ISR_callback; 	err_donnee1 = 7; }
    }
    return 0;
}

// Extraction d’un message LORA de la queue
// retour : 0:message dispo pour l'envoi, 1:fifo vide, sinon:erreur
uint8_t mess_LORA_dequeue(out_message_t* mess)
{
	osStatus_t status = osMutexAcquire(lora_bufferMutex, 5000);
	if (status != osOK) return 4;

	uint16_t tail_prov = lora_tail;

    if (lora_head == lora_tail) {
        osMutexRelease(lora_bufferMutex);
        return 1; // FIFO vide
    }


    uint16_t size = lora_buff[lora_tail];


	//osDelay(300);
	//LOG_INFO("dequeue1:head:%d tail:%d", head, tail);
    //osDelay(300);

    // Vérif longueur valide
	if ((size < 5) || (size > MESS_LG_MAX) || tail_prov >= MESS_BUFFER_SIZE)
	{
		lora_head=0;
		lora_tail=0;
		osMutexRelease(lora_bufferMutex);
		return 2; // corruption détectée
	}

	// Vérif que les données tiennent dans la FIFO actuelle
	uint16_t available = (lora_head >= tail_prov) ?
						 (lora_head - tail_prov) :
						 (MESS_BUFFER_SIZE - (tail_prov - lora_head));

	if (available < size) {
		lora_head=0;
		lora_tail=0;
		osMutexRelease(lora_bufferMutex);
		return 3; // corruption : message incomplet
	}

		uint8_t* mess_ptr = (uint8_t*)mess;
			for (uint16_t i = 0; i < size+4; i++) {
				mess_ptr[i] = lora_buff[tail_prov];
				tail_prov = (tail_prov + 1) % MESS_BUFFER_SIZE;
			}
		lora_tail = tail_prov;

		//osDelay(300);
		//LOG_INFO("dequeue2:head:%d tail:%d lg:%d", head, tail, *len);
		LOG_INFO("dequeuelora:head:%d tail:%d mess:%s len:%i", lora_head, lora_tail, mess->data, mess->length);

		//osDelay(300);

    // Analyser s'il y a un autre message à envoyer dans la pile
    // TODO  à faire
    // Si oui, mettre le bit 0 de mess.param à 0, sinon mettre 1

    osMutexRelease(lora_bufferMutex);
    return 0;
}


uint8_t mess_LORA_dequeue_fictif(void)
{

    if (lora_head == lora_tail) {
        return 1; // FIFO vide
    }

    uint16_t size = lora_buff[lora_tail];

    // Vérif longueur valide
	if ((size < 5) || (size > MESS_LG_MAX) || lora_tail >= MESS_BUFFER_SIZE)
	{
		return 2; // corruption détectée
	}

	// Vérif que les données tiennent dans la FIFO actuelle
	uint16_t available = (lora_head >= lora_tail) ?
						 (lora_head - lora_tail) :
						 (MESS_BUFFER_SIZE - (lora_tail - lora_head));

	if (available < size) {
		return 3; // corruption : message incomplet
	}

    return 0;
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

// ==============================
//  Comptage LPTIM1 / Calculs B
// ==============================
void lora_on_lptim1_10s_tick(void)
{
    g_lptim1_10s_since_beacon++;
}

void lora_get_time_since_last_and_to_next(uint32_t* elapsed_ms_since_last,
                                          uint32_t* remaining_ms_to_next)
{
    if (elapsed_ms_since_last == NULL || remaining_ms_to_next == NULL) return;

    uint32_t now_ms = lptim_get_seconds() * 1000U;

    if (g_next_beacon_at_ms == 0)
    {
        *elapsed_ms_since_last = g_lptim1_10s_since_beacon * 10000U;
        *remaining_ms_to_next = 0;
        return;
    }

    uint32_t last_beacon_ms = (g_next_beacon_at_ms >= 180000U) ? (g_next_beacon_at_ms - 180000U) : 0U;
    *elapsed_ms_since_last = (now_ms >= last_beacon_ms) ? (now_ms - last_beacon_ms) : 0U;
    *remaining_ms_to_next = (g_next_beacon_at_ms > now_ms) ? (g_next_beacon_at_ms - now_ms) : 0U;
}
