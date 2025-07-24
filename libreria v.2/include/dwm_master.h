#ifndef DWM_SPI_MASTER_RPI_H
#define DWM_SPI_MASTER_RPI_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>      
#include <unistd.h>    
#include <sys/ioctl.h>  
#include <linux/spi/spidev.h> 
#include <stddef.h>

#define START_BYTE 0xAA


/*
Tutti i possibili comandi che possono essere inviati al dispositivo slave (dwm1001-dev)
che verranno processati e che, a seconda del comando, prevederanno una risposta.
*/
#define CMD_GET_DISTANCES         0x01

#define CMD_SET_MODE_INIT         0x10
#define CMD_SET_MODE_RESP         0x11

#define CMD_SET_ID                0x20 // Seguito da 8 byte ID

#define CMD_ENABLE_ANCHOR         0x30 // Seguito da 8 byte ID
#define CMD_DISABLE_ANCHOR        0x31 // Seguito da 8 byte ID

#define CMD_ENTER_CONFIG_MODE     0x40 // Comando per entrare in modalità configurazione e settare parametri come ID comunicazioni
#define CMD_EXIT_CONFIG_MODE      0x41 // Esci dalla modalità di configurazione e salva le modifiche
#define CMD_SET_NUM_DEVICES       0x42 // Imposta il numero di dispositivi con cui si vuole comunicare
#define CMD_SET_DEVICE_ID_AT      0x43 // Imposta l'id del dispositivo all'index desiderato nella lista di ID
#define CMD_GET_NUM_DEVICES       0x44 // Restituisce il numero di dispositivi abilitati

#define CMD_MEASURE_DISTANCE      0x50 // Esegue misurazioni multiple e restituisce media, dato id
#define CMD_MEASURE_ALL_DISTANCES 0x51 // Misura le distanze average da tutti i dispositivi settati

#define CMD_SET_NLOS_MODE         0x60 // comando che permette di setuppare la modalità NLOS usando pacchetti più grandi
#define CMD_SET_LOS_MODE          0x61 // imposta a LoS con pacchetti più piccoli
#define CMD_SET_ANTENNA_TX_DELAY  0x62 // Permette di modificare il delay dell'antenna per calibrazione in transmission
#define CMD_SET_ANTENNA_RX_DELAY  0x63 // idem ma in ricezione

#define CMD_GET_INFO              0xFE
#define CMD_GET_HELP              0xFF

// --- Id risposte ---
#define RESP_ACK 0x01
#define RESP_NACK 0x00

#define MAX_DWM_RESPONDERS 16 // Massimo numero atteso dal DWM, modificabile in caso di modifica
// al firmware base del dispositivo

/*-----Strutture di Utilità-------*/

// --- Struttura per contenere i dati delle distanze ---
typedef struct {
  uint8_t id; // Id del dispositivo 
  double distance; // Distanza ottenuta
  int valid; // Flag booleana
} ResponderInfo;
 
// -- Struttura per misurazioni medie --
typedef struct {
  uint64_t id; // Id del dispositivo misurato
  double average_distance; // Distanza media ottenuta
  uint8_t samples_count; // numero di misurazioni effettuate
  uint8_t requested_samples; // numero richiesto originariamente
  int valid; // flag booleana che indica se le misurazioni sono valide
} AverageMeasurement;

// -- Struttura per info di configurazione --
typedef struct {
  uint8_t device_mode; // 0 initiator, 1 responder
  uint64_t device_id; // id del dispositivo
  uint8_t config_mod_active; // 1 se in modalità configurazione
} DeviceInfo;

/*----Funzioni di inizializzazione e gestione UART -----*/


/** 
 * @brief Inizializza l'interfaccia UART.
 * 
 * @param device Il nome del dispositivo UART (es. "/dev/ttyS0").
 * @param baudrate La velocità di trasmissione in baud.
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
*/
int dwm_uart_init(const char* device, uint32_t baudrate);

/**
 * @brief Chiude l'interfaccia UART.
 */
int dwm_uart_close(void);

/**
 * @brief Invia pacchetto sulla UART.
 * 
 * @param command Il comando da inviare.
 * @param payload Il payload del pacchetto.
 * @param payload_len La lunghezza del payload.
 *
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_send_packet(uint8_t command, const uint8_t* payload, size_t payload_len);

/**
 * @brief Riceve un byte da UART, con timeout.
 * 
 * @param timeout_ms Timeout in millisecondi.
 * 
 * @return Il byte ricevuto, o -1 in caso di timeout o errore.
 */
int read_byte_uart(int timeout_ms);

/**
 * @brief Riceve un pacchetto completo dalla UART.
 * 
 * @param resp_buffer Buffer in cui memorizzare la risposta.
 * @param max_len Lunghezza massima del buffer di risposta.
 * @param out_len Puntatore a variabile in cui memorizzare la lunghezza del pacchetto ricevuto.
 * @param timeout_ms Timeout in millisecondi.
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_receive_packet(uint8_t* resp_buffer, uint16_t max_len, uint16_t* out_len, int timeout_ms);

/**
 * @brief Invia un comando semplice con risposta ACK/NACK.
 * 
 * @param command Il comando da inviare.
 * @param payload Il payload del comando.
 * @param payload_len La lunghezza del payload.
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_simple_cmd_ack(uint8_t command, const uint8_t* payload, size_t payload_len);


/*--- Funzioni di comunicazione effettive per DWM1001-Dev */

/**
 * @brief Imposta il dispositivo come iniziatore.
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_initiator(void);

/**
 * @brief Imposta il dispositivo come rispondente.
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_responder(void);

/**
 * @brief Imposta l'ID del dispositivo.
 * 
 * @param new_id Il nuovo ID da impostare.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_id(uint64_t new_id);

/**
 * @brief Abilita un'ancora specifica.
 * 
 * @param anchor_id L'ID dell'ancora da abilitare.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_enable_device(uint64_t anchor_id);

/**
 * @brief Disabilita un'ancora specifica.
 * 
 * @param anchor_id L'ID dell'ancora da disabilitare.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_disable_device(uint64_t anchor_id);

/**
 * @brief Imposta il dispositivo in modalità configurazione.
 * 
 *  @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_enter_config_mode(void);

/**
 * @brief Esce dalla modalità configurazione e applica le modifiche.
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_exit_config_mode(void);

/**
 * @brief Imposta il numero di dispositivi con cui comunicare.
 * 
 * @param num_devices Il numero di dispositivi.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_num_devices(uint8_t num_devices);

/**
 * @brief Imposta l'ID del dispositivo all'indice specificato.
 * 
 * @param index L'indice dell'array dei dispositivi.
 * @param device_id L'ID del dispositivo da impostare.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_device_id_at(uint8_t index, uint64_t device_id);

/**
 * @brief Imposta gli ID dei dispositivi dati un array di ID.
 * 
 * @param device_ids Array di ID a 64 bit da impostare.
 * @param num_devices Numero di dispositivi.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_devices_id(uint64_t* device_ids, uint8_t num_devices);

/**
 * @brief Esegue misurazioni verso un dispositivo specifico e restituisce la media.
 * 
 * @param target_id L'ID del dispositivo a cui si vuole effettuare la misurazione.
 * @param num_samples Numero di campioni di misurazioni da effettuare.
 * @param result Puntatore a struttura AverageMeasurement popolata.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_measure(uint64_t target_id, uint8_t num_samples, AverageMeasurement* result);

/**
 * @brief Esegue misurazioni verso tutti i dispositivi configurati e restituisce le medie.
 * 
 * @param num_samples Numero di campioni di misurazioni da effettuare.
 * @param results Array di AverageMeasurement popolato.
 * @param max_results Dimensione massima dell'array results.
 * @param out_valid_count Numero di misurazioni valide ottenute.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_measure_all(uint8_t num_samples, AverageMeasurement* results, int max_results, uint8_t* out_valid_count);

/**
 * @brief Imposta il dispositivo in modalità NLOS (Non Line of Sight).
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_nlos_mode(void);

/**
 * @brief Imposta il dispositivo in modalità LOS (Line of Sight).
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_los_mode(void);

/**
 * @brief Imposta il delay dell'antenna in modalità trasmissione.
 * 
 * @param delay Il delay da impostare in microsecondi.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_antenna_tx_delay(uint16_t delay);

/**
 * @brief Imposta il delay dell'antenna in modalità ricezione.
 * 
 * @param delay Il delay da impostare in microsecondi.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_antenna_rx_delay(uint16_t delay);

/**
 * @brief Ottiene le informazioni sul dispositivo.
 * 
 * @param info Puntatore a struttura DeviceInfo in cui memorizzare le informazioni.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_get_info(DeviceInfo* info);

#if 0
/**
 * @brief Imposta id del dispositivo (64 bit) connesso via SPI.
 * 
 * @param new_id Id da impostare
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_id(uint64_t new_id);

/**
 * @brief Abilita un'ancora specifica, ovvero permette di abilitare
 * un dispositivo nell'array di dispositivi abilitati per la comunicazione
 * sul dwm1001-dev.
 * 
 * @param anchor_id del dispositivo da abilitare
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_enable_device(uint64_t anchor_id);

/**
 * @brief Disabilita un'ancora specifica, disabilitandola nell'array di dispositivi abilitati sul
 * dwm1001-dev.
 * 
 * @param anchor_id del dispositivo da disabilitare
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_disable_device(uint64_t anchor_id);

/**
 * @brief Imposta numero dispositivi attualmente in funzione con cui effettuare la comunicazione
 * 
 * @param num_devices numero dispositivi
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_num_devices(uint8_t num_devices);

/**
 * @brief Imposta id del dispositivo dato un certo index
 * dell'array dei dispositivi sul dwm1001-dev
 * 
 * @param index dell'array dei dispositivi sul DWM
 * @param device_id a 64 bit del dispositivo che si vuole
 * impostare
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_device_id_at(uint8_t index, uint64_t device_id);


/**
 * @brief Dato un vettore di id, imposta gli id dei dispositivi sul DWM
 * 
 * @param device_ids Array di id a 64 bit da impostare
 * @param num_devices numero di dispositivi
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_set_devices_id(uint64_t* device_ids, uint8_t num_devices);

/**
 * @brief Esegue misurazioni dato un certo dispositivo
 * con un certo id e ne restituisce la media
 * 
 * @param target_id del dispositivo a cui si vuole effettuare
 * la misurazione
 * @param num_samples Numero di campioni di misurazioni da
 * effettuare, 10 è standard
 * @param result Puntatore a struttura AverageMeasurement
 * popolata
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_measure(uint64_t target_id, uint8_t num_samples, AverageMeasurement* result);

/**
 * @brief Esegue misurazioni verso tutti i dispositivi
 * dati gli id configurati su uno di essi, e li restituisce
 * 
 * @param num_samples numero di misurazioni standard, 10 è lo standard
 * @param results Array di AverageMeasurement popolato
 * @param max_results Dimensione massima dell'array resylts
 * @param out_valid_count numero di misurazioni valide ottenute
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_measure_all(uint8_t num_samples, AverageMeasurement* results, int max_results, uint8_t* out_valid_count);
#endif

#endif // DWM_SPI_MASTER_RPI_H