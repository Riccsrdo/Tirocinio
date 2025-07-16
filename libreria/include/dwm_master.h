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

#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED 2000000 // 2 MHz
#define SPI_MODE 0
#define SPI_BITS 8


/*
Tutti i possibili comandi che possono essere inviati al dispositivo slave (dwm1001-dev)
che verranno processati e che, a seconda del comando, prevederanno una risposta.
*/
#define CMD_GET_DISTANCES  0x01
#define CMD_SET_MODE_INIT  0x10
#define CMD_SET_MODE_RESP  0x11
#define CMD_SET_ID         0x20 // Seguito da 8 byte ID
#define CMD_ENABLE_ANCHOR  0x30 // Seguito da 8 byte ID
#define CMD_DISABLE_ANCHOR 0x31 // Seguito da 8 byte ID
#define CMD_ENTER_CONFIG_MODE 0x40 // Comando per entrare in modalità configurazione e settare parametri come ID comunicazioni
#define CMD_EXIT_CONFIG_MODE 0x41 // Esci dalla modalità di configurazione e salva le modifiche
#define CMD_SET_NUM_DEVICES 0x42 // Imposta il numero di dispositivi con cui si vuole comunicare
#define CMD_SET_DEVICE_ID_AT 0x43 // Imposta l'id del dispositivo all'index desiderato nella lista di ID
#define CMD_MEASURE_DISTANCE 0x50 // Esegue misurazioni multiple e restituisce media, dato id
#define CMD_MEASURE_ALL_DISTANCES 0x51 // Misura le distanze average da tutti i dispositivi settati
#define CMD_GET_INFO       0xFE
#define CMD_GET_HELP       0xFF

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

/*----Funzioni di inizializzazione e gestione SPI -----*/

/**
 * @brief Inizializza l'interfaccia SPI. Setta alcuni parametri come velocità trasmissione, modalità, ecc.
 *
 * @param device Path del device SPI (es. "/dev/spidev0.0").
 * @param speed Velocità SPI in Hz (es. 2000000).
 * @param mode Modalità SPI (0, 1, 2, o 3).
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_spi_init(const char* device, uint32_t speed, uint8_t mode);

/**
 * @brief Chiude l'interfaccia SPI.
 */
void dwm_spi_close(void);

/**
 * @brief Invia un comando a singolo byte.
 *
 * @param command Il byte del comando da inviare.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_send_command(uint8_t command);

/**
 * @brief Invia un comando seguito da un argomento (2 byte totali).
 *
 * @param command Il byte del comando.
 * @param argument Il byte dell'argomento.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_send_command_with_arg(uint8_t command, uint8_t argument);

/*--- Funzioni di comunicazione effettive per DWM1001-Dev */


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


#endif // DWM_SPI_MASTER_RPI_H