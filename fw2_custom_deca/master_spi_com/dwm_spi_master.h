#ifndef DWM_SPI_MASTER_RPI_H
#define DWM_SPI_MASTER_RPI_H

#include <stdint.h>
#include <stddef.h> // Per size_t

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

// --- Struttura per contenere i dati delle distanze ---
typedef struct {
  uint8_t id;
  double distance;
  int valid; // Usiamo int come flag booleano (0 = non valido, 1 = valido)
} ResponderInfo;
 
// -- Struttura per misurazioni medie --
typedef struct {
  uint64_t id;
  double average_distance;
  uint8_t samples_count;
  uint8_t requested_samples;
  int valid;
} AverageMeasurement;

// -- Struttura per info di configurazione --
typedef struct {
  uint8_t device_mode; // 0 initiator, 1 responder
  uint64_t device_id;
  uint8_t config_mod_active; // 1 se in modalità configurazione
} DeviceInfo;

/**
 * @brief Inizializza l'interfaccia SPI.
 *
 * @param device Path del device SPI (es. "/dev/spidev0.0").
 * @param speed Velocità SPI in Hz (es. 2000000).
 * @param mode Modalità SPI (0, 1, 2, o 3). Mode 0 è tipico per nRF52.
 * @return 0 in caso di successo, -1 in caso di errore.
 */
int dwm_spi_init(const char* device, uint32_t speed, uint8_t mode);

/**
 * @brief Chiude l'interfaccia SPI.
 */
void dwm_spi_close(void);

/**
 * @brief Esegue una transazione SPI (invio e ricezione simultanei).
 *
 * @param tx_buf Buffer con i dati da inviare.
 * @param rx_buf Buffer dove memorizzare i dati ricevuti.
 * @param len Numero di byte da trasferire.
 * @return 0 in caso di successo, -1 in caso di errore.
 */
int dwm_spi_transfer(uint8_t* tx_buf, uint8_t* rx_buf, size_t len);

/**
 * @brief Invia un comando a singolo byte.
 *
 * @param command Il byte del comando da inviare.
 * @return 0 in caso di successo, -1 in caso di errore.
 */
int dwm_send_command(uint8_t command);

/**
 * @brief Invia un comando seguito da un argomento (2 byte totali).
 *
 * @param command Il byte del comando.
 * @param argument Il byte dell'argomento.
 * @return 0 in caso di successo, -1 in caso di errore.
 */
int dwm_send_command_with_arg(uint8_t command, uint8_t argument);

/**
 * @brief Richiede e legge i dati delle distanze dal DWM1001.
 *
 * @param responder_array Un array di struct ResponderInfo fornito dal chiamante,
 * che verrà popolato con i dati ricevuti.
 * @param max_responders La dimensione dell'array responder_array.
 * @param out_valid_count Puntatore a uint8_t dove verrà scritto il numero
 * effettivo di ancore valide ricevute.
 * @return 0 in caso di successo (anche se 0 distanze ricevute), -1 in caso di errore di comunicazione.
 */
int dwm_request_distances(ResponderInfo* responder_array, int max_responders, uint8_t* out_valid_count);

// --- Aggiungi qui prototipi per altre funzioni se necessario ---
// int dwm_set_id(uint8_t new_id);
// int dwm_enable_anchor(uint8_t anchor_id);

/**
 * @brief Imposta id del dispositivo (64 bit)
 * 
 * @param new_id Id da impostare
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_set_id(uint64_t new_id);

/**
 * @brief Abilita un'ancora specifica
 * 
 * @param anchor_id del dispositivo da abilitare
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_enable_anchor(uint64_t anchor_id);

/**
 * @brief Disabilita un'ancora specifica
 * 
 * @param anchor_id del dispositivo da disabilitare
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_disable_anchor(uint64_t anchor_id);

/**
 * @brief Entra in modalità configurazione, permettendo di
 * impostare id dei dispositivi, e altre impostazioni
 * 
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_enter_config_mode(void);

/**
 * @brief Imposta numero dispositivi attualmente in funzione
 * 
 * @param num_devices numero dispositivi
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_set_num_devices(uint8_t num_devices);

/**
 * @brief Imposta id del dispositivo dato un certo index
 * dell'array
 * 
 * @param index dell'array dei dispositivi sul DWM
 * @param device_id a 64 bit del dispositivo che si vuole
 * impostare
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_set_device_id_at(uint8_t index, uint64_t device_id);

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
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_measure_average(uint64_t target_id, uint8_t num_samples, AverageMeasurement* result);

/**
 * @brief Esegue misurazioni verso tutti i dispositivi
 * dati gli id configurati su uno di essi, e li restituisce
 * 
 * @param num_samples numero di misurazioni standard, 10 è lo standard
 * @param results Array di AverageMeasurement popolato
 * @param max_results Dimensione massima dell'array resylts
 * @param out_valid_count numero di misurazioni valide ottenute
 * @return 0 in caso di successo, -1 di fallimento
 */
int dwm_measure_average_all(uint8_t num_samples, AverageMeasurement* results, int max_results, uint8_t* out_valid_count);


#endif // DWM_SPI_MASTER_RPI_H