#ifndef UTILS_H
#define UTILS_H

#define POLL_RX_TO_RESP_TX_DLY_UUS  2000
#define POLL_RX_TO_RESP_TX_DLY_UUS_NLOS 6000  // Valore maggiore per NLoS


#include <stdbool.h>
#include <stdint.h>

/* Numero massimo di risponditori supportati */
#define MAX_RESPONDERS 16

/* Variabile globale che tiene conto della modalitÃ  di utilizzo del dispositivo */
typedef enum
{
  DEVICE_MODE_INITIATOR = 0, /* Dispositivo in modalitÃ  iniziatore */
  DEVICE_MODE_RESPONDER = 1, /* Dispositivo in modalitÃ  risponditore */
} device_mode_t;

/* Struttura dati per memorizzare informazioni sui risponditori */
typedef struct {
  uint64_t id;
  bool valid;
  double distance;
  bool nlos_suspection; // Bool value set to true if NLOS detected
  double first_path_power; // used in the analysis of the reception of signal to identify NLOS
  double peak_path_power; // used just like first_path_power
} responder_distance_t;

/* Dichiarazioni delle variabili condivise */
extern volatile device_mode_t device_mode;
extern volatile bool bool_mode_changed;
extern volatile uint64_t anchor_ids[MAX_RESPONDERS];
extern volatile uint64_t DEVICE_ID;
extern volatile bool anchor_enabled[MAX_RESPONDERS];
extern responder_distance_t distances[MAX_RESPONDERS];
extern volatile bool new_spi_command_received;
extern volatile bool nlos_mode;

/* Dichiarazioni delle funzioni condivise */
extern int ss_init_run(uint64_t anchor_id);
extern int ss_resp_run(uint64_t anchor_id);
extern void ss_main_task_function(void *pvParameter);
extern void ss_initiator_task_function(void *pvParameter);

#endif
