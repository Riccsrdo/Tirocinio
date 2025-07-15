#ifndef DWM_UTILITIES_H
#define DWM_UTILITIES_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stddef.h>

extern int spi_fd;
extern uint8_t spi_mode;
extern uint8_t spi_bits;
extern uint32_t spi_speed;

/**
 * @brief Esegue una transazione SPI (invio e ricezione simultanei). 
 *
 * @param tx_buf Buffer con i dati da inviare.
 * @param rx_buf Buffer dove memorizzare i dati ricevuti.
 * @param len Numero di byte da trasferire.
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_spi_transfer(uint8_t* tx_buf, uint8_t* rx_buf, size_t len);

/**
 * @brief Entra in modalità configurazione, permettendo di
 * impostare id dei dispositivi, e altre impostazioni
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_enter_config_mode(void);

/**
 * @brief Esco dalla modalità configurazione, tornando al normale
 * funzionamento
 * 
 * @return EXIT_SUCCESS in caso di successo, EXIT_FAILURE in caso di errore.
 */
int dwm_exit_config_mode(void);

// --------------------- Filtro di Kalman ---------------------

// Definisco gli elementi del filtro di Kalman
typedef struct
{
    double x[2];                 // Stato del sistema: posizione e velocità
    double P[2][2];              // Matrice di covarianza dello stato
    double Q[2][2];              // Matrice di processo di rumore
    double R;                    // Varianza del rumore di osservazione
    double F[2][2];              // Matrice di transizione di stato
    double H[2];                 // Matrice di osservazione
    struct timespec last_update; // Timestamp dell'ultimo aggiornamento
    int initialized;             // Flag per verificare se il filtro è stato inizializzato
} KalmanFilter;

/**
 * @brief Inizializza il filtro di Kalman con parametri iniziali. 
 * @param kf Puntatore al filtro di Kalman da inizializzare.
 * @param distanza_iniziale Distanza iniziale da utilizzare per l'inizializzazione.
 * @param r_std Deviazione standard del rumore di osservazione.
 * @param q_std Deviazione standard del rumore di processo.
 */
//void kalman_init(KalmanFilter *kf, double distanza_iniziale, double r_std, double q_std);

/**
 * @brief Aggiorna il filtro di Kalman con una nuova misura della distanza.
 * @param kf Puntatore al filtro di Kalman da aggiornare.
 * @param distanza_misurata Distanza misurata da utilizzare per l'aggiornamento.
 * @param r_std Deviazione standard del rumore di osservazione.
 * @param q_std Deviazione standard del rumore di processo.
 * 
 * @return Nuova distanza stimata dal filtro di Kalman.
 */
//double kalman_update(KalmanFilter *kf, double distanza_misurata, double r_std, double q_std);

#endif // DWM_UTILITIES_H