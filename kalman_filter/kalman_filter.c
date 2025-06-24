/*
Implementazione in C del filtro di Kalman, ispirato dal filtro implementato nella libreria
'filterpy' di Python.
*/

// Include delle librerie standard
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

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

void kalman_init(KalmanFilter *kf, double distanza_iniziale, double r_std, double q_std)
{
    /*
    Inizializzo il filtro di Kalman con la distanza iniziale e le varie matrici e valori
    */

    // inizio dallo stato del sistema
    kf->x[0] = distanza_iniziale; // Posizione iniziale
    kf->x[1] = 0.0;               // Velocità iniziale

    // poi passo alla matrice di covarianza dello stato
    kf->P[0][0] = r_std * r_std; // Varianza della posizione
    kf->P[0][1] = 0.0;           // Covarianza posizione-velocità
    kf->P[1][0] = 0.0;           // Covarianza velocità-posizione
    kf->P[1][1] = 10.0;          // Varianza della velocità, all'inizio alta per maggiore incertezza

    // poi passo alla matrice di osservazione, si misura solo distanza e non velocità
    kf->H[0] = 1.0; // Osservazione della posizione
    kf->H[1] = 0.0; // Non si osserva la velocità

    // inizializzo varianza del rumore di misura
    kf->R = r_std * r_std; // Varianza del rumore di osservazione

    // ottengo il primo timestamp
    clock_gettime(CLOCK_MONOTONIC, &kf->last_update);
    kf->initialized = 1; // Segno che il filtro è stato inizializzato
}

double kalman_update(KalmanFilter *kf, double distanza_misurata, double r_std, double q_std)
{
    /*
    Aggiorno il filtro di Kalman con la nuova misura della distanza
    */
    if (kf->initialized == 0)
    {
        fprintf(stderr, "Errore: il filtro di Kalman non è stato inizializzato.\n");
        return -1.0; // Errore, filtro non inizializzato
    }

    // Calcolo il dt dall'ultimo aggiornamento
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = (now.tv_sec - kf->last_update.tv_sec) + 
                (now.tv_nsec - kf->last_update.tv_nsec) / 1e9; // Converto in secondi
    kf->last_update = now; // Aggiorno il timestamp dell'ultimo aggiornamento

    // Aggiorno matrici dipendenti da dt
    // matrice di transizione di stato
    kf->F[0][0] = 1.0; // Posizione rimane la stessa
    kf->F[0][1] = dt;  // Posizione aggiornata in base al tempo
    kf->F[1][0] = 0.0; // Velocità non cambia
    kf->F[1][1] = 1.0; // Velocità rimane la stessa

    double dt2 = dt * dt; // Calcolo dt al quadrato
    double dt3 = dt2 * dt; // Calcolo dt al cubo
    double dt4 = dt3 * dt; // Calcolo dt alla quarta potenza
    double q_std = q_std * q_std; // Varianza del rumore di processo

    // Matrice di processo di rumore
    // traduzione della funzione Q_discrete_white_noise della libreria filterpy per modello a v. costante
    kf->Q[0][0] = dt4 /4.0 * q_std;
    kf->Q[0][1] = dt3 /2.0 * q_std;
    kf->Q[1][0] = dt3 /2.0 * q_std;
    kf->Q[1][1] = dt2 * q_std;

    // effettuo la predizione dello stato
    double x_pred[2];
    x_pred[0] = kf->F[0][0] * kf->x[0] + kf->F[0][1] * kf->x[1]; // nuova_distanza = vecchia_distanza + velocità * dt
    x_pred[1] = kf->F[1][0] * kf->x[0] + kf->F[1][1] * kf->x[1]; // nuova_velocità = vecchia_velocità

    // Effettuo anche la predizione della matrice di covarianza dello stato
    // aggiorno l'incertezza dello stato
    // P = F * P * F^T + Q // aggiungo Q (rumore) per rappresentare l'incertezza della velocità costante, non sempre lo è
    // ad esempio se il veicolo accelera o decelera c'è un cambiamento
    double P_pred[2][2]; 
    double temp_P[2][2];

    // temp_P = F * P
    temp_P[0][0] = kf->F[0][0] * kf->P[0][0] + kf->F[0][1] * kf->P[1][0];
    temp_P[0][1] = kf->F[0][0] * kf->P[0][1] + kf->F[0][1] * kf->P[1][1];
    temp_P[1][0] = kf->F[1][0] * kf->P[0][0] + kf->F[1][1] * kf->P[1][0];
    temp_P[1][1] = kf->F[1][0] * kf->P[0][1] + kf->F[1][1] * kf->P[1][1];

    // Calcolo P_pred = temp_P * F^T + Q
    // con temp_P * F^T dato da: 
    // somma delle righe di temp_P moltiplicate per le colonne di F
    P_pred[0][0] = temp_P[0][0]*kf->F[0][0] + temp_P[0][1]*kf->F[0][1] + kf->Q[0][0];
    P_pred[0][1] = temp_P[0][0]*kf->F[1][0] + temp_P[0][1]*kf->F[1][1] + kf->Q[0][1];
    P_pred[1][0] = temp_P[1][0]*kf->F[0][0] + temp_P[1][1]*kf->F[0][1] + kf->Q[1][0];
    P_pred[1][1] = temp_P[1][0]*kf->F[1][0] + temp_P[1][1]*kf->F[1][1] + kf->Q[1][1];

    // Eseguo l'update con la misura della distanza
    // calcolo l'innovazione y, data da z (ovvero la misurazione ricevuta, distanza_misurata) 
    // meno l'osservazione predetta H * x_pred
    // y = z - H * x_pred
    double y = distanza_misurata - (kf->H[0] * x_pred[0] + kf->H[1] * x_pred[1]);

    // calcolo S, la matrice di covarianza dell'innovazione
    // S = H * P_pred * H^T + R
    double S = (kf->H[0] * (P_pred[0][0] * kf->H[0] + P_pred[0][1] * kf->H[1]) +
                kf->H[1] * (P_pred[1][0] * kf->H[0] + P_pred[1][1] * kf->H[1])) + kf->R;


    // calcolo la matrice di guadagno K
    // K = P_pred * H^T * S^-1
    double K[2];
    K[0] = (P_pred[0][0] * kf->H[0] + P_pred[0][1] * kf->H[1]) / S;
    K[1] = (P_pred[1][0] * kf->H[0] + P_pred[1][1] * kf->H[1]) / S;

    // aggiorno lo stato del sistema in base al guadagno K e all'innovazione
    kf->x[0] = x_pred[0] + K[0] * y; // Nuova distanza
    kf->x[1] = x_pred[1] + K[1] * y; // Nuova velocità

    // aggiorno la covarianza dello stato P
    // P = (I - K * H) * P_pred
    double I_KH[2][2];
    I_KH[0][0] = 1.0 - K[0] * kf->H[0]; I_KH[0][1] = - K[0] * kf->H[1];
    I_KH[1][0] = - K[1] * kf->H[0];     I_KH[1][1] = 1.0 - K[1] * kf->H[1];
    
    kf->P[0][0] = I_KH[0][0]*P_pred[0][0] + I_KH[0][1]*P_pred[1][0];
    kf->P[0][1] = I_KH[0][0]*P_pred[0][1] + I_KH[0][1]*P_pred[1][1];
    kf->P[1][0] = I_KH[1][0]*P_pred[0][0] + I_KH[1][1]*P_pred[1][0];
    kf->P[1][1] = I_KH[1][0]*P_pred[0][1] + I_KH[1][1]*P_pred[1][1];

    // ritorno la nuova distanza stimata
    return kf->x[0];
}
