#ifndef UKF_H
#define UKF_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

typedef struct {
    int dim_x; // dimensione del vettore di stato
    int dim_z; // dimensione del vettore di osservazione
    int num_sigma_points; // numero di punti sigma

    // funzioni
    void (*fx)(const gsl_vector *x, gsl_vector *x_out, double dt); // funzione di transizione di stato
    void (*hx)(const gsl_vector *x, gsl_vector *z_out); // funzione di osservazione

    // Vettori e matrici di stato
    gls_vector *x; // vettore di stato
    gsl_matrix *P; // matrice di covarianza dello stato

    gsl_matrix *Q; // matrice di covarianza del rumore di processo
    gsl_matrix *R; // matrice di covarianza del rumore di osservazione

    // Sigma Points e pesi
    gsl_matrix *sigma_points; // matrice dei punti sigma
    gsl_vector *Wm; // pesi per la media
    gsl_vector *Wc; // pesi per la covarianza

    // Variabili temporanee
    gsl_vector *x_prior; // vettore di stato a priori
    gsl_matrix *P_prior; // matrice di covarianza a priori
    gsl_matrix *sigmas_h; // matrice dei punti sigma trasformati dalla funzione di osservazione
    gsl_vector *z_pred; // vettore di osservazione predetto
    gsl_matrix *S; // matrice di covarianza dell'osservazione
    gsl_matrix *Pxz; // matrice di covarianza tra stato e osservazione
    gsl_vector *y; // innovazione (residuo)
    gsl_matrix *K; // matrice di guadagno di Kalman

    // Variabili per la gestione della memoria GSL
    gsl_vector *tmp_x;
    gsl_vector *tmp_x2;
    gsl_vector *tmp_z;
    gsl_vector *tmp_z2;
    gsl_matrix *tmp_P;

} ukf_state;

/**
 * @brief Inizializza lo stato dell'UKF.
 * 
 * @param dim_x Dimensione del vettore di stato.
 * @param dim_z Dimensione del vettore di osservazione.
 * @param alpha Parametro di scaling per la generazione dei punti sigma.
 * @param beta Parametro per incorporare informazioni sulla distribuzione.
 * @param kappa Parametro di scaling secondario
 * @param fx Funzione di transizione di stato.
 * @param hx Funzione di osservazione.
 * 
 * @return Puntatore a una struttura ukf_state inizializzata.
 * 
 */
ukf_state* ukf_init(int dim_x, int dim_z, double alpha, double beta,
                   double kappa, void (*fx)(const gsl_vector *x, gsl_vector *x_out, double dt),
                   void (*hx)(const gsl_vector *x, gsl_vector *z_out));



                   
/**
 * @brief Esegue un passo di predizione dell'UKF.
 * 
 * @param state Puntatore alla struttura ukf_state contenente lo stato dell'UKF.
 * @param dt Intervallo di tempo per la predizione.
 */
void ukf_predict(ukf_state *state, double dt);

/**
 * @brief Esegue un passo di aggiornamento dell'UKF.
 * 
 * @param state Puntatore alla struttura ukf_state contenente lo stato dell'UKF.
 * @param z Osservazione da utilizzare per l'aggiornamento.
 */
void ukf_update(ukf_state *state, const gsl_vector *z);

/**
 * @brief Libera la memoria allocata per lo stato dell'UKF.
 * 
 * @param state Puntatore alla struttura ukf_state da liberare.
 */
void ukf_free(ukf_state *state);

#endif // UKF_H