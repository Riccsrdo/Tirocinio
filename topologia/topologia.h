#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

#define MAX_NODES 16 /* Configurabile se configurato anche su Firmware*/
#define UWB_ERRORE 0.15 // circa 15cm
#define NUM_ITERATIONS 2000 // iterazioni per convergere dati rumorosi
#define LEARNING_RATE 0.01 // tasso apprendimento ottimizzazione (valore di moltiplicazione per riduzione temperatura)
#define TEMP_INIZIALE 1.0 // temperatura per simulated annealing
#define TEMP_FINALE 0.001 // temperatura finale di accettazione
#define NUM_CAMPIONI 5 // numero di posizioni casuali da provare nel multistart approach


typedef struct {
    double x;
    double y;
} Point2D;

/**
 * @brief Prende due Point2D e calcola la distanza
 * euclidea tra i due punti
 * 
 * @param p1 primo punto
 * @param p2 secondo punto
 * 
 * @return distanza euclidea tra p1 e p2
 */
double calc_dist(Point2D p1, Point2D p2);

/**
 * @brief Genera un valore random nell'intervallo dato da minimo e massimo
 * 
 * @param min minimo valore
 * @param max massimo valore
 * 
 * @return valore random generato
 */
double random_double(double min, double max);

/**
 * @brief Funzione che si occupa di calcolare errore sulla base 
 * dell'incertezza definita per le misurazioni UWB
 * 
 * @param distMisurata la distanza effettiva di UWB
 * @param distCalcolata la distanza calcolata tra i nodi nella configurazione ipotetica
 * 
 * @return errore calcolato
 */
double calcolaErrore(double distMisurata, double distCalcolata); 

/**
 * @brief Calcola errore della topologia corrente mettendola in
 * comparazione con le distanze effettive tra i nodi
 * 
 * @param distanze matrice 2d di distanze tra i nodi
 * @param coordinate coordinate della topologia calcolata
 * @param numNodi numero di nodi effettivo di cui vogliamo effettuare calcolo
 * 
 * @return errore calcolato
 */
double erroreTopologia(double distanze[MAX_NODES][MAX_NODES], Point2D coordinate[], int numNodi);

/**
 * @brief Costruisce la topologia dei nodi dello spazio
 * date le distanze tra tutti i nodi verso tutti i nodi.
 * 
 * @param distanze Vettore di distanze di dimensione MAX_NODES x MAX_NODES
 * che contiene le distanze tra tutti i nodi verso tutti i nodi
 * @param numNodi numero di nodi effettivo di cui si calcolano
 * le distanze
 * @param coordinate vettore di coordinate di Point2D in cui
 * salvare le coordinate dei nodi
 */
void costruisci_topologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]);

/**
 * @brief Funzione che si occupa di ottimizzare l'errore di calcolo
 * della topologia
 * 
 * @param distanze Vettore di distanze di dimensione MAX_NODES x MAX_NODES
 * che contiene le distanze tra tutti i nodi verso tutti i nodi
 * @param numNodi numero di nodi effettivo di cui si calcolano
 * le distanze
 * @param coordinate vettore di coordinate di Point2D in cui
 * salvare le coordinate dei nodi
 */
void ottimizza_topologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]);

/**
 * @brief Funzione che dati i valori salvati, printa
 * su stdout la topologia costruita
 * 
 * @param coordinate vettore di Point2D
 * @param numNodi numero di nodi di cui printare topologia
 */
void print_topologia(Point2D coordinate[], int numNodi);

/**
 * @brief Funzione che presa una topologia costruita, verifica che non
 * si distacchi più del margine dei errore di UWB dalle distanze effettivamente
 * calcolate
 * 
 * @param distanze matrice 2d di distanze
 * @param coordinate configurazione dei nodi calcolata
 */
void verificaConsistenza(double distanze[MAX_NODES][MAX_NODES], Point2D coordinate[], int numNodi);

