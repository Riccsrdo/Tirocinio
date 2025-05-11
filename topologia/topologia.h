#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_NODES 16 /* Configurabile se configurato anche su Firmware*/
#define UWB_ERRORE 0.40 // circa 40cm
#define NUM_ITERATIONS 8000 // iterazioni per convergere dati rumorosi
#define LEARNING_RATE 0.01 // tasso apprendimento ottimizzazione (valore di moltiplicazione per riduzione temperatura)
#define TEMP_INIZIALE 2.0 // temperatura per simulated annealing
#define TEMP_FINALE 0.0005 // temperatura finale di accettazione
#define NUM_CAMPIONI 20 // numero di posizioni casuali da provare nel multistart approach
#define NUM_MEDIANA 9 // campioni mediana
#define CONVERGENCE_THRESHOLD 1e-5 // soglia di convergenza

typedef struct {
    double x;
    double y;
    double confidenza; // confidenza della posizione
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
 * @brief Funzione che si occupa di calcolare l'intersezione
 * tra due cerchi dati i loro centri e raggi
 * 
 * @param p1 centro del primo cerchio
 * @param r1 raggio del primo cerchio
 * @param p2 centro del secondo cerchio
 * @param r2 raggio del secondo cerchio
 * @param intersezione1 primo punto di intersezione
 * @param intersezione2 secondo punto di intersezione
 */
int calcolaIntersezioneCerchi(Point2D p1, double r1, Point2D p2, double r2, Point2D* intersezione1, Point2D* intersezione2);

/**
 * @brief Funzione che si occupa di rilevare outlier
 * attraverso il metodo RANSAC, e correggerli
 * 
 * @param distanze matrice 2d di distanze tra i nodi
 * @param numNodi numero di nodi effettivo di cui vogliamo effettuare calcolo
 */
void ransac_outlier(double distanze[MAX_NODES][MAX_NODES], int numNodi);

/**
 * @brief Funzione che si occupa di rilevare e correggere outlier
 * che si discostano di molto dalle misurazioni
 * 
 * @param distanze vettore di distanze da tutti i nodi verso tutti i nodi
 * @param numNodi numero di nodi effettivamente impiegati
 */
void rilevamentoOutlier(double distanze[MAX_NODES][MAX_NODES], int numNodi);

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
 * @brief Funzione che rileva e corregge misurazioni inconsistenti
 * 
 * @param distanze matrice 2d di distanze tra i nodi
 * @param numNodi numero di nodi effettivo di cui vogliamo effettuare calcolo
 */
void correggiMisurazioni(double distanze[MAX_NODES][MAX_NODES], int numNodi);

/**
 * @brief Posiziona i nodi in modo iniziale considerando incertezza
 * 
 * @param distanze matrice 2d di distanze tra i nodi
 * @param numNodi numero di nodi effettivo di cui vogliamo effettuare calcolo
 * @param coordinate coordinate della topologia calcolata
 */
void posizionamentoIniziale(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]);

/**
 * @brief Fa uso del simulated annealing per ottimizzare topologia
 * 
 * @param distanze matrice 2d di distanze tra i nodi
 * @param numNodi numero di nodi effettivo di cui vogliamo effettuare calcolo
 * @param coordinate coordinate della topologia calcolata
 * 
 */
void ottimizzaTopologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]);

/**
 * @brief Filtra i valori mediani delle coordinate calcolate
 * 
 * @param coordinate vettore di coordinate della topologia
 * @param numNodi numero di nodi effettivi
 * @param numCampioni numero di campioni 
 */
void filtroMediana(Point2D coordinate[], int numNodi, int numCampioni);

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

void visualizzaTopologiaAvanzata(Point2D coordinate[], int numNodi);