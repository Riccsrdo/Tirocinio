#include "topologia.h"

int main(){
    //double distanze[MAX_NODES][MAX_NODES];
    Point2D coordinate[MAX_NODES];
    int numNodi;

    // TODO: popola in qualche modo il vettore delle distanze

    printf("Inserisci il numero di nodi: ");
    scanf("%d", &numNodi);
    
    if (numNodi > MAX_NODES) {
        printf("Troppi nodi! Il massimo è %d.\n", MAX_NODES);
        return 1;
    }
    
    #if 0
    printf("Inserisci la matrice delle distanze (in metri, con errore UWB di circa %.2f metri):\n", 
           UWB_ERRORE);
    for (int i = 0; i < numNodi; i++) {
        for (int j = 0; j < numNodi; j++) {
            if (i == j) {
                distanze[i][j] = 0.0;
            } else if (j > i) {
                printf("Distanza da nodo %d a nodo %d (m): ", i, j);
                scanf("%lf", &distanze[i][j]);
                distanze[j][i] = distanze[i][j]; // La matrice è simmetrica
            }
        }
    }
        #endif
    
    double distanze[MAX_NODES][MAX_NODES] = {
        // Nodo 0  Nodo 1  Nodo 2  Nodo 3  Nodo 4  Nodo 5
        {  0.00,   3.22,   2.54,   4.50,   2.13,   5.48 },  // Nodo 0
        {  3.22,   0.00,   3.12,   2.43,   1.69,   2.19 },  // Nodo 1
        {  2.54,   3.12,   0.00,   3.35,   4.21,   5.10 },  // Nodo 2
        {  4.50,   2.43,   3.35,   0.00,   3.75,   3.30 },  // Nodo 3
        {  2.13,   1.69,   4.21,   3.75,   0.00,   2.86 },  // Nodo 4
        {  5.48,   2.19,   5.10,   3.30,   2.86,   0.00 }   // Nodo 5
    };

    costruisci_topologia(distanze, numNodi, coordinate);

    visualizzaTopologiaAvanzata(coordinate, numNodi);

    //print_topologia(coordinate, numNodi);

    //verifica_consistenza(distanze, coordinate, numNodi);

    return 0;
}