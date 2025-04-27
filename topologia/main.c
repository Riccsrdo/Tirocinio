#include "topologia.h"

int main(){
    double distanze[MAX_NODES][MAX_NODES];
    Point2D coordinate[MAX_NODES];
    int numNodi;

    // TODO: popola in qualche modo il vettore delle distanze

    costruisci_topologia(distanze, numNodi, coordinate);

    print_topologia(coordinate, numNodi);

    verifica_consistenza(distanze, coordinate, numNodi);

    return 0;
}