#include "topologia.h"

double calc_dist(Point2D p1, Point2D p2){
    return sqrt(pow(p2.x-p1.x,2) + pow(p2.y-p1.y, 2));
}

double random_double(double min, double max){
    return min + (max - min) * ((double)rand() / RAND_MAX);
}

double calcolaErrore(double distMisurata, double distCalcolata){
    // Controllo se la differenza è nel range di errore UWB
    double diff = fabs(distCalcolata - distMisurata);
    if(diff < UWB_ERRORE){
        return 0.0;
    } else {
        return pow(diff - UWB_ERRORE, 2);
    }
}

double erroreTopologia(double distanze[MAX_NODES][MAX_NODES], Point2D coordinate[], int numNodi){
    double errore = 0.0;
    
    for(int i = 0; i < numNodi; i++){
        for(int j = i + 1; j < numNodi; j++){
            double distCalcolata = calc_dist(coordinate[i], coordinate[j]);
            errore += calcolaErrore(distanze[i][j], distCalcolata);
        }
    }

    return errore;
}

void costruisci_topologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]){
    
    /*
    if(numNodi<2){
        printf("Due nodi minimo\n");
        return;
    }
    // Posiziono il primo nodo all'origine (0,0)
    coordinate[0].x = 0.0;
    coordinate[0].y = 0.0;

    // Posiziono il secondo nodo sull'asse positivo a distanza d_{1,2} dall'origine
    coordinate[1].x = distanze[0][1];
    coordinate[1].y = 0.0;

    // Per ogni nodo successivo
    for(int i = 2; i < numNodi; i++){
        // Effettuo la triangolazione a partire dai primi due nodi
        double d1 = distanze[0][i]; // distanza dal nodo 0
        double d2 = distanze[1][i]; // distanza dal nodo 1
        double d3 = coordinate[1].x; // Distanza tra nodo 0 e 1

        // uso legge del coseno e determino angolo rispetto ai primi due
        // cos(C) = (a^2 + b^2 - c^2) / (2*a*b)
        double cos = (d1*d1 + d3*d3 - d2*d2) / (2*d1*d3);

        // se il coseno va fuori dal range [-1.0,1.0] lo normalizzo
        if(cos<-1.0) cos = -1.0;
        if(cos>1.0) cos = 1.0;

        double angle = acos(cos); //mi trovo angolo dall'origine

        // ne calcolo le coordinate
        // prima y, con la regola h = sin(alpha) * d1
        coordinate[i].y = sin(angle) * d1;

        // poi x, usando il coseno
        coordinate[i].x = cos * d1;

        // verifico coerenza dagli altri nodi già posizionati
        int rifletti = 0; // flag booleana
        for (int j = 2; j < i; j ++){
            double distCalc = calc_dist(coordinate[i], coordinate[j]); // mi calcolo la distanza
            // tra i due nodi posizionati temporaneamente
            if(fabs(distCalc - distanze[i][j])>0.001){  // se è diversa, allora vuol dire che
                                                        // il posizionamento delle coordinate è sbagliato, e va corretto
                rifletti = 1; // setto la flag booleana
                break;
            }
        }
        if(rifletti){
            coordinate[i].y = -coordinate[i].y;
        }
    }
        */
        
}

void verificaConsistenza(double distanze[MAX_NODES][MAX_NODES], Point2D coordinate[], int numNodi){


}