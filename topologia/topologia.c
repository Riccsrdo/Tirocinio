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

void correggiMisurazioni(double distanze[MAX_NODES][MAX_NODES], int numNodi){

    // In primo luogo effettuo una verifica della disuguaglianza triangolare
    for(int i = 0; i < numNodi; i++){
        for(int j = i+1; j < numNodi; j++){
            for(int k = 0; k < numNodi; k++){
                if (k!=i && k !=j){ // prendo un terzo nodo che non sia ne i ne j
                    // verifico se dik + dkj < dij - 2*errore_uwb
                    if(distanze[i][k]+distanze[k][j]<distanze[i][j] - 2 * UWB_ERRORE){
                        // Correggi la distanza i-j che sembra essere sovrastimata
                        distanze[i][j] = distanze[j][i] = distanze[i][k] + distanze[k][j] - UWB_ERRORE;
                    }
                }
            }
        }
    }
}

void posizionamentoIniziale(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]){
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
        // Ci sono due possibilità di coordinate, una con la y negativa
        // l'altra positiva, le si calcola entrambe e si sceglie quella
        // con errore minore

        Point2D pos1 = {cos * d1, d1*sin(angle)};
        Point2D pos2 = {cos * d1, -d1*sin(angle)};

        // Calcolo errore per entrambe le possibilità
        double err1=0.0, err2=0.0;
        for(int j = 0; j<i; j++){
            double dist1=calc_dist(pos1, coordinate[j]);
            double dist2=calc_dist(pos2, coordinate[j]);
            err1 += calcolaErrore(distanze[i][j], dist1);
            err2 += calcolaErrore(distanze[i][j], dist2);
        }

        // scelgo distanza con minor errore
        if(err1<=err2){
            coordinate[i]=pos1;
        } else {
            coordinate[i]=pos2;
        }

        
    }
    
}

void ottimizzaTopologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]){
    // Imposto la temperatura iniziale per essere sempre accettante
    double temperatura = TEMP_INIZIALE;
    double fattore_raffreddamento = pow(TEMP_FINALE/TEMP_INIZIALE, 1.0/NUM_ITERATIONS);
    double erroreAttuale = erroreTopologia(distanze, coordinate, numNodi);

    Point2D migliori_coordinate[MAX_NODES]; // vettore temporaneo in cui salvo coordinate più precise
    double bestErrore = erroreAttuale;

    // Inserisco nel vettore delle migliori coordinate la config. attuale
    for(int i = 0; i< numNodi;i++){
        migliori_coordinate[i] = coordinate[i];
    }

    Point2D temp_coord[MAX_NODES]; // salvo le coordinate
    for(int i=0; i<NUM_ITERATIONS;i++){

        // copio coordinate attuali
        for(int j=0;j<numNodi;j++){
            temp_coord[j]=coordinate[j];
        }

        // seleziono un nodo tra quelli nella rete, esclusi
        // i primi due che sono fissati
        int nodoRandom = 2 + rand() % (numNodi-2);

        // Perturbo posizione in modo randomico sulla base
        // della temperatura attuale
        temp_coord[nodoRandom].x += random_double(-temperatura, temperatura);
        temp_coord[nodoRandom].y+= random_double(-temperatura, temperatura);

        // Calcolo il nuovo errore
        double new_error = erroreTopologia(distanze, coordinate, numNodi);

        // decido se accettare la nuova configurazione
        double delta = new_error - erroreAttuale;
        // se è negativo, accetto, altrimenti accetto con probabilità P
        double probabilità = exp(-delta / temperatura);

        if(delta < 0 || random_double(0,1) < probabilità){
            // Allora accetto il valore
            coordinate[nodoRandom] = temp_coord[nodoRandom];
            erroreAttuale = new_error;

            // aggiorno migliore configurazione
            if(new_error < bestErrore){
                for(int j = 0; j<numNodi; j++){
                    migliori_coordinate[j] = coordinate[j];
                }

                bestErrore = new_error;
            }
        }
        temperatura *= fattore_raffreddamento; // riduco temperatura
    }

    // ottenuta configurazione migliorata, la salvo
    for(int i=0; i<numNodi;i++){
        coordinate[i] = migliori_coordinate[i];
    }
}

void filtroMediana(Point2D coordinate[], int numNodi, int numCampioni){
    
    Point2D** campioni = (Point2D**)malloc(numNodi * sizeof(Point2D*));

    // alloco memoria per ogni campione
    for(int i=0;i<numNodi;i++){
        campioni[i] = (Point2D*)malloc(numCampioni*sizeof(Point2D));
    }

    // Genero diversi posizionamenti con piccole perturbazioni
    for(int i=0;i<numCampioni;i++){
        // Il primo posizionamento è quello originale
        if(i==0){
            for (int j = 0; j < numNodi; j++) {
                campioni[j][i] = coordinate[j];
            }
        } else{
            // Gli altri sono perturbazioni dell'originale
            for (int j = 0; j < numNodi; j++) {
                if (j < 2) {  // Mantieni fissi i primi due nodi
                    campioni[j][i] = coordinate[j];
                } else {
                    // Aggiungi un piccolo rumore gaussiano
                    campioni[j][i].x = coordinate[j].x + random_double(-UWB_ERRORE/2, UWB_ERRORE/2);
                    campioni[j][i].y = coordinate[j].y + random_double(-UWB_ERRORE/2, UWB_ERRORE/2);
                }
            }
        }
    }

     // Calcola la mediana delle coordinate per ogni nodo
     for (int i = 2; i < numNodi; i++) {  // Solo per i nodi dopo i primi due
        double *xValues = (double*)malloc(numCampioni * sizeof(double));
        double *yValues = (double*)malloc(numCampioni * sizeof(double));

        if (xValues == NULL || yValues == NULL) {
            printf("Errore: impossibile allocare memoria per il filtro di mediana\n");
            exit(1);
        }
        
        for (int j = 0; j < numCampioni; j++) {
            xValues[j] = campioni[i][j].x;
            yValues[j] = campioni[i][j].y;
        }
        
        // Ordinamento semplice per trovare la mediana
        for (int j = 0; j < numCampioni-1; j++) {
            for (int k = j+1; k < numCampioni; k++) {
                if (xValues[j] > xValues[k]) {
                    double temp = xValues[j];
                    xValues[j] = xValues[k];
                    xValues[k] = temp;
                }
                if (yValues[j] > yValues[k]) {
                    double temp = yValues[j];
                    yValues[j] = yValues[k];
                    yValues[k] = temp;
                }
            }
        }
        
        // Aggiorna le coordinate con la mediana
        coordinate[i].x = xValues[numCampioni/2];
        coordinate[i].y = yValues[numCampioni/2];

        free(xValues);
        free(yValues);
    }


    // libero memoria per ogni campione
    for(int i=0;i<numNodi;i++){
        free(campioni[i]);
    }
    free(campioni);
}


void verificaConsistenza(double distanze[MAX_NODES][MAX_NODES], Point2D coordinate[], int numNodi){
    int i,j;
    double erroreMax = 0.0;
    int nodo1 = -1, nodo2 = -1;

    for(i = 0; i < numNodi; i++){
        for(j = i + 1; j < numNodi; j++){
            double distCalcolata = distanza(coordinate[i], coordinate[j]);
            double errore = calcolaErrore(distanze[i][j], distCalcolata);

            if(errore>erroreMax){
                erroreMax = errore;
                nodo1 = i;
                nodo2 = j;
            }

            if(errore > UWB_ERRORE){
                printf("Superato errore max UWB\r\n");
            }
        }
    }

    printf("Errore massimo: %.2f cm tra nodi %d e %d\n", erroreMax * 100, nodo1, nodo2);
    printf("Errore medio: %.2f cm\n", erroreTopologia(distanze, coordinate, numNodi));

}


void costruisci_topologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]){
    int tentativi;
    double erroreMinimo = DBL_MAX;
    Point2D tempCoord[MAX_NODES];

    // Generatore numeri casuali per generare configurazioni random
    srand(time(NULL));

    for(int tentativi = 0; tentativi < NUM_CAMPIONI; tentativi++){

        // Effettuo una correzione delle misurazioni
        correggiMisurazioni(distanze, numNodi); // Verifica le misurazioni effettuate
        // con UWB, e in caso di incosistenze grandi prova a correggere

        // posiziona in modo temporaneo i nodi
        posizionamentoIniziale(distanze, numNodi, tempCoord);

        // Ottimizza le posizioni
        ottimizzaTopologia(distanze, numNodi, tempCoord);

        // Calcola errore attuale in questa soluzione
        double erroreAttuale = erroreTopologia(distanze, tempCoord, numNodi);

        // Aggiorna la soluzione migliore
        if(erroreAttuale < erroreMinimo){
            erroreMinimo = erroreAttuale;
            for(int i = 0; i< numNodi;i++){
                coordinate[i] = tempCoord[i];
            }
        }

    }

    // applico filtro mediana
    filtroMediana(coordinate, numNodi, 5);
}

