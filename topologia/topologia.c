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
    if(diff <= UWB_ERRORE * 0,5){ // transazione graduale
        return 0.0;
    } else if( diff <= UWB_ERRORE){
        double ratio = (diff - UWB_ERRORE * 0.5)/ (UWB_ERRORE * 0.5);
        return pow(ratio * (diff - UWB_ERRORE * 0.5), 2);
    }else {
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
    // Imposto i livelli di confidenza iniziali
    for (int i = 0; i<numNodi; i++){
        coordinate[i].confidenza = 1.0;
    }
    
    // Posiziono il primo nodo all'origine (0,0)
    coordinate[0].x = 0.0;
    coordinate[0].y = 0.0;

    // Posiziono il secondo nodo sull'asse positivo a distanza d_{1,2} dall'origine
    coordinate[1].x = distanze[0][1];
    coordinate[1].y = 0.0;

    // Per ogni nodo successivo
    for(int i = 2; i < numNodi; i++){
        // Effettuo la triangolazione con molteplici riferimenti
        //double bestX = 0.0, bestY = 0.0;
        double minError = DBL_MAX;

        // Prima effettuo la triangolazione a partire dai primi due nodi
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
            minError = err1;
        } else {
            coordinate[i]=pos2;
            minError = err2;
        }

        // Calcolo livello di confidenza basato su errore
        coordinate[i].confidenza = 1.0 / (1.0 + minError);

        
    }
    
}

void ottimizzaTopologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]){
    // Imposto la temperatura iniziale per essere sempre accettante
    double temperatura = TEMP_INIZIALE;
    double fattore_raffreddamento = pow(TEMP_FINALE/TEMP_INIZIALE, 1.0/NUM_ITERATIONS);
    double erroreAttuale = erroreTopologia(distanze, coordinate, numNodi);
    int iterSenzaMiglioramento = 0;

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

        // seleziono nodi tra quelli nella rete, esclusi
        // i primi due che sono fissati
        // strategia adattiva: perturba più nodi quando ottimizzazione rallenta
        int numNodi = 1;
        if (iterSenzaMiglioramento > 200 ) numNodi = 2;
        if (iterSenzaMiglioramento > 500 ) numNodi = 3;
        

        for (int p = 0; p < numNodi; p++){
            int nodoRandom = 2 + rand() % (numNodi-2);

            // Calcolo di quanto perturbare sulla base della temperatura
            // e della confidenza attuale del nodo randomicamente scelto
            double perturbazione = temperatura / coordinate[nodoRandom].confidenza;

            // Perturbo posizione in modo randomico sulla base
            // della temperatura attuale
            temp_coord[nodoRandom].x += random_double(-perturbazione, perturbazione);
            temp_coord[nodoRandom].y+= random_double(-perturbazione, perturbazione);
        }

        

        // Calcolo il nuovo errore
        double new_error = erroreTopologia(distanze, coordinate, numNodi);

        // decido se accettare la nuova configurazione
        double delta = new_error - erroreAttuale;
        // se è negativo, accetto, altrimenti accetto con probabilità P
        double probabilità = exp(-delta / temperatura);

        if(delta < 0 || random_double(0,1) < probabilità){
            // Allora accetto il valore
            for (int k = 2; k<numNodi;k++){
                coordinate[k] = temp_coord[k];
            }
            erroreAttuale = new_error;

            // Se il delta calcolato è troppo basso
            // allora aggiorno il numero di iterazioni che non hanno
            // comportato dei miglioramenti
            if (delta < -CONVERGENCE_THRESHOLD){
                iterSenzaMiglioramento = 0;
            } else {
                iterSenzaMiglioramento++;
            }

            // aggiorno migliore configurazione
            if(new_error < bestErrore){
                for(int j = 0; j<numNodi; j++){
                    migliori_coordinate[j] = coordinate[j];
                }

                bestErrore = new_error;
            } else {
                iterSenzaMiglioramento++;
            }
        } else {
            iterSenzaMiglioramento++;
        }
        temperatura *= fattore_raffreddamento; // riduco temperatura

        if(iterSenzaMiglioramento>1000){
            // incremento temperatura per fornire margine maggiore di accettazione nuovi valori perturbati
            temperatura = temperatura*1.5;
            iterSenzaMiglioramento=0;
        }
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
    for(int i=0;i<NUM_MEDIANA;i++){
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
                    // Aggiungi un piccolo rumore gaussiano in modo inversamente proporzionale alla confidenza
                    double noise = UWB_ERRORE * (1.0 - coordinate[j].confidenza);
                    campioni[j][i].x = coordinate[j].x + random_double(-noise, noise);
                    campioni[j][i].y = coordinate[j].y + random_double(-noise, noise);
                    campioni[i][j].confidenza = coordinate[j].confidenza; // aggiorno la confidenza
                }
            }
        }
    }

     // Calcola la mediana delle coordinate per ogni nodo
     for (int i = 2; i < numNodi; i++) {  // Solo per i nodi dopo i primi due
        double *xValues = (double*)malloc(NUM_MEDIANA * sizeof(double));
        double *yValues = (double*)malloc(NUM_MEDIANA * sizeof(double));
        double *weights = (double*)malloc(NUM_MEDIANA * sizeof(double));

        if (xValues == NULL || yValues == NULL || weights == NULL) {
            printf("Errore: impossibile allocare memoria per il filtro di mediana\n");
            exit(1);
        }
        
        // Copio valori e assegno pesi sulla base di confidenza e posizione originale
        for (int j = 0; j < numCampioni; j++) {
            xValues[j] = campioni[i][j].x;
            yValues[j] = campioni[i][j].y;

            weights[j] = campioni[i][j].confidenza;
            if (j==0) weights[j] * 2.0; // raddoppio peso per posizione originale
        }
        
        #if 0
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
            #endif
        // Ordinamento per coordinate x
        for (int j = 0; j < NUM_MEDIANA-1; j++) {
            for (int k = j+1; k < NUM_MEDIANA; k++) {
                if (xValues[j] > xValues[k]) {
                    // Scambio x
                    double temp = xValues[j];
                    xValues[j] = xValues[k];
                    xValues[k] = temp;
                    
                    // Scambio peso corrispondente
                    temp = weights[j];
                    weights[j] = weights[k];
                    weights[k] = temp;
                }
            }
        }
        
        // Ordinamento per coordinate y
        for (int j = 0; j < NUM_MEDIANA-1; j++) {
            for (int k = j+1; k < NUM_MEDIANA; k++) {
                if (yValues[j] > yValues[k]) {
                    // Scambio y
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
        free(weights);
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

    // TODO: funzione che rileva outlier di misurazione

    for(int tentativi = 0; tentativi < NUM_CAMPIONI; tentativi++){

        // Effettuo una correzione delle misurazioni
        //correggiMisurazioni(distanze, numNodi); // Verifica le misurazioni effettuate
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

