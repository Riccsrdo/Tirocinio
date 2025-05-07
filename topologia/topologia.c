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
    if(diff <= UWB_ERRORE * 0.5){ // transazione graduale
        return 0.0;
    } else if( diff <= UWB_ERRORE){
        double ratio = (diff - UWB_ERRORE * 0.5)/ (UWB_ERRORE * 0.5);
        return pow(ratio * (diff - UWB_ERRORE * 0.5), 2);
    }else {
        return pow(diff - UWB_ERRORE, 2);
    }
}

void ransac_outlier(double distanze[MAX_NODES][MAX_NODES], int numNodi){
    // Creo dei placeholder per i nodi migliori per il modello di partenza
    Point2D bestP1, bestP2, bestP3;
    int indici_nodi[3] = {-1, -1, -1};
    int bestInlierCount = 0; // Contatore per il numero di inlier

    // Array in cui dico se un nodo è inlier
    bool* inlier = (bool*)malloc(numNodi * sizeof(bool));


    // Itero per x volte per permettere il ciclo di iterazioni RANSAC
    for(int i = 0; i< 200;i++){
        // Prendo tre nodi casualmente
        int nodo1 = rand() % numNodi;
        int nodo2 = rand() % numNodi;
        int nodo3 = rand() % numNodi;
        // Assicuro che i nodi siano diversi
        while(nodo1 == nodo2 || nodo1 == nodo3 || nodo2 == nodo3){
            nodo2 = rand() % numNodi;
            nodo3 = rand() % numNodi;
        }
        // Ottengo le distanze tra i nodi
        double d12 = distanze[nodo1][nodo2];
        double d13 = distanze[nodo1][nodo3];
        double d23 = distanze[nodo2][nodo3];
        // Verifico se il triangolo è valido con la disuguaglianza triangolare
        if(d12 + d13 < d23 - 2 * UWB_ERRORE|| d12 + d23 < d13 - 2 * UWB_ERRORE || d13 + d23 < d12 - 2 * UWB_ERRORE){
            // Triangolo non valido, salto
            continue;
        }
        // Se è valido, creo dei punti 2D temporanei ponendo 1 sull'origine, 2 su x
        // 3 lo calcolo con legge del coseno
        Point2D p1 = {0.0, 0.0, 1.0};
        Point2D p2 = {d12, 0.0, 1.0};
        double cos = (d12*d12 + d13*d13 - d23*d23) / (2*d12*d13);
        if(cos < -1.0) cos = -1.0;
        if(cos > 1.0) cos = 1.0;
        double angle = acos(cos);
        Point2D p3 = {d13*cos, d13*sin(angle), 1.0};

        bool* inlierAttuali = (bool*)malloc(numNodi * sizeof(bool)); // Array temporaneo per gli inlier
        int inlierCount = 0; // Contatore per gli inlier attuali
        // I nodi 1,2,3 sono inlier per definizione
        inlierAttuali[nodo1] = true;
        inlierAttuali[nodo2] = true;
        inlierAttuali[nodo3] = true;
        inlierCount += 3; // Incremento il contatore degli inlier

        

        // Prendo tutti gli altri nodi e verifico se sono consistenti con il modello entro UWB_ERRORE
        for(int j = 0; j<numNodi; j++){
            if(j == nodo1 || j == nodo2 || j == nodo3){
                continue; // salto i nodi già considerati
            }
            // Verifico se la distanza 1-j, 2-j e 3-j sono consistenti, ovvero se 1,2,3 permettono di stimare
            // la posizione di j in modo coerente alle distanze misurate, se si allora le distanze sono inlier
            double d1j = distanze[nodo1][j];
            double d2j = distanze[nodo2][j];
            double d3j = distanze[nodo3][j];
            


        }
        

    }

    free(inlier); // Libero la memoria allocata per gli inlier
    
}


void rilevamentoOutlier(double distanze[MAX_NODES][MAX_NODES], int numNodi) {
    int i, j, k;
    const int MIN_SUPPORT = 3;  // Numero minimo di triangoli per validare
    
    // Verifica della disuguaglianza triangolare su più percorsi
    for (i = 0; i < numNodi; i++) {
        for (j = i + 1; j < numNodi; j++) {
            int violazioniTriangolo = 0;
            int triangoliValidi = 0;
            
            for (k = 0; k < numNodi; k++) {
                if (k != i && k != j) {
                    triangoliValidi++;
                    // Verifica se dik + dkj < dij - 2*errore (violazione grave)
                    if (distanze[i][k] + distanze[k][j] < distanze[i][j] - 2*UWB_ERRORE) {
                        violazioniTriangolo++;
                    }
                }
            }
            
            // Se più della metà dei triangoli mostrano violazioni, la misura è un outlier
            if (violazioniTriangolo >= MIN_SUPPORT && 
                violazioniTriangolo > triangoliValidi * 0.3) {
                
                // Calcolo la distanza mediana attraverso i punti intermedi
                double* distanzeIndirette = (double*)malloc(triangoliValidi * sizeof(double));
                int idx = 0;
                
                for (k = 0; k < numNodi; k++) {
                    if (k != i && k != j) {
                        distanzeIndirette[idx++] = distanze[i][k] + distanze[k][j];
                    }
                }
                
                // Ordino le distanze indirette
                for (int p = 0; p < idx-1; p++) {
                    for (int q = p+1; q < idx; q++) {
                        if (distanzeIndirette[p] > distanzeIndirette[q]) {
                            double temp = distanzeIndirette[p];
                            distanzeIndirette[p] = distanzeIndirette[q];
                            distanzeIndirette[q] = temp;
                        }
                    }
                }
                
                // Uso la mediana delle distanze indirette
                double distanzaCorretta = distanzeIndirette[idx/2];
                printf("Outlier rilevato: Distanza tra nodo %d e %d corretta da %.2f a %.2f metri\n", 
                       i, j, distanze[i][j], distanzaCorretta);
                       
                distanze[i][j] = distanzaCorretta;
                distanze[j][i] = distanzaCorretta;
                
                free(distanzeIndirette);
            }
        }
    }
    
    // Verifica della coerenza complessiva attraverso MDS
    double centroidi[MAX_NODES][2] = {0};  // Coordinate stimate grezze
    
    // Posiziono arbitrariamente il primo nodo nell'origine
    centroidi[0][0] = 0;
    centroidi[0][1] = 0;
    
    // Posiziono grossolanamente gli altri nodi basandosi sulle distanze dal primo
    for (i = 1; i < numNodi; i++) {
        double angle = random_double(0, 2 * M_PI); // angoli casuali
        centroidi[i][0] = distanze[0][i] * cos(angle);
        centroidi[i][1] = distanze[0][i] * sin(angle);
    }
    
    // Semplice ottimizzazione delle posizioni 
    for (int iter = 0; iter < 25; iter++) {
        for (i = 1; i < numNodi; i++) {  
            double forceX = 0, forceY = 0;
            for (j = 0; j < numNodi; j++) {
                if (i == j) continue;
                
                double dx = centroidi[i][0] - centroidi[j][0];
                double dy = centroidi[i][1] - centroidi[j][1];
                double distAttuale = sqrt(dx*dx + dy*dy);
                
                if (distAttuale > 1e-6) {  // Controllo che non sia 0
                    double scale = (distAttuale - distanze[i][j]) / distAttuale;
                    forceX -= scale * dx * 0.1;  // Fattore di smorzamento
                    forceY -= scale * dy * 0.1;
                }
            }
            centroidi[i][0] += forceX;
            centroidi[i][1] += forceY;
        }
    }
    
    // Identifica distanze che sono incoerenti con il layout grossolano
    for (i = 0; i < numNodi; i++) {
        for (j = i + 1; j < numNodi; j++) {
            double dx = centroidi[i][0] - centroidi[j][0];
            double dy = centroidi[i][1] - centroidi[j][1];
            double distStimata = sqrt(dx*dx + dy*dy);
            
            // Se la differenza è troppo grande, potrebbe essere un outlier
            if (fabs(distStimata - distanze[i][j]) > 4 * UWB_ERRORE) {
                
                double nuovaDistanza = 0.7 * distanze[i][j] + 0.3 * distStimata;

                if (fabs(nuovaDistanza - distanze[i][j]) > UWB_ERRORE) {
                    // Limita la correzione al margine di errore UWB
                    if (nuovaDistanza > distanze[i][j]) {
                        nuovaDistanza = distanze[i][j] + UWB_ERRORE;
                    } else {
                        nuovaDistanza = distanze[i][j] - UWB_ERRORE;
                    }
                }

                printf("Incoerenza topologica: Distanza tra nodo %d e %d aggiustata da %.2f a %.2f metri\n", 
                       i, j, distanze[i][j], nuovaDistanza);
                distanze[i][j] = nuovaDistanza;
                distanze[j][i] = nuovaDistanza;
            }
        }
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

        printf("Posiziono il nodo %d alle coordinate (%f, %f)\n", i, coordinate[i].x, coordinate[i].y);

        // Calcolo livello di confidenza basato su errore
        coordinate[i].confidenza = 1.0 / (1.0 + minError);

        
    }

    // Correzione posizionamento
    for (int i = 2; i < numNodi; i++) {
        for (int j = 0; j < i; j++) {
            double dist = calc_dist(coordinate[i], coordinate[j]);
            if (dist < 0.1) {  // Se due nodi sono troppo vicini
                printf("ATTENZIONE: Nodi %d e %d quasi sovrapposti. Correggo...\n", i, j);
                // Sposta il nodo in una posizione più ragionevole
                double angle = random_double(0, 2 * M_PI);
                double dist_corr = distanze[i][j] * 0.9;  // 90% della distanza misurata
                coordinate[i].x = coordinate[j].x + dist_corr * cos(angle);
                coordinate[i].y = coordinate[j].y + dist_corr * sin(angle);
            }
        }
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
        int numNodiPert = 1;
        if (iterSenzaMiglioramento > 200 ) numNodiPert = 2;
        if (iterSenzaMiglioramento > 500 ) numNodiPert = 3;
        

        for (int p = 0; p < numNodiPert; p++){
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
        double new_error = erroreTopologia(distanze, temp_coord, numNodi);

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

            if(new_error < bestErrore && i % 500 == 0) {
                printf("Iterazione %d: Errore migliorato a %.6f\n", i, new_error);
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
                    // Aggiungi un piccolo rumore gaussiano in modo inversamente proporzionale alla confidenza
                    double noise = UWB_ERRORE * (1.0 - coordinate[j].confidenza);
                    campioni[j][i].x = coordinate[j].x + random_double(-noise, noise);
                    campioni[j][i].y = coordinate[j].y + random_double(-noise, noise);
                    campioni[j][i].confidenza = coordinate[j].confidenza; // aggiorno la confidenza
                }
            }
        }
    }

     // Calcola la mediana delle coordinate per ogni nodo
     for (int i = 2; i < numNodi; i++) {  // Solo per i nodi dopo i primi due
        double *xValues = (double*)malloc(numCampioni * sizeof(double));
        double *yValues = (double*)malloc(numCampioni * sizeof(double));
        double *weights = (double*)malloc(numCampioni * sizeof(double));

        if (xValues == NULL || yValues == NULL || weights == NULL) {
            printf("Errore: impossibile allocare memoria per il filtro di mediana\n");
            exit(1);
        }
        
        // Copio valori e assegno pesi sulla base di confidenza e posizione originale
        for (int j = 0; j < numCampioni; j++) {
            xValues[j] = campioni[i][j].x;
            yValues[j] = campioni[i][j].y;

            weights[j] = campioni[i][j].confidenza;
            if (j==0) weights[j] *= 2.0; // raddoppio peso per posizione originale
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
        for (int j = 0; j < numCampioni-1; j++) {
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
        for (int j = 0; j < numCampioni-1; j++) {
            for (int k = j+1; k < numCampioni; k++) {
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
            double distCalcolata = calc_dist(coordinate[i], coordinate[j]);
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

    rilevamentoOutlier(distanze, numNodi);

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
    filtroMediana(coordinate, numNodi, NUM_MEDIANA);

    // Verifica finale per evitare sovrapposizioni
    for (int i = 0; i < numNodi; i++) {
        for (int j = 0; j < i; j++) {
            double dist = calc_dist(coordinate[i], coordinate[j]);
            if (dist < 0.1 && i != j) {
                printf("CORREZIONE FINALE: Nodi %d e %d sovrapposti - separazione\n", i, j);
                double angle = random_double(0, 2 * M_PI);
                double separazione = fmax(0.5, distanze[i][j] * 0.8);
                coordinate[i].x = coordinate[j].x + separazione * cos(angle);
                coordinate[i].y = coordinate[j].y + separazione * sin(angle);
            }
        }
    }

    // Stampa configurazione finale prima del filtro mediana
    printf("\nConfigurazione finale prima del filtro mediana:\n");
    for (int i = 0; i < numNodi; i++) {
        printf("Nodo %d: (%.2f, %.2f)\n", i, coordinate[i].x, coordinate[i].y);
    }
}

// Funzione per visualizzare la topologia con livelli di confidenza
void visualizzaTopologiaAvanzata(Point2D coordinate[], int numNodi) {
    int i;
    printf("\nTopologia ricostruita (con livelli di confidenza):\n");
    printf("-----------------------------------------------------\n");
    printf("| Nodo |    X    |    Y    | Confidenza | Precisione |\n");
    printf("-----------------------------------------------------\n");
    
    for (i = 0; i < numNodi; i++) {
        // Calcola un indicatore di precisione stimata in centimetri
        double precisioneStimata = UWB_ERRORE * (1.0 - coordinate[i].confidenza) * 100;
        // Limita la precisione stimata a un valore ragionevole
        if (precisioneStimata < 5.0) precisioneStimata = 5.0;
        if (precisioneStimata > UWB_ERRORE * 100) precisioneStimata = UWB_ERRORE * 100;
        
        printf("| %4d | %7.2f | %7.2f | %10.2f | ±%6.1f cm |\n", 
               i, coordinate[i].x, coordinate[i].y, coordinate[i].confidenza, precisioneStimata);
    }
    
    printf("-----------------------------------------------------\n");
    printf("Nota: La precisione stimata è basata sul livello di confidenza\n");
    printf("      e rappresenta l'incertezza nella posizione del nodo.\n");
}

