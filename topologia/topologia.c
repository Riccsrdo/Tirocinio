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

int calcolaIntersezioneCerchi(Point2D p1, double r1, Point2D p2, double r2, Point2D* intersezione1, Point2D* intersezione2) {
    // Calcola la distanza tra i centri dei cerchi
    double d = calc_dist(p1, p2);
    
    // Se i cerchi non si intersecano
    if (d > r1 + r2 || d < fabs(r1 - r2) || d == 0 && r1!=r2) {
        return 0; // Nessuna intersezione
    }

    if(d<1e-9){
        if(fabs(r1-r2)<1e-9){
            // Se i cerchi coincidono, non c'è un punto di intersezione unico
            return 0; // Nessuna intersezione
        } else {
            // Se i cerchi sono concentrici ma con raggi diversi, non c'è intersezione
            return 0; // Nessuna intersezione
        }
    }

    // Calcolo del punto di intersezione
    double a = (r1*r1 - r2*r2 + d*d) / (2*d);
    double h_sq = r1*r1 - a*a;

    if(h_sq < 0 && fabs(h_sq) < 1e-9){
        h_sq = 0; // Se h^2 è negativo, significa che i cerchi si toccano in un punto
    } else if(h_sq < 0){
        return 0; // Nessuna intersezione
    }
    double h = sqrt(h_sq);
    
    Point2D p0 = {p1.x + a * (p2.x - p1.x) / d, p1.y + a * (p2.y - p1.y) / d, 0.0};

    intersezione1->x = p0.x + h * (p2.y - p1.y) / d;
    intersezione1->y = p0.y - h * (p2.x - p1.x) / d;
    intersezione1->confidenza = 0.0;
    
    if(fabs(h)>1e-9){
        intersezione2->x = p0.x - h * (p2.y - p1.y) / d;
        intersezione2->y = p0.y + h * (p2.x - p1.x) / d;
        intersezione2->confidenza = 0.0;

        return 2; // Due punti di intersezione
    } 

    if (intersezione2 != NULL) { // Buona pratica anche se qui è sempre fornito
        intersezione2->x = intersezione1->x;
        intersezione2->y = intersezione1->y;
        intersezione2->confidenza = intersezione1->confidenza;
    }
    
    return 1; // un punto di intersezione
}

void ransac_outlier(double distanze[MAX_NODES][MAX_NODES], int numNodi, bool inlier[MAX_NODES]){
    // Creo dei placeholder per i nodi migliori per il modello di partenza
    Point2D bestP1, bestP2, bestP3;
    int indici_nodi[3] = {-1, -1, -1};
    int bestInlierCount = -1; // Contatore per il numero di inlier

    // Array in cui dico se un nodo è inlier
    //bool* inlier = (bool*)calloc(numNodi, sizeof(bool));
    if(!inlier){
        perror("Errore allocazione memoria per inlier");
        exit(EXIT_FAILURE);
        return;
    }


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
        if(d12 + d13 < d23 -  UWB_ERRORE|| d12 + d23 < d13 -  UWB_ERRORE || d13 + d23 < d12 -  UWB_ERRORE){
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

        bool* inlierAttuali = (bool*)calloc(numNodi, sizeof(bool)); // Array temporaneo per gli inlier
        if(!inlierAttuali){
            perror("Errore allocazione memoria per inlier attuali");
            exit(EXIT_FAILURE);
            return;
        }
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

            // A questo punto usando il punto 1 e 2 e le distanze 1-j e 2-j
            // determino i punti di intersezione tra i due cerchi centrati in 1 e 2
            // e raggio d1j e d2j rispettivamente
            Point2D intersezione1, intersezione2;
            int num_intersezioni = calcolaIntersezioneCerchi(p1, d1j, p2, d2j, &intersezione1, &intersezione2);

            if(num_intersezioni > 0){
                 // Controllo che il primo punto rientri nel range di errore di UWB per la distanza j
                 bool consistente = false;
                 if(fabs(calc_dist(p3, intersezione1) -d3j) < UWB_ERRORE){
                    // Se la distanza tra il punto 3 e il primo punto di intersezione è nel range di errore
                    // allora il nodo j è un inlier
                    consistente = true;
                 }
                 if(!consistente && num_intersezioni == 2){
                    // Se il primo punto non è consistente, verifico il secondo
                    if(fabs(calc_dist(p3, intersezione2) -d3j) < UWB_ERRORE){
                        consistente = true;
                    }
                 }

                 if(consistente){
                    // Se il nodo j è un inlier, lo segno come tale
                    inlierAttuali[j] = true;
                    inlierCount++;
                 }
            }
        }

        if(inlierCount > bestInlierCount){
            // Se il numero di inlier attuali è maggiore del migliore trovato finora
            // allora aggiorno i nodi migliori e il contatore degli inlier
            bestInlierCount = inlierCount;
            indici_nodi[0] = nodo1;
            indici_nodi[1] = nodo2;
            indici_nodi[2] = nodo3;
            bestP1 = p1;
            bestP2 = p2;
            bestP3 = p3;

            // aggiorno il vettore degli inlier migliori
            for(int k = 0; k<numNodi; k++){
                inlier[k] = inlierAttuali[k];
            }
        }

        free(inlierAttuali); // Libero la memoria allocata per gli inlier attuali
    }

    // Fase di correzione delle distanze usando miglior modello
    // Controllo se c'è un modello valido da cui partire
    if(bestInlierCount > 2 &&  indici_nodi[0] != -1) {

        Point2D* coordinate_stimate = (Point2D*)malloc(numNodi * sizeof(Point2D));
        if(!coordinate_stimate){
            perror("Errore allocazione memoria per coordinate stimate");
            exit(EXIT_FAILURE);
            return;
        }
        bool* flag_coordinate = (bool*)calloc(numNodi, sizeof(bool));
        if(!flag_coordinate){
            perror("Errore allocazione memoria per flag coordinate");
            exit(EXIT_FAILURE);
            return;
        }

        // Inizializzo le coordinate stimate e i flag
        coordinate_stimate[indici_nodi[0]] = bestP1;
        coordinate_stimate[indici_nodi[1]] = bestP2;
        coordinate_stimate[indici_nodi[2]] = bestP3;
        flag_coordinate[indici_nodi[0]] = true;
        flag_coordinate[indici_nodi[1]] = true;
        flag_coordinate[indici_nodi[2]] = true;

        // Stimo coordinate degli altri nodi inlier
        for(int k = 0; k < numNodi; ++k){
            if(inlier[k] && !flag_coordinate[k]){
                double dk_to_best0 = distanze[k][indici_nodi[0]];
                double dk_to_best1 = distanze[k][indici_nodi[1]];
                double dk_to_best2 = distanze[k][indici_nodi[2]];

                // Ricalcolo i nodi usando calcolo intersezione cerchi
                Point2D intersezione1, intersezione2;
                int num_intersezioni = calcolaIntersezioneCerchi(bestP1, dk_to_best0, bestP2, dk_to_best1, &intersezione1, &intersezione2);
                if(num_intersezioni > 0){
                    if(num_intersezioni == 1){
                        // Se c'è un solo punto di intersezione, lo uso
                        coordinate_stimate[k] = intersezione1;
                    } else {
                        // Se ci sono due punti di intersezione, prendo quello più vicino al nodo 3
                        if(fabs(calc_dist(bestP3, intersezione1) - dk_to_best2) < fabs(calc_dist(bestP3, intersezione2) - dk_to_best2)){
                            coordinate_stimate[k] = intersezione1;
                        } else {
                            coordinate_stimate[k] = intersezione2;
                        }
                    }
                    flag_coordinate[k] = true; // Segno che ho calcolato le coordinate
                } else {
                    inlier[k] = false; // Se non riesco a calcolare le coordinate, non è un inlier  
                }
            }
        }

        // Correggo le distanze tra tutti i nodi che sono stati posizionati 
        // in modo da essere consistenti con il modello migliore
        for(int k = 0; k < numNodi; ++k){
            if(!inlier[k] || !flag_coordinate[k]){
                continue; // Se non è un inlier o non ho calcolato le coordinate, salto
            }
            // Calcolo la distanza tra il nodo k e gli altri
            for(int c = k + 1; c < numNodi; ++c){
                if(!inlier[c] || !flag_coordinate[c]){
                    continue; // Se non è un inlier o non ho calcolato le coordinate, salto
                }
                // Calcolo la distanza tra i nodi k e c
                double dkc = calc_dist(coordinate_stimate[k], coordinate_stimate[c]);
                // Correggo la distanza nella matrice delle distanze
                if(fabs(distanze[k][c] - dkc) > UWB_ERRORE * 0.5){
                    distanze[k][c] = dkc;
                    distanze[c][k] = dkc;
                }
                
            }
        }
        free(coordinate_stimate); // Libero la memoria allocata per le coordinate stimate
        free(flag_coordinate); // Libero la memoria allocata per i flag delle coordinate

    } else {
        printf("Nessun modello valido trovato.\n");
    }

    //free(inlier); // Libero la memoria allocata per gli inlier
    
}

void jacobi_eigenvalue(int N, double **A, double **V, double *autovalori, int max_iter, double tol) {
    // Inizializzo V come matrice identità
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i == j) {
                V[i][j] = 1.0;
            } else {
                V[i][j] = 0.0;
            }
        }
    }

    for(int iter = 0; iter < max_iter; iter++) {
        double max_off_diag = 0.0;
        int p = -1, q = -1;

        // Trovo elemento fuori diagonale massimo
        for (int i = 0; i < N; i++) {
            for (int j = i + 1; j < N; j++) {
                if (fabs(A[i][j]) > max_off_diag) {
                    max_off_diag = fabs(A[i][j]);
                    p = i;
                    q = j;
                }
            }
        }

        // Se il massimo è sotto la tolleranza, esco
        if (max_off_diag < tol) {
            break;
        }

        // Calcolo gli autovalori e autovettori
        double app = A[p][p];
        double aqq = A[q][q];
        double apq = A[p][q];

        double tau = (aqq - app) / (2 * apq);
        double t;


        if(fabs(apq)<DBL_EPSILON){
            t = 0.0;
        } else if(fabs(tau)<DBL_EPSILON) {
            t = (tau >= 0) ? 1.0 : -1.0; // no div per zero
            if(tau==0) t = (app<=apq) ? 1.0 : -1.0;
        } else if (fabs(tau) > 1.0/DBL_EPSILON){
            t = 1.0 / (2.0 * tau);
        } else {
            t = ((tau>=0)? 1.0 : -1.0) / (fabs(tau) + sqrt(1.0 + tau*tau));
        }

        double c = 1.0 / sqrt(1 + t*t);
        double s = t * c;

        // Aggiorno la matrice A
        A[p][p] = app - t * apq;
        A[q][q] = aqq + t * apq;
        A[p][q] = 0.0;
        A[q][p] = 0.0;

        for (int i = 0; i < N; i++) {
            if (i != p && i != q) {
                double aip = A[i][p];
                double aiq = A[i][q];
                A[i][p] = c * aip - s * aiq;
                A[i][q] = s * aip + c * aiq;
                A[p][i] = A[i][p]; // simmetria
                A[q][i] = A[i][q]; // simmetria
            }
        }

        // Aggiorno la matrice V
        for (int i = 0; i < N; i++) {
            double vip = V[i][p];
            double viq = V[i][q];
            V[i][p] = c * vip - s * viq;
            V[i][q] = s * vip + c * viq;
        }

        // Estrazione degli autovalori
        for (int i = 0; i < N; i++) {
            autovalori[i] = A[i][i];
        }

    }
    

}

void MDS_classico(double distanze[MAX_NODES][MAX_NODES], bool flag_inlier[MAX_NODES], 
    int numNodi, Point2D coordinate[]){

    // L'algoritmo si basa sul calcolo dei nodi inlier fatto in precedenza con RANSAC
    // Dato quindi il vettore di inlier, determino quanti sono
    int numInlier = 0;
    for(int i = 0; i<numNodi; i++){
        if(flag_inlier[i]){
            numInlier++;
        } 
    }

    // Controllo che gli inlier siano almeno 3
    if(numInlier < 3){
        printf("Non ci sono abbastanza inlier per eseguire MDS.\n");
        return;
    }

    // Creo un vettore in cui salvo gli indici degli inlier
    int* indici_inlier = (int*)malloc(numInlier * sizeof(int));
    if(!indici_inlier){
        perror("Errore allocazione memoria per indici inlier");
        exit(EXIT_FAILURE);
        return;
    }

    // Popolo il vettore degli indici inlier
    int index = 0;
    for(int i = 0; i<numNodi; i++){
        if(flag_inlier[i]){
            indici_inlier[index++] = i;
        }
    }

    // Creo la matrice di distanza quadratica
    double** D_sq = (double**)malloc(numInlier * sizeof(double*));
    if(!D_sq){
        perror("Errore allocazione memoria per matrice D_sq");
        //libero la memoria allocata per altro
        free(indici_inlier);
        exit(EXIT_FAILURE);
        return;
    }
    for(int i = 0; i<numInlier; i++){
        D_sq[i] = (double*)malloc(numInlier * sizeof(double));
        if(!D_sq[i]){
            perror("Errore allocazione memoria per matrice D_sq");
            //libero la memoria allocata per altro
            for(int j = 0; j<i; j++){
                free(D_sq[j]);
            }
            free(D_sq);
            free(indici_inlier);
            exit(EXIT_FAILURE);
            return;
        }
        for(int j = 0; j<numInlier; j++){ // Determino le entry della matrice
            if(i == j){
                D_sq[i][j] = 0.0;
            } else {
                D_sq[i][j] = pow(distanze[indici_inlier[i]][indici_inlier[j]], 2);
            }
        }
    }

    // Applico la doppia centratura
    double* row_means = (double*)calloc(numInlier, sizeof(double));
    double* col_means = (double*)calloc(numInlier, sizeof(double));
    double grand_mean = 0.0;

    if(!row_means || !col_means){
        perror("Errore allocazione memoria per row_means o col_means");
        //libero la memoria allocata per altro
        for(int i = 0; i<numInlier; i++){
            free(D_sq[i]);
        }
        free(D_sq);
        free(indici_inlier);
        if(row_means) free(row_means);
        if(col_means) free(col_means);
        exit(EXIT_FAILURE);
        return;
    }

    // Popolo i vettori delle medie
    for(int i = 0; i<numInlier; i++){
        for(int j = 0; j<numInlier; j++){
            row_means[i] += D_sq[i][j];
            col_means[j] += D_sq[i][j];
        }
    }

    // Determino la media globale
    for(int i = 0; i<numInlier; i++){
        grand_mean += row_means[i];
        row_means[i] /= numInlier; // Media della riga
        col_means[i] /= numInlier; // Media della colonna
    }
    grand_mean /= (numInlier * numInlier); // Media globale

    // Applico la doppia centratura
    double** B = (double**)malloc(numInlier * sizeof(double*));
    if(!B){
        perror("Errore allocazione memoria per matrice B");
        //libero la memoria allocata per altro
        for(int i = 0; i<numInlier; i++){
            free(D_sq[i]);
        }
        free(D_sq);
        free(indici_inlier);
        free(row_means);
        free(col_means);
        exit(EXIT_FAILURE);
        return;
    }

    for(int i = 0; i<numInlier; i++){
        B[i] = (double*)malloc(numInlier * sizeof(double));
        if(!B[i]){
            perror("Errore allocazione memoria per matrice B");
            //libero la memoria allocata per altro
            for(int j = 0; j<i; j++){
                free(B[j]);
            }
            free(B);
            for(int j = 0; j<numInlier; j++){
                free(D_sq[j]);
            }
            free(D_sq);
            free(indici_inlier);
            free(row_means);
            free(col_means);
            exit(EXIT_FAILURE);
            return;
        }
        for(int j = 0; j<numInlier; j++){
            B[i][j] = -0.5 * (D_sq[i][j] - row_means[i] - col_means[j] + grand_mean);
        }
    }

    // Libero memoria
    for(int i = 0; i<numInlier; i++){
        free(D_sq[i]);
    }
    free(D_sq);
    free(row_means);
    free(col_means);

    // Calcolo gli autovalori e autovettori della matrice B
    double* autovalori = (double*)malloc(numInlier * sizeof(double));
    double** autovettori = (double**)malloc(numInlier * sizeof(double*));
    if(!autovalori || !autovettori){
        perror("Errore allocazione memoria per autovalori o autovettori");
        //libero la memoria allocata per altro
        for(int i = 0; i<numInlier; i++){
            free(B[i]);
        }
        free(B);
        free(indici_inlier);
        if(autovalori) free(autovalori);
        if(autovettori){
            for(int i = 0; i<numInlier; i++){
                free(autovettori[i]);
            }
            free(autovettori);
        }
        exit(EXIT_FAILURE);
        return;
    }

    for(int i = 0; i<numInlier; i++){
        autovettori[i] = (double*)malloc(numInlier * sizeof(double));
        if(!autovettori[i]){
            perror("Errore allocazione memoria per autovettori");
            //libero la memoria allocata per altro
            for(int j = 0; j<i; j++){
                free(autovettori[j]);
            }
            free(autovettori);
            for(int j = 0; j<numInlier; j++){
                free(B[j]);
            }
            free(B);
            free(indici_inlier);
            free(autovalori);
            exit(EXIT_FAILURE);
            return;
        }
    }

    // Uso la funzione di estrazione degli autovalori e autovettori su B
    jacobi_eigenvalue(numInlier, B, autovettori, autovalori, 100, 1e-9);

    // Seleziono i due autovettori e autovalori più grandi, ordinando il vettore
    typedef struct {
        double value;
        int index;
    } EigenPair;

    EigenPair* eigenPairs = (EigenPair*)malloc(numInlier * sizeof(EigenPair));
    if(!eigenPairs){
        perror("Errore allocazione memoria per eigenPairs");
        //libero la memoria allocata per altro
        for(int i = 0; i<numInlier; i++){
            free(B[i]);
        }
        free(B);
        free(indici_inlier);
        free(autovalori);
        for(int i = 0; i<numInlier; i++){
            free(autovettori[i]);
        }
        free(autovettori);
        exit(EXIT_FAILURE);
        return;
    }

    for(int i = 0; i<numInlier; i++){
        eigenPairs[i].value = autovalori[i];
        eigenPairs[i].index = i;
    }

    // Ordino gli autovalori in ordine decrescente
    for(int i = 0; i < numInlier -1; i++){
        for(int j=0; j< numInlier -i -1; j++){
            if(eigenPairs[j].value < eigenPairs[j+1].value){
                EigenPair temp = eigenPairs[j];
                eigenPairs[j] = eigenPairs[j+1];
                eigenPairs[j+1] = temp;
            }
        }
    }

    // Calcolo le coordinate
    // Uso solo i primi due autovalori e autovettori
    // X = V_k * sqrt(lambda_k)
    // con V_k matrice degli autovettori e lambda_k matrice diagonale degli autovalori

    for(int i=0; i< numInlier; i++){
        int global_index = indici_inlier[i]; // mi trovo l'indice dell'inlier nella mappa globale
        coordinate[global_index].x = 0.0; 
        coordinate[global_index].y = 0.0; // Salvo il valore temporaneo

        for(int d=0; d<2 && d < numInlier; d++){ // 2 sta per le due dimensioni
            double lambda = eigenPairs[d].value;
            if(lambda < 0){
                lambda = 0; // Se l'autovalore è negativo, lo metto a 0
            }

            int eigenIndex = eigenPairs[d].index; // Indice dell'autovettore
            double component = autovettori[i][eigenIndex] *sqrt(lambda); // 
            
            coordinate[global_index].x = component; 
            coordinate[global_index].y = component;

        }
        
    }

    // Calcolo le coordinate
    // X = V_k * sqrt(lambda_k)
    // con V_k matrice degli autovettori e lambda_k matrice diagonale degli autovalori
    for(int i=0; i< numInlier; i++){
        int global_index = indici_inlier[i];

        // Inizializza coordinate
        coordinate[global_index].x = 0.0;
        coordinate[global_index].y = 0.0;

        // Assegna coordinata X usando il primo autovalore/autovettore (il più grande)
        if (0 < numInlier && eigenPairs[0].value > 0) { // Controlla che l'autovalore sia positivo
             int eigenIndex0 = eigenPairs[0].index;
             coordinate[global_index].x = autovettori[i][eigenIndex0] * sqrt(eigenPairs[0].value);
        }

        // Assegna coordinata Y usando il secondo autovalore/autovettore
        if (1 < numInlier && eigenPairs[1].value > 0) { // Controlla che l'autovalore sia positivo
             int eigenIndex1 = eigenPairs[1].index;
             coordinate[global_index].y = autovettori[i][eigenIndex1] * sqrt(eigenPairs[1].value);
        }
    }

    // Pulisco memoria
    for(int i=0; i < numInlier; i++){
        free(B[i]);
        if(autovettori[i] != NULL) free(autovettori[i]);
    }
    free(B);
    free(autovalori);
    free(autovettori);
    free(eigenPairs);
    free(indici_inlier);

    //return numInlier;

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

double erroreTopologia(double distanze[MAX_NODES][MAX_NODES], Point2D coordinate[], int numNodi, bool inlier_mask[MAX_NODES][MAX_NODES]){
    double errore = 0.0;
    int inlierCount = 0;
    
    for(int i = 0; i < numNodi; i++){
        for(int j = i + 1; j < numNodi; j++){
            // Considero inlier se distanza è inlier
            if(inlier_mask[i][j]){
                double distCalcolata = calc_dist(coordinate[i], coordinate[j]);
                errore += calcolaErrore(distanze[i][j], distCalcolata);
                inlierCount++;
                //double distCalcolata = calc_dist(coordinate[i], coordinate[j]);
            }
          
        }
    }

    // Normalizzo l'errore per il numero di inlier
    if(inlierCount > 0){
        errore /= inlierCount;
    } else {
        errore = DBL_MAX; // Se non ci sono inlier, l'errore è infinito
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

#if 0
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

#endif

double random_gaussian(double mu, double sigma){
    static int usa_valore_precedente = 0;
    static double valore_precedente;
    double x1, x2, raggio_quadro, fattore;

    if(usa_valore_precedente){
        usa_valore_precedente = 0;
        return mu + sigma * valore_precedente;
    }
    do {
        x1 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        x2 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        raggio_quadro = x1 * x1 + x2 * x2;
    } while (raggio_quadro >= 1.0 || raggio_quadro == 0.0);

    fattore = sqrt(-2.0 * log(raggio_quadro) / raggio_quadro);
    valore_precedente = x2 * fattore;
    usa_valore_precedente = 1;
    return mu + sigma * x1 * fattore;
}

void ottimizzaTopologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[], bool inlier_mask[MAX_NODES][MAX_NODES]){
    // Imposto la temperatura iniziale per essere sempre accettante
    double temperatura = TEMP_INIZIALE;
    //double fattore_raffreddamento = pow(TEMP_FINALE/TEMP_INIZIALE, 1.0/NUM_ITERATIONS);
    double fattore_raffreddamento = 0.99; // Fattore di raffreddamento
    
    double erroreAttuale = erroreTopologia(distanze, coordinate, numNodi, inlier_mask); // TODO: da modificare per usare inlie_mask
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
        if (iterSenzaMiglioramento > 500 ) numNodiPert = fmin(3, numNodi-2);
        if (numNodi-2 <= 0 && numNodi > 0){
            numNodiPert = 1;
        }
        

        for (int p = 0; p < numNodiPert; p++){
            int nodoRandom = 2 + rand() % (numNodi-2);

            // Calcolo un valore non lineare per capire quanto perturbare in base alla temperatura
            double fattore_perturbazione = sqrt(temperatura/TEMP_INIZIALE);

            double perturbazione_standard = UWB_ERRORE * 1.5;

            // Prendo confidenza del nodo (controllando che non sia zero)
            double confidenza = coordinate[nodoRandom].confidenza;
            if (confidenza < 0.01) confidenza = 0.01;

            // Calcolo la perturbazione gaussiana
            double ampiezza_perturbazione = (perturbazione_standard * fattore_perturbazione) / confidenza;

            // Storico dei miglioramenti
            double fattore_escalation = 1.0;
            // Applico nel caso di molteplici tentativi
            if (iterSenzaMiglioramento > 700) {
                fattore_escalation = 1.5;
            } else if (iterSenzaMiglioramento > 300) {
                fattore_escalation = 1.2;
            }

            ampiezza_perturbazione *= fattore_escalation; // aggiorno ampiezza perturbazione in base a n. tentativi senza miglioramenti

            // Definisco un sigma per la perturbazione gaussiana così che
            // il 99% delle perturbazioni siano comprese nell'errore massimo
            double sigma = ampiezza_perturbazione / 3.0;

            temp_coord[nodoRandom].x += random_gaussian(0.0, sigma);
            temp_coord[nodoRandom].y += random_gaussian(0.0, sigma);
        }

        

        // Calcolo il nuovo errore
        double new_error = erroreTopologia(distanze, temp_coord, numNodi, inlier_mask); // TODO: da modificare per usare inlier_mask

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

            if( erroreAttuale < bestErrore - CONVERGENCE_THRESHOLD){
                bestErrore = erroreAttuale; // aggiorno errore migliore
                for(int j = 0; j<numNodi; j++){
                    migliori_coordinate[j] = coordinate[j];
                }
                iterSenzaMiglioramento = 0; // resetto contatore

            } else {
                iterSenzaMiglioramento++;
            }


            #if 0
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
                #endif
        } else {
            iterSenzaMiglioramento++;
        }



        //temperatura *= fattore_raffreddamento; // riduco temperatura

        if(iterSenzaMiglioramento>1000){
            // Aumento la temperatura ad una frazione della temperatura iniziale
            // o un valore basato su differenza tra errore attuale e migliore

            double fattore_riscaldamento = 0.1;
            temperatura = fmax(temperatura * 1.2, TEMP_INIZIALE * fattore_riscaldamento);

            iterSenzaMiglioramento = 0; // resetto contatore
        }

        temperatura *= fattore_raffreddamento; // riduco temperatura
    }

    // ottenuta configurazione migliorata, la salvo
    for(int i=0; i<numNodi;i++){
        coordinate[i] = migliori_coordinate[i];
    }

}


Point2D calcolaMedianaGeometrica(Point2D campioni[], int numCampioni, int maxIter, double tolleranza){
    Point2D mediana = {0.0, 0.0, 0.0};

    // controllo quanti siano i campioni
    if(numCampioni <= 0){
        printf("Errore: numero di campioni non valido\n");
        return mediana;
    }
    if(numCampioni == 1){
        return campioni[0]; // La mediana di un singolo punto è il campione stesso
    }

    // In primo luogo determino il baricentro dei campioni
    for(int i=0; i<numCampioni; i++){
        mediana.x += campioni[i].x;
        mediana.y += campioni[i].y;
    }
    mediana.x /= numCampioni;
    mediana.y /= numCampioni;

    // Itero per calcolare la mediana geometrica
    for(int iter=0; iter<maxIter; iter++){
        double sommaX = 0.0;
        double sommaY = 0.0;
        double sommaConfidenza = 0.0;
        Point2D vecchia_mediana = mediana;

        bool coincide_con_campione = false;
        for(int i = 0; i < numCampioni; i++){
            double distanza = calc_dist(mediana, campioni[i]);

            // Se la distanza è zero, significa che la mediana coincide con un campione
            if(distanza < 1e-9){
                distanza = fmax(1e-9, distanza); 
                if(distanza == 0.0){
                    coincide_con_campione = true;
                }
            }

            // Calcolo il peso inversamente proporzionale alla distanza
            double peso = 1.0 / (distanza);

            sommaX += campioni[i].x * peso;
            sommaY += campioni[i].y * peso;
            sommaConfidenza += peso;
        }

        if(sommaConfidenza < 1e-9){
            // La mediana precedente è l'unica valida
            break;
        }

        mediana.x = sommaX / sommaConfidenza;
        mediana.y = sommaY / sommaConfidenza;


        // Controllo che la distanza tra mediana e quella precedente sia minore della tolleranza
        double distanza = calc_dist(mediana, vecchia_mediana);
        if(distanza < tolleranza){
            if(coincide_con_campione && iter < 2){
                // Se coincide subito, potrebbe trattarsi di un falso positivo, si continua
            }
            else {
                break; // Converge
            }
        }  
    }

    return mediana;
}

void filtroMediana(Point2D coordinate[], int numNodi, int numCampioni){
    
    Point2D** campioni = (Point2D**)malloc(numNodi * sizeof(Point2D*));
    if (campioni == NULL) {
        printf("Errore: impossibile allocare memoria per i campioni\n");
        exit(1);
    }

    // alloco memoria per ogni campione
    for(int i=0;i<numNodi;i++){
        campioni[i] = (Point2D*)malloc(numCampioni*sizeof(Point2D));
        if (campioni[i] == NULL) {
            printf("Errore: impossibile allocare memoria per i campioni\n");
            // libero la memoria già allocata
            for(int j=0;j<i;j++){
                free(campioni[j]);
            }
            free(campioni);
            exit(1);
        }
    }

    // Genero campioni perturbando le coordinate iniziali
    for(int i = 0; i < numCampioni; i++){
        for (int j = 0; j < numNodi; j++) {
            if(i==0){ // Primo campione, nessuna perturbazione
                campioni[j][i] = coordinate[j];
            } else {
                // I primi due nodi sono fissi 
                if (j < 2) {
                    campioni[j][i] = coordinate[j];
                } else {
                    // Sistemo valore di "noise" in base alla confidenza
                    // confidenza = 0.0 -> rumore massimo
                    // confidenza = 1.0 -> nessun rumore
                    double confidenza = coordinate[j].confidenza;
                    if (confidenza < 0.01) confidenza = 0.01; 
                    if(confidenza > 0.99) confidenza = 0.99; 

                    double noise = UWB_ERRORE * (1.0 - confidenza);

                    // Definisco un minimo e un massimo per il rumore
                    noise = fmax(noise, UWB_ERRORE * 0.05); // minimo rumore
                    noise = fmin(noise, UWB_ERRORE * 1.5); // massimo rumore

                    campioni[j][i].x = coordinate[j].x + random_double(-noise, noise);
                    campioni[j][i].y = coordinate[j].y + random_double(-noise, noise);
                    campioni[j][i].confidenza = coordinate[j].confidenza; 


                }
            }
        }   
    }


     // Calcola la mediana delle coordinate per ogni nodo
    for (int i = 2; i < numNodi; i++) {  // Solo per i nodi dopo i primi due
        // Calcola la mediana geometrica per il nodo i
        Point2D mediana = calcolaMedianaGeometrica(campioni[i], numCampioni, 100, 0.01);

        // Aggiorna le coordinate del nodo i con la mediana calcolata
        coordinate[i].x = mediana.x;
        coordinate[i].y = mediana.y;
        coordinate[i].confidenza = mediana.confidenza; 
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
    //printf("Errore medio: %.2f cm\n", erroreTopologia(distanze, coordinate, numNodi));

}


void costruisci_topologia(double distanze[MAX_NODES][MAX_NODES], int numNodi, Point2D coordinate[]){
    int tentativi;
    double erroreMinimo = DBL_MAX;
    Point2D tempCoord[MAX_NODES];

    // Generatore numeri casuali per generare configurazioni random
    srand(time(NULL));

    // -- RANSAC --
    // Verifico con RANSAC gli outlier definendo una maschera
    bool inlier[MAX_NODES];
    // Inizializzo la maschera a false
    for(int i = 0; i < numNodi; i++){
        inlier[i] = false;
    }
    ransac_outlier(distanze, numNodi, inlier);

    // Costruisco inlier_mask
    bool inlier_mask[MAX_NODES][MAX_NODES];
    for (int i = 0; i < numNodi; ++i) {
        for (int j = 0; j < numNodi; ++j) {
            if (i == j) {
                inlier_mask[i][j] = false;
            } else {
                inlier_mask[i][j] = inlier[i] && inlier[j];
            }
        }
    }

    // -- MDS --
    // Utile per posizionamento iniziale
    // Creo vettore di coordinate iniziali temporanee
    //Point2D tempCoord[MAX_NODES];
    for(int i=0; i<numNodi; i++){
        tempCoord[i].x = 0.0;
        tempCoord[i].y = 0.0;
        tempCoord[i].confidenza = 0.0;
    }

    // Richiamo MDS classico
    MDS_classico(distanze, inlier, numNodi, tempCoord);

    // Imposta la confidenza iniziale dopo MDS
    // I nodi usati da MDS (inlier) avranno coordinate aggiornate.
    for(int i = 0; i < numNodi; ++i) {
        if(inlier[i]) { // Nodi processati da MDS
            tempCoord[i].confidenza = 1.0; // Alta confidenza iniziale
        } else { // Nodi non processati da MDS 
            // Mantengono coordinate iniziali e bassa confidenza
            tempCoord[i].confidenza = 0.1; // Bassa confidenza
        }
    }

    // -- Simulated Annealing --
    Point2D migliori_coordinate[MAX_NODES]; // vettore temporaneo in cui salvo coordinate più precise
    double bestErrore = DBL_MAX;

    for(int start = 0; start < NUM_CAMPIONI; start++){
        Point2D tempSACoord[MAX_NODES];

        for (int i = 0; i < numNodi; ++i) {
            tempSACoord[i] = tempCoord[i];
            // if (start > 0 && i >=2) { // dati i nodi 0 e 1 sono ancore fisse
            //    tempSACoord[i].x += random_double(-0.05, 0.05) * UWB_ERRORE;
            //    tempSACoord[i].y += random_double(-0.05, 0.05) * UWB_ERRORE;
            // }
        }

        ottimizzaTopologia(distanze, numNodi, tempSACoord, inlier_mask);
        
        // Applico filtro mediana
        filtroMediana(tempSACoord, numNodi, NUM_MEDIANA);

        // Valuto errore attuale
        double erroreAttuale = erroreTopologia(distanze, tempSACoord, numNodi, inlier_mask);

        // Aggiorna la soluzione migliore trovata finora
        if (erroreAttuale < bestErrore) {
            bestErrore = erroreAttuale;
            for (int i = 0; i < numNodi; ++i) {
                migliori_coordinate[i] = tempSACoord[i];
            }
        }

    }

    // Copia le migliori coordinate trovate nel vettore di output
    for (int i = 0; i < numNodi; ++i) {
        coordinate[i] = migliori_coordinate[i];
    }

    visualizzaTopologiaAvanzata(coordinate, numNodi);
    

    #if 0
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
    #endif
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

