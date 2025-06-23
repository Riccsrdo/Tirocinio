# Filtro UKF base per correzione distanze soggette a NLOS o MP

## Introduzione
Il filtro UKF, ovvero Unscented Kalman Filter, si occuperà di gestire le misurazioni fallate ottenute con la misurazione UWB e di correggerle sulla base di altri dati, quali l'accelerazione del dispositivo.
Se la distanza misurata con UWB si distacca troppo dalla distanza predetta, il filtro tenterà di correggerla.

## Funzionamento Teorico
Il filtro opera facendo uso di una serie di elementi di algebra lineare, di seguito introdotti.

### Vettore di Stato
Il vettore di stato `x` descrive lo stato del sistema in un determinato istante, si tratta della migliore ipotesi attuale effettuata dal filtro. Esso contiene dati riguardo la distanza e la velocità relativa. Dato che operiamo in uno spazio bidimensionale, allora conterrà i seguenti dati `[d_x, d_y, v_x, v_y]`, ovvero le componenti sia sull'asse x che sull'asse y della distanza e della velocità.

### Modello di moto
Si tratta di una "funzione" `f(x)` che prende lo stato attuale e tenta di predirre la nuova posizione dopo un intervallo di tempo `delta_t`. La teoria è basata sulla fisica, per cui dato `x`, se conosco la distanza, velocità, e l'intervallo di tempo, potrò calcolare la nuova posizione con `nuova_distanza = vecchia_distanza + velocità * tempo`.

### La misurazione
La misurazione `z` è invece data dal vettore di dati provenienti dal sensore UWB.

### La matrice di covarianza dello stato
Si tratta di una matrice, `P`, che descrive l'incertezza riguardo alla stima dello stato. Con un valore grande si ha una grande incertezza, il che implica che non si è sicuri riguardo all'effettivo valore della distanza, mentre con un valore più piccolo il filtro è più certo. L'obiettivo del filtro è rendere questa matrice il più piccola possibile, per cui nella fase di previsione del filtro il valore di P aumenta, ma nella fase di correzione si cerca di farlo diminuire.

### La covarianza del rumore di processo
La matrice `Q` descrive l'incertezza del modello di moto. Tale incertezza è dovuta ai possibili movimenti repentini che il futuro robot potrà avere, questo perché esso potrebbe girarsi, decelerare, mentre il modello assume che la velocità sia costante. Proprio questa convinzione viene rappresentata dal livello di incertezza data dalla matrice Q, che se vanta di un valore elevato allora il filtro si fiderà di più delle nuove misurazioni, mentre con valori bassi si fiderà maggiornamente delle predizioni. 

### La covarianza del rumore di misurazione
Si tratta della matrice `R` che ha lo scopo di descrivere l'incertezza della misurazione del sensore, basata sull'incertezza del dato ottenuto tramite misurazioni UWB, con un valore di R alto si ha che il sensore è inaffidabile, mentre con R basso ho una buona fiducia del sensore. 

### Guadagno di Kalman
La matrice `K` viene calcolata ad ogni passo e bilancia l'incertezza della misurazione con quella della previsione. Se il valore P è piccolo ed R grande (quindi filtro sicuro dello stato e misure UWB non sicure) allora il valore K sarà basso. Al contrario se P è grande ed R è piccolo il valore K sarà alto. 

### Punti sigma e trasformazione unscented - NON PRESENTI NEL FILTRO DI KALMAN STANDARD
Al contrario dell'EKF, che tenta di linearizzare i dati, portando potenzialmente a molti errori di approssimazione, l'UKF prende la migliore ipotesi, genera una serie di punti campione attorno ad essi detti `sigma point`, e per tutti questi punti generati calcola f(x), ne calcola la posizione media finale ottenuta e la nuova dispersione. 

### Ciclo di funzionamento

1) Il filtro calcola f(x) e prevede la nuova posizione, con l'insicurezza P che incrementa all'incrementare della matrice di covarianza del rumore di processo Q. 
2) Riceve la misura z dal sensore UWB, con l'incertezza R associata.
3) Vengono calcolati:
    - Guadagno di Kalman K
    - Nuova previsione, creando un mix tra previsione e misurazione
    - Aggiorna la sua insicurezza P