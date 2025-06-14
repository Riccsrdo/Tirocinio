# Filtro di Kalman Esteso per la correzione misurazioni UWB

## Introduzione
In questa cartella sarà contenuto il codice dell'implementazione del filtro di Kalman esteso per la correzione delle misurazioni UWB. [TBU]

Tale filtro dovrà essere applicato sui singoli dispositivi, dopo aver collezionato le misurazioni, per tentare di correggere le misurazioni con tecnologia UWB sottoposte ad un rumore a causa di un ambiente NLOS o MP.

## Funzionamento
Il filtro esteso di Kalman è utile per rappresentare modelli non lineari.
Fa uso delle derivate parziali (rappresentate con matrice jacobiana) per linearizzare il modello, e di una serie di operazioni tra matrici.

Tra le matrici fondamentali di cui fa uso si ha:
- Vettore x, ovvero un vettore in R^4 contenente posizione su x, posizione su y, velocità su x e velocità su y
- Matrice F, ovvero la matrice di transizione, si tratta di una matrice identità 4x4 dove nella cella [0,2] e nella cella [1,3] c'è il valore del delta t, ovvero la differenza temporale dall'ultima esecuzione del filtro
- Matrice di covarianza Q, aiuta a gestire il rumore dovuto a cambiamenti di accelerazione, angolo, ecc.
- vettore z delle distanze, date n distanze allora sarà in R^n
- una funzione h(x), che calcola le distanze attese da uno stato x
- matrice jacobiana della misura, si tratta di una matrice di dimensione nx4 (con n numero dei dispositivi) 