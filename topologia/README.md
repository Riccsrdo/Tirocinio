## Introduzione
L'algoritmo è sviluppato con lo scopo di calcolare una topologia dei punti nello spazio date le distanze di tutti i nodi verso tutti gli altri nodi della rete, calcolate con tecnologia UWB, tentando di correggere anche l'errore di misurazione.

## Funzionamento
In primis prende in input una matrice NxN di distanze con la cella [i,j] che indica distanza tra nodo i e j.

Successivamente vengono analizzate le distanze per verificare misurazioni possibili e controllare errore, per farlo si fa uso del principio di disuguaglianza triangolare. Esso ci dice che in un qualsiasi triangolo, la somma dei due lati deve essere maggiore del terzo. Se c'è una violazione significativa, che va oltre il doppio del margine di errore di UWB, allora c'è una correzione. Quello che si fa è effettuare l'analisi per ogni coppia di nodi (i,j) verificando per ogni altro nodo k. Se più di 3 nodi k portano ad una disuguaglianza triangolare allora c'è la possibilità che la distanza tra i e j sia sovrastimata. Per poterla correggere si procede allora con una correzione che calcola la media delle distanze indirettamente.
Per sistemare eventuali misurazioni errate che si distaccano di molto dal valore effettivo si procede poi con un algoritmo di Multidimensional Scaling (MDS) per cui si posizionano in modo grossolano i nodi all'inizio, ponendo sempre il primo nell'origine e gli altri in modo circolare. Successivamente attraverso una serie di iterazioni si tenta di correggere il posizionamento calcolando per ogni coppia di nodi il deltaX e il deltaY e quindi la distanza tra i due nodi. Si calcola una scala di aggiornamento del posizionamento sulla base della differenza tra la distanza attuale calcolata sulla base dei delta, con quella effettiva che viene fornita nel vettore.
Infine, per ogni coppia di nodi, si ricalcola la distanza ottenuta con i delta e la si confronta con quella effettiva. Se supera di molto il margine di errore di UWB allora la distanza stimata è probabilmente un outlier e quindi va corretta.

Viene poi creato il sistema di riferimento settando il primo nodo all'origine, e il secondo nodo successivo sull'asse positivo delle x con la x uguale alla distanza tra il primo e secondo nodo.
Per ogni nodo successivo si fa uso della legge del coseno per determinare l'angolo tra il nodo e l'origine data da `cos = (d_{0,i}^2 + d_{0,1}^2 - d_{1,i}^2) / (2 * d_{0,i} * d_{0,1})` e si calcolano le coordinate potenziali con `x = d_{0,i} * cos` e `y = d_{0,i} * sin`. Successivamente verifico se le distanze tra i nodi già posizionati sul piano cartesiano e quelle effettive calcolate con UWB corrispondono o sono diverse, se sono diverse allora devo invertire il valore di y.

Si procede poi con un'ottimizzazione robusta facendo uso del Simulated Annealing. 
Il simulated annealing è un algoritmo ispirato alla ricottura dei metalli, per cui un materiale continua ad essere riscaldato e poi raffreddato per eliminare difetti e raggiungere stato di minima energia. A livello concreto quello che fa è comportarsi come una ricerca casuale, per cui se esiste una configurazione peggiore la si accetta all'inizio con alta probabilità, per poi ridurre di volta in volta questa probabilità in maniera graduale. Alla fine l'algoritmo avrà una probabilità molto bassa di accettare valori che si distaccano dalla configurazione ideale.
La formula di cui fa uso è `P(accettare) = e^(-ΔE/T)`, dove `ΔE` è l'aumento dell'errore, `T` è la temperatura attuale. Se si ha un valore di ΔE negativo allora P sarà maggiore di 1 e quindi si accetta sempre una configurazione, mentre se è positivo P diminuisce con l'aumentare di ΔE e con il diminuire di T.
Allora quello che si fa è valutare la configurazione iniziale, impostando una temperatura di T=1.0. Dato poi un numero di iterazioni fisso, si seleziona casualmente un nodo (eccetto i primi due), si perturba la posizione con ampiezza proporzionale alla temperatura, si calcola errore della nuova configurazione, e se l'errore è diminuito rispetto a prima si accetta sempre la configurazione, se è aumentato si accetta in base alla probabilità come spiegato in precedenza. Si conserva sempre la migliore configurazione trovata.

Successivamente si fa uso di un approccio multi-start per evitare minimi locali, effettuando 5 tentativi partendo da condizioni leggermente diverse e conservando la soluzione migliore trovata, ovvero quella con errore minore.

Si ha poi una stabilizzazione con filtro di mediana. Si prendono tutti i nodi, eccetto i due fissati all'inizio, generando posizioni alternative con piccole perturbazioni (entro il margine di errore di UWB), ordinando i valori di x e y ottenuti in modo separato e prendendo il valore mediano per entrambe le coordinate. La mediana è più efficace della media in quanto è resistente all'influenza di valori estremi.

Si verifica poi la consistenza, calcolando le distanze tra i nodi nella topologia costruita e confrontandole con quelle originariamente passate. Si verifica se ci sono discrepanze maggiori rispetto al margine di errore UWB, e si calcolano statistiche come errore massimo e medio.




