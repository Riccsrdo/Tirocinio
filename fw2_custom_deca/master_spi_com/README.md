## Funzionamento
Il codice in questa cartella può essere runnato su un microcontrollore per gestire le chiamate
al dispositivo dwm1001-dev tramite SPI.
Per farlo basta compilare con `gcc main.c dwm_spi_master.c -o dwm_master_app` e poi eseguire con `./dwm_master_app`.
La lista di comandi inviabili al dispositivo è consultabile nel file `.h`.

Per avviare opportunamente la comunicazione, occorre eseguire la funzione:
```C
int ret = dwm_spi_init(SPI_DEVICE, SPI_SPEED, SPI_MODE);
if (ret != 0) {
    fprintf(stderr, "Impossibile inizializzare SPI.\n");
    return EXIT_FAILURE;
}
```
Essa si occuperà di avviare la comunicazione con il dispositivo DWM usando SPI, ed i parametri definiti nel file .h.

E' poi possibile eseguire le funzioni desiderate.

Infine, prima di ritornare, eseguire il seguente frammento di codice:
```C
dwm_spi_close();
```
