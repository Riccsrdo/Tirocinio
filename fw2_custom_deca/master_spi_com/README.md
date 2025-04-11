## Funzionamento
Il codice in questa cartella può essere runnato su un microcontrollore per gestire le chiamate
al dispositivo dwm1001-dev tramite SPI.
Per farlo basta compilare con `gcc main.c dwm_spi_master.c -o dwm_master_app` e poi eseguire con `./dwm_master_app`.
La lista di comandi inviabili al dispositivo è consultabile nel file `.h`.