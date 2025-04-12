#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> 
#include <signal.h> 
#include "dwm_spi_master.h"

// Definizione del device SPI da usare (SPI0, Chip Select 0)
#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED 2000000 // 2 MHz
#define SPI_MODE 2

// Array per memorizzare i dati delle distanze
ResponderInfo distance_data[MAX_DWM_RESPONDERS];

// Flag per gestire l'uscita pulita con Ctrl+C
volatile sig_atomic_t running = 1;

void sigint_handler(int sig) {
    (void)sig; // Evita warning unused parameter
    printf("\nCtrl+C ricevuto, terminazione...\n");
    running = 0;
}

int main() {
    uint8_t valid_count = 0;
    int ret;

    // Imposta handler per SIGINT (Ctrl+C)
    signal(SIGINT, sigint_handler);

    // Inizializza SPI
    ret = dwm_spi_init(SPI_DEVICE, SPI_SPEED, SPI_MODE);
    if (ret != 0) {
        fprintf(stderr, "Impossibile inizializzare SPI.\n");
        return EXIT_FAILURE;
    }
    
    // printf("Invio comando SET_MODE_INIT...\n");
    // dwm_send_command(CMD_SET_MODE_INIT);
    // sleep(1); // Pausa per dare tempo al DWM di cambiare modo

    printf("Avvio ciclo di richiesta distanze (premi Ctrl+C per uscire)...\n");

    while (running) {
        printf("\nRichiesta distanze...\n");
        ret = dwm_request_distances(distance_data, MAX_DWM_RESPONDERS, &valid_count);

        if (ret == 0) {
            if (valid_count > 0) {
                 printf("--- Distanze Ricevute (%u valide) ---\n", valid_count);
                 for (int i = 0; i < MAX_DWM_RESPONDERS; ++i) {
                    if (distance_data[i].valid) {
                        printf("  Anchor %d: %.3f m\n",
                               distance_data[i].id,
                               distance_data[i].distance);
                    }
                }
            }
            // Se valid_count è 0, la funzione dwm_request_distances ha già stampato il messaggio.
        } else {
            fprintf(stderr, "Errore durante la richiesta delle distanze.\n");
            // Potrebbe essere utile aggiungere una pausa maggiore in caso di errore
            sleep(2);
        }

        sleep(1); // Attendi 1 secondo tra le richieste
    }

    // Cleanup
    dwm_spi_close();

    return EXIT_SUCCESS;
}