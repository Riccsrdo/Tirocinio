#include "dwm_spi_master_rpi.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>      
#include <unistd.h>    
#include <sys/ioctl.h>  
#include <linux/spi/spidev.h> 

// Variabili statiche per la gestione del device SPI
static int spi_fd = -1; // File descriptor per il device SPI
static uint8_t spi_mode = 0;
static uint8_t spi_bits = 8;
static uint32_t spi_speed = 2000000; // Default 2 MHz

// Funzione interna per gestire errori
static void print_spi_error(const char* action) {
    perror(action); // Stampa il messaggio di errore di sistema
}


int dwm_spi_init(const char* device, uint32_t speed, uint8_t mode) {
    int ret;

    if (spi_fd >= 0) { // se il file descriptor è stato già inizializzato, stampa errore
        fprintf(stderr, "Errore: SPI già inizializzato.\n");
        return -1; // Già aperto
    }

    spi_fd = open(device, O_RDWR); // altrimenti apri la connessione SPI
    if (spi_fd < 0) {
        print_spi_error("Errore apertura device SPI");
        return -1;
    }

    spi_mode = mode; // imposta la modalità (0,1,2,3)
    spi_speed = speed; // e la velocità

    // Imposta modalità SPI (CPOL, CPHA)
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (ret == -1) {
        print_spi_error("Errore impostazione SPI mode (WR)");
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (ret == -1) {
        print_spi_error("Errore lettura SPI mode (RD)");
        // Continua comunque? Forse non critico.
    }

    // Imposta bits per word
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
    if (ret == -1) {
        print_spi_error("Errore impostazione bits per word (WR)");
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits);
    if (ret == -1) {
        print_spi_error("Errore lettura bits per word (RD)");
    }

    // Imposta velocità massima
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (ret == -1) {
        print_spi_error("Errore impostazione velocità massima (WR)");
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
     if (ret == -1) {
        print_spi_error("Errore lettura velocità massima (RD)");
    }

    printf("SPI Inizializzato: %s\n", device);
    printf("  Mode: %d, Bits: %d, Speed: %d Hz\n", spi_mode, spi_bits, spi_speed);

    return 0; // Successo
}

void dwm_spi_close(void) {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
        printf("Interfaccia SPI chiusa.\n");
    }
}

int dwm_spi_transfer(uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    if (spi_fd < 0) {
        fprintf(stderr, "Errore: SPI non inizializzato.\n");
        return -1;
    }

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = len,
        .delay_usecs = 0, // Nessun ritardo tra i byte
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
        // .cs_change = 0, // Non cambiare CS durante il trasferimento (se len > 1)
    };

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr); // Invia 1 struttura spi_ioc_transfer
    if (ret < 1) { // ioctl ritorna 0 o -1 in caso di fallimento, >0 in caso di successo
        print_spi_error("Errore durante SPI transfer (ioctl)");
        return -1;
    }
    return 0; // Successo
}

int dwm_send_command(uint8_t command) {
    uint8_t tx = command;
    uint8_t rx; // Buffer di ricezione (ignorato)
    return dwm_spi_transfer(&tx, &rx, 1);
}

int dwm_send_command_with_arg(uint8_t command, uint8_t argument) {
    uint8_t tx[2] = {command, argument};
    uint8_t rx[2]; // Buffer di ricezione (ignorato)
    return dwm_spi_transfer(tx, rx, 2);
}

int dwm_request_distances(ResponderInfo* responder_array, int max_responders, uint8_t* out_valid_count) {
    if (!responder_array || !out_valid_count || max_responders <= 0) {
        fprintf(stderr, "Errore: Parametri non validi per dwm_request_distances.\n");
        return -1;
    }

    uint8_t command = CMD_GET_DISTANCES;
    uint8_t rx_buffer[sizeof(double)]; // Buffer per leggere i byte della distanza + 1 per count/id
    uint8_t tx_dummy = 0x00; // Byte dummy da inviare per generare clock
    int ret;

    *out_valid_count = 0; // Inizializza conteggio a 0

    // Invalida tutti i record nell'array fornito
    for (int i = 0; i < max_responders; ++i) {
        responder_array[i].valid = 0;
    }

    // 1. Invia il comando GET_DISTANCES
    //    La prima risposta va ignorata in quanto essendo full duplex è quella inutile ottenuta mentre si invia comando
    ret = dwm_spi_transfer(&command, rx_buffer, 1);
    if (ret != 0) return -1;

    // 2. Leggi il byte del conteggio inviando un dummy byte
    usleep(100); // Attesa 100 microsecondi 
    ret = dwm_spi_transfer(&tx_dummy, rx_buffer, 1); // Legge count in rx_buffer[0]
    if (ret != 0) return -1;

    uint8_t count = rx_buffer[0];
    if (count == 0) {
        printf("  Nessuna distanza valida riportata.\n");
        return 0; // Successo, ma 0 distanze
    }
    if (count > MAX_DWM_RESPONDERS || count > max_responders) {
        fprintf(stderr, "  Errore: Conteggio ancore non valido ricevuto: %d\n", count);
        return -1; // Errore nei dati ricevuti
    }

    printf("  Conteggio ancore valide ricevuto: %d\n", count);
    *out_valid_count = count;

    // 3. Leggi ID e distanza per ogni ancora
    for (uint8_t i = 0; i < count; ++i) {
        // Leggi ID
        ret = dwm_spi_transfer(&tx_dummy, rx_buffer, 1); // Legge ID in rx_buffer[0]
        if (ret != 0) return -1;
        uint8_t current_id = rx_buffer[0];

        // Leggi 8 byte della distanza
        ret = dwm_spi_transfer(&tx_dummy, rx_buffer, sizeof(double)); // Legge 8 byte in rx_buffer
        if (ret != 0) return -1;

        // Popola l'array del chiamante
        if (current_id < max_responders) {
             responder_array[current_id].id = current_id;
             memcpy(&(responder_array[current_id].distance), rx_buffer, sizeof(double));
             responder_array[current_id].valid = 1; // Marca come valido
             printf("    ID: %d, Distanza bytes: %02x%02x%02x%02x%02x%02x%02x%02x, Val: %.3f m\n",
                    current_id, rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3],
                    rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7],
                    responder_array[current_id].distance);
        } else {
            fprintf(stderr, "  Errore: ID ancora non valido (%d) ricevuto.\n", current_id);
             // Potrebbe essere necessario continuare a leggere per svuotare il buffer SPI
        }
    }

    return 0; // Successo
}