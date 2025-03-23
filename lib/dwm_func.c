#include"dwm_func.h"

int spi_fd = -1;

// Inizializza SPI
int spi_init() {
    int ret;
    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) {
        perror("Impossibile aprire il dispositivo SPI");
        return -1;
    }
    
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0) { perror("Impossibile impostare la modalità SPI"); return -1; }
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0) { perror("Impossibile impostare i bits per word"); return -1; }
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0) { perror("Impossibile impostare la velocità massima"); return -1; }
    
    printf("SPI inizializzato: modalità %d, %d bits per word, %d Hz velocità massima\n", mode, bits, speed);
    return 0;
}

// Funzione di trasferimento SPI
int spi_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, int length) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = length,
        .speed_hz = speed,
        .delay_usecs = 0,
        .bits_per_word = bits,
    };
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("Impossibile inviare messaggio SPI");
        return -1;
    }
    return ret;
}

// Chiudi SPI
void spi_close() {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
        printf("SPI chiuso\n");
    }
}

// Forza lo stato Idle
void ensure_device_in_idle_state() {
    uint8_t reset_byte = 0xFF;
    uint8_t dummy;
    printf("Esecuzione sequenza di reset SPI...\n");
    for (int i = 0; i < 3; i++) {
        spi_transfer(&reset_byte, &dummy, 1);
        usleep(1000);
    }
    usleep(5000); // Pausa dopo il reset
    printf("Reset SPI completato - dispositivo in stato 'Idle'\n");
}

// Transazione SPI con DWM1001
int dwm_spi_transaction(uint8_t *tx_data, int tx_len, uint8_t *rx_data, int max_rx_len) {
    uint8_t rx_cmd[32] = {0};
    printf("Invio comando TLV: ");
    for (int i = 0; i < tx_len; i++) printf("0x%02x ", tx_data[i]);
    printf("\n");
    spi_transfer(tx_data, rx_cmd, tx_len);
    
    usleep(50000); // Aumentato a 50ms per dare più tempo
    
    uint8_t tx_size[2] = {0xFF, 0xFF};
    uint8_t rx_size[2] = {0, 0};
    int retries = 0;
    
    do {
        spi_transfer(tx_size, rx_size, 2);
        retries++;
        if (retries > 500) { // Timeout a 500ms
            printf("Timeout leggendo SIZE/NUM (ultimi rx_size: 0x%02x 0x%02x)\n", rx_size[0], rx_size[1]);
            return -1;
        }
        usleep(1000);
    } while (rx_size[0] == 0 || rx_size[0] == 0xFF);
    
    int data_size = rx_size[0];
    int transfers = rx_size[1];
    printf("SIZE/NUM ricevuti: SIZE=%d, NUM=%d\n", data_size, transfers);
    
    if (data_size > max_rx_len) {
        printf("Buffer troppo piccolo per i dati ricevuti (%d > %d)\n", data_size, max_rx_len);
        return -1;
    }
    
    uint8_t tx_dummy[256] = {0};
    memset(tx_dummy, 0xFF, sizeof(tx_dummy));
    spi_transfer(tx_dummy, rx_data, data_size);
    
    printf("Dati ricevuti: ");
    for (int i = 0; i < data_size; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");
    
    return data_size;
}

int read_status() {
    //ensure_device_in_idle_state();
    uint8_t tx_data[2] = {0x32, 0x00};
    uint8_t rx_data[7] = {0};
    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 7);
    
    if (answer_len < 0) {
        printf("Errore durante la lettura dello status\n");
        return -1;
    }
    printf("Status data: ");
    for (int i = 0; i < answer_len; i++) {
        printf("0x%02x ", rx_data[i]);
    }
    printf("\n");

    if(rx_data[0]==0x40 && rx_data[1]==0x01 && rx_data[2]==0x00){
        uint8_t type = rx_data[3];
        uint8_t length = rx_data[4];

        // Leggo i singoli bit del byte in rx_data[5] e l'unico non riservato in rx_data[6]
        bool loc_ready = rx_data[5] & 0x01;
        bool uwbmac_joined = rx_data[5] & 0x02;
        bool bh_data_ready = rx_data[5] & 0x04;
        bool bh_status_changed = rx_data[5] & 0x08;
        // bit 4 riservato
        bool uwb_scan_ready = rx_data[5] & 0x20;
        bool usr_data_ready = rx_data[5] & 0x40;
        bool usr_data_sent = rx_data[5] & 0x80;
        bool fwup_in_progress = rx_data[6] & 0x01;
        // bit dal 9 al 15 riservati

    } else {
        printf("Risposta non valida per status\n");
        return -1;
    }

    return 0;
}

int dwm_panid_set(uint16_t panid) {
    //ensure_device_in_idle_state();
    uint8_t tx_data[4] = {0x2E, 0x02, (uint8_t)(panid & 0xFF), (uint8_t)(panid >> 8 & 0xFF)};
    uint8_t rx_data[8] = {0};
    int len = dwm_spi_transaction(tx_data, 4, rx_data, sizeof(rx_data));
    
    if (len < 0) {
        printf("Errore in panid_set, ma il comando potrebbe essere stato eseguito\n");
        return -1;
    }
    
    printf("Risposta panid_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");
    
    if (len >= 3 && rx_data[0] == 0x40 && rx_data[1] == 0x01) {
        return rx_data[2]; // 0 = successo
    } else {
        printf("Risposta non valida per panid_set, ma il PANID potrebbe essere stato impostato\n");
        return -1;
    }
}

int dwm_panid_get() {
    uint8_t tx_data[2] = {0x2F, 0x00};
    uint8_t rx_data[7] = {0};
    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 7);
    
    if (answer_len < 0) {
        printf("Errore durante la lettura del PANID\n");
        return -1;
    }
    printf("Risposta panid_get: ");
    for (int i = 0; i < answer_len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");
    
    if (answer_len >= 7 && rx_data[0] == 0x40 && rx_data[1] == 0x01 && rx_data[2] == 0x00 &&
        rx_data[3] == 0x4D && rx_data[4] == 0x02) {
        printf("PANID: 0x%02x%02x\n", rx_data[6], rx_data[5]);
    } else {
        printf("Risposta incompleta o non valida per panid_get (len=%d)\n", answer_len);
        return -1;
    }

    return 0;
}

/* 
Funzione di settaggio della posizione nello spazio.
Quest'ultima non è usata quando il dispositivo è in modalità tag, ma è salvata.

Prende in input la struttura dwm_pos_t con i valori x, y, z e qf (quality factor).
Le posizioni sono in millimetri
*/
int dwm_pos_set(dwm_pos_t *pos) {
    //ensure_device_in_idle_state();
    uint8_t tx_data[15] = {0x01, 0x0D
        , (uint8_t)(pos->x & 0xFF), (uint8_t)(pos->x >> 8 & 0xFF), (uint8_t)(pos->x >> 16 & 0xFF), (uint8_t)(pos->x >> 24 & 0xFF)
        , (uint8_t)(pos->y & 0xFF), (uint8_t)(pos->y >> 8 & 0xFF), (uint8_t)(pos->y >> 16 & 0xFF), (uint8_t)(pos->y >> 24 & 0xFF)
        , (uint8_t)(pos->z & 0xFF), (uint8_t)(pos->z >> 8 & 0xFF), (uint8_t)(pos->z >> 16 & 0xFF), (uint8_t)(pos->z >> 24 & 0xFF),
        pos->qf
    };
    uint8_t rx_data[3] = {0};

    int len = dwm_spi_transaction(tx_data, 15, rx_data, 3);

    if (len < 0) {
        printf("Errore in pos_set, ma il comando potrebbe essere stato eseguito\n");
        return -1;
    }

    printf("Risposta pos_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    return 0;

}

/* Funzione per ottenere la posizione attuale nello spazio*/
int dwm_pos_get(dwm_pos_t *pos) {
    uint8_t tx_data[2] = {0x02, 0x00};
    uint8_t rx_data[18] = {0};

    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 18);
    
    if (answer_len < 0) {
        printf("Errore durante la lettura della posizione\n");
        return -1;
    }

    printf("Risposta pos_get: ");
    for (int i = 0; i < answer_len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    // salva dal 6 al 9 byte la x nella struct
    pos->x = (int32_t)(rx_data[5] | rx_data[6] << 8 | rx_data[7] << 16 | rx_data[8] << 24);
    // salva dal 10 al 13 byte la y nella struct
    pos->y = (int32_t)(rx_data[9] | rx_data[10] << 8 | rx_data[11] << 16 | rx_data[12] << 24);
    // salva dal 14 al 17 byte la z nella struct
    pos->z = (int32_t)(rx_data[13] | rx_data[14] << 8 | rx_data[15] << 16 | rx_data[16] << 24);
    // salva il 18 esimo byte nel quality factor
    pos->qf = rx_data[17];

    return 0;
    
}

/* 
Funzione utile per configurare il dispositivo come tag dati una serie di parametri facendo
uso della struttura dwm_cfg_tag_t.
Questa funzione setta i parametri, per rendere effettivi i cambiamenti bisogna fare un reset
al dispositivo.
*/
int dwm_cfg_tag_set(dwm_cfg_tag_t* p_cfg) {
    uint8_t tx_data[4] = {
        0x05, // Comando (tipo TLV)
        0x02, // Lunghezza (2 byte)
        0x00, // BYTE 0 - 
        0x00  // BYTE 1 - 
    };
    
    // BYTE 0
    if (p_cfg->low_power_en)
        tx_data[2] |= 0x80; // bit 7
    
    if (p_cfg->loc_engine_en)
        tx_data[2] |= 0x40; // bit 6
    
    if (p_cfg->common.enc_en)
        tx_data[2] |= 0x20; // bit 5
    
    if (p_cfg->common.led_en)
        tx_data[2] |= 0x10; // bit 4
    
    if (p_cfg->common.ble_en)
        tx_data[2] |= 0x08; // bit 3
    
    if (p_cfg->common.fw_update_en)
        tx_data[2] |= 0x04; // bit 2
    
    // Bits 0-1 per la modalità UWB
    tx_data[2] |= (p_cfg->common.uwb_mode & 0x03); // bits 0-1
    
    // BYTE 1
    if (p_cfg->stnry_en)
        tx_data[3] |= 0x04; // bit 2
    
    // Bits 0-1 per la modalità di misurazione
    tx_data[3] |= (p_cfg->meas_mode & 0x03); // bits 0-1

    uint8_t rx_data[3] = {0};

    int len = dwm_spi_transaction(tx_data, 4, rx_data, 3);

    if (len < 0) {
        printf("Errore in cfg_tag_set, ma il comando potrebbe essere stato eseguito\n");
        return -1;
    }

    printf("Risposta cfg_tag_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    return 0;
}

/*
Funzione utile per ottenere la configurazione attuale del dispositivo.
*/
int dwm_cfg_get(dwm_cfg_t *cfg){
    uint8_t tx_data[2] = {0x08, 0x00};
    uint8_t rx_data[7] = {0};

    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 7);

    if (answer_len < 0) {
        printf("Errore durante la lettura della configurazione\n");
        return -1;
    }

    printf("Risposta cfg_get: ");
    for (int i = 0; i < answer_len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    if(rx_data[0]==0x40 && rx_data[1]==0x01 && rx_data[2]==0x00){
        uint8_t byte0 = rx_data[5]; // Primo byte di configurazione
        uint8_t byte1 = rx_data[6]; // Secondo byte di configurazione
        
        // Estrazione dei bit dal BYTE0
        cfg->common.uwb_mode = byte0 & 0x03;         // bits 0-1
        cfg->common.fw_update_en = (byte0 >> 2) & 0x01; // bit 2
        cfg->common.ble_en = (byte0 >> 3) & 0x01;    // bit 3
        cfg->common.led_en = (byte0 >> 4) & 0x01;    // bit 4
        cfg->common.enc_en = (byte0 >> 5) & 0x01;    // bit 5
        cfg->loc_engine_en = (byte0 >> 6) & 0x01;    // bit 6
        cfg->low_power_en = (byte0 >> 7) & 0x01;     // bit 7
        
        // Estrazione dei bit dal BYTE1
        cfg->meas_mode = byte1 & 0x03;              // bits 0-1
        cfg->stnry_en = (byte1 >> 2) & 0x01;        // bit 2
        cfg->bridge = (byte1 >> 3) & 0x01;          // bit 3
        cfg->initiator = (byte1 >> 4) & 0x01;       // bit 4
        cfg->mode = (byte1 >> 5) & 0x01;            // bit 5
        // bits 6-7 sono riservati

    } else {
        printf("Risposta non valida per cfg_get\n");
        return -1;
    }

    return 0;
}

/*
Funzione utile al settaggio del nodo come ancora.
La funzione da sola non basta per applicare le modifiche, chiamare anche dwm_reset().
*/
int dwm_cfg_anchor_set(dwm_cfg_anchor_t *p_cfg) {
    uint8_t tx_data[4] = {
        0x07, // Comando
        0x02, // Lunghezza
        0x00, // BYTE 0
        0x00  // BYTE 1
    };

    // setting dei bit nel BYTE 0
    if (p_cfg->initiator)
        tx_data[2] |= 0x80; // bit 7: initiator
    
    if (p_cfg->bridge)
        tx_data[2] |= 0x40; // bit 6: bridge
    
    if (p_cfg->common.enc_en)
        tx_data[2] |= 0x20; // bit 5: enc_en
    
    if (p_cfg->common.led_en)
        tx_data[2] |= 0x10; // bit 4: led_en
    
    if (p_cfg->common.ble_en)
        tx_data[2] |= 0x08; // bit 3: ble_en
    
    if (p_cfg->common.fw_update_en)
        tx_data[2] |= 0x04; // bit 2: fw_update_en
    
    // Bits 0-1 per la modalità UWB
    tx_data[2] |= (p_cfg->common.uwb_mode & 0x03); // bits 0-1

    uint8_t rx_data[3] = {0};

    int len = dwm_spi_transaction(tx_data, 4, rx_data, 3);

    if (len < 0) {
        printf("Errore in cfg_anchor_set, ma il comando potrebbe essere stato eseguito\n");
        return -1;
    }

    printf("Risposta cfg_anchor_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    return 0;

}


/*
Funzione utile per l'applicazione delle modifiche alla configurazione del dispositivo.
*/
int dwm_reset(){
    uint8_t tx_data[2] = {0x14, 0x00};
    uint8_t rx_data[3] = {0};

    int len = dwm_spi_transaction(tx_data, 2, rx_data, 3);

    if (len < 0) {
        printf("Errore in reset, ma il comando potrebbe essere stato eseguito\n");
        return -1;
    }

    printf("Risposta reset (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    return 0;
}

/*
Funzione utile all'ottenimento delle distanze tra dispositivi ancora e tag.
Prende in input un vettore in cui alloca dinamicamente memoria e salva le distanze che trova.
*/
int dwm_loc_get(uint32_t **distances) {

    uint8_t tx_data[2] = {0x0C, 0x00};
    uint8_t rx_data[256] = {0};  // Buffer ampio per i dati

    // Controllo che il puntatore del vettore delle distanze sia corretto
    if(distances == NULL) {
        printf("Puntatore nullo per il vettore delle distanze\n");
        return -1;
    }

    *distances = NULL;

    int len = dwm_spi_transaction(tx_data, 2, rx_data, 256);

    if (len < 0) {
        printf("Errore durante la lettura dei dati di localizzazione\n");
        return -1;
    }

    printf("Risposta loc_get: ");
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    int pos = 0;
    
    // Controllo che la risposta contenga almeno l'header
    if (len >= 3 && rx_data[0] == 0x40 && rx_data[1] == 0x01) { // I campi header devono essere conformi
        if (rx_data[2] != 0x00) { // C'è stato un errore
            printf("Errore nel comando loc_get: 0x%02x\n", rx_data[2]);
            return -1;
        }
        
        pos = 3;  // index dopo header
        
        // Verifica se ci sono dati di posizione
        if (pos < len && rx_data[pos] == 0x41) {
            pos++;  // Salta il tipo
            
            if (pos < len) {
                uint8_t pos_len = rx_data[pos++];  // Lunghezza dei dati di posizione
                
                if (pos + pos_len <= len) { // ci sono altri dati da controllare
                    // Salvo la x,y,z del dispositivo, non è la distanza dagli altri
                    int32_t x = (int32_t)rx_data[pos] | 
                                ((int32_t)rx_data[pos+1] << 8) | 
                                ((int32_t)rx_data[pos+2] << 16) | 
                                ((int32_t)rx_data[pos+3] << 24);
                    pos += 4;
                    
                    int32_t y = (int32_t)rx_data[pos] | 
                                ((int32_t)rx_data[pos+1] << 8) | 
                                ((int32_t)rx_data[pos+2] << 16) | 
                                ((int32_t)rx_data[pos+3] << 24);
                    pos += 4;
                    
                    int32_t z = (int32_t)rx_data[pos] | 
                                ((int32_t)rx_data[pos+1] << 8) | 
                                ((int32_t)rx_data[pos+2] << 16) | 
                                ((int32_t)rx_data[pos+3] << 24);
                    pos += 4;
                    
                    uint8_t qf = rx_data[pos++];
                    
                    printf("Posizione: x=%ld, y=%ld, z=%ld, qf=%u\n", x, y, z, qf);
                }
            }
        }
        
        // Verifica se ci sono dati sulle distanze
        if (pos < len && (rx_data[pos] == 0x48 || rx_data[pos] == 0x49)) {
            uint8_t type = rx_data[pos++];  // Tipo delle distanze (0x48 per ancora, 0x49 per tag)
            
            uint8_t length = rx_data[pos++]; // Lunghezza dei dati ottenuti 

            if (pos < len) {
                uint8_t count = rx_data[pos++];  // Numero di distanze
                printf("Distanze: %d\n", count);

                // Allocazione del vettore delle distanze
                *distances = (uint32_t*)malloc(count * sizeof(uint32_t));
                if (*distances == NULL) {
                    printf("Errore nell'allocazione del vettore delle distanze\n");
                    return -1;
                }
                
                for (int i = 0; i < count && pos < len; i++) {
                    if (type == 0x49) {  // Formato tag
                        // Lettura in formato corretto per l'ID
                        uint16_t id = ((uint16_t)rx_data[pos]) | 
                                        ((uint16_t)rx_data[pos+1] << 8);
                        // Stampa per debug dei byte originali
                        printf("  Byte originali ID: 0x%02X 0x%02X\n", rx_data[pos], rx_data[pos+1]);
                        pos += 2;
                        
                        uint32_t dist = (uint32_t)rx_data[pos] | 
                                        ((uint32_t)rx_data[pos+1] << 8) | 
                                        ((uint32_t)rx_data[pos+2] << 16) | 
                                        ((uint32_t)rx_data[pos+3] << 24);
                        pos += 4;

                        (*distances)[i] = dist;
                        
                        uint8_t qf = rx_data[pos++];
                        
                        // Per i tag, leggi anche la posizione dell'ancora
                        int32_t ax = 0, ay = 0, az = 0;
                        uint8_t aqf = 0;
                        
                        if (pos + 12 < len) {
                            ax = (int32_t)rx_data[pos] | 
                                ((int32_t)rx_data[pos+1] << 8) | 
                                ((int32_t)rx_data[pos+2] << 16) | 
                                ((int32_t)rx_data[pos+3] << 24);
                            pos += 4;
                            
                            ay = (int32_t)rx_data[pos] | 
                                ((int32_t)rx_data[pos+1] << 8) | 
                                ((int32_t)rx_data[pos+2] << 16) | 
                                ((int32_t)rx_data[pos+3] << 24);
                            pos += 4;
                            
                            az = (int32_t)rx_data[pos] | 
                                ((int32_t)rx_data[pos+1] << 8) | 
                                ((int32_t)rx_data[pos+2] << 16) | 
                                ((int32_t)rx_data[pos+3] << 24);
                            pos += 4;
                            
                            aqf = rx_data[pos++];
                        }
                        
                        printf("  Distanza %d: ID=0x%04X, dist=%lu mm, qf=%u, pos=[%ld,%ld,%ld]\n",
                                i+1, id, dist, qf, ax, ay, az);
                        
                        
                    } else {  // Formato ancora
                        uint64_t id = 0;
                        // Lettura little-endian esplicita per uint64_t
                        for (int j = 0; j < 8 && pos < len; j++) {
                            id |= (uint64_t)rx_data[pos++] << (j * 8);
                        }
                        
                        uint32_t dist = (uint32_t)rx_data[pos] | 
                                        ((uint32_t)rx_data[pos+1] << 8) | 
                                        ((uint32_t)rx_data[pos+2] << 16) | 
                                        ((uint32_t)rx_data[pos+3] << 24);
                        pos += 4;

                        (*distances)[i] = dist;
                        
                        uint8_t qf = rx_data[pos++];
                        
                        printf("  Distanza %d: ID=0x%016llX, dist=%lu mm, qf=%u\n",
                                i+1, (unsigned long long)id, dist, qf);
                    }
                }
            }
        }
    } else {
        printf("Risposta non valida per loc_get\n");
        return -1;
    }


    return 0;
}

/*
Funzione di utilità nel settaggio delle configurazioni UWB per un nodo.
Gli viene passato il ritardo e la potenza di trasmissione.
*/
int dwm_uwb_cfg_set(uint8_t pg_delay, uint32_t tx_power) {
    uint8_t tx_data[7] = {
        0x17, // Comando
        0x05, // Lunghezza
        pg_delay, // Delay
        (uint8_t)(tx_power & 0xFF), (uint8_t)(tx_power >> 8 & 0xFF), // Potenza di trasmissione
        (uint8_t)(tx_power >> 16 & 0xFF), (uint8_t)(tx_power >> 24 & 0xFF) // Potenza di trasmissione
    };
    uint8_t rx_data[3] = {0}; // dati in ricezione

    int len = dwm_spi_transaction(tx_data, 7, rx_data, 3); // invio comando

    if (len < 0) {
        printf("Errore in uwb_cfg_set, ma il comando potrebbe essere stato eseguito\n");
        return -1;
    }

    printf("Risposta uwb_cfg_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    if( rx_data[0] == 0x40 && rx_data[1] == 0x01 && rx_data[2] == 0x00) { // controllo successo
        printf("Configurazione UWB impostata con successo\n");
    } else {
        printf("Errore durante il settaggio della configurazione UWB\n");
        return -1;
    }

    return 0;
}

int dwm_uwb_cfg_get(dwm_uwb_cfg_t *cfg) {
    uint8_t tx_data[2] = {0x18, 0x00};
    uint8_t rx_data[15] = {0};

    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 15);

    if (answer_len < 0) {
        printf("Errore durante la lettura della configurazione UWB\n");
        return -1;
    }

    printf("Risposta uwb_cfg_get: ");
    for (int i = 0; i < answer_len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    if (rx_data[0] == 0x40 && rx_data[1] == 0x01 && rx_data[2] == 0x00) {
        uint8_t type = rx_data[3];
        uint8_t length = rx_data[4];

        int pos = 5;
        if (pos < length) {
            cfg->pg_delay = rx_data[pos++];
            cfg->tx_power = (uint32_t)rx_data[pos] | 
                            ((uint32_t)rx_data[pos+1] << 8) | 
                            ((uint32_t)rx_data[pos+2] << 16) | 
                            ((uint32_t)rx_data[pos+3] << 24);
            pos += 4;

            cfg->compensated.pg_delay = rx_data[pos++];
            cfg->compensated.tx_power = (uint32_t)rx_data[pos] | 
                                 ((uint32_t)rx_data[pos+1] << 8) | 
                                 ((uint32_t)rx_data[pos+2] << 16) | 
                                 ((uint32_t)rx_data[pos+3] << 24);
            pos += 4;
            
            printf("Configurazione UWB: Delay=%u, Potenza=%lu\n", cfg->pg_delay, cfg->tx_power);
        }
    } else {
        printf("Risposta non valida per uwb_cfg_get\n");
        return -1;
    }

    return 0;
}

/*
Funzione utile ad ottenere l'id del nodo su cui il codice è in esecuzione.
*/
int dwm_nodeid_get(){
    uint8_t tx_data[2] = {0x30, 0x00};
    uint8_t rx_data[13] = {0};

    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 13);

    if (answer_len < 0) {
        printf("Errore durante la lettura dell'ID del nodo\n");
        return -1;
    }

    printf("Risposta nodeid_get: ");
    for (int i = 0; i < answer_len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    if(rx_data[0]==0x40 && rx_data[1]==0x01 && rx_data[2]==0x00){
        uint64_t id = 0;
        for (int i = 5; i < 13; i++) {
            id |= (uint64_t)rx_data[i] << (i * 8);
        }
        printf("ID del nodo: 0x%016llX\n", (unsigned long long)id);
    } else {
        printf("Risposta non valida per nodeid_get\n");
        return -1;
    }

    return 0;
}

/*
Funzione che permette di settare il rate di aggiornamento di ottenimento dati.
Il valore urs deve essere sempre maggiore o uguale a ur

Esempio:
dwm_upd_rate_set(10, 50) indica update rate di 1 sec quando in movimento, 5 s quando stazionario



*/
int dwm_upd_rate_set(uint16_t ur, uint16_t urs){

    if(urs < ur) {
        printf("Il valore di update rate stazionario deve essere maggiore o uguale a quello dinamico\n");
        return -1;
    }

    uint8_t tx_data[6] = {
        0x03, // Comando
        0x04, // Lunghezza
        (uint8_t)(ur & 0xFF), (uint8_t)(ur >> 8 & 0xFF), // Update rate dinamico
        (uint8_t)(urs & 0xFF), (uint8_t)(urs >> 8 & 0xFF) // Update rate stazionario
    };

    uint8_t rx_data[3] = {0};

    int len = dwm_spi_transaction(tx_data, 6, rx_data, 3);

    if (len < 0) {
        printf("Errore in upd_rate_set, ma il comando potrebbe essere stato eseguito\n");
        return -1;
    }

    printf("Risposta upd_rate_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    if( rx_data[0] == 0x40 && rx_data[1] == 0x01 && rx_data[2] == 0x00) { // controllo successo
        printf("Update rate impostato con successo\n");
    } else {
        printf("Errore durante il settaggio dell'update rate\n");
        return -1;
    }

    usleep(500000); // aspetto 500ms
    return 0;
}




/*
EXTRA:


void dwm_anchor_list_get(){
    uint8_t tx_data[3] = {
        0x0B, // Comando
        0x01,  // Lunghezza
        0x00 // Numero di pagina
    }
    uint8_t rx_data[256] = {0}; // Vettore abbastanza grande da raccogliere info sulle posizioni dei dispositivi

    int len = dwm_spi_transaction(tx_data, 3, rx_data, 256);

    if (len < 0) {
        printf("Errore durante la lettura della lista degli anchor\n");
        return;
    }

    if (answer_len >= 5 && rx_data[0] == 0x40 && rx_data[1] == 0x01 && rx_data[2] == 0x00) {
        // Risposta valida
        if (rx_data[3] == 0x40 && rx_data[4] == 0x01) {
            // Contiene l'elenco degli ancoraggi
            uint8_t anchor_count = rx_data[5];
            printf("Numero di ancoraggi trovati: %d\n", anchor_count);
            
            if (anchor_count == 0) {
                printf("Nessun ancoraggio trovato nella pagina %d\n", page_number);
                return 0;
            }
            
            // Inizia a elaborare i dati degli ancoraggi
            int offset = 6; // Posizione iniziale dei dati del primo ancoraggio
            
            for (int i = 0; i < anchor_count && offset < answer_len; i++) {
                // Estrai l'ID dell'ancoraggio (2 byte)
                uint16_t anchor_id = rx_data[offset] | (rx_data[offset + 1] << 8);
                offset += 2;
                
                // Estrai le coordinate x, y, z (12 byte)
                int32_t x = rx_data[offset] | (rx_data[offset + 1] << 8) | 
                           (rx_data[offset + 2] << 16) | (rx_data[offset + 3] << 24);
                offset += 4;
                
                int32_t y = rx_data[offset] | (rx_data[offset + 1] << 8) | 
                           (rx_data[offset + 2] << 16) | (rx_data[offset + 3] << 24);
                offset += 4;
                
                int32_t z = rx_data[offset] | (rx_data[offset + 1] << 8) | 
                           (rx_data[offset + 2] << 16) | (rx_data[offset + 3] << 24);
                offset += 4;
                
                // Estrai RSSI
                int8_t rssi = (int8_t)rx_data[offset];
                offset += 1;
                
                // Estrai informazioni aggiuntive
                uint8_t info = rx_data[offset];
                uint8_t seat = info & 0x1F;           // bit 0-4: numero di posto
                bool neighbor_network = (info >> 5) & 0x01; // bit 5: rete vicina
                // bit 6-7: riservati
                offset += 1;
                
                printf("Ancoraggio %d: ID=0x%04X, Posizione=[%ld,%ld,%ld], RSSI=%d, "
                       "Seat=%u, Rete vicina=%s\n", 
                       i+1, anchor_id, x, y, z, rssi, seat, 
                       neighbor_network ? "Sì" : "No");
            }
            
            return anchor_count;
        } else {
            printf("Formato risposta non valido o nessun ancoraggio trovato\n");
            return 0;
        }
    } else {
        printf("Risposta non valida per anchor_list_get\n");
        return -1;
    }


}
*/

/*
void dwm_loc_get() {
    uint8_t tx_data[2] = {0x0C, 0x00};
    uint8_t rx_data[256] = {0};  // Buffer ampio per i dati

    int len = dwm_spi_transaction(tx_data, 2, rx_data, 256);

    if (len < 0) {
        printf("Errore durante la lettura dei dati di localizzazione\n");
        return;
    }

    printf("Risposta loc_get: ");
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

    int pos = 0;
    
    // Controllo che la risposta contenga almeno l'header
    if (len >= 3 && rx_data[0] == 0x40 && rx_data[1] == 0x01) {
        if (rx_data[2] != 0x00) {
            printf("Errore nel comando loc_get: 0x%02x\n", rx_data[2]);
            return;
        }
        
        pos = 3;  // Posizione dopo l'header di risposta
        
        // Verifica se ci sono dati di posizione
        if (pos < len && rx_data[pos] == 0x41) {
            pos++;  // Salta il tipo
            
            if (pos < len) {
                uint8_t pos_len = rx_data[pos++];  // Lunghezza dei dati di posizione
                
                if (pos + pos_len <= len) {
                    int32_t x = rx_data[pos] | (rx_data[pos+1] << 8) | 
                               (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                    pos += 4;
                    
                    int32_t y = rx_data[pos] | (rx_data[pos+1] << 8) | 
                               (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                    pos += 4;
                    
                    int32_t z = rx_data[pos] | (rx_data[pos+1] << 8) | 
                               (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                    pos += 4;
                    
                    uint8_t qf = rx_data[pos++];
                    
                    printf("Posizione: x=%ld, y=%ld, z=%ld, qf=%u\n", x, y, z, qf);
                }
            }
        }
        
        // Verifica se ci sono dati sulle distanze
        if (pos < len && (rx_data[pos] == 0x48 || rx_data[pos] == 0x49)) {
            uint8_t type = rx_data[pos++];  // Tipo delle distanze (0x48 per ancora, 0x49 per tag)
            
            if (pos < len) {
                uint8_t count = rx_data[pos++];  // Numero di distanze
                printf("Distanze: %d\n", count);
                
                for (int i = 0; i < count && pos < len; i++) {
                    if (type == 0x49) {  // Formato tag
                        uint16_t id = rx_data[pos] | (rx_data[pos+1] << 8);
                        pos += 2;
                        
                        uint32_t dist = rx_data[pos] | (rx_data[pos+1] << 8) | 
                                       (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                        pos += 4;
                        
                        uint8_t qf = rx_data[pos++];
                        
                        // Per i tag, leggi anche la posizione dell'ancora
                        int32_t ax = 0, ay = 0, az = 0;
                        uint8_t aqf = 0;
                        
                        if (pos + 12 < len) {
                            ax = rx_data[pos] | (rx_data[pos+1] << 8) | 
                                (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                            pos += 4;
                            
                            ay = rx_data[pos] | (rx_data[pos+1] << 8) | 
                                (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                            pos += 4;
                            
                            az = rx_data[pos] | (rx_data[pos+1] << 8) | 
                                (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                            pos += 4;
                            
                            aqf = rx_data[pos++];
                        }
                        
                        printf("  Distanza %d: ID=0x%04X, dist=%lu mm, qf=%u, pos=[%ld,%ld,%ld]\n",
                               i+1, id, dist, qf, ax, ay, az);
                    } else {  // Formato ancora
                        uint64_t id = 0;
                        for (int j = 0; j < 8 && pos < len; j++) {
                            id |= (uint64_t)rx_data[pos++] << (j * 8);
                        }
                        
                        uint32_t dist = rx_data[pos] | (rx_data[pos+1] << 8) | 
                                       (rx_data[pos+2] << 16) | (rx_data[pos+3] << 24);
                        pos += 4;
                        
                        uint8_t qf = rx_data[pos++];
                        
                        printf("  Distanza %d: ID=0x%016llX, dist=%lu mm, qf=%u\n",
                               i+1, (unsigned long long)id, dist, qf);
                    }
                }
            }
        }
    } else {
        printf("Risposta non valida per loc_get\n");
    }
}
    */