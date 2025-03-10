#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <stdbool.h>

// Struttura per la posizione
typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t qf;
} dwm_pos_t;

/* Le seguenti strutture fino all'asterisco sono prese dall'API ufficiale */
/**
 * @brief Position measurement modes
 */
typedef enum {
	DWM_MEAS_MODE_TWR = 0,//!< DWM_MEAS_MODE_TWR
	DWM_MEAS_MODE_TDOA = 1//!< DWM_MEAS_MODE_TDOA
} dwm_meas_mode_t;

/**
 * @brief Device modes
 */
typedef enum {
	DWM_MODE_TAG = 0,  //!< DWM_MODE_TAG
	DWM_MODE_ANCHOR = 1//!< DWM_MODE_ANCHOR
} dwm_mode_t;

typedef enum {
	DWM_UWB_MODE_OFF = 0,
	DWM_UWB_MODE_PASSIVE = 1,
	DWM_UWB_MODE_ACTIVE = 2
}dwm_uwb_mode_t;

typedef enum {
	DWM_UWB_BH_ROUTING_OFF = 0,
	DWM_UWB_BH_ROUTING_ON = 1,
	DWM_UWB_BH_ROUTING_AUTO = 2,
} dwm_uwb_bh_routing_t;

typedef struct dwm_cfg_common {
	dwm_uwb_mode_t uwb_mode;
	bool fw_update_en;
	bool ble_en;
	bool led_en;
	bool enc_en;
} dwm_cfg_common_t;

typedef struct dwm_cfg_anchor {
	dwm_cfg_common_t common;
	bool bridge;
	bool initiator;
	dwm_uwb_bh_routing_t uwb_bh_routing;
} dwm_cfg_anchor_t;

typedef struct dwm_cfg_tag {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
} dwm_cfg_tag_t;

typedef struct dwm_cfg {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
	dwm_uwb_bh_routing_t uwb_bh_routing;
	bool bridge;
	bool initiator;
	dwm_mode_t mode;
} dwm_cfg_t;

/* * */

// File descriptor SPI globale
int spi_fd = -1;

// Configurazione SPI
static const char *device = "/dev/spidev0.0"; // Bus 0, Device 0
static uint32_t speed = 1000000; // 1 MHz
static uint8_t mode = 0; // CPOL=0, CPHA=0
static uint8_t bits = 8;

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

void read_status() {
    //ensure_device_in_idle_state();
    uint8_t tx_data[2] = {0x32, 0x00};
    uint8_t rx_data[7] = {0};
    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 7);
    
    if (answer_len < 0) {
        printf("Errore durante la lettura dello status\n");
        return;
    }
    printf("Status data: ");
    for (int i = 0; i < answer_len; i++) {
        printf("0x%02x ", rx_data[i]);
    }
    printf("\n");
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

void dwm_panid_get() {
    //ensure_device_in_idle_state();
    uint8_t tx_data[2] = {0x2F, 0x00};
    uint8_t rx_data[7] = {0};
    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 7);
    
    if (answer_len < 0) {
        printf("Errore durante la lettura del PANID\n");
        return;
    }
    printf("Risposta panid_get: ");
    for (int i = 0; i < answer_len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");
    
    if (answer_len >= 7 && rx_data[0] == 0x40 && rx_data[1] == 0x01 && rx_data[2] == 0x00 &&
        rx_data[3] == 0x4D && rx_data[4] == 0x02) {
        printf("PANID: 0x%02x%02x\n", rx_data[6], rx_data[5]);
    } else {
        printf("Risposta incompleta o non valida per panid_get (len=%d)\n", answer_len);
    }
}

/* 
Funzione di settaggio della posizione nello spazio.
Quest'ultima non è usata quando il dispositivo è in modalità tag, ma è salvata.

Prende in input la struttura dwm_pos_t con i valori x, y, z e qf (quality factor).
Le posizioni sono in millimetri
*/
void dwm_pos_set(dwm_pos_t *pos) {
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
        return;
    }

    printf("Risposta pos_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

}

/* Funzione per ottenere la posizione attuale */
void dwm_pos_get(dwm_pos_t *pos) {
    uint8_t tx_data[2] = {0x02, 0x00};
    uint8_t rx_data[18] = {0};

    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 18);
    
    if (answer_len < 0) {
        printf("Errore durante la lettura della posizione\n");
        return;
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
    
}

/* 
Funzione utile per configurare il dispositivo come tag dati una serie di parametri facendo
uso della struttura dwm_cfg_tag_t.
Questa funzione setta i parametri, per rendere effettivi i cambiamenti bisogna fare un reset
al dispositivo.
*/
void dwm_cfg_tag_set(dwm_cfg_tag_t* p_cfg) {
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
        return;
    }

    printf("Risposta cfg_tag_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

}

/*
Funzione utile per ottenere la configurazione attuale del dispositivo.
*/
void dwm_cfg_get(dwm_cfg_t *cfg){
    uint8_t tx_data[2] = {0x08, 0x00};
    uint8_t rx_data[7] = {0};

    int answer_len = dwm_spi_transaction(tx_data, 2, rx_data, 7);

    if (answer_len < 0) {
        printf("Errore durante la lettura della configurazione\n");
        return;
    }

    printf("Risposta cfg_get: ");
    for (int i = 0; i < answer_len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

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
}

/*
Funzione utile al settaggio del nodo come ancora.
La funzione da sola non basta per applicare le modifiche, chiamare anche dwm_reset().
*/
void dwm_cfg_anchor_set(dwm_cfg_anchor_t *p_cfg) {
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
        return;
    }

    printf("Risposta cfg_anchor_set (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");

}

void dwm_anchor_list_get(){
    uint8_t tx_data[3] = {
        0x0B, // Comando
        0x01,  // Lunghezza
        0x01 // Numero di pagina
    }
    uint8_t rx_data[256] = {0}; // Vettore abbastanza grande da raccogliere info sulle posizioni dei dispositivi

    int len = dwm_spi_transaction(tx_data, 3, rx_data, 256);

    if (len < 0) {
        printf("Errore durante la lettura della lista degli anchor\n");
        return;
    }

    
}

/*
Funzione utile per l'applicazione delle modifiche alla configurazione del dispositivo.
*/
void dwm_reset(){
    uint8_t tx_data[2] = {0x14, 0x00};
    uint8_t rx_data[3] = {0};

    int len = dwm_spi_transaction(tx_data, 2, rx_data, 3);

    if (len < 0) {
        printf("Errore in reset, ma il comando potrebbe essere stato eseguito\n");
        return;
    }

    printf("Risposta reset (len=%d): ", len);
    for (int i = 0; i < len; i++) printf("0x%02x ", rx_data[i]);
    printf("\n");
}



int main() {
    if (spi_init() < 0) return -1;
    
    read_status();
    usleep(10000);
    
    spi_close();
    return 0;
}