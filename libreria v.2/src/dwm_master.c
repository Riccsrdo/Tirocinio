#include "dwm_master.h"
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <unistd.h>


// Variabili statiche per la gestione di UART
static int uart_fd = -1; // File descriptor per il device UART

int dwm_send_packet(uint8_t command, const uint8_t* payload, size_t payload_len) {

    if(uart_fd < 0) {
        fprintf(stderr, "Errore: UART non inizializzata.\n");
        return EXIT_FAILURE; // UART non inizializzata
    }

    // Calcolo la dimensione del pacchetto
    uint16_t packet_size = 5 + payload_len; // 4 byte per header + payload + 1 byte per checksum
    uint8_t packet[packet_size];

    packet[0] = START_BYTE; // Byte di partenza
    packet[1] = command; // ID del comando
    packet[2] = (uint8_t)(payload_len & 0xFF); // Lunghezza LSB
    packet[3] = (uint8_t)((payload_len >> 8) & 0xFF); // Lunghezza MSB

    if(payload && payload_len > 0){
        memcpy(&packet[4], payload, payload_len);
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < packet_size - 1; i++) {
        checksum += packet[i];
    }

    packet[packet_size - 1] = checksum; // Checksum

    int bytes_written = write(uart_fd, packet, packet_size);
    if(bytes_written != packet_size) {
        fprintf(stderr, "Errore durante l'invio del pacchetto UART: %s\n", strerror(errno));
        return EXIT_FAILURE; // Errore di scrittura
    }

    return EXIT_SUCCESS; // Successo
}

int read_byte_uart(int timeout_ms) {
    fd_set read_fds;
    struct timeval timeout;
    int ret;
    uint8_t byte;

    FD_ZERO(&read_fds); // Inizializza il set di file descriptor
    FD_SET(uart_fd, &read_fds); // Aggiungi il file descriptor UART

    timeout.tv_sec = timeout_ms / 1000; // Imposta i secondi
    timeout.tv_usec = (timeout_ms % 1000) * 1000; // Imposta i microsecondi

    ret = select(uart_fd + 1, &read_fds, NULL, NULL, &timeout);
    if(ret < 0) {
        perror("Errore durante select");
        return -1; // Errore di select
    } else if(ret == 0) {
        //fprintf(stderr, "Timeout durante la lettura del byte UART.\n");
        return -1; // Timeout
    } else {
        // Leggo il byte dalla UART
        if(read(uart_fd, &byte, 1) > 0) {
            return byte; // Successo, ritorno il byte letto
        } else {
            perror("Errore durante la lettura del byte UART");
            return -1; // Errore di lettura
        }
    }
}


int dwm_receive_packet(uint8_t* resp_buffer, uint16_t max_len, uint16_t* out_len, int timeout_ms) {

    int byte;
    uint16_t packet_idx = 0;
    uint16_t payload_len = 0;

    enum {
        STATE_RECEIVE_ID, STATE_RECEIVE_LEN_LSB,
        STATE_RECEIVE_LEN_MSB, STATE_RECEIVE_PAYLOAD, STATE_RECEIVE_CHECKSUM
    } state = STATE_RECEIVE_ID;

    int timeout_generale = timeout_ms + 200; // Timeout generale

    while(timeout_generale > 0) {

        byte = read_byte_uart(100);
        if(byte < 0) {
            // This is now a timeout, not a critical error
            timeout_generale -= 100;
            if(timeout_generale <= 0) {
                 fprintf(stderr, "Timeout durante la ricezione del pacchetto UART.\n");
                 return EXIT_FAILURE;
            }
            continue;
        }

        printf("0x%02X", (uint8_t)byte); // Stampa il byte ricevuto in esadecimale


        if (packet_idx >= max_len) {
            fprintf(stderr, "Errore: Buffer di risposta troppo piccolo.\n");
            return EXIT_FAILURE; // Buffer troppo piccolo
        }

        resp_buffer[packet_idx++] = (uint8_t)byte;

        switch(state) {
            case STATE_RECEIVE_ID:
                state = STATE_RECEIVE_LEN_LSB;
                //printf("Passo a ricezione len lsb\n");
                break;
            case STATE_RECEIVE_LEN_LSB:
                payload_len = byte; // LSB della lunghezza del payload
                state = STATE_RECEIVE_LEN_MSB;
                //printf("Passo a ricezione len msb\n");
                break;
            case STATE_RECEIVE_LEN_MSB:
                payload_len |= ((uint16_t)byte << 8); // MSB della lunghezza del payload
                if(payload_len == 0) state = STATE_RECEIVE_CHECKSUM; // Nessun payload
                else state = STATE_RECEIVE_PAYLOAD; // Aspetto il payload
                //printf("Passo al prossimo\n");
                break;
            case STATE_RECEIVE_PAYLOAD:
                if(packet_idx >= (payload_len + 3)) { // 3 byte per ID e lunghezza
                    state = STATE_RECEIVE_CHECKSUM; // Ho ricevuto tutto il payload
                }
                //printf("passo al calcolo checksum\n");
                break;

            case STATE_RECEIVE_CHECKSUM:
                // Calcolo checksum
                uint8_t checksum = 0;
                for (uint16_t i = 0; i < packet_idx - 1; i++) {
                    checksum += resp_buffer[i];
                }
                if(checksum == (uint8_t)byte) {
                    *out_len = packet_idx; // Imposto la lunghezza del pacchetto ricevuto
                    return EXIT_SUCCESS; // Pacchetto ricevuto correttamente
                } else {
                    fprintf(stderr, "Errore: Checksum non valido.\n");
                    return EXIT_FAILURE; // Checksum non valido
                }
        }
    }

    fprintf(stderr, "Errore: Timeout generale durante la ricezione del pacchetto UART.\n");
    return EXIT_FAILURE; // Timeout generale

}

int dwm_uart_init(const char* device, uint32_t baudrate) {
    if(uart_fd >= 0) {
        fprintf(stderr, "Errore: UART già inizializzata.\n");
        return EXIT_FAILURE; // Già inizializzata
    }

    uart_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(uart_fd < 0) {
        perror("Errore apertura device UART");
        return EXIT_FAILURE; // Errore di apertura
    }

    struct termios tty;
    if(tcgetattr(uart_fd, &tty) != 0) { // Ottengo le impostazioni attuali
        perror("Errore lettura impostazioni UART");
        close(uart_fd);
        uart_fd = -1;
        return EXIT_FAILURE; // Errore di lettura
    }

    cfsetospeed(&tty, baudrate); // Imposto la velocità di trasmissione
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bit di dati
    tty.c_cflag |= (CLOCAL | CREAD); // Abilita la porta e la lettura
    tty.c_cflag &= ~PARENB; // Disabilita il bit di parità
    tty.c_cflag &= ~CSTOPB; // 1 bit di stop
    tty.c_cflag &= ~CRTSCTS; // Disabilita il controllo di flusso hardware

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Modalità non canonica, senza echo
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disabilita il controllo di flusso software
    tty.c_oflag &= ~OPOST; // Modalità raw

    // Configurazione letture non bloccanti
    tty.c_cc[VMIN] = 0; // Nessun byte minimo da leggere
    tty.c_cc[VTIME] = 5; // Timeout di 0.5 secondi

    if(tcsetattr(uart_fd, TCSANOW, &tty) != 0) { // Applico le impostazioni
        perror("Errore impostazione UART");
        close(uart_fd);
        uart_fd = -1;
        return EXIT_FAILURE; // Errore di impostazione
    }

    printf("UART Inizializzata: %s a %d baud\n", device, (int)baudrate);
    return EXIT_SUCCESS; // Successo
}

int dwm_uart_close(void) {
    if(uart_fd < 0) {
        fprintf(stderr, "Errore: UART non inizializzata.\n");
        return EXIT_FAILURE; // Non inizializzata
    }

    if(close(uart_fd) < 0) {
        perror("Errore chiusura UART");
        return EXIT_FAILURE; // Errore di chiusura
    }
    uart_fd = -1; // Resetta il file descriptor
    printf("UART chiusa correttamente.\n");
    return EXIT_SUCCESS; // Successo
}

int dwm_simple_cmd_ack(uint8_t command, const uint8_t* payload, size_t payload_len){
    uint8_t response[256];
    uint16_t resp_len;

    if(dwm_send_packet(command, payload, payload_len) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante l'invio del comando 0x%02X.\n", command);
        return EXIT_FAILURE; // Errore di invio
    }

    usleep(50000); // Attendo un breve periodo per garantire che il comando sia elaborato

    if(dwm_receive_packet(response, sizeof(response), &resp_len, 1000) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante la ricezione della risposta per il comando 0x%02X.\n", command);
        return EXIT_FAILURE; // Errore di ricezione
    }

    if(response[0] == command){ // Controllo che ACK riguardi comando inviato
        // Se c'è payload di un byte, controllo che sia 0x01 (successo)
        if(resp_len == 5 && response[3] == 0x00) { // NACK
            fprintf(stderr, "Comando 0x%02X non riuscito.\n", command);
            return EXIT_FAILURE; // NACK

        }

        return EXIT_SUCCESS; // Comando riuscito
    }

    fprintf(stderr, "Errore: Risposta non valida per il comando 0x%02X.\n", command);
    return EXIT_FAILURE; // Risposta non valida
}

int dwm_set_initiator(void) {
    return dwm_simple_cmd_ack(CMD_SET_MODE_INIT, NULL, 0);
    #if 0
    uint8_t command = CMD_SET_MODE_INIT;
    uint8_t response[5]; // comando + len + status + checksum
    uint16_t resp_len;

    if(dwm_send_packet(command, NULL, 0) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante l'invio del comando per impostare la modalità iniziatore.\n");
        return EXIT_FAILURE; // Errore di invio
    }

    // Attendo risposta con ACK o NACK
    if(dwm_receive_packet(response, sizeof(response), &resp_len, 300) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante la ricezione della risposta per la modalità iniziatore.\n");
        return EXIT_FAILURE; // Errore di ricezione
    }

    if(response[3] == 0x01) { // Risposta positiva
        printf("Modalità Iniziatore impostata correttamente.\n");
        return EXIT_SUCCESS; // Successo
    } else {
        fprintf(stderr, "Errore: Modalità Iniziatore non impostata correttamente.\n");
        return EXIT_FAILURE; // Errore di risposta
    }

    return EXIT_FAILURE; // Non dovrebbe mai arrivare qui
    #endif

}

int dwm_set_responder(void) {
    return dwm_simple_cmd_ack(CMD_SET_MODE_RESP, NULL, 0);
    #if 0
    uint8_t command = CMD_SET_MODE_RESP;
    uint8_t response[5]; // comando + len + status + checksum
    uint16_t resp_len;

    if(dwm_send_packet(command, NULL, 0) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante l'invio del comando per impostare la modalità rispondente.\n");
        return EXIT_FAILURE; // Errore di invio
    }

    usleep(50000); // Attendo un breve periodo per garantire che il comando sia elaborato

    // Attendo risposta con ACK o NACK
    if(dwm_receive_packet(response, sizeof(response), &resp_len, 1000) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante la ricezione della risposta per la modalità rispondente.\n");
        return EXIT_FAILURE; // Errore di ricezione
    }

    /*
    if(response[3] == 0x01) { // Risposta positiva
        printf("Modalità rispondente impostata correttamente.\n");
        return EXIT_SUCCESS; // Successo
    } else {
        fprintf(stderr, "Errore: Modalità rispondente non impostata correttamente.\n");
        return EXIT_FAILURE; // Errore di risposta
    }
        */

    return EXIT_FAILURE; // Non dovrebbe mai arrivare qui
    #endif

}

int dwm_set_id(uint64_t new_id){
    uint8_t payload[8];
    memcpy(payload, &new_id, sizeof(new_id)); // Copio l'ID nei primi 8 byte del payload
    return dwm_simple_cmd_ack(CMD_SET_ID, payload, sizeof(payload));
}

int dwm_enable_device(uint64_t anchor_id) {
    uint8_t payload[8];
    memcpy(payload, &anchor_id, sizeof(uint64_t));
    return dwm_simple_cmd_ack(CMD_ENABLE_ANCHOR, payload, sizeof(payload));
}

int dwm_disable_device(uint64_t anchor_id) {
    uint8_t payload[8];
    memcpy(payload, &anchor_id, sizeof(uint64_t));
    return dwm_simple_cmd_ack(CMD_DISABLE_ANCHOR, payload, sizeof(payload));
}

int dwm_enter_config_mode(void) {
    return dwm_simple_cmd_ack(CMD_ENTER_CONFIG_MODE, NULL, 0);
}

int dwm_exit_config_mode(void) {
    return dwm_simple_cmd_ack(CMD_EXIT_CONFIG_MODE, NULL, 0);
}

int dwm_set_num_devices(uint8_t num_devices) {
    return dwm_simple_cmd_ack(CMD_SET_NUM_DEVICES, &num_devices, 1);
}

int dwm_set_device_id_at(uint8_t index, uint64_t device_id) {
    uint8_t payload[9];
    payload[0] = index;
    memcpy(&payload[1], &device_id, sizeof(uint64_t));
    return dwm_simple_cmd_ack(CMD_SET_DEVICE_ID_AT, payload, sizeof(payload));
}

int dwm_set_devices_id(uint64_t* device_ids, uint8_t num_devices) {
    if (num_devices > MAX_DWM_RESPONDERS) {
        fprintf(stderr, "Errore: numero di dispositivi troppo elevato.\n");
        return EXIT_FAILURE;
    }

    for (uint8_t i = 0; i < num_devices; i++) {
        if (dwm_set_device_id_at(i, device_ids[i]) != EXIT_SUCCESS) {
            fprintf(stderr, "Errore nell'impostare l'ID per il dispositivo all'indice %d.\n", i);
            return EXIT_FAILURE;
        }
        usleep(50000); // Piccola pausa tra i comandi
    }
    return EXIT_SUCCESS;
}

int dwm_measure(uint64_t target_id, uint8_t num_samples, AverageMeasurement* result) {
    if (!result) return EXIT_FAILURE;

    uint8_t payload[9];
    uint8_t response[256];
    uint16_t resp_len;

    memcpy(&payload[0], &target_id, sizeof(uint64_t));
    payload[8] = num_samples;

    if (dwm_send_packet(CMD_MEASURE_DISTANCE, payload, sizeof(payload)) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }

    // Timeout più lungo per permettere le misurazioni
    if (dwm_receive_packet(response, sizeof(response), &resp_len, 2000) != EXIT_SUCCESS) {
        fprintf(stderr, "Nessuna risposta di misurazione ricevuta.\n");
        return EXIT_FAILURE;
    }
    
    // Il payload della risposta è 19 byte
    if (response[0] == CMD_MEASURE_DISTANCE && resp_len >= (3 + 1 + 19)) {
        uint8_t* resp_payload = &response[3];
        result->valid = resp_payload[0];
        result->requested_samples = resp_payload[1];
        result->samples_count = resp_payload[2];
        memcpy(&result->id, &resp_payload[3], sizeof(uint64_t));
        memcpy(&result->average_distance, &resp_payload[11], sizeof(double));
        return EXIT_SUCCESS;
    }

    return EXIT_FAILURE;
}

int dwm_measure_all(uint8_t num_samples, AverageMeasurement* results, int max_results, uint8_t* out_valid_count) {
    if (!results || !out_valid_count || max_results <= 0) return EXIT_FAILURE;
    
    uint8_t response[512];
    uint16_t resp_len;

    // Invio comando che richiede numero dispositivi abilitati
    if (dwm_send_packet(CMD_GET_NUM_DEVICES, NULL, 0) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante l'invio del comando measure_all.\n");
        return EXIT_FAILURE; // Errore di invio
    }

    // Attendo risposta con ACK o NACK
    usleep(50000); // Attendo un breve periodo per garantire che il comando sia elaborato

    if (dwm_receive_packet(response, sizeof(response), &resp_len, 1000) != EXIT_SUCCESS) {
        fprintf(stderr, "Errore durante la ricezione della risposta per il comando measure_all.\n");
        return EXIT_FAILURE; // Errore di ricezione
    }

    uint8_t num_devices = -1;

    if(response[0] == CMD_GET_NUM_DEVICES && resp_len == 5){
        uint16_t payload_len = response[1] | (response[2] << 8);
        if(payload_len == 1){
            num_devices = response[3]; // Numero di dispositivi abilitati
            printf("Numero di dispositivi abilitati: %d\n", num_devices);
        } else {
            fprintf(stderr, "Errore: Risposta non valida per il comando get_num_devices.\n");
            return EXIT_FAILURE; // Risposta non valida
        }
    }

    if (num_devices == -1) {
        fprintf(stderr, "Errore: Numero di dispositivi abilitati non ricevuto correttamente.\n");
        return EXIT_FAILURE; // Errore nel numero di dispositivi
    }

    // Invio il comando per misurare le distanze da tutti i dispositivi
    if (num_devices > max_results) {
        fprintf(stderr, "Errore: Numero di dispositivi abilitati supera il buffer dei risultati.\n");
        return EXIT_FAILURE; // Troppi dispositivi
    }

    if (num_devices == 0) {
        *out_valid_count = 0; // Nessun dispositivo abilitato
        return EXIT_SUCCESS; // Successo con zero dispositivi
    }

    int wait_time_per_device_ms = 200; // Tempo di attesa per ogni dispositivo
    int total_wait_time_ms = num_devices * num_samples * wait_time_per_device_ms; // Tempo totale di attesa

    // Invia il comando per misurare le distanze da tutti i dispositivi
    if (dwm_send_packet(CMD_MEASURE_ALL_DISTANCES, &num_samples, 1) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }

    // Timeout molto lungo per permettere tutte le misurazioni
    if (dwm_receive_packet(response, sizeof(response), &resp_len, total_wait_time_ms) != EXIT_SUCCESS) {
        fprintf(stderr, "Nessuna risposta per measure_all ricevuta.\n");
        return EXIT_FAILURE;
    }

    if (response[0] == CMD_MEASURE_ALL_DISTANCES) {
        uint8_t* payload = &response[3];
        uint8_t num_valid = payload[0];
        *out_valid_count = (num_valid > max_results) ? max_results : num_valid;
        
        int current_pos = 1;
        const int bytes_per_meas = sizeof(uint64_t) + sizeof(uint8_t) + sizeof(double);

        for (int i = 0; i < *out_valid_count; i++) {
            memcpy(&results[i].id, &payload[current_pos], sizeof(uint64_t));
            current_pos += sizeof(uint64_t);
            results[i].samples_count = payload[current_pos++];
            memcpy(&results[i].average_distance, &payload[current_pos], sizeof(double));
            current_pos += sizeof(double);

            results[i].valid = 1;
            results[i].requested_samples = num_samples;
        }
        return EXIT_SUCCESS;
    }
    
    return EXIT_FAILURE;
}

int dwm_get_info(DeviceInfo* info) {
    if (!info) return EXIT_FAILURE;

    uint8_t response[256];
    uint16_t resp_len;
    
    if (dwm_send_packet(CMD_GET_INFO, NULL, 0) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }

    if (dwm_receive_packet(response, sizeof(response), &resp_len, 500) != EXIT_SUCCESS) {
        fprintf(stderr, "Nessuna risposta per get_info.\n");
        return EXIT_FAILURE;
    }
    
    if (response[0] == CMD_GET_INFO && resp_len >= (3 + 1 + 11)) { // 11 byte payload
        uint8_t* payload = &response[3];
        info->device_mode = payload[0];
        memcpy(&info->device_id, &payload[1], sizeof(uint64_t));
        // Il campo config_mod_active non è presente nella risposta del firmware, si può omettere o aggiungere.
        info->config_mod_active = 0; // Placeholder
        return EXIT_SUCCESS;
    }

    return EXIT_FAILURE;
}

int dwm_set_nlos_mode(void) {
    return dwm_simple_cmd_ack(CMD_SET_NLOS_MODE, NULL, 0);
}

int dwm_set_los_mode(void) {
    return dwm_simple_cmd_ack(CMD_SET_LOS_MODE, NULL, 0);
}

int dwm_set_antenna_tx_delay(uint16_t delay) {
    uint8_t payload[2];
    payload[0] = (uint8_t)(delay & 0xFF);
    payload[1] = (uint8_t)((delay >> 8) & 0xFF);
    return dwm_simple_cmd_ack(CMD_SET_ANTENNA_TX_DELAY, payload, sizeof(payload));
}

int dwm_set_antenna_rx_delay(uint16_t delay) {
    uint8_t payload[2];
    payload[0] = (uint8_t)(delay & 0xFF);
    payload[1] = (uint8_t)((delay >> 8) & 0xFF);
    return dwm_simple_cmd_ack(CMD_SET_ANTENNA_RX_DELAY, payload, sizeof(payload));
}

#if 0
int dwm_disable_device(uint64_t anchor_id){
    uint8_t tx_buff[9]; // comando + 8 byte id
    uint8_t rx_buff[9]; // ignorato

    tx_buff[0] = CMD_DISABLE_ANCHOR;
    //popolo array con id scomposto in 8 byte
    for(int i=0; i<8; i++){
        tx_buff[i+1] = (uint8_t)((anchor_id >> (i*8)) & 0xFF);
    }

    // invio e ricevo risposta
    int ret = dwm_spi_transfer(tx_buff, rx_buff, sizeof(tx_buff));
    if(ret!=0){
        fprintf(stderr, "Errore nell'invio comando DISABLE_ANCHOR\n");
        return EXIT_FAILURE;
    }

    printf("Disabilitata ancora con id 0x%llx\n", (unsigned long long)anchor_id);
    return EXIT_SUCCESS;
}

int dwm_set_num_devices(uint8_t num_devices){
    int ret = dwm_send_command_with_arg(CMD_SET_NUM_DEVICES, num_devices);
    if(ret!=0){
        fprintf(stderr, "Errore nell'invio comando SET_NUM_DEVICES\n");
        return EXIT_FAILURE;
    }

    printf("Numero di dispositivi impostato a: %d!\n", num_devices);
    return EXIT_SUCCESS;
}

int dwm_set_device_id_at(uint8_t index, uint64_t device_id){
    // Prima controllo che l'indce non superi il numero massimo di dispositiv
    // consentiti
    if(index >= MAX_DWM_RESPONDERS){
        fprintf(stderr, "Errore: Indice non valido. Deve essere tra 0 e %d.\n", MAX_DWM_RESPONDERS - 1);
        return EXIT_FAILURE;
    }

    uint8_t tx_buf[10]; // comando + index + 8 byte id
    uint8_t rx_buf[10]; // ignorato

    tx_buf[0] = CMD_SET_DEVICE_ID_AT;
    tx_buf[1] = index;

    //popolo array con id scomposto in 8 byte
    for(int i=0; i<8; i++){
        tx_buf[i+2] = (uint8_t)((device_id >> (i*8)) & 0xFF);
    }

    int ret = dwm_spi_transfer(tx_buf, rx_buf, sizeof(tx_buf));
    if (ret != 0) {
        fprintf(stderr, "Errore durante l'impostazione dell'ID del dispositivo all'indice %d.\n", index);
        return EXIT_FAILURE;
    }
    
    printf("ID dispositivo all'indice %d impostato a: 0x%llx\n", index, (unsigned long long)device_id);
    return EXIT_SUCCESS;
}

int dwm_set_devices_id(uint64_t *device_ids, uint8_t num_devices){
    // Controllo che il numero di dispositivi non superi il massimo
    if(num_devices > MAX_DWM_RESPONDERS){
        fprintf(stderr, "Errore: Numero di dispositivi non valido. Deve essere tra 0 e %d.\n", MAX_DWM_RESPONDERS - 1);
        return EXIT_FAILURE;
    }

    // Controllo che il puntatore non sia nullo
    if(!device_ids){
        fprintf(stderr, "Errore: Puntatore nullo per device_ids.\n");
        return EXIT_FAILURE;
    }

    // per ogni dispositivo, invio il comando di settaggio id
    for(int i=0; i<num_devices; i++){
        int ret = dwm_set_device_id_at(i, device_ids[i]);
        if(ret!=0){
            fprintf(stderr, "Errore durante l'impostazione dell'ID del dispositivo all'indice %d.\n", i);
            return EXIT_FAILURE;
        }
    }
    printf("ID dei dispositivi impostati correttamente!\n");
    return EXIT_SUCCESS;
}

int dwm_measure(uint64_t target_id, uint8_t num_samples, AverageMeasurement* result){
    // Controllo che il puntatore sia valido
    if(!result) {
        fprintf(stderr, "Errore: Parametri non validi per dwm_measure_average.\n");
        return EXIT_FAILURE;
    }

    // setto memoria
    memset(result, 0, sizeof(AverageMeasurement));

    uint8_t tx_buf[10]; // comando + 8 byte ID + 1 byte num_misurazioni
    uint8_t rx_buf[sizeof(double) + 12]; // buffer lettura risposta
    uint8_t tx_dummy = 0x00;

    tx_buf[0] = CMD_MEASURE_DISTANCE;

    for (int i = 0; i < 8; i++) {
        tx_buf[i+1] = (uint8_t)((target_id >> (i * 8)) & 0xFF);
    }

    size_t cmd_len = 10;
    tx_buf[9] = 10;

    int ret = dwm_spi_transfer(tx_buf, rx_buf, cmd_len);
    if (ret != 0) {
        fprintf(stderr, "Errore durante l'invio del comando MEASURE_AVERAGE.\n");
        return EXIT_FAILURE;
    }

    // Attesa misurazioni, 200ms per campione
    int wait_time = 1000; // ms
    usleep(wait_time * 1000);

    // leggo flag di validità
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, 1);
    if (ret != 0) return EXIT_FAILURE;
    result->valid = rx_buf[0];

    // Leggi numero di campioni richiesti
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, 1);
    if (ret != 0) return EXIT_FAILURE;
    result->requested_samples = rx_buf[0];
    
    // Leggi numero di campioni validi
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, 1);
    if (ret != 0) return EXIT_FAILURE;
    result->samples_count = rx_buf[0];
    
    // Leggi ID target (8 byte)
    uint64_t id = 0;
    for (int i = 0; i < 8; i++) {
        ret = dwm_spi_transfer(&tx_dummy, &rx_buf[i], 1);
        if (ret != 0) return EXIT_FAILURE;
        id |= ((uint64_t)rx_buf[i] << (i * 8));
    }
    result->id = id;
    
    // Leggi distanza media
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, sizeof(double));
    if (ret != 0) return EXIT_FAILURE;
    memcpy(&(result->average_distance), rx_buf, sizeof(double));
    
    if (result->valid) {
        printf("Misurazione media completata. Target: 0x%llx, Campioni: %d/%d, Distanza: %.3f m\n",
               (unsigned long long)result->id, result->samples_count, result->requested_samples,
               result->average_distance);
    } else {
        printf("Misurazione media fallita. Target: 0x%llx, Campioni: %d/%d\n",
               (unsigned long long)result->id, result->samples_count, result->requested_samples);
    }
    
    return EXIT_SUCCESS;

}

int dwm_measure_all(uint8_t num_samples, AverageMeasurement* results, int max_results, uint8_t* out_valid_count){
    // Controllo che i puntatori  e i parametri siano corretti
    if( !results || !out_valid_count || max_results <= 0){
        fprintf(stderr, "Errore nei parametri della funzione di measure_average_all\n");
        return EXIT_FAILURE;
    }

    uint8_t tx_buf[2]; // comando + num_samples
    uint8_t rx_count_buf[1]; // conteggio misurazioni lette
    uint8_t tx_dummy = 0x00;
    int ret;

    // Invalido risultati salvati attualmente nell'array
    for(int i=0;i<MAX_DWM_RESPONDERS;i++){
        results[i].valid=0;
    }

    //Salvo comando nel buffer di invio
    tx_buf[0]=CMD_MEASURE_ALL_DISTANCES;

    //Imposto num__samples
    tx_buf[1]=10;

    // invio comando
    ret = dwm_spi_transfer(tx_buf, rx_count_buf, 2);
    if(ret!=0){
        fprintf(stderr, "Errore nell'invio comando di misurazione tutte distanze\n");
        return EXIT_FAILURE;
    }

    printf("Avviate misurazioni verso tutti i dispositivi\n");

    // Attendo 
    int wait_time = num_samples * MAX_DWM_RESPONDERS * 200;
    usleep(wait_time*1000);

    // Leggo numero misurazioni valide
    ret = dwm_spi_transfer(&tx_dummy, rx_count_buf, 1);
    if(ret!=0) return EXIT_FAILURE;

    uint8_t count = rx_count_buf[0];
    *out_valid_count = count;

    if(count == 0){
        printf("Nessuna misurazione riportata\n");
        return EXIT_SUCCESS;
    }

    if(count > max_results){
        fprintf(stderr, "Troppe misurazioni catturate, errore\n");
        count=max_results;
    }

    printf("Ricevute %d misurazioni valide\n", count);

    const int bytes_per_measurement = 8 + 1 + 8; // 8 byte id + 1 byte conteggio + 8 byte distanza media
    const int total_response_size = count * bytes_per_measurement;

    // Alloco buffer che contenga l'intera risposta
    uint8_t* response_buffer = (uint8_t*)malloc(total_response_size);
    if(!response_buffer){
        fprintf(stderr, "Errore nell'allocazione memoria per il buffer di risposta\n");
        return EXIT_FAILURE; 
    }

    // Leggo la risposta, creando un buffer di dummy bytes da inviare
    uint8_t* tx_dummy_buf = (uint8_t*)malloc(total_response_size);
    if (!tx_dummy_buf) {
        fprintf(stderr, "Errore nell'allocazione memoria per il buffer di invio\n");
        free(response_buffer);
        return EXIT_FAILURE;
    }
    memset(tx_dummy_buf, 0, total_response_size);
    ret = dwm_spi_transfer(tx_dummy_buf, response_buffer, total_response_size);
    free(tx_dummy_buf);
    if(ret!=0){
        fprintf(stderr, "Errore nella lettura risposta completa\n");
        free(response_buffer);
        return EXIT_FAILURE;
    }


    // Analizzo le singole entry della risposta
    for(int i=0;i<count; i++){

        // preparo offset di lettura nel buffer di risposta
        int offset = i * bytes_per_measurement;

        // Estraggo ID
        uint64_t id = 0;
        for(int j=0; j<8;j++){
            id |= ((uint64_t)response_buffer[offset + j] << (j*8));
        }

        results[i].id = id;

        //Estraggo numero campioni validi
        results[i].samples_count = response_buffer[offset+8];

        // Estraggo distanza media
        memcpy(&(results[i].average_distance), &response_buffer[offset+9], sizeof(double));

        // imposto altri campi
        results[i].valid = 1;
        results[i].requested_samples = num_samples > 0 ? num_samples : 10;

        printf("Misurazione %d: Target: 0x%llx, Campioni: %d, Distanza: %.3f m\n",
        i+1, (unsigned long long)results[i].id, results[i].samples_count, results[i].average_distance);
    }

    free(response_buffer); // libero puntatore a fine funzione
    return EXIT_SUCCESS;

}
#endif