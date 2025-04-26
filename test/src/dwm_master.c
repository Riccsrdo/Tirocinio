#include "dwm_spi_master.h"

// Variabili statiche per la gestione del device SPI
static int spi_fd = -1; // File descriptor per il device SPI
static uint8_t spi_mode = 2; // Modalità di connessione SPI
static uint8_t spi_bits = 8; // Bits in trasferimento
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
        // .cs_change = 0, 
    };

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr); // Invia 1 struttura spi_ioc_transfer
    if (ret < 1) { // 
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

int dwm_set_id(uint64_t new_id){
    uint8_t tx_buff[9]; // comando + 8 byte id
    uint8_t rx_buff[9]; // buffer di ricezione ignorato

    tx_buff[0] = CMD_SET_ID;
    // popolo il resto del buffer con il nuovo id
    // scomposto in 8 byte
    for(int i=0; i<8; i++){
        tx_buff[i+1] = (uint8_t)((new_id >> (i*8)) & 0xFF);
    }

    // Invio comando e id
    int ret = dwm_spi_transfer(tx_buff, rx_buff, sizeof(tx_buff));
    if(ret!=0){
        fprintf(stderr, "Errore durante invio comando SET_ID\n");
        return -1;
    }

    printf("Ide del dispositivo impostato a 0x%llx\n", (unsigned long long)new_id);
    return 0;
}

int dwm_enable_anchor(uint64_t anchor_id){
    uint8_t tx_buff[9]; // comando + 8 byte id
    uint8_t rx_buff[9]; // ignorato

    tx_buff[0] = CMD_ENABLE_ANCHOR;
    //popolo array con id scomposto in 8 byte
    for(int i=0; i<8; i++){
        tx_buff[i+1] = (uint8_t)((anchor_id >> (i*8)) & 0xFF);
    }

    // invio e ricevo risposta
    int ret = dwm_spi_transfer(tx_buff, rx_buff, sizeof(tx_buff));
    if(ret!=0){
        fprintf(stderr, "Errore nell'invio comando ENABLE_ANCHOR\n");
        return -1;
    }

    printf("Abilitata ancora con id 0x%llx\n", (unsigned long long)anchor_id);
}

int dwm_disable_anchor(uint64_t anchor_id){
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
        return -1;
    }

    printf("Disabilitata ancora con id 0x%llx\n", (unsigned long long)anchor_id);
}

int dwm_enter_config_mode(void){
    int ret = dwm_send_command(CMD_ENTER_CONFIG_MODE);
    if(ret!=0){
        fprintf(stderr, "Errore nell'invio comando ENTER_CONFIG_MODE\n");
        return -1;
    }

    printf("Dispositivo ora in modalità config!\n");
    return 0;
}


int dwm_exit_config_mode(void){
    int ret = dwm_send_command(CMD_EXIT_CONFIG_MODE);
    if(ret!=0){
        fprintf(stderr, "Errore nell'invio comando EXIT_CONFIG_MODE\n");
        return -1;
    }

    printf("Dispositivo uscito dalla modalità configurazione!\n");
    return 0;
}

int dwm_set_num_devices(uint8_t num_devices){
    int ret = dwm_send_command_with_arg(CMD_SET_NUM_DEVICES, num_devices);
    if(ret!=0){
        fprintf(stderr, "Errore nell'invio comando SET_NUM_DEVICES\n");
        return -1;
    }

    printf("Numero di dispositivi impostato a: %d!\n", num_devices);
    return 0;
}

int dwm_set_device_id_at(uint8_t index, uint64_t device_id){
    // Prima controllo che l'indce non superi il numero massimo di dispositiv
    // consentiti
    if(index >= MAX_DWM_RESPONDERS){
        fprintf(stderr, "Errore: Indice non valido. Deve essere tra 0 e %d.\n", MAX_DWM_RESPONDERS - 1);
        return -1;
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
        return -1;
    }
    
    printf("ID dispositivo all'indice %d impostato a: 0x%llx\n", index, (unsigned long long)device_id);
    return 0;
}

int dwm_measure_average(uint64_t target_id, uint8_t num_samples, AverageMeasurement* result){
    // Controllo che il puntatore sia valido
    if(!result) {
        fprintf(stderr, "Errore: Parametri non validi per dwm_measure_average.\n");
        return -1;
    }

    // alloco memoria
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
        return -1;
    }

    // Attesa misurazioni, 200ms per campione
    int wait_time = 1000; // ms
    usleep(wait_time * 1000);

    // leggo flag di validità
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, 1);
    if (ret != 0) return -1;
    result->valid = rx_buf[0];

    // Leggi numero di campioni richiesti
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, 1);
    if (ret != 0) return -1;
    result->requested_samples = rx_buf[0];
    
    // Leggi numero di campioni validi
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, 1);
    if (ret != 0) return -1;
    result->samples_count = rx_buf[0];
    
    // Leggi ID target (8 byte)
    uint64_t id = 0;
    for (int i = 0; i < 8; i++) {
        ret = dwm_spi_transfer(&tx_dummy, &rx_buf[i], 1);
        if (ret != 0) return -1;
        id |= ((uint64_t)rx_buf[i] << (i * 8));
    }
    result->id = id;
    
    // Leggi distanza media
    ret = dwm_spi_transfer(&tx_dummy, rx_buf, sizeof(double));
    if (ret != 0) return -1;
    memcpy(&(result->average_distance), rx_buf, sizeof(double));
    
    if (result->valid) {
        printf("Misurazione media completata. Target: 0x%llx, Campioni: %d/%d, Distanza: %.3f m\n",
               (unsigned long long)result->id, result->samples_count, result->requested_samples,
               result->average_distance);
    } else {
        printf("Misurazione media fallita. Target: 0x%llx, Campioni: %d/%d\n",
               (unsigned long long)result->id, result->samples_count, result->requested_samples);
    }
    
    return 0;

}

int dwm_measure_average_all(uint8_t num_samples, AverageMeasurement* results, int max_results, uint8_t* out_valid_count){
    // Controllo che i puntatori  e i parametri siano corretti
    if( !results || !out_valid_count || max_results <= 0){
        fprintf(stderr, "Errore nei parametri della funzione di measure_average_all\n");
        return -1;
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
        return -1;
    }

    printf("Avviate misurazioni verso tutti i dispositivi\n");

    // Attendo 
    int wait_time = num_samples * MAX_DWM_RESPONDERS * 200;
    usleep(wait_time*1000);

    // Leggo numero misurazioni valide
    ret = dwm_spi_transfer(&tx_dummy, rx_count_buf, 1);
    if(ret!=0) return -1;

    uint8_t count = rx_count_buf[0];
    *out_valid_count = count;

    if(count == 0){
        printf("Nessuna misurazione riportata\n");
        return 0;
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
        return -1; 
    }

    // Leggo la risposta, creando un buffer di dummy bytes da inviare
    uint8_t* tx_dummy_buf = (uint8_t*)malloc(total_response_size);
    if (!tx_dummy_buf) {
        fprintf(stderr, "Errore nell'allocazione memoria per il buffer di invio\n");
        free(response_buffer);
        return -1;
    }
    memset(tx_dummy_buf, 0, total_response_size);
    ret = dwm_spi_transfer(tx_dummy_buf, response_buffer, total_response_size);
    free(tx_dummy_buf);
    if(ret!=0){
        fprintf(stderr, "Errore nella lettura risposta completa\n");
        free(response_buffer);
        return -1;
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
    return 0;

}