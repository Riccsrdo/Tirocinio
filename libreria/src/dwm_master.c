#include "dwm_master.h"
#include "dwm_utilities.h"

// Variabili statiche per la gestione del device SPI
static int spi_fd = -1; // File descriptor per il device SPI
//static uint8_t spi_mode = 2; // Modalità di connessione SPI
//static uint8_t spi_bits = 8; // Bits in trasferimento
//static uint32_t spi_speed = 2000000; // Default 2 MHz

// Funzione interna per gestire errori
static void print_spi_error(const char* action) {
    perror(action); // Stampa il messaggio di errore di sistema
}


int dwm_spi_init(const char* device, uint32_t speed, uint8_t mode) {
    int ret;

    if (spi_fd >= 0) { // se il file descriptor è stato già inizializzato, stampa errore
        fprintf(stderr, "Errore: SPI già inizializzato.\n");
        return EXIT_FAILURE; // Già aperto
    }

    spi_fd = open(device, O_RDWR); // altrimenti apri la connessione SPI
    if (spi_fd < 0) {
        print_spi_error("Errore apertura device SPI");
        return EXIT_FAILURE;
    }

    spi_mode = mode; // imposta la modalità (0,1,2,3)
    spi_speed = speed; // e la velocità

    // Imposta modalità SPI (CPOL, CPHA)
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (ret == -1) {
        print_spi_error("Errore impostazione SPI mode (WR)");
        close(spi_fd);
        spi_fd = -1;
        return EXIT_FAILURE;
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
        return EXIT_FAILURE;
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
        return EXIT_FAILURE;
    }

    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
     if (ret == -1) {
        print_spi_error("Errore lettura velocità massima (RD)");
    }

    printf("SPI Inizializzato: %s\n", device);
    printf("  Mode: %d, Bits: %d, Speed: %d Hz\n", spi_mode, spi_bits, spi_speed);

    return EXIT_SUCCESS; // Successo
}

void dwm_spi_close(void) {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
        printf("Interfaccia SPI chiusa.\n");
    }
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
        return EXIT_FAILURE;
    }

    printf("Ide del dispositivo impostato a 0x%llx\n", (unsigned long long)new_id);
    return EXIT_SUCCESS;
}

int dwm_enable_device(uint64_t anchor_id){
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
        return EXIT_FAILURE;
    }

    printf("Abilitata ancora con id 0x%llx\n", (unsigned long long)anchor_id);

    return EXIT_SUCCESS;
}

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