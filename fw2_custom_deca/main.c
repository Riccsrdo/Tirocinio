
/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

 #include "sdk_config.h"
 #include "FreeRTOS.h"
 #include "task.h"
 #include "timers.h"
 #include "bsp.h"
 #include "boards.h"
 #include "nordic_common.h"
 #include "nrf_drv_clock.h"
 //#include "nrf_drv_spi.h"
 //#include "../components/drivers_nrf/spi_slave/nrf_drv_spis.h"
 //#include "nRF5_SDK_14.2.0\components\drivers_nrf\spi_slave/nrf_drv_spis.h"
 #include "nRF5_SDK_14.2.0/components/drivers_nrf/spi_slave/nrf_drv_spis.h"
 #include "nrf_uart.h"
 #include "app_util_platform.h"
 #include "nrf_gpio.h"
 #include "nrf_delay.h"
 #include "nrf_log.h"
 #include "nrf.h"
 #include "app_error.h"
 #include "app_util_platform.h"
 #include "app_error.h"
 #include <string.h>
 #include "port_platform.h"
 #include "deca_types.h"
 #include "deca_param_types.h"
 #include "deca_regs.h"
 #include "deca_device_api.h"
 #include "uart.h"
 //#include "ss_init_main.h"
 #include "nrf_drv_gpiote.h"
 #include "utils.h"
 #include "math.h"
 #include "sys/time.h"
 //#include "UART.h"
 #include "nrf_drv_uart.h" // Funzioni di utilità e gestione UART
 
 /* Dichiaro in precedenza le funzioni che dovrÃ² usare successivamente */
 extern int ss_init_run(uint64_t anchor_id);           // Funzione per utilizzo come iniziatore comunicazione
 extern int ss_resp_run(uint64_t anchor_id);           // Funzione per utilizzo come risponditore comunicazione
 extern void ss_main_task_function(void *pvParameter); // Funzione per gestione task
 extern void ss_initiator_task_function(void *pvParameter);

 // Callbacks for DW1000 Interrupts
 extern void tx_conf_cb(const dwt_cb_data_t *cb_data);
 extern void rx_ok_cb(const dwt_cb_data_t *cb_data);
 extern void rx_to_cb(const dwt_cb_data_t *cb_data);
 extern void rx_err_cb(const dwt_cb_data_t *cb_data);

  static void prepare_distance_data_for_spi(void);
  static void prepare_info_data_for_spi(void);
  static void prepare_config_info_for_spi(void);
  static void prepare_avg_measurement_for_spi(void);
  static void prepare_all_measurements_for_spi(void);


 // ------------------ Finite State Machine per SPI ---------------------

 typedef enum {
    STATE_IDLE, // il dispositivo è in attesa di comandi
    STATE_DATA_READY, // i dati sono pronti per essere letti dal master
    STATE_PROCESSING, // esecuzione operazione logica
    STATE_ERROR // errore verificato
 } device_state_t;

 static volatile device_state_t current_state = STATE_IDLE;  // parte da idle

 
 //-----------------dw1000----------------------------
 
 // Configurazione per condizioni di LoS
 static dwt_config_t config = {
     5,               /* Channel number. */
     DWT_PRF_64M,     /* Pulse repetition frequency. */
     DWT_PLEN_128,    /* Preamble length. Used in TX only. */
     DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
     10,              /* TX preamble code. Used in TX only. */
     10,              /* RX preamble code. Used in RX only. */
     0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
     DWT_BR_6M8,      /* Data rate. */
     DWT_PHRMODE_STD, /* PHY header mode. */
     (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
 };

 // Configurazione per condizioni di NLoS
 static dwt_config_t nlos_config = {
     5,               /* Channel number. */
     DWT_PRF_16M,     /* Pulse repetition frequency. */
     DWT_PLEN_1024,    /* Preamble length. Used in TX only. */
     DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
     10,              /* TX preamble code. Used in TX only. */
     10,              /* RX preamble code. Used in RX only. */
     0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
     DWT_BR_850K,      /* Data rate. Reduced to 850K, increasing receiver sensibility */
     DWT_PHRMODE_STD, /* PHY header mode. */
     (1025 + 64 - 32)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
 };
 
 /* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
 #define PRE_TIMEOUT 1000
 
 /* Delay between frames, in UWB microseconds. See NOTE 1 below. */
 #define POLL_TX_TO_RESP_RX_DLY_UUS 100
 
 /*Should be accurately calculated during calibration*/
 #ifndef TX_ANT_DLY
 #define TX_ANT_DLY 16300
 #endif
 #define RX_ANT_DLY 16456

 // Delay Antenne
 volatile int RECEIVER_DELAY = 16456;
 volatile int TRANSMITTER_DELAY = 16300;
 
 //--------------dw1000---end---------------
 
 #define TASK_DELAY 200    /**< Task delay. Delays a LED0 task for 200 ms */
 #define TIMER_PERIOD 2000 /**< Timer period. LED1 timer will expire after 1000 ms */
 #define RNG_DELAY_MS 100  /**< Delay between two ranging requests. */
 
 /* Variabile globale che tiene conto della modalitÃ  di utilizzo del dispositivo */
 #if 0
 typedef enum
 {
   DEVICE_MODE_INITIATOR = 0, /* Dispositivo in modalitÃ  iniziatore */
   DEVICE_MODE_RESPONDER = 1, /* Dispositivo in modalitÃ  risponditore */
 } device_mode_t;

 #define MAX_RESPONDERS 16 // da aggiornare anche su ss_init_main.c in caso di cambiamento
 #endif
 
 volatile device_mode_t device_mode = DEVICE_MODE_INITIATOR; /* Imposto modalitÃ  iniziale come iniziatore */
 volatile bool bool_mode_changed = false;                         /* Flag booleana per inidicare che la modalitÃ  Ã¨ cambiata */
 
 /* Impostazioni modalitÃ  multi-risponditore
 Configurare in base al numero di risponditori utilizzati */

 volatile uint64_t anchor_ids[MAX_RESPONDERS] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
                                                           9, 10, 11, 12, 13, 14, 15}; /* ID dei risponditori */
 volatile uint64_t DEVICE_ID = 0; /* ID del dispositivo in uso */                // DA CONFIGURARE IN BASE AL DISPOSITIVO
 volatile bool anchor_enabled[MAX_RESPONDERS] = {true, true, false, false, false, false, false, false,
                                                        false, false, false, false, false, false, false, false}; /* Abilitazione dei risponditori */


 volatile bool nlos_mode=false; // flag booleana di check se dispositivo in modalità NLoS

 
 
#if 1

 /*-------------------------------SPI--------------------------------*/

 #define SPIS_INSTANCE 2 
 static const nrf_drv_spis_t m_spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE); // < SPIS instance.


 // Pin: DEPRECATI
#define MY_SPIS_CSN_PIN    NRF_GPIO_PIN_MAP(0, 28) // Modulo Pin 29 -> Connettore Pin 24 (CS_RPI)
#define MY_SPIS_SCK_PIN    NRF_GPIO_PIN_MAP(0, 8)  // Modulo Pin 25 -> Connettore Pin 23 (SPI1_CLK) 
#define MY_SPIS_MOSI_PIN   NRF_GPIO_PIN_MAP(0, 30) // Modulo Pin 27 -> Connettore Pin 19 (SPI1_MOSI)
#define MY_SPIS_MISO_PIN   NRF_GPIO_PIN_MAP(0, 31) // Modulo Pin 26 -> Connettore Pin 21 (SPI1_MISO)

 // Dimensione buffer per ricezione/invio dati dal/al master
 #define SPI_CMD_BUFFER_SIZE 32 // Max command length from master
 #define SPI_TX_DATA_BUFFER_SIZE 255 // Max data length to master
 
 // Buffer di salvataggio dati da inviare via SPI
 static uint8_t m_spi_rx_buf[SPI_CMD_BUFFER_SIZE]; 
 static uint8_t m_spi_tx_buf[SPI_TX_DATA_BUFFER_SIZE]; 

 // Variabili per il buffer di comandi
 static uint8_t spi_cmd_buffer[SPI_CMD_BUFFER_SIZE]; // Separate buffer to copy cmd into
 static volatile uint8_t spi_cmd_length = 0;         // Length of command in spi_cmd_buffer
 volatile bool new_spi_command_received = false; // Flag for newly received SPI command

 // Comandi possibili inviabili via SPI
 #define SPI_CMD_GET_DISTANCES 0x01 // Permette di ottenere le distanze
    // da implementare: comando che dato un id permette di ottenere la distanza dal dispositivo con quell'id

 #define SPI_CMD_GET_STATUS 0x02 // permette di ottenere lo stato della FSM in cui si trova il dispositivo
 #define SPI_CMD_READ_RESULTS 0x03 // master richiede risultati pronti
 #define SPI_CMD_ECHO_TEST     0x04 // Comando di test per l'echo
  
 #define SPI_CMD_SET_MODE_INIT 0x10 // Permette di settare il dispositivo come iniziatore e misurare le distanze dagli
 // altri dispositivi
 #define SPI_CMD_SET_MODE_RESP 0x11 // Permette di mettersi in ascolto e attendere richieste di comunicazione
 // dagli iniziatori
 #define SPI_CMD_SET_ID        0x20 // Permette di settare id per il dispositivo, deve essere seguito da 8 byte
 // che indica l'ID che si vuole settare
 #define SPI_CMD_ENABLE_ANCHOR 0x30 // Permette di abilitare l'ancora nell'array delle ancore disabilitate/abilitate, deve
 // essere seguito da un byte come ID
 #define SPI_CMD_DISABLE_ANCHOR 0x31 // Come sopra, serve id byte
 #define SPI_CMD_GET_INFO      0xFE // Informazioni sul sistema
 #define SPI_CMD_GET_HELP      0xFF // Dummy byte, usando per ottenere aiuto

 #define SPI_CMD_ENTER_CONFIG_MODE 0x40 // Comando per entrare in modalità configurazione e settare parametri come ID comunicazioni
 #define SPI_CMD_EXIT_CONFIG_MODE 0x41 // Esci dalla modalità di configurazione e salva le modifiche
 #define SPI_CMD_SET_NUM_DEVICES 0x42 // Imposta il numero di dispositivi con cui si vuole comunicare
 #define SPI_CMD_SET_DEVICE_ID_AT 0x43 // Imposta l'id del dispositivo all'index desiderato nella lista di ID
 #define SPI_CMD_MEASURE_DISTANCE 0x50 // Esegue misurazioni multiple e restituisce media, dato id
 #define SPI_CMD_MEASURE_ALL_DISTANCES 0x51 // Misura le distanze average da tutti i dispositivi settati

 #define SPI_SET_NLOS_MODE 0x60 // comando che permette di setuppare la modalità NLOS usando pacchetti più grandi
 #define SPI_SET_LOS_MODE 0x61 // imposta a LoS con pacchetti più piccoli
 #define SPI_SET_ANTENNA_TX_DELAY 0x62 // Permette di modificare il delay dell'antenna per calibrazione in transmission
 #define SPI_SET_ANTENNA_RX_DELAY 0x63 // idem ma in ricezione

 // Misurazioni media
 typedef struct {
  uint64_t target_id;
  bool valid;
  double average_distance;
  uint8_t samples_count; // campioni validi ottenuti
  uint8_t requested_samples; // campioni richiesti
 } average_measurement_t;

 // buffer globale misurazione media corrente
 static average_measurement_t current_avg_measurement = {0};
 //flag booleana che indica misurazioni in corso
 static volatile bool performing_avg_measurement = false;
 static average_measurement_t measurements[MAX_RESPONDERS];
 static uint8_t num_valid_measurements = 0;

 extern responder_distance_t distances[MAX_RESPONDERS];


 // Flag di configurazione e buffer
 static volatile bool in_config_mode = false;
 static volatile uint8_t config_num_devices = 0;
 static volatile uint64_t config_anchor_ids[MAX_RESPONDERS];
 static volatile bool config_anchor_enabled[MAX_RESPONDERS];
 static volatile bool continous_measurements = true;
 static volatile uint8_t current_spi_command = 0; // Store command being processed



 /*------------------------------fine-------------------------------*/





 /*------------------------------UART-------------------------------*/

  #define UART_RX_PIN 11 // Pin nRF52832 per gestione ricezione UART
  #define UART_TX_PIN 5 // Pin nRF52832 per gestione trasmissione UART

  /*
  Struttura pacchetto UART:

  - 1: Byte di partenza del pacchetto, indicato da 0xAA
  - 2: Id del comando, valore in esadecimale che indica comando
  - 3: Lunghezza del payload: due byte che indicano la lunghezza dei dati successivi
  - 4: Payload, contenuto effettivo inviato
  - 5: Checksum, controllo degli errori

  */

  #define START_BYTE 0xAA
  #define MAX_PAYLOAD_SIZE 512
  #define PACKET_BUFFER_SIZE (MAX_PAYLOAD_SIZE + 5) // Con 5 si intendono i 5 byte extra (1 inizio, 1 cmd, 2 lunghezza, 1 checksum)
  
  static char uart_packet_buffer[PACKET_BUFFER_SIZE]; 
  static uint16_t uart_packet_index = 0;
  static uint16_t uart_payload_len = 0;
  volatile bool uart_new_command = false; // TODO: Verifica se funziona con dichiarazione extern, sennò rimetti static qui (prima di volatile)

  // FSM per UART
  typedef enum {
      STATE_WAIT_FOR_START,
      STATE_RECEIVE_CMD,
      STATE_RECEIVE_LEN_LSB,
      STATE_RECEIVE_LEN_MSB,
      STATE_RECEIVE_PAYLOAD,
      STATE_RECEIVE_CHECKSUM
  } uart_rx_state_t;

  static uart_rx_state_t uart_state = STATE_WAIT_FOR_START; // parte in attesa di comandi

  // payload risposta
  static uint8_t success_payload[1];

  // Istanza driver uart
  static const nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);
  static uint8_t m_uart_rx_byte; // byte ricezione

  /*------------------------------fine-------------------------------*/





 /* Gestione del buffer di comandi provenienti da UART */
//#define CMD_BUFFER_SIZE 32
//static char cmd_buffer[CMD_BUFFER_SIZE];
//static uint8_t cmd_buffer_index = 0;
 


 /* DWM1000 interrupt initialization and handler definition */

 /*!
* Interrupt handler calls the DW1000 ISR API. Call back corresponding to each event defined in ss_init_main
*/
void vInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  dwt_isr(); // DW1000 interrupt service routine 
}

/*!
* @brief Configure an IO pin as a positive edge triggered interrupt source.
*/
void vInterruptInit (void)
{
  ret_code_t err_code;

  if (nrf_drv_gpiote_is_init())
    printf("nrf_drv_gpiote_init already installed\n");
  else
    nrf_drv_gpiote_init();

  // input pin, +ve edge interrupt, no pull-up
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

  // Link this pin interrupt source to its interrupt handler
  err_code = nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, vInterruptHandler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);
}

/*-------------------------------------misurazioni average----------------------*/

// Funzione di utilità per la comparazione di numeri double
int compare_doubles(const void * a, const void * b){
  if(*(double*)a > *(double*)b){
    return 1;
  }
  else if (*(double*)a < *(double*)b){
    return -1;
  } else
    return 0;
}

 static average_measurement_t perform_average_measurement(uint64_t target_id, uint8_t num_samples) {
    average_measurement_t result = {
      .target_id = target_id,
      .valid = false,
      .average_distance = 0.0,
      .samples_count = 0,
      .requested_samples = num_samples
    };

    // Effettuo misurazioni solo se in modalità Iniziatore
    if (device_mode != DEVICE_MODE_INITIATOR){
      printf("Dispositivo non in modalità iniziatore, misurazioni non possibili\r\n");
      return result;
    }

    // trovo index target nell'array
    int target_index = -1;
    for (int i = 0; i<MAX_RESPONDERS; i++){
      if(anchor_ids[i]==target_id){
        target_index = -1;
        break;
      }
    }
    
    uint32_t id_high = (uint32_t)(target_id>>32);
    uint32_t id_low = (uint32_t)(target_id & 0xFFFFFFFFUL);
    printf("Inizio misurazioni verso target con id: %08lX%08lX\r\n", id_high, id_low);
  
    // Resetto misurazioni per quel target fatte in precedenza
    distances[target_index].valid=false;
    distances[target_index].distance=0.0;

    double sum_distances = 0.0;
    uint8_t valid_samples = 0;

    // Buffer per valori mediani
    double measurements[num_samples];

    // Eseguo 10 misurazioni
    for(uint8_t i = 0; i< num_samples; i++){
      
      ss_init_run(target_id);

      // controllo se la misurazione è valida
      if(distances[target_index].valid){
        sum_distances += distances[target_index].distance;
        measurements[valid_samples] = distances[target_index].distance;
        valid_samples++;
      }

      nrf_delay_ms(50); // breve ritardo
      
      // In caso di comando SPI ricevuto, si interrompe
      if(new_spi_command_received){
        break;
      }
    }

    if(valid_samples>0){
      // Effettuo un sort per prendere il valore mediano
      qsort(measurements, valid_samples, sizeof(double), compare_doubles);

      if(valid_samples % 2 == 1) { // Se l'array vanta di un numero dispari di misurazioni
        result.average_distance = measurements[valid_samples/2];
      } else { // altrimenti se sono pari
        result.average_distance = (measurements[valid_samples/2 - 1] + measurements[valid_samples/2])/2.0;
      }

      
      result.valid=true;
      // result.average_distance = sum_distances / valid_samples; // rimosso per il nuovo metodo che usa mediana
      result.samples_count = valid_samples;

      // Calcolo deviazione standard per verificare se sono in condizioni di NLoS
      double sum_sq_diff = 0.0;
      for(int i=0;i<valid_samples; i++){
        double diff = measurements[i] - result.average_distance;
        sum_sq_diff += diff * diff;
      }

      double std_dev = sqrt(sum_sq_diff / valid_samples);

      // se la deviazione standard è troppo alta, c'è la possibilità che siamo in situazione di NLoS
      // TODO: Implementare controllo e gestire cambio automatico a modalità NLoS?

      

      printf("Distanza media calcolata dal dispositivo %08lX%08lX è: %f\r\n", id_high, id_low, result.average_distance);
    }


    return result;
    
 }

 static average_measurement_t measure_all_distances(uint8_t num_samples){
    // Controllo se il dispositivo è in configurazione non accetta
    if(device_mode == DEVICE_MODE_RESPONDER){
      printf("Misurazioni non possibili in modalità risponditore\r\n");
    }

    average_measurement_t temp_buf;

    for(int i=0; i< MAX_RESPONDERS; i++){
      if(anchor_enabled[i] && anchor_ids[i]!=DEVICE_ID) {
        temp_buf = perform_average_measurement(anchor_ids[i], num_samples);

        measurements[i] = temp_buf;

        if(measurements[i].valid){
          num_valid_measurements++;
        }
      }
      if(new_spi_command_received){
        break;
      }
    }
 }



/*-------------------------------------fine------------------------------------*/









/*-------------------------------------Configurazione------------------------------------*/

// Funzione che permette di inizializzare array di id con le impostazioni attuali
static void init_config_arrays(void){
  
  // Per prima cosa effettuo una copia degli array che ho già salvato
  for (int i=0; i<MAX_RESPONDERS; i++){
    config_anchor_ids[i]=anchor_ids[i];
    config_anchor_enabled[i]=anchor_enabled[i];
  }

  // Poi conto il numero di dispositivi attualmente abilitati
  config_num_devices = 0;
  for (int i=0; i < MAX_RESPONDERS; i++){
    if(anchor_enabled[i]){
      config_num_devices++;
    }
  }
}

// Funzione che, dopo le modifiche effettuate in modalità configurazione
// le applica agli array effettivi
static void apply_config_changes(void){
  
  // Applico solo se in modalità configurazione
  if (in_config_mode){
    
    for(int i=0; i<MAX_RESPONDERS; i++){
      anchor_ids[i] = config_anchor_ids[i];
      anchor_enabled[i] = config_anchor_enabled[i];
    }

    printf("Nuova configurazione applicata!\r\n");
  }
}

/*-------------------------------------fine------------------------------------*/











/*---------------------------------------Funzioni UART--------------------------------*/

/**
* @brief Costruisce e invia un pacchetto di risposta via UART.
*
* @param response_id ID del messaggio di risposta.
* @param p_payload    Puntatore al buffer del payload. Se NULL, payload non è inviato
* @param len          Lunghezza del payload.
*/
void send_response_packet(uint8_t response_id, const uint8_t *p_payload, uint16_t len){
    // La dimensione del pacchetto di risposta è 4 + dimensione payload
    uint16_t dim_pacchetto = 4 + len;
    // Definisco un buffer in cui salvo la risposta
    uint8_t response_buffer[dim_pacchetto];

    // Popolo il buffer
    response_buffer[0] = response_id;
    response_buffer[1] = (uint8_t)(len & 0xFF); // len LSB
    response_buffer[2] = (uint8_t)((len>>8) & 0xFF); // len MSB

    // salvo il payload, se presente
    if(p_payload!=NULL && len>0){
        memcpy(&response_buffer[3], p_payload, len);
    }

    // calcolo checksum, per permettere check su rpi
    uint8_t checksum = 0;
    for (int i=0;i<dim_pacchetto-1;i++){
        checksum += response_buffer[i];
    }

    response_buffer[dim_pacchetto-1] = checksum;

    // invio pacchetto via UART
    nrf_drv_uart_tx(&m_uart, response_buffer, dim_pacchetto);

}

static void process_packet(void){
    uint8_t command_id = uart_packet_buffer[1];
    uint16_t len = (uart_packet_buffer[3]<<8) | uart_packet_buffer[2];
    uint8_t* payload = &uart_packet_buffer[4];

    switch (command_id)
    {   
        case SPI_CMD_ECHO_TEST:
            send_response_packet(command_id, payload, len); // rimando indietro ciò che ho ricevuto
            break;

        case SPI_CMD_SET_MODE_INIT:
            if (device_mode != DEVICE_MODE_INITIATOR) {
                device_mode = DEVICE_MODE_INITIATOR;
                bool_mode_changed = true; // Signal UWB logic in main loop
                //printf("Cambiamento modalità a Iniziatore!\r\n");
                // Invio risposta senza payload di ACK
                send_response_packet(command_id, NULL, 0);
            }
            break;

        case SPI_CMD_SET_MODE_RESP:
             if (device_mode != DEVICE_MODE_RESPONDER) {
                device_mode = DEVICE_MODE_RESPONDER;
                bool_mode_changed = true; // Signal UWB logic
                //printf("Cambiamento modalità a Responder!\r\n");
                send_response_packet(command_id, NULL, 0);
            }
            break;

        case SPI_CMD_SET_ID:
            if (len == 8) { // 8 byte dispositivo
                uint64_t device_id = 0;
                // Costruisco a partire dagli 8 byte ricevuti
                for (int i = 0; i < 8; i++){
                  device_id |= ((uint64_t)payload[i] << (i * 8));
                }
                DEVICE_ID = device_id;

                uint32_t id_high = (uint32_t)(device_id>>32);
                uint32_t id_low = (uint32_t)(device_id & 0xFFFFFFFFUL);
                //printf("ID del dispositivo settato a %08lX%08lX\r\n", id_high, id_low);

                } else { /* Handle invalid ID */ }
            break;

         case SPI_CMD_ENABLE_ANCHOR:
             if (len == 8) {
                uint64_t device_id = 0;
                // Costruisco a partire dagli 8 byte ricevuti
                for (int i = 0; i < 8; i++){
                  device_id |= ((uint64_t)payload[i] << (i * 8));
                }

                int idx;
                for(int i=0; i<MAX_RESPONDERS;i++){
                    if (anchor_ids[i]==device_id){
                        anchor_enabled[i]=true;
                        idx=i;
                        break;
                    }
                }

                if(idx!=-1) { // ho trovato entry nella lista ancore
                    success_payload[0] = 0x01;
                    send_response_packet(command_id, success_payload, 1);
                } else { // niente entry, invio payload con 0x00
                    success_payload[0] = 0x00;
                    send_response_packet(command_id, success_payload, 1);
                }
                
             } else { /* Handle invalid ID */ }
             break;

         case SPI_CMD_DISABLE_ANCHOR:
              if (len == 8) {
                uint64_t device_id = 0;
                // Costruisco a partire dagli 8 byte ricevuti
                for (int i = 0; i < 8; i++){
                  device_id |= ((uint64_t)payload[i] << (i * 8));
                }

                int idx = -1;
                for(int i=0; i<MAX_RESPONDERS;i++){
                    if (anchor_ids[i]==device_id){
                        anchor_enabled[i]=false;
                        idx=i;
                        break;
                    }
                }
                
             
                if(idx!=-1) { // ho trovato entry nella lista ancore
                    success_payload[0] = 0x01;
                    send_response_packet(command_id, success_payload, 1);
                } else { // niente entry, invio payload con 0x00
                    success_payload[0] = 0x00;
                    send_response_packet(command_id, success_payload, 1);
                }


                
             } else { /* Handle invalid ID */ }
             break;
          case SPI_CMD_ENTER_CONFIG_MODE:
              if(!in_config_mode){
                in_config_mode = true;
                init_config_arrays();
                //printf("Entrato in modalità configurazione \r\n");
                send_response_packet(command_id, NULL, 0);
              }

            case SPI_CMD_EXIT_CONFIG_MODE:
              if (in_config_mode) {
                apply_config_changes();
                in_config_mode = false;
                //printf("Uscito dalla modalità configurazione\r\n");
                send_response_packet(command_id, NULL, 0);
              }

            case SPI_CMD_SET_NUM_DEVICES:
              if (in_config_mode && len > 1) {
                uint8_t num = payload[0];
                if (num < MAX_RESPONDERS) {
                  config_num_devices = num;
                  //printf("Numero di dispositivi settato a: %d\r\n", config_num_devices);
                  success_payload[0] = 0x01;
                  send_response_packet(command_id, success_payload, 1);
                } 
              } else if (!in_config_mode) {
                //printf("Comando ignorato, dispositivo non in config mode\r\n");
                success_payload[0] = 0x00;
                  send_response_packet(command_id, success_payload, 1);
              }
              break;

            case SPI_CMD_SET_DEVICE_ID_AT:
              if (in_config_mode && len == 9) // 1 byte index array + 8 byte id
              {
                uint8_t idx = payload[0];
                if(idx < MAX_RESPONDERS){
                  uint64_t device_id = 0;

                  // Costruisco id a 64 bit a partire da 8 byte
                  for(int i = 0; i<8; i++){
                    device_id |= ((uint64_t)payload[1+i] << (i * 8));
                  }

                  config_anchor_ids[idx] = device_id;
                  config_anchor_enabled[idx] = true;

                  uint32_t id_high = (uint32_t)(device_id>>32);
                  uint32_t id_low = (uint32_t)(device_id & 0xFFFFFFFFUL);
                  //printf("ID all'indice %d settato a %08lX%08lX\r\n", idx, id_high, id_low);

                  success_payload[0] = 0x01;
                  send_response_packet(command_id, success_payload, 1);
                }
              } else if (!in_config_mode){
                printf("Dispositivo non in config mode\r\n");
                success_payload[0] = 0x00;
                send_response_packet(command_id, success_payload, 1);
              }
              else {
                success_payload[0] = 0x00;
                send_response_packet(command_id, success_payload, 1);
                printf("Formato del comando non valido\r\n");
              }

              break;

            case SPI_CMD_MEASURE_DISTANCE:
                if(len == 9) { // Payload: 8 byte ID + 1 byte num_samples
                    uint64_t target_id = 0;
                    memcpy(&target_id, &payload[0], sizeof(uint64_t)); // Modo più sicuro per copiare
                    uint8_t num_samples = payload[8];
                  
                    // Esegui la misurazione
                    average_measurement_t result = perform_average_measurement(target_id, num_samples);

                    // Prepara e invia la risposta
                    uint8_t response_payload[19]; // valid(1) + req(1) + count(1) + id(8) + dist(8)
                    response_payload[0] = result.valid ? 1 : 0;
                    response_payload[1] = result.requested_samples;
                    response_payload[2] = result.samples_count;
                    memcpy(&response_payload[3], &result.target_id, sizeof(uint64_t));
                    memcpy(&response_payload[11], &result.average_distance, sizeof(double));
                
                    send_response_packet(command_id, response_payload, sizeof(response_payload));
                }
                break;

            case SPI_CMD_MEASURE_ALL_DISTANCES:
                if (len == 1) { // Payload: 1 byte num_samples
                    uint8_t num_samples = payload[0];
                
                    // Avvia le misurazioni verso tutti i dispositivi
                    measure_all_distances(num_samples); // Questa funzione popola `measurements` e `num_valid_measurements`

                    // Costruisci il payload della risposta
                    // Formato: num_valid(1) + [id(8)+samples(1)+dist(8)] * num_valid
                    const int bytes_per_meas = sizeof(uint64_t) + sizeof(uint8_t) + sizeof(double);
                    uint16_t response_payload_len = 1 + (num_valid_measurements * bytes_per_meas);
                    uint8_t response_payload[response_payload_len];

                    response_payload[0] = num_valid_measurements;
                    int current_pos = 1;

                    for (int i = 0; i < num_valid_measurements; i++) {
                        memcpy(&response_payload[current_pos], &measurements[i].target_id, sizeof(uint64_t));
                        current_pos += sizeof(uint64_t);
                        response_payload[current_pos++] = measurements[i].samples_count;
                        memcpy(&response_payload[current_pos], &measurements[i].average_distance, sizeof(double));
                        current_pos += sizeof(double);
                    }
                
                    send_response_packet(command_id, response_payload, response_payload_len);
                }
                break;

            case SPI_CMD_GET_INFO:
                {
                    uint8_t info_payload[11]; // mode(1) + id(8) + enabled_mask(2)
                    uint16_t enabled_mask = 0;

                    info_payload[0] = (uint8_t)device_mode;
                    for(int i = 0; i < 8; i++) {
                        info_payload[1 + i] = (DEVICE_ID >> (i * 8)) & 0xFF;
                    }

                    for(int i = 0; i < MAX_RESPONDERS; ++i) {
                        if (anchor_enabled[i]) {
                            enabled_mask |= (1 << i);
                        }
                    }
                    info_payload[9] = (uint8_t)(enabled_mask & 0xFF);
                    info_payload[10] = (uint8_t)((enabled_mask >> 8) & 0xFF);
                
                    send_response_packet(command_id, info_payload, sizeof(info_payload));
                }
                break;

            case SPI_CMD_GET_HELP:
                 {
                    char help_str[] = "Comandi supportati: GET_INFO, SET_MODE, MEASURE, ecc.";
                    send_response_packet(command_id, (uint8_t*)help_str, strlen(help_str));
                 }
                 break;

            case SPI_SET_NLOS_MODE:
                nlos_mode = true;
                dwt_configure(&nlos_config);
                send_response_packet(command_id, NULL, 0); // ACK
                break;

            case SPI_SET_LOS_MODE:
                nlos_mode = false;
                dwt_configure(&config);
                send_response_packet(command_id, NULL, 0); // ACK
                break;

            case SPI_SET_ANTENNA_RX_DELAY:
                if (len == 2) { // Payload: 2 byte delay
                    uint16_t delay = payload[0] | (payload[1] << 8);
                    RECEIVER_DELAY = delay;
                    dwt_setrxantennadelay(RECEIVER_DELAY);
                    send_response_packet(command_id, NULL, 0); // ACK
                }
                break;

            case SPI_SET_ANTENNA_TX_DELAY:
                if (len == 2) {
                    uint16_t delay = payload[0] | (payload[1] << 8);
                    TRANSMITTER_DELAY = delay;
                    dwt_settxantennadelay(TRANSMITTER_DELAY);
                    send_response_packet(command_id, NULL, 0); // ACK
                }
                break;

            default:
                // Comando non riconosciuto, potresti inviare una risposta NACK (Not Acknowledged)
                send_response_packet(0xFF, NULL, 0); // 0xFF come ID di errore generico
                break;
  
    }


}


static void uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context){

    if(p_event->type == NRF_DRV_UART_EVT_RX_DONE){
    
        // Prendo il byte ricevuto
        uint8_t data = p_event->data.rxtx.p_data[0];

        switch(uart_state){
        
            case STATE_WAIT_FOR_START:
                if (data==START_BYTE){
                    
                    uart_packet_index = 0;
                    uart_packet_buffer[uart_packet_index++] = data;
                    uart_state = STATE_RECEIVE_CMD; // Aspetto di ricevere comando

                }
                break;
            case STATE_RECEIVE_CMD:
                uart_packet_buffer[uart_packet_index++] = data;
                uart_state = STATE_RECEIVE_LEN_LSB;
                break;
            
            case STATE_RECEIVE_LEN_LSB:
                uart_payload_len = data;
                uart_packet_buffer[uart_packet_index++] = data;
                uart_state = STATE_RECEIVE_LEN_MSB;
                break;

            case STATE_RECEIVE_LEN_MSB:

                // Combino i byte ricevuti per formare lunghezza a 16 bit
                uart_payload_len |= ((uint16_t)data<<8);
                uart_packet_buffer[uart_packet_index++] = data;

                // Verifico che la lunghezza non superi quella massima
                if(uart_payload_len > MAX_PAYLOAD_SIZE){
                    uart_state = STATE_WAIT_FOR_START; // resetto, scartando il pacchetto
                } else if (uart_payload_len == 0){
                    uart_state = STATE_RECEIVE_CHECKSUM; // nessun payload necessario per questo comando
                } else {
                    uart_state = STATE_RECEIVE_PAYLOAD;
                }

                break;

            case STATE_RECEIVE_PAYLOAD:

                // continuo a salvare dati, fin tanto quando 
                // la differenza tra l'indice (sottratto ai 4 byte iniziali) non raggiunge 
                // la lunghezza del payload
                uart_packet_buffer[uart_packet_index++] = data;
                if((uart_packet_index-4)>=uart_payload_len){
                    uart_state = STATE_RECEIVE_CHECKSUM;
                }

                break;

            
            case STATE_RECEIVE_CHECKSUM:
                uart_packet_buffer[uart_packet_index++] = data;
                
                uint8_t checksum = 0;
                for (int i=0;i<uart_packet_index-1;i++){
                    checksum += uart_packet_buffer[i];
                }

                if(checksum==data){ // pacchetto ricevuto correttamente
                    uart_new_command = true;
                }

                // resetto lo stato
                uart_state = STATE_WAIT_FOR_START;

                break;
    
        }

        nrf_drv_uart_rx(&m_uart, &m_uart_rx_byte, 1);
    
    }

}

/**
*@brief Inizializza UART.
*/
static void uart_init(void)
{
    uint32_t err_code;
    const nrf_drv_uart_config_t config = {
        .pseltxd = UART_TX_PIN, .pselrxd = UART_RX_PIN,
        .pselcts = NRF_UART_PSEL_DISCONNECTED, .pselrts = NRF_UART_PSEL_DISCONNECTED,
        .p_context = NULL, .hwfc = NRF_UART_HWFC_DISABLED,
        .parity = NRF_UART_PARITY_EXCLUDED, .baudrate = NRF_UART_BAUDRATE_115200,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    err_code = nrf_drv_uart_init(&m_uart, &config, uart_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_uart_rx_enable(&m_uart);
    nrf_drv_uart_rx(&m_uart, &m_uart_rx_byte, 1);
}






/*-------------------------------------fine------------------------------------*/







#if 1
/*---------------------------------------Funzioni SPI--------------------------------*/

/* 
Funzione usata per preparare la risposta a seguito della ricezione di un certo tipo di comando
ricevuto dal master.
*/
static void prepare_spi_response(uint8_t command_processed){
  switch (command_processed)
    {   case SPI_CMD_ECHO_TEST:
            break;
        case SPI_CMD_GET_DISTANCES:
        case SPI_CMD_SET_MODE_INIT:
        case SPI_CMD_SET_MODE_RESP:
        case SPI_CMD_SET_ID:
        case SPI_CMD_ENABLE_ANCHOR:
        case SPI_CMD_DISABLE_ANCHOR:
             // Prepare distance data in the TX buffer
             prepare_distance_data_for_spi();
             break;

        case SPI_CMD_GET_INFO:
             prepare_info_data_for_spi();
             break;

        case SPI_CMD_GET_HELP:
             // Prepare a simple help message or code
             snprintf((char*)m_spi_tx_buf, SPI_TX_DATA_BUFFER_SIZE, "CMDS: 01=Dist, 10=Init, 11=Resp, 20=SetID, 30/31=En/Dis, FE=Info");
             break;
        
        case SPI_CMD_ENTER_CONFIG_MODE:
        case SPI_CMD_EXIT_CONFIG_MODE:
        case SPI_CMD_SET_NUM_DEVICES:
        case SPI_CMD_SET_DEVICE_ID_AT:
          prepare_config_info_for_spi();
          break;
        
        case SPI_CMD_MEASURE_DISTANCE:
          prepare_avg_measurement_for_spi();
          break;
        
        case SPI_CMD_MEASURE_ALL_DISTANCES:
          prepare_all_measurements_for_spi();
          break;

        case SPI_SET_LOS_MODE:
        case SPI_SET_NLOS_MODE:
        case SPI_SET_ANTENNA_RX_DELAY:
        case SPI_SET_ANTENNA_TX_DELAY:

        default:
             // Unknown command or error during processing
             m_spi_tx_buf[0] = 0xFF; // Example error code
             m_spi_tx_buf[1] = command_processed; // Echo faulty command
             memset(&m_spi_tx_buf[2], 0, SPI_TX_DATA_BUFFER_SIZE - 2);
             break;
    }
}

static void prepare_all_measurements_for_spi(void) {
  // Prima posizione: numero di misurazioni valide
    m_spi_tx_buf[0] = num_valid_measurements;
    
    uint8_t pos = 1;
    
    // Per ogni misurazione valida
    for (int i = 0; i < num_valid_measurements; i++) {
        // Verifica se abbiamo spazio sufficiente nel buffer
        if (pos + 18 > SPI_TX_DATA_BUFFER_SIZE) {
            printf("Warning: SPI buffer too small for all measurements\r\n");
            break;  // Evita overflow del buffer
        }
        
        average_measurement_t *meas = &measurements[i];
        
        // Bytes 0-7: ID del target (little-endian)
        for (int j = 0; j < 8; j++) {
            m_spi_tx_buf[pos++] = (uint8_t)((meas->target_id >> (j * 8)) & 0xFF);
        }
        
        // Byte 8: Numero di campioni validi
        m_spi_tx_buf[pos++] = meas->samples_count;
        
        // Bytes 9-16: Distanza media
        memcpy(&m_spi_tx_buf[pos], &meas->average_distance, sizeof(double));
        pos += sizeof(double);
    }
    
    // Pulisci il resto del buffer
    if (pos < SPI_TX_DATA_BUFFER_SIZE) {
        memset(&m_spi_tx_buf[pos], 0, SPI_TX_DATA_BUFFER_SIZE - pos);
    }



}

// Funzione per preparare dati in risposta SPI
static void prepare_avg_measurement_for_spi(void){
  uint8_t tx_len = 0;

  // Byte 0: flag di validità
  m_spi_tx_buf[tx_len++] = current_avg_measurement.valid ? 1: 0;

  // Byte 1: Numero di campioni richiesti
  m_spi_tx_buf[tx_len++] = current_avg_measurement.requested_samples;

  // byte 2: numero di campioni utilizzati
  m_spi_tx_buf[tx_len++] = current_avg_measurement.samples_count;

  // byte 3-10: id del target
  for (int i = 0; i<8; i++){
    m_spi_tx_buf[tx_len++] = (uint8_t)((current_avg_measurement.target_id >> (i*8)) & 0xFF);
  }

  // byte 11-18: Distanza media (se valida)
  if(current_avg_measurement.valid){
    memcpy(&m_spi_tx_buf[tx_len], &current_avg_measurement.average_distance, sizeof(double));
  } else {
    memset(&m_spi_tx_buf[tx_len], 0, sizeof(double));
  }

  tx_len += sizeof(double);
  
  // Pongo tutti zeri nel resto del vettore non usato
  if (tx_len < SPI_TX_DATA_BUFFER_SIZE) {
     memset(&m_spi_tx_buf[tx_len], 0, SPI_TX_DATA_BUFFER_SIZE - tx_len);
  }

}

/*
Funzione adibita alla preparazione dei dati della distanza ottenuti e salvati nel vettore.
*/
static void prepare_distance_data_for_spi(void)
{
    uint8_t * p_tx_buf = m_spi_tx_buf; 
    uint8_t tx_len = 0;
    uint8_t valid_count = 0;

    // 1. In primo luogo effettuo un conteggio delle distanze valaide
    for (int i = 0; i < MAX_RESPONDERS; i++) {
        // DA AGGIUNGERE: mutex
        if (anchor_enabled[i] && distances[i].valid && i!=DEVICE_ID) {
            valid_count++;
        }
    }

    if ((1 + valid_count * (1 + sizeof(double))) > SPI_TX_DATA_BUFFER_SIZE) {
        // Se ci sono troppi dati
        p_tx_buf[0] = 0; // Indicate error or zero count
        tx_len = 1;
    } else {
        // altrimenti
        p_tx_buf[0] = valid_count; // imposto il primo byte come numero distanze
        tx_len = 1;
        for (int i = 0; i < MAX_RESPONDERS; i++) {
            if (anchor_enabled[i] && distances[i].valid && i!=DEVICE_ID) {
                p_tx_buf[tx_len++] = distances[i].id; // Salvo id della distanza
                memcpy(&p_tx_buf[tx_len], &distances[i].distance, sizeof(double)); // Seguito dalla distanza
                tx_len += sizeof(double); // incremento distanza
            }
        }
    }
    // Pongo tutti zeri nel resto del vettore non usato
    if (tx_len < SPI_TX_DATA_BUFFER_SIZE) {
       memset(&p_tx_buf[tx_len], 0, SPI_TX_DATA_BUFFER_SIZE - tx_len);
    }
}

static void prepare_info_data_for_spi(void)
{
    uint8_t tx_len = 0;
    m_spi_tx_buf[tx_len++] = (uint8_t)device_mode; //salvo la modalità del dispositivo...
    
    // Device id
    for (int i = 0; i < 8; i++) {
      m_spi_tx_buf[tx_len++] = (uint8_t)((DEVICE_ID >> (i*8)));
    }


    uint16_t enabled_mask = 0;
    for(int i=0; i<MAX_RESPONDERS; ++i) {
        if (anchor_enabled[i]) {
            enabled_mask |= (1 << i);
        }
    }
    m_spi_tx_buf[tx_len++] = (uint8_t)(enabled_mask & 0xFF);        // LSB
    m_spi_tx_buf[tx_len++] = (uint8_t)((enabled_mask >> 8) & 0xFF); // MSB

    // Pulisci buffer rimanente
    if (tx_len < SPI_TX_DATA_BUFFER_SIZE) {
        memset(&m_spi_tx_buf[tx_len], 0, SPI_TX_DATA_BUFFER_SIZE - tx_len);
    }
}

// Funzione per preparare info di configurazione per il buffer di trasmissione SPI
static void prepare_config_info_for_spi(void){
  uint8_t tx_len = 0;

  // Byte 0: Status di configurazione
  m_spi_tx_buf[tx_len++] = in_config_mode ? 1 : 0; // 1 indica che sta in configurazione
  
  // Byte 1: numero di dispositivi configurati
  m_spi_tx_buf[tx_len++] = config_num_devices;

  // Byte 2-3: Maschera dispositivi abilitati
  // NON IMPLEMENTATA

  // Pulisco buffer rimanente
  if (tx_len < SPI_TX_DATA_BUFFER_SIZE) {
    memset(&m_spi_tx_buf[tx_len], 0, SPI_TX_DATA_BUFFER_SIZE - tx_len);
  }

}


/*
Funzione che processa i comandi SPI ricevuti dal master.
*/
static void process_spi_command(uint8_t *cmd_data, uint8_t cmd_len)
{
    if (cmd_len == 0) return;

    uint8_t command = cmd_data[0];

    // Gestisco comandi informativi, quali SPI_CMD_GET_STATUS o SPI_CMD_GET_INFO
    if (command == SPI_CMD_GET_STATUS || command == SPI_CMD_GET_INFO) {
        return; // gestiti in prepare_spi_response()
    }

    // Gestisco poi gli altri comandi in base allo stato del dispositivo
    switch(current_state){
        case STATE_IDLE: // eseguo comandi 
            
            switch (command)
            {   
                case SPI_CMD_ECHO_TEST:
                    // Prepara i dati ricevuti per essere rispediti al mittente.
                    // cmd_data[0] è il comando, il payload inizia da cmd_data[1].
                    if (cmd_len > 1) {
                        uint8_t payload_len = cmd_len - 1;
                        // Copia il payload nel buffer di trasmissione per il prossimo ciclo
                        memcpy(m_spi_tx_buf, &cmd_data[1], payload_len);
                        // Pulisce il resto del buffer di trasmissione
                        memset(&m_spi_tx_buf[payload_len], 0, SPI_TX_DATA_BUFFER_SIZE - payload_len);
                    } else {
                        // Se non c'è payload, pulisce l'intero buffer
                        memset(m_spi_tx_buf, 0, SPI_TX_DATA_BUFFER_SIZE);
                    }
                    printf("Echo command received. Preparing %d bytes for response.\r\n", cmd_len > 1 ? cmd_len - 1 : 0);
                    break;

                case SPI_CMD_SET_MODE_INIT:
                    if (device_mode != DEVICE_MODE_INITIATOR) {
                        device_mode = DEVICE_MODE_INITIATOR;
                        bool_mode_changed = true; // Signal UWB logic in main loop
                        printf("Cambiamento modalità a Iniziatore!\r\n");
                    }
                    break;

                case SPI_CMD_SET_MODE_RESP:
                     if (device_mode != DEVICE_MODE_RESPONDER) {
                        device_mode = DEVICE_MODE_RESPONDER;
                        bool_mode_changed = true; // Signal UWB logic
                        printf("Cambiamento modalità a Responder!\r\n");
                    }
                    break;

                case SPI_CMD_SET_ID:
                    if (cmd_len >= 9) { // comando + 8 byte dispositivo
                        uint64_t device_id = 0;
                        // Costruisco a partire dagli 8 byte ricevuti
                        for (int i = 0; i < 8; i++){
                          device_id |= ((uint64_t)cmd_data[1+i] << (i * 8));
                        }
                        DEVICE_ID = device_id;

                        uint32_t id_high = (uint32_t)(device_id>>32);
                        uint32_t id_low = (uint32_t)(device_id & 0xFFFFFFFFUL);
                        printf("ID del dispositivo settato a %08lX%08lX\r\n", id_high, id_low);

                        } else { /* Handle invalid ID */ }
                    break;

                 case SPI_CMD_ENABLE_ANCHOR:
                     if (cmd_len > 1) {
                        int id = cmd_data[1];
                        if (id >= 0 && id < MAX_RESPONDERS) {
                             anchor_enabled[id] = true;
                         } else { /* Handle invalid ID */ }
                     }
                     break;

                 case SPI_CMD_DISABLE_ANCHOR:
                      if (cmd_len > 1) {
                        int id = cmd_data[1];
                        if (id >= 0 && id < MAX_RESPONDERS) {
                            anchor_enabled[id] = false;
                        } else { /* Handle invalid ID */ }
                     }
                     break;
                  case SPI_CMD_ENTER_CONFIG_MODE:
                      if(!in_config_mode){
                        in_config_mode = true;
                        init_config_arrays();
                        printf("Entrato in modalità configurazione \r\n");
                      }
        
                    case SPI_CMD_EXIT_CONFIG_MODE:
                      if (in_config_mode) {
                        apply_config_changes();
                        in_config_mode = false;
                        printf("Uscito dalla modalità configurazione\r\n");
                      }
        
                    case SPI_CMD_SET_NUM_DEVICES:
                      if (in_config_mode && cmd_len > 1) {
                        uint8_t num = cmd_data[1];
                        if (num < MAX_RESPONDERS) {
                          config_num_devices = num;
                          printf("Numero di dispositivi settato a: %d\r\n", config_num_devices);
                        }
                      } else if (!in_config_mode) {
                        printf("Comando ignorato, dispositivo non in config mode\r\n");
                      }
                      break;

                    case SPI_CMD_SET_DEVICE_ID_AT:
                      if (in_config_mode && cmd_len >= 10) // comando + 1 byte index array + 8 byte id
                      {
                        uint8_t idx = cmd_data[1];
                        if(idx < MAX_RESPONDERS){
                          uint64_t device_id = 0;

                          // Costruisco id a 64 bit a partire da 8 byte
                          for(int i = 0; i<8; i++){
                            device_id |= ((uint64_t)cmd_data[2+i] << (i * 8));
                          }

                          config_anchor_ids[idx] = device_id;
                          config_anchor_enabled[idx] = true;

                          uint32_t id_high = (uint32_t)(device_id>>32);
                          uint32_t id_low = (uint32_t)(device_id & 0xFFFFFFFFUL);
                          printf("ID all'indice %d settato a %08lX%08lX\r\n", idx, id_high, id_low);
                        }
                      } else if (!in_config_mode){
                        printf("Dispositivo non in config mode\r\n");
                      }
                      else {
                        printf("Formato del comando non valido\r\n");
                      }
                      break;
        
                    case SPI_CMD_MEASURE_DISTANCE:
                      if(cmd_len>=10) { // Comando + 8 byte id + numero misurazioni

                        // Passo a modalità Initiator, se non lo è già
                        if(device_mode!=DEVICE_MODE_INITIATOR){
                            device_mode = DEVICE_MODE_INITIATOR;
                            // Applica le configurazioni hardware per l'initiator
                            dwt_setrxtimeout(65000);
                            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                        }
            
                        // Estraggo id target
                        uint64_t target_id = 0;
                        for(int i=0; i<8; i++){
                          target_id |= ((uint64_t)cmd_data[1+i] << (i*8));
                        }

                        uint8_t num_samples = cmd_data[9];
                        if(num_samples>50) num_samples=50; // massimo 50
             
                        uint32_t id_high = (uint32_t)(target_id>>32);
                        uint32_t id_low = (uint32_t)(target_id & 0xFFFFFFFFUL);
                        printf("inizio misurazioni verso dispositivo %08lX%08lX\r\n", id_high, id_low);

                        performing_avg_measurement = true;
            
                        // effettuo misurazioni
                        current_avg_measurement = perform_average_measurement(target_id, num_samples);

                        performing_avg_measurement = false;

                        // Ritorno a responder
                        device_mode = DEVICE_MODE_RESPONDER;
                        // Applica le configurazioni hardware per il responder
                        dwt_setrxtimeout(0);
                        dwt_rxreset(); // Resetta la ricezione
                      }
                      current_state = STATE_PROCESSING;
                      break;

                    case SPI_CMD_MEASURE_ALL_DISTANCES:
                      // Passo a modalità Initiator, se non lo è già
                        if(device_mode!=DEVICE_MODE_INITIATOR){
                            device_mode = DEVICE_MODE_INITIATOR;
                            // Applica le configurazioni hardware per l'initiator
                            dwt_setrxtimeout(65000);
                            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                        }

                        measure_all_distances(10);

                        // Ritorno a responder
                        device_mode = DEVICE_MODE_RESPONDER;
                        // Applica le configurazioni hardware per il responder
                        dwt_setrxtimeout(0);
                        dwt_rxreset(); // Resetta la ricezione
                        current_state = STATE_PROCESSING;
                        break;

                    case SPI_CMD_GET_INFO:
                         // Action is to prepare response
                         break;

                    case SPI_CMD_GET_HELP:
                         // Action is to prepare response
                         break;
        
                    case SPI_SET_NLOS_MODE:
                        // imposto la modalità a NLoS
                        nlos_mode = true;
                        dwt_configure(&nlos_config);
                        break;
        
                    case SPI_SET_LOS_MODE:
                        // imposto modalità a LoS
                        nlos_mode=false;
                        dwt_configure(&config);
                        break;
        
                    case SPI_SET_ANTENNA_RX_DELAY:
                        if(cmd_len>=3){ // comando + 2 byte delay
                            uint16_t delay = cmd_data[1] | (cmd_data[2]<<8);
                            RECEIVER_DELAY = delay;
                            dwt_setrxantennadelay(RECEIVER_DELAY);
                            printf("Antenna receiver delay set to: %u\r\n", RECEIVER_DELAY);
                        } else {
                            printf("Command for rx delay setting received, but content is not valid!\r\n");
                        }
                        break;

                    case SPI_SET_ANTENNA_TX_DELAY:
                        if(cmd_len>=3){
                            uint16_t delay = cmd_data[1] | (cmd_data[2]<<8);
                            TRANSMITTER_DELAY = delay;
                            dwt_settxantennadelay(TRANSMITTER_DELAY);
                            printf("Antenna transmitter delay set to: %u\r\n", TRANSMITTER_DELAY);
                        } else {
                            printf("Command for tx delay setting received, but content is not valid!\r\n");
                        }

                        break;

                    default:
                        // Unknown command - prepare_spi_response will handle this
                         break;
          
            }
            break;
        case STATE_PROCESSING: // il dispositivo sta processando un comando, e gestisce solo i comandi informativi
            printf("Comando 0x%02X ignorato: dispositivo occupato. \r\n");
            break;
        
        case STATE_DATA_READY: // con i dati pronti, si accettano solo comandi per leggerli
            
            switch (command) {
                case SPI_CMD_READ_RESULTS:
                    // dopo la lettura, torna in idle
                    current_state = STATE_IDLE;
                    break;
                default:
                    printf("Comando 0x%02X ignorato: in attesa di lettura risultati vecchia operazione. \r\n");
                    break;
            }

            break;
        case STATE_ERROR:
            // da gestire
            break;
    
    }


}

/*
Funzione che gestisce eventi SPI con interrupt.
*/
static void spis_event_handler(nrf_drv_spis_event_t event)
{
    printf("SPI HANDLER TRIGGERED! RX bytes: %d\r\n", event.rx_amount);
    LEDS_INVERT(BSP_LED_0_MASK);
    LEDS_INVERT(BSP_LED_1_MASK);
    LEDS_INVERT(BSP_LED_2_MASK);
    LEDS_INVERT(BSP_LED_0_MASK);
    LEDS_INVERT(BSP_LED_1_MASK);
    LEDS_INVERT(BSP_LED_2_MASK);

    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {   
        // Check if there are incoming commands
        if (!new_spi_command_received)
        {   
            
        
            uint8_t bytes_received = event.rx_amount;
            if (bytes_received > SPI_CMD_BUFFER_SIZE) {
                bytes_received = SPI_CMD_BUFFER_SIZE;
            }
            if (bytes_received > 0) {
                memcpy(spi_cmd_buffer, m_spi_rx_buf, bytes_received);
                spi_cmd_length = bytes_received;
                new_spi_command_received = true;
            } else {
                spi_cmd_length = 0;
            }
        
        }

        // Set up buffers for the next transaction
        APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&m_spis, m_spi_tx_buf, SPI_TX_DATA_BUFFER_SIZE, m_spi_rx_buf, SPI_CMD_BUFFER_SIZE));
        #if 0
        printf("SPI XFER DONE, RX bytes: %d\r\n", event.rx_amount);
        // 1. Copia il comando ricevuto e segnala
        if (!new_spi_command_received) {
            uint8_t bytes_received = event.rx_amount;
            memcpy(spi_cmd_buffer, m_spi_rx_buf, bytes_received);
            spi_cmd_length = bytes_received;
            new_spi_command_received = true;
        }

        // 2. Prepara la risposta PRIMA di riconfigurare i buffer
        prepare_spi_response(current_spi_command); // Aggiungi questa linea

        // 3. Riconfigura i buffer con i nuovi dati
        APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&m_spis, m_spi_tx_buf, SPI_TX_DATA_BUFFER_SIZE, m_spi_rx_buf, SPI_CMD_BUFFER_SIZE));
        //#endif
        #endif
    }
}

/*
Funzione che inizializza dispositivo come slave SPI
*/

static void spi_slave_init(void)
{   
    memset(m_spi_tx_buf, 0xAA, SPI_TX_DATA_BUFFER_SIZE); // Inizializza con pattern alternato
    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin   = 3; //MY_SPIS_CSN_PIN;
    spis_config.csn_pullup = NRF_GPIO_PIN_PULLUP;
    spis_config.miso_pin  = 7; //MY_SPIS_MISO_PIN;
    spis_config.mosi_pin  = 6; // MY_SPIS_MOSI_PIN;
    spis_config.sck_pin   = 4; // MY_SPIS_SCK_PIN;
    spis_config.mode      = NRF_DRV_SPIS_MODE_1;
    spis_config.bit_order = NRF_DRV_SPIS_BIT_ORDER_MSB_FIRST;
    spis_config.irq_priority = APP_IRQ_PRIORITY_LOW;

    APP_ERROR_CHECK(nrf_drv_spis_init(&m_spis, &spis_config, spis_event_handler));

    // Prepare the buffer for transmission
    //prepare_distance_data_for_spi();

    // Initialize buffers for transmissions
    APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&m_spis, m_spi_tx_buf, SPI_TX_DATA_BUFFER_SIZE, m_spi_rx_buf, SPI_CMD_BUFFER_SIZE));

    new_spi_command_received = false;
    spi_cmd_length = 0;
}

/*----------------------------------------fine----------------------------------------*/


 
 #endif
 
 int main(void)
 {  

    struct timeval t1,t2;
    double elapsedTime;


   /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
    LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
    //LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );
 
   /*Initialization UART*/
   // boUART_Init(); // sostituita con altra inizializzazione

   /* Initialize clock */
   APP_ERROR_CHECK(nrf_drv_clock_init());
   nrf_drv_clock_lfclk_request(NULL);

   // Inizializzazione UART
   uart_init();

   /* Initialize SPI Slave */
   //spi_slave_init();
   //printf("SPI Inizializzato!\r\n");
   //nrf_delay_ms(1000);
 
   //set_uart_rx_handler(uart_event_handler);
 
   printf("\r\n\r\n--- Unified system per multi-responder ---\r\n");
   printf("Send 'INIT' for Initiator mode, 'RESP' for Responder mode\r\n");
   printf("For help with all commands, send HELP via UART\r\n");
 
   //-------------dw1000  ini------------------------------------
 
   /* Setup DW1000 IRQ pin */
   //nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); // irq

   /* Setupd NRF52832 interrupt on GPIO 25 : connected to DW1000 iRQ*/
   vInterruptInit();
 
   /* Reset DW1000 */
   reset_DW1000();
 
   /* Set SPI clock to 2MHz */
   port_set_dw1000_slowrate();
 
   /* Init the DW1000 */
   if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
   {
     // Init of DW1000 Failed
     while (1)
     {
     };
   }
 
   // Set SPI to 8MHz clock
   port_set_dw1000_fastrate();
 
   /* Configure DW1000. */
   uint8_t ntm1_value;
   uint16_t ntm2_value;

   // If NLOS_MODE
   if(nlos_mode){
      dwt_configure(&nlos_config); // Setup NLOS configuration for the device

      // Configure leading edge algorithm for first path identification
      dwt_write8bitoffsetreg(0x2E, LDE_CFG1_OFFSET, 0x09);
      dwt_write16bitoffsetreg(0x2E, LDE_CFG2_OFFSET, 0x0004);

   } else {
      dwt_configure(&config);
      
      // We use standard values for LOS conditions
      dwt_write8bitoffsetreg(0x2E, LDE_CFG1_OFFSET, 0x0D);
      dwt_write16bitoffsetreg(0x2E, LDE_CFG2_OFFSET, 0x0007);
   }

   // We then modify the LDE_REPC register to make the configuration more robust
   if(nlos_mode){
      // Forces chip to use a lower LDE value, good for 16MHz preambles
      dwt_write16bitoffsetreg(0x2E, LDE_REPC_OFFSET, 0x4000); // for 16MHz PRF
   } else {
      dwt_write16bitoffsetreg(0x2E, LDE_REPC_OFFSET, 0x28F0); // Standard value
   }

   /* Initialization of the DW1000 interrupt */
   dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

   /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
   dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

   /* Apply default antenna delay value. See NOTE 2 below. */
   dwt_setrxantennadelay(RECEIVER_DELAY); //RX_ANT_DLY);
   dwt_settxantennadelay(TRANSMITTER_DELAY); // TX_ANT_DLY);
 
   /* Set preamble timeout for expected frames. See NOTE 3 below. */
   // dwt_setpreambledetecttimeout(0); // PRE_TIMEOUT

   if(device_mode == DEVICE_MODE_INITIATOR){
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(65000); // Maximum value timeout with DW1000 is 65ms 
   }
   else{
    //dwt_setrxaftertxdelay(POLL_RX_TO_RESP_TX_DLY_UUS);
    dwt_setrxtimeout(0);
   }
   
 
   //-------------dw1000  ini------end---------------------------
   // IF WE GET HERE THEN THE LEDS WILL BLINK
 
 
   printf("Starting in %s mode \r\n", device_mode == DEVICE_MODE_INITIATOR ? "Initiator" : "Responder");
   if(device_mode == DEVICE_MODE_RESPONDER){
    uint32_t id_alta = (uint32_t)(DEVICE_ID >> 32);
    uint32_t id_bassa = (uint32_t)(DEVICE_ID & 0xFFFFFFFFUL); // o solo (uint32_t)DEVICE_ID
    // Stampa le due parti in esadecimale, 8 cifre ciascuna con padding
    printf("Device id: %08lX%08lX \r\n", id_alta, id_bassa);
   }

   //uint8_t current_spi_command = 0; // Store command being processed
  
  /*
  // Test misurazione tempo
  if(device_mode==DEVICE_MODE_INITIATOR){
    gettimeofday(&t1, NULL);
    ss_init_run(1);
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    printf("%f ms.\n", elapsedTime);
  }
  */
  
  // Ciclo di esecuzione del DWM
  while(true){

      // 1. Controllo arrivo nuovo pacchetto UART
      if(uart_new_command){
          
          // Ho ricevuto un pacchetto nuovo, resetto il flag
          uart_new_command=false;
          // e chiamo la funzione di gestione
          process_packet();
      }

      // 2. Eseguo loop di polling con dispositivo in modalità rispondente
      if(device_mode==DEVICE_MODE_RESPONDER && !in_config_mode){
          ss_resp_run(DEVICE_ID);
      }

  
  }



  while(true){ // DA RIMUOVERE
      // === INIZIO SEZIONE DI TEST ===
    // Commenta temporaneamente tutta la logica UWB e la macchina a stati
    // per isolare e testare solo la comunicazione SPI.

    if (new_spi_command_received) {
        printf("SPI CMD Received: 0x%02X (len %d)\r\n", spi_cmd_buffer[0], spi_cmd_length);
        uint8_t local_cmd_buf[SPI_CMD_BUFFER_SIZE];
        uint8_t local_cmd_len = 0;

        CRITICAL_REGION_ENTER();
        if (new_spi_command_received)
        {
            local_cmd_len = spi_cmd_length;
            memcpy(local_cmd_buf, spi_cmd_buffer, local_cmd_len);
            new_spi_command_received = false;
            spi_cmd_length = 0;
        }
        CRITICAL_REGION_EXIT();

        if (local_cmd_len > 0) {
             printf("Processing SPI CMD: 0x%02X (len %d)\r\n", local_cmd_buf[0], local_cmd_len);
             process_spi_command(local_cmd_buf, local_cmd_len);
             // Non c'è bisogno di chiamare prepare_spi_response qui
             // perché la logica di echo la gestisce già.
             printf("SPI Response prepared for next transaction.\r\n");
        }
    } else {
        //printf("Nessun comando ricevuto\r\n");
    }

    // Aggiungi un piccolo ritardo per non saturare la CPU
    //nrf_delay_ms(500);
  }
  

  while (true)
  {  
  #if 1
    /*
    while(i<MAX_RESPONDERS){
      if(anchor_enabled[i] && i!= DEVICE_ID) ss_init_run(i);
      i++;
      vTaskDelay(10);
    }
    i=0;
    //ss_init_run(1);
    /* Delay a task for a given number of ticks */
    //vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */

    // Controllo la ricezione di comandi SPI

    //printf("Ricomincio il loop!\r\n");
    nrf_delay_ms(200);
    //printf("Inizio raccolta dati SPI\r\n");
    if (new_spi_command_received) {
      
        LEDS_INVERT(BSP_LED_1_MASK);
        LEDS_INVERT(BSP_LED_2_MASK);
        LEDS_INVERT(BSP_LED_1_MASK);
        LEDS_INVERT(BSP_LED_2_MASK);
        printf("SPI CMD Received: 0x%02X (len %d)\r\n", spi_cmd_buffer[0], spi_cmd_length);
        uint8_t local_cmd_buf[SPI_CMD_BUFFER_SIZE];
        uint8_t local_cmd_len = 0;

        // Abilita sezione critica per evitare race conditions
        CRITICAL_REGION_ENTER(); // Disabilita interrupts
        if (new_spi_command_received) // controllo di nuovo se ci sono comandi
        {
            local_cmd_len = spi_cmd_length; // salvo lunghezza
            memcpy(local_cmd_buf, spi_cmd_buffer, local_cmd_len); // lo salvo in un buffer locale
            new_spi_command_received = false; // pulisco la flag
            spi_cmd_length = 0;
        }
         CRITICAL_REGION_EXIT(); // riabilita interrupts
         //  chiudi sezione critica

        if (local_cmd_len > 0) {
             printf("Processing SPI CMD: 0x%02X (len %d)\r\n", local_cmd_buf[0], local_cmd_len);
             //NRF_LOG_FLUSH();
             current_spi_command = local_cmd_buf[0]; // Store command code
             process_spi_command(local_cmd_buf, local_cmd_len);

             // Prepare the response buffer for the *next* transaction
             prepare_spi_response(current_spi_command);
             printf("SPI Response prepared.\r\n");
             //NRF_LOG_FLUSH();
        }
    }

    // Controllo lo stato attuale della FSM in cui si trova il dispositivo, e sulla base di quello eseguo azioni
    switch (current_state) {
        case STATE_IDLE:
            // Se sono in modalità responder, ascolto in UWB
            if (device_mode == DEVICE_MODE_RESPONDER) {
                ss_resp_run(DEVICE_ID);
            } else if (device_mode == DEVICE_MODE_INITIATOR){
                //ss_init_run(1);
            }
            // Altrimenti attendo comandi SPI
            break;
        case STATE_PROCESSING:
            break;
        case STATE_DATA_READY:
            break;
        case STATE_ERROR:
            break;
    }
        
          
          #if 0
          // --- 2. Handle UWB Mode Switching ---
          //printf("Gestisco cambio di modalità\r\n");
          if (bool_mode_changed) {
            printf("Mode change detected.\r\n");
            // Reset device state for new mode
            if (device_mode == DEVICE_MODE_INITIATOR) {
                dwt_setrxtimeout(65000);
                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                printf("Switched to INITIATOR mode\r\n");
            } else {
                dwt_setrxtimeout(0);
                printf("Switched to RESPONDER mode (ID: %d)\r\n", DEVICE_ID);
            }
            dwt_rxreset(); // Reset reception

            // Reset distance tracking
            // No mutex needed here if only main loop modifies distances
            for (int i = 0; i < MAX_RESPONDERS; i++) {
                distances[i].distance = 0.0;
                distances[i].valid = false;
            }

            bool_mode_changed = false; // Reset the flag
            //NRF_LOG_FLUSH();
          }
          #endif
          
          if(!in_config_mode) { //&& device_mode == DEVICE_MODE_RESPONDER){
          #if 0
          // --- 3. Perform UWB Ranging ---
          //printf("Gestisco misurazioni\r\n");
          if (device_mode == DEVICE_MODE_INITIATOR) {
              // Cycle through enabled anchors
              //printf("Inizio misurazioni\r\n");
              for (int i = 0; i < MAX_RESPONDERS; i++) {
                  if (anchor_enabled[i] && i != DEVICE_ID) {
                      LEDS_INVERT(BSP_LED_1_MASK); // Activity indicator
                      // NRF_LOG_DEBUG("Ranging with %d", i); NRF_LOG_FLUSH();
                      ss_init_run(i); // This function polls DW1000 flags internally
                      LEDS_INVERT(BSP_LED_1_MASK);

                      // Check for SPI commands *between* ranging attempts
                        if (new_spi_command_received) break; // Process SPI cmd sooner

                      nrf_delay_ms(RNG_DELAY_MS); // Delay between anchors
                  }
                    // Allow processing SPI commands if loop takes time
                    if (new_spi_command_received) break;
              }
          } else { // Responder mode
              LEDS_INVERT(BSP_LED_1_MASK); // Activity indicator
              // NRF_LOG_DEBUG("Running as responder %d", DEVICE_ID); NRF_LOG_FLUSH();
              ss_resp_run(DEVICE_ID); // This function polls DW1000 flags internally
              LEDS_INVERT(BSP_LED_1_MASK);
                // Responder mode is mostly waiting, check SPI often is implicit.
                // Add a small delay if ss_resp_run returns immediately without receiving
                nrf_delay_ms(10); // Small delay to prevent busy-waiting if no poll received
          }
          
          if(device_mode==DEVICE_MODE_RESPONDER && !performing_avg_measurement) {
              // dispositivo in ascolto
              ss_resp_run(DEVICE_ID);

          } else if (device_mode==DEVICE_MODE_INITIATOR) {
              ss_init_run(1);
              nrf_delay_ms(10);
          
          } else {
              nrf_delay_ms(10); // busy wait
          }
          #endif

        }
        
  
    #else
    // Run appropriate function based on current mode
    if (device_mode == DEVICE_MODE_INITIATOR) {
          //printf("entro qui\r\n");
          // Communicate with all enabled responders in sequence
          for (int i = 0; i < MAX_RESPONDERS; i++) {
              if (anchor_enabled[i] && i != DEVICE_ID) {
                  ss_init_run(i);
                  nrf_delay_ms(100); // Small delay between anchors
              }
          }
        
    } else {
        // Responder mode - respond using our anchor ID
        ss_resp_run(DEVICE_ID);
    }

    // Check if mode has changed
    if (bool_mode_changed) {
        // Reset device state for new mode
        if (device_mode == DEVICE_MODE_INITIATOR) {
            // Initiator specific setup
            dwt_setrxtimeout(65000); // Maximum value timeout with DW1000 is 65ms
            dwt_setrxaftertxdelay(POLL_RX_TO_RESP_TX_DLY_UUS);
            printf("Switched to INITIATOR mode\r\n");
        } else {
            // Responder specific setup
            dwt_setrxtimeout(0); // set to NO receive timeout for responder
            printf("Switched to RESPONDER mode\r\n");
        }
        
        // Reset reception
        dwt_rxreset();
        
        bool_mode_changed = false;
    }
    #endif
  }
    
 #if 0
    if (device_mode == DEVICE_MODE_INITIATOR) {
        // Initiator specific setup
        //dwt_setrxtimeout(65000); // Maximum value timeout with DW1000 is 65ms
        dwt_setrxaftertxdelay(POLL_RX_TO_RESP_TX_DLY_UUS);
    } else {
        // Responder specific setup
        dwt_setrxtimeout(0); // set to NO receive timeout for responder
    }
 
   // No RTOS task here so just call the main loop here.
   // Loop forever responding to ranging requests.
   while (1)
   {
    
      
     /* Polling della UART */
    uint8_t data;
    if (boUART_getc(&data))
    {
      /* Gestione caratteri di controllo (terminatori di riga) */
      if (data == '\r' || data == '\n')
      {
        if (cmd_buffer_index > 0)
        {
          /* Termina la stringa */
          cmd_buffer[cmd_buffer_index] = '\0';
          
          /* Elabora il comando */
          process_uart_command(cmd_buffer);
          
          /* Reset per il prossimo comando */
          cmd_buffer_index = 0;
        }
      }
      /* Gestione del backspace */
      else if (data == 8 || data == 127) /* Backspace o Delete */
      {
        if (cmd_buffer_index > 0)
        {
          cmd_buffer_index--;
          /* Echo del backspace */
          printf("\b \b");
        }
      }
      /* Accumula caratteri nel buffer */
      else if (cmd_buffer_index < CMD_BUFFER_SIZE - 1)
      {
        cmd_buffer[cmd_buffer_index++] = data;
        /* Echo del carattere */
        printf("%c", data);
      }
    }

     if (device_mode == DEVICE_MODE_INITIATOR){
       // gestisco la comunicazione con tutti i risponditori
       for (int i = 0; i< MAX_RESPONDERS; i++){
         if(anchor_enabled[i] && i!=DEVICE_ID) {
           ss_init_run(i);
           nrf_delay_ms(100);
         }
       }
     } else {
       ss_resp_run(DEVICE_ID);
     }
 
     // Small delay
     nrf_delay_ms(RNG_DELAY_MS);
   }
 #endif
 
 #endif
 
   return 0; 
 }

 
 
 /*****************************************************************************************************************************************************
  * NOTES:
  *
  * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
  *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
  *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
  *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
  *    process.  NB:SEE ALSO NOTE 11.
  * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
  *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate. For a real production application, each
  *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
  * 3. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
  *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
  *    6.8M data rate used (around 200 ï¿½s).
  * 4. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
  *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
  * 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
  *     DW1000 API Guide for more details on the DW1000 driver functions.
  *
  ****************************************************************************************************************************************************/
 