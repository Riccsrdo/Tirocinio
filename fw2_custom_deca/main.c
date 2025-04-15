
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
 //#include "UART.h"
 
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
 
 //-----------------dw1000----------------------------
 
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
 
 /* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
 #define PRE_TIMEOUT 1000
 
 /* Delay between frames, in UWB microseconds. See NOTE 1 below. */
 #define POLL_TX_TO_RESP_RX_DLY_UUS 100
 
 /*Should be accurately calculated during calibration*/
 #ifndef TX_ANT_DLY
 #define TX_ANT_DLY 16300
 #endif
 #define RX_ANT_DLY 16456
 
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
 volatile bool anchor_enabled[MAX_RESPONDERS] = {true, true, true, false, false, false, false, false,
                                                        false, false, false, false, false, false, false, false}; /* Abilitazione dei risponditori */
 
#if 1

 /*-------------------------------SPI--------------------------------*/

 #define SPIS_INSTANCE 2 
 static const nrf_drv_spis_t m_spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE); // < SPIS instance.


 // Pin
#define MY_SPIS_CSN_PIN    NRF_GPIO_PIN_MAP(0, 28) // Modulo Pin 29 -> Connettore Pin 24 (CS_RPI)
#define MY_SPIS_SCK_PIN    NRF_GPIO_PIN_MAP(0, 8)  // Modulo Pin 25 -> Connettore Pin 23 (SPI1_CLK) <- NOTA: usa P0.08!
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
 #define SPI_CMD_SET_MODE_INIT 0x10 // Permette di settare il dispositivo come iniziatore e misurare le distanze dagli
 // altri dispositivi
 #define SPI_CMD_SET_MODE_RESP 0x11 // Permette di mettersi in ascolto e attendere richieste di comunicazione
 // dagli iniziatori
 #define SPI_CMD_SET_ID        0x20 // Permette di settare id per il dispositivo, deve essere seguito da un byte
 // che indica l'ID che si vuole settare
 #define SPI_CMD_ENABLE_ANCHOR 0x30 // Permette di abilitare l'ancora nell'array delle ancore disabilitate/abilitate, deve
 // essere seguito da un byte come ID
 #define SPI_CMD_DISABLE_ANCHOR 0x31 // Come sopra, serve id byte
 #define SPI_CMD_GET_INFO      0xFE // Informazioni sul sistema
 #define SPI_CMD_GET_HELP      0xFF // Dummy byte, usando per ottenere aiuto

 extern responder_distance_t distances[MAX_RESPONDERS];


 /*------------------------------fine-------------------------------*/

 #endif

 /* Gestione del buffer di comandi provenienti da UART */
#define CMD_BUFFER_SIZE 32
static char cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_buffer_index = 0;
 
 #ifdef USE_FREERTOS
 
 TaskHandle_t ss_initiator_task_handle; /**< Reference to SS TWR Initiator FreeRTOS task. */
 TaskHandle_t led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
 TimerHandle_t led_toggle_timer_handle; /**< Reference to LED1 toggling FreeRTOS timer. */
 TaskHandle_t uart_task_handle;
 TaskHandle_t ss_main_task_handle;
 
 /**@brief LED0 task entry function.
  *
  * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
  */
 static void led_toggle_task_function(void *pvParameter)
 {
   UNUSED_PARAMETER(pvParameter);
   while (true)
   {
     LEDS_INVERT(BSP_LED_0_MASK);
     /* Delay a task for a given number of ticks */
     vTaskDelay(TASK_DELAY);
     /* Tasks must be implemented to never return... */
   }
 }
 
 /**@brief The function to call when the LED1 FreeRTOS timer expires.
  *
  * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
  */
 static void led_toggle_timer_callback(void *pvParameter)
 {
   UNUSED_PARAMETER(pvParameter);
   LEDS_INVERT(BSP_LED_1_MASK);
 }
 
 #endif
 
 /* Funzione che gestisce i comandi di setting inviati via UART*/
static void process_uart_command(char *cmd)
{
  /* Rimuovi spazi iniziali e finali */
  char *start = cmd;
  while (*start == ' ' || *start == '\t') start++;
  
  char *end = start + strlen(start) - 1;
  while (end > start && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) {
    *end = '\0';
    end--;
  }
  
  /* Se la stringa ï¿½ vuota, non fare nulla */
  if (strlen(start) == 0) return;

  if (strncmp(start, "INIT", 4) == 0 || strncmp(start, "init", 4) == 0)
  {
    // Setto il dispositivo come iniziatore della comunicazione
    if (device_mode != DEVICE_MODE_INITIATOR)
    {
      device_mode = DEVICE_MODE_INITIATOR;
      bool_mode_changed = true;
      printf("Device set to INITIATOR mode\r\n");
    }
    else
    {
      printf("Already in INITIATOR mode\r\n");
    }
  }
  else if (strncmp(start, "RESP", 4) == 0 || strncmp(start, "resp", 4) == 0)
  {
    // Setto il dispositivo come risponditore
    if (device_mode != DEVICE_MODE_RESPONDER)
    {
      device_mode = DEVICE_MODE_RESPONDER;
      bool_mode_changed = true;
      printf("Device set to RESPONDER mode\r\n");
    }
    else
    {
      printf("Already in RESPONDER mode\r\n");
    }
  }
  else if (strncmp(start, "INFO", 4) == 0 || strncmp(start, "info", 4) == 0)
  {
    printf("Device mode: %s\r\n", device_mode == DEVICE_MODE_INITIATOR ? "Initiator" : "Responder");
    printf("Device ID: %d\r\n", DEVICE_ID);
    printf("Enabled responders: ");
    for (int i = 0; i < MAX_RESPONDERS; i++)
    {
      if (anchor_enabled[i])
      {
        printf("%d ", i);
      }
    }
    printf("\r\n");
  }
  else if (strncmp(start, "SETID", 5) == 0 || strncmp(start, "setid", 5) == 0)
  {
    // Setto l'id del dispositivo
    int id = 0;
    if (sscanf(start + 5, "%d", &id) == 1 && id >= 0 && id < MAX_RESPONDERS)
    {
      DEVICE_ID = (uint8_t)id;
      printf("Device ID set to: %d\r\n", DEVICE_ID);
    }
    else
    {
      printf("Invalid ID. Please specify a value between 0 and %d\r\n", MAX_RESPONDERS - 1);
    }
  }
  else if (strncmp(start, "ENABLE", 6) == 0 || strncmp(start, "enable", 6) == 0)
  {
    // Abilita la comunicazione con un certo risponditore
    int id = 0;
    if (sscanf(start + 6, "%d", &id) == 1 && id >= 0 && id < MAX_RESPONDERS)
    {
      anchor_enabled[id] = true;
      printf("Responder with ID %d enabled\r\n", id);
    }
    else
    {
      printf("Invalid ID. Please specify a value between 0 and %d\r\n", MAX_RESPONDERS - 1);
    }
  }
  else if (strncmp(start, "DISABLE", 7) == 0 || strncmp(start, "disable", 7) == 0)
  {
    // Disabilita la comunicazione con un certo risponditore
    int id = 0;
    if (sscanf(start + 7, "%d", &id) == 1 && id >= 0 && id < MAX_RESPONDERS)
    {
      anchor_enabled[id] = false;
      printf("Responder with ID %d disabled\r\n", id);
    }
    else
    {
      printf("Invalid ID. Please specify a value between 0 and %d\r\n", MAX_RESPONDERS - 1);
    }
  }
  else if (strncmp(start, "HELP", 4) == 0 || strncmp(start, "help", 4) == 0)
  {
    printf("Available commands:\r\n");
    printf("  INIT - Switch to initiator mode\r\n");
    printf("  RESP - Switch to responder mode\r\n");
    printf("  INFO - Show current device settings\r\n");
    printf("  SETID <n> - Set device ID (0-%d)\r\n", MAX_RESPONDERS - 1);
    printf("  ENABLE <n> - Enable responder with ID n\r\n");
    printf("  DISABLE <n> - Disable responder with ID n\r\n");
    printf("  HELP - Show this help message\r\n");
  }
  else
  {
    printf("Unknown command: %s\r\n", start);
    printf("Type HELP for available commands\r\n");
  }
}

static void uart_task_function(void *pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  
  printf("\r\n\r\n--- UWB Ranging System ---\r\n");
  printf("Send 'HELP' for command list\r\n");

  while (true)
  {
    uint8_t data;
    
    /* Polling della UART */
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
    
    /* Breve delay per non saturare la CPU */
    vTaskDelay(10);
  }
}

 
 /* Funzione che gestisce i vari settaggi effettuati via UART */
 void uart_event_handler(uint8_t data)
 {
   if (data == '\r' || data == '\n')
   {
     if (cmd_buffer_index > 0)
     {
       cmd_buffer[cmd_buffer_index] = '\0';
       // Process the command
       process_uart_command((char*)cmd_buffer);
       cmd_buffer_index = 0;
     }
   }
   else if (cmd_buffer_index < CMD_BUFFER_SIZE - 1)
   {
     cmd_buffer[cmd_buffer_index++] = data;
   }
 }


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


#if 1
/*---------------------------------------Funzioni SPI--------------------------------*/

/* 
Funzione usata per preparare la risposta a seguito della ricezione di un certo tipo di comando
ricevuto dal master.
*/
static void prepare_spi_response(uint8_t command_processed){
  switch (command_processed)
    {
        case SPI_CMD_GET_DISTANCES:
        case SPI_CMD_SET_MODE_INIT: // Often good to return current state after setting
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

        default:
             // Unknown command or error during processing
             m_spi_tx_buf[0] = 0xFF; // Example error code
             m_spi_tx_buf[1] = command_processed; // Echo faulty command
             memset(&m_spi_tx_buf[2], 0, SPI_TX_DATA_BUFFER_SIZE - 2);
             break;
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
    m_spi_tx_buf[tx_len++] = DEVICE_ID; //...seguita dall'id del dispositivo

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

/*
Funzione che processa i comandi SPI ricevuti dal master.
*/
static void process_spi_command(uint8_t *cmd_data, uint8_t cmd_len)
{
    if (cmd_len == 0) return;

    uint8_t command = cmd_data[0];

    switch (command)
    {
        case SPI_CMD_GET_DISTANCES:
            // Action is to prepare response, done after this function returns
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
            if (cmd_len > 1) {
                int id = cmd_data[1];
                if (id >= 0 && id < MAX_RESPONDERS) {
                    DEVICE_ID = (uint8_t)id;
                } else { /* Handle invalid ID */ }
            }
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

        case SPI_CMD_GET_INFO:
             // Action is to prepare response
             break;

        case SPI_CMD_GET_HELP:
             // Action is to prepare response
             break;

        default:
            // Unknown command - prepare_spi_response will handle this
             break;
    }
}

/*
Funzione che gestisce eventi SPI con interrupt.
*/
static void spis_event_handler(nrf_drv_spis_event_t event)
{
    printf("SPI HANDLER TRIGGERED! RX bytes: %d\r\n", event.rx_amount);
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {   
        printf("SPI XFER DONE, RX bytes: %d\r\n", event.rx_amount);
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
    }
}

/*
Funzione che inizializza dispositivo come slave SPI
*/
static void spi_slave_init(void)
{
    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin   = 3; //MY_SPIS_CSN_PIN;
    spis_config.miso_pin  = 7; //MY_SPIS_MISO_PIN;
    spis_config.mosi_pin  = 6; // MY_SPIS_MOSI_PIN;
    spis_config.sck_pin   = 4; // MY_SPIS_SCK_PIN;
    spis_config.mode      = NRF_DRV_SPIS_MODE_2;
    spis_config.bit_order = NRF_DRV_SPIS_BIT_ORDER_MSB_FIRST;
    spis_config.irq_priority = APP_IRQ_PRIORITY_LOW;

    APP_ERROR_CHECK(nrf_drv_spis_init(&m_spis, &spis_config, spis_event_handler));

    // Prepare the buffer for transmission
    prepare_distance_data_for_spi();

    // Initialize buffers for transmissions
    APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&m_spis, m_spi_tx_buf, SPI_TX_DATA_BUFFER_SIZE, m_spi_rx_buf, SPI_CMD_BUFFER_SIZE));

    new_spi_command_received = false;
    spi_cmd_length = 0;
}

/*----------------------------------------fine----------------------------------------*/
 
 #endif
 
 int main(void)
 {
   /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
    LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
    LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );

  #ifdef USE_FREERTOS
    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    /* Create task for SS TWR Initiator set to 2 */
    UNUSED_VARIABLE(xTaskCreate(ss_initiator_task_function, "SSTWR_INIT", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_main_task_handle));
  #endif // #ifdef USE_FREERTOS
 
   /*Initialization UART*/
   boUART_Init();

   /* Initialize clock */
   APP_ERROR_CHECK(nrf_drv_clock_init());
   nrf_drv_clock_lfclk_request(NULL);

   /* Initialize SPI Slave */
   spi_slave_init();
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
   dwt_configure(&config);

   /* Initialization of the DW1000 interrupt */
   dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

   /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
   dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

   /* Apply default antenna delay value. See NOTE 2 below. */
   dwt_setrxantennadelay(RX_ANT_DLY);
   dwt_settxantennadelay(TX_ANT_DLY);
 
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

   uint8_t current_spi_command = 0; // Store command being processed
 
 
 #ifdef USE_FREERTOS
   /* Start FreeRTOS scheduler. */
   vTaskStartScheduler();	
 
   while(1) 
   {};
 #else

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
        //printf("Inizio raccolta dati SPI\r\n");
        if (new_spi_command_received) {
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

        // --- 3. Perform UWB Ranging ---
        //printf("Gestisco misurazioni\r\n");
        if (device_mode == DEVICE_MODE_INITIATOR) {
            // Cycle through enabled anchors
            printf("Inizio misurazioni\r\n");
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
 