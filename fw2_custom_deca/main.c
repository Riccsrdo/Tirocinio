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
 #include "nrf_drv_spi.h"
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
 extern void ss_init_run(uint8_t anchor_id);           // Funzione per utilizzo come iniziatore comunicazione
 extern void ss_resp_run(uint8_t anchor_id);           // Funzione per utilizzo come risponditore comunicazione
 extern void ss_main_task_function(void *pvParameter); // Funzione per gestione task
 extern void ss_initiator_task_function(void *pvParameter);

 // Callbacks for DW1000 Interrupts
 extern void tx_conf_cb(const dwt_cb_data_t *cb_data);
 extern void rx_ok_cb(const dwt_cb_data_t *cb_data);
 extern void rx_to_cb(const dwt_cb_data_t *cb_data);
 extern void rx_err_cb(const dwt_cb_data_t *cb_data);
 
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
 typedef enum
 {
   DEVICE_MODE_INITIATOR = 0, /* Dispositivo in modalitÃ  iniziatore */
   DEVICE_MODE_RESPONDER = 1, /* Dispositivo in modalitÃ  risponditore */
 } device_mode_t;
 
 volatile device_mode_t device_mode = DEVICE_MODE_INITIATOR; /* Imposto modalitÃ  iniziale come iniziatore */
 volatile bool bool_mode_changed = false;                         /* Flag booleana per inidicare che la modalitÃ  Ã¨ cambiata */
 
 /* Impostazioni modalitÃ  multi-risponditore
 Configurare in base al numero di risponditori utilizzati */
 #define MAX_RESPONDERS 16 // da aggiornare anche su ss_init_main.c in caso di cambiamento
 volatile uint8_t anchor_ids[MAX_RESPONDERS] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
                                                           9, 10, 11, 12, 13, 14, 15}; /* ID dei risponditori */
 volatile uint8_t DEVICE_ID = 0; /* ID del dispositivo in uso */                // DA CONFIGURARE IN BASE AL DISPOSITIVO
 volatile bool anchor_enabled[MAX_RESPONDERS] = {true, true, false, false, false, false, false, false,
                                                        false, false, false, false, false, false, false, false}; /* Abilitazione dei risponditori */
 
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
   
 
   //-------------dw1000  ini------end---------------------------
   // IF WE GET HERE THEN THE LEDS WILL BLINK
 
 
   printf("Starting in %s mode \r\n", device_mode == DEVICE_MODE_INITIATOR ? "Initiator" : "Responder");
   if(device_mode == DEVICE_MODE_RESPONDER){
     printf("Device id: %d \r\n", DEVICE_ID);
   }
 
 
 #ifdef USE_FREERTOS
   /* Start FreeRTOS scheduler. */
   vTaskStartScheduler();	
 
   while(1) 
   {};
 #else

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
 