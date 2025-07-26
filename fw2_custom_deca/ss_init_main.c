/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
//#include "ss_init_main.h"
#include "utils.h"
#include <math.h>

#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 150

#if 0
/* Setting del dispositivo in funzione */
typedef enum {
  DEVICE_MODE_INITIATOR = 0,
  DEVICE_MODE_RESPONDER = 1
} device_mode_t;


/* Variabili esterne prese dal main */
#define MAX_RESPONDERS 16 // da aggiornare anche sul main in caso di cambiamento
extern volatile device_mode_t device_mode;
extern volatile bool bool_mode_changed;
extern volatile uint8_t anchor_ids[MAX_RESPONDERS];
extern volatile uint8_t DEVICE_ID;
extern volatile bool anchor_enabled[MAX_RESPONDERS];


/* Struttura e array in cui salvo informazioni dei risponditori */
typedef struct {
  uint8_t id;
  bool valid;
  double distance;
} responder_distance_t;

static responder_distance_t distances[MAX_RESPONDERS]; // Vettore delle distanze per il dispositivo
#else
responder_distance_t distances[MAX_RESPONDERS];

#endif

/* Setting globali validi per entrambe le modalitÃ  */
//#define POLL_RX_TO_RESP_TX_DLY_UUS  2000
//#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT 299702547

/* Impostazioni di timeout */
#define MAX_TIMEOUT_COUNT 10
#define DW1000_MAX_TIMEOUT_MS 65
#define EXTENDED_TIMEOUT_MS 200

/*
Nuova struttura per supportare id a 64 bit:
Byte 0-1: Frame Control
Byte 2: Sequence Number
Byte 3-4: PAN ID 
Byte 5-6: Destination address
Byte 7-8: Source address
Byte 9: Function code (E0 per poll, E1 per resp)
Byte 10-17: ID del dispositivo target (per Poll) o sorgente (per Resp)
Byte 18-21: Timestamp poll rx (solo in Resp)
Byte 22-25: Timestamp Resp TX (solo in Resp)
Byte 26-27: Checksum (automatico)


*/

/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN_OLD 10
/* Indexes to access some of the fields in the frames defined above. */
#define ID_LEN 8 // Lunghezza degli ID sotto forma di 8 byte
#define ALL_MSG_FC_IDX 9 // indice function code
#define ALL_MSG_ID_START_IDX 10 // indice di partenza dell'id nel messaggio
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_TS_LEN 4
#define RESP_MSG_POLL_RX_TS_IDX (ALL_MSG_ID_START_IDX + ID_LEN) // nuovo indice di partenza Timestamp
#define RESP_MSG_RESP_TX_TS_IDX (RESP_MSG_POLL_RX_TS_IDX + RESP_MSG_TS_LEN) // Nuovo indice Timestamp 2
#define ALL_MSG_COMMON_LEN (ALL_MSG_ID_START_IDX) // lunghezza fino a prima dell'id
#define POLL_FRAME_LEN (ALL_MSG_ID_START_IDX + ID_LEN + 2) // lunghezza poll: common + id + checksum
#define RESP_FRAME_LEN (RESP_MSG_RESP_TX_TS_IDX + RESP_MSG_TS_LEN + 2) // len resp: fino a t2 + checksum 
#define ALL_MSG_DEST_ID_INDEX 8 // Indice da cui viene preso id del dispositivo nel messaggio inviato
#define ALL_MSG_DEST_ID_IDX ALL_MSG_DEST_ID_INDEX


/* Interrupt flag */
static volatile int tx_int_flag = 0; // Transmit success interrupt flag
static volatile int rx_int_flag = 0; // receive success interrupt flag
static volatile int to_int_flag = 0; // timeout interrupt flag
static volatile int er_int_flag = 0; // error interrupt flag

/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter 


// Funzioni di utilità per convertire uint64 in 8 byte
static inline void uint64_to_bytes(uint64_t val, uint8_t *bytes) {
  memcpy(bytes, &val, sizeof(uint64_t));
}

static inline uint64_t bytes_to_uint64(const uint8_t* bytes){
  uint64_t val;
  memcpy(&val, bytes, sizeof(uint64_t));
  return val;
}


// -------------------- Initiator ------------------------------



/* Frames used in the ranging process. See NOTE 1,2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, // Frame control
                              0, // Sequence number
                              0xCA, 0xDE, // PAN ID
                              'D', 'E',  // Destination address
                              'V', 1, // Source address
                              0xE0, // Function code
                              0, 0, 0, 0, 0, 0, 0, 0, // Placeholder 8 byte per ID
                              0, 0}; // Grandezza = 20
      
static uint8 rx_resp_msg[] = {0x41, 0x88, // Frame control
                              0, // sequence number
                              0xCA, 0xDE, // PAN ID
                              'V', 1, // Source address
                              'D', 'E',  // Destination address
                              0xE1, // Function code
                              0, 0, 0, 0, 0, 0, 0, 0, // Placeholder per ID
                              0, 0, 0, 0, // POLL RX TS
                              0, 0, 0, 0, // RESP TX TS
                              0, 0}; // Grandezza = 28

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN RESP_FRAME_LEN // lunghezza massima
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);




/*! ------------------------------------------------------------------------------------------------------------------
* @fn ss_init_run()
*
* @brief Initiator function using interrupts
*
* @param  dev_id of the device to comunicate with
*
* @return none
*/
int ss_init_run(uint64_t dev_id)
{ 
  /* Reset interrupts flags*/
  //tx_int_flag = 0;
  //rx_int_flag = 0;
  //to_int_flag = 0;
  //er_int_flag = 0;


  // Imposto il valore dell'id del dispositivo con il quale voglio comunicare nel messaggio tx
  //tx_poll_msg[ALL_MSG_DEST_ID_INDEX] = dev_id;

  /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

  // Copio ID a 64 bit nel messaggio
  uint64_to_bytes(dev_id, &tx_poll_msg[ALL_MSG_ID_START_IDX]);

  //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  //tx_count++;
  

  
  /*
  // We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 4 below. *
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  { 
  
  };
  */

  

  // Waiting for transmission success flag
  while(!(tx_int_flag))
  {};

    #if 0  // include if required to help debug timeouts.
    int temp = 0;		
    if(status_reg & SYS_STATUS_RXFCG )
    temp =1;
    else if(status_reg & SYS_STATUS_ALL_RX_TO )
    temp =2;
    if(status_reg & SYS_STATUS_ALL_RX_ERR )
    temp =3;
    #endif
   

  if (tx_int_flag)
  {
    tx_count++;
    uint32_t id_alta = (uint32_t)(dev_id >> 32);
    uint32_t id_bassa = (uint32_t)(dev_id & 0xFFFFFFFFUL); // o solo (uint32_t)DEVICE_ID
    // Stampa le due parti in esadecimale, 8 cifre ciascuna con padding
    //printf("Transmission to responder %08lX%08lX (#%d) \r\n", id_alta, id_bassa, tx_count);

    // Resetting tx interrupt flag
    tx_int_flag = 0;

  }

  /* Wait for reception, timeout or error interrupt flag*/
  while (!(rx_int_flag || to_int_flag|| er_int_flag))
  {};

  //printf("Flag rx_int_flag: %d\r\n", rx_int_flag);

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (rx_int_flag)
  {		
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the responder we contacted.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    // Allora:
    // 1. Controllo parte comune fino al function code
    // 2. Controllo il function code
    // 3. Estraggo ID dal buffer ricevuto
    // 4. Confronto ID ricevuto con quello atteso (dev_id)
    
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if(frame_len >= (RESP_MSG_RESP_TX_TS_IDX + RESP_MSG_TS_LEN) && // lunghezza minima
       memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0 && // compara parte fissa iniziale
       rx_buffer[ALL_MSG_FC_IDX] == 0xE1) //function code = response
    {
      uint64_t received_id = bytes_to_uint64(&rx_buffer[ALL_MSG_ID_START_IDX]); // salvo id ricevuto

      if(received_id == dev_id) // Controllo che l'id sia quello del dispositivo a cui volevo inviare originariamente
      {
        rx_count++;
        uint32_t id_alta = (uint32_t)(dev_id >> 32);
        uint32_t id_bassa = (uint32_t)(dev_id & 0xFFFFFFFFUL); // o solo (uint32_t)DEVICE_ID
        // Stampa le due parti in esadecimale, 8 cifre ciascuna con padding
        //printf("Reception from responder %08lX%08lX (#%d)\r\n", id_alta, id_bassa, rx_count);
        uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32 rtd_init, rtd_resp;
        float clockOffsetRatio ;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 5 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 7 below. */
        clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

        /* Get timestamps embedded in response message. */
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
        distance = tof * SPEED_OF_LIGHT;

        //printf("Distance to responder: %f\r\n", distance);
        printf("%d: %f\r\n", rx_count, distance);

        /* Resetting receive interrupt flag */
        rx_int_flag = 0;

        // Check diagnostic data to identify NLOS
        uint32_t fqual = dwt_read32bitreg(RX_FQUAL_ID);
        uint16_t pp_ampl = (uint16_t) (fqual >> 16); // peak path amplitude
        uint16_t std_noise = (uint16_t) dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x00);
        uint16_t fp_ampl1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET); // check first path power
        uint16_t fp_ampl2 = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x02);
        uint16_t fp_ampl3 = dwt_read16bitoffsetreg(0x2E, 0x04); 
        uint16_t cir_pwr = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x06);

        // Check if noise is null
        if(std_noise == 0) std_noise = 1;

        // Calculate power estimation in dB
        double f1 = (double)fp_ampl1;
        double f2 = (double)fp_ampl2;
        double f3 = (double)fp_ampl3;
        double N = (double)std_noise;
        double C_raw = (double)cir_pwr;

        // A constant for power formula
        // For PRF 16MHz, the values is 121.74
        double A_const = 121.74;

        double firstPathPower = 10.0 * log10((f1*f1 + f2*f2 + f3*f3) / (N*N));
        double channelPower = 10.0 * log10((C_raw * pow(2.0, 17.0)) / (N*N));

        // Check the difference between total channel power and first path power
        // Value is tunable
        bool is_nlos = (channelPower - firstPathPower) > 8.0;

        // Cerco indice dell'array di ancore a cui è memorizzato il device con dev_id
        int target_index = -1;
        for(int i=0; i<MAX_RESPONDERS; i++){
          if(anchor_ids[i] == dev_id) {
            target_index = i;
            break;
          }
        }

        if(target_index >= 0 && // ha trovato id corretto nella lista
           target_index < MAX_RESPONDERS)
        {
          distances[target_index].id = dev_id;
          distances[target_index].distance = distance;
          distances[target_index].valid = true;
          distances[target_index].nlos_suspection = is_nlos;
        }
      } 
      else {
        uint32_t id_alta = (uint32_t)(dev_id >> 32);
        uint32_t id_bassa = (uint32_t)(dev_id & 0xFFFFFFFFUL); // o solo (uint32_t)DEVICE_ID
        // Stampa le due parti in esadecimale, 8 cifre ciascuna con padding
        printf("Received distance from wrong id, expected %08lX%08lX\r\n", id_alta, id_bassa);
      }

      /* Resetting receive interrupt flag */
      rx_int_flag = 0;
    
    }
    
  }

  if (to_int_flag || er_int_flag)
  {
    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();

    /*Reseting interrupt flag*/
    to_int_flag = 0 ;
    er_int_flag = 0 ;
  }

  /* Execute a delay between ranging exchanges. */
  //     deca_sleep(RNG_DELAY_MS);

  //	return(1);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_ok_cb()
*
* @brief Callback to process RX good frame events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  rx_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #1 */
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_to_cb()
*
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  to_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #2 */
  //printf("TimeOut\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_err_cb()
*
* @brief Callback to process RX error events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  er_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #3 */
  printf("Transmission Error : may receive package from different UWB device\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn tx_conf_cb()
*
* @brief Callback to process TX confirmation events
*
* @param  cb_data  callback data
*
* @return  none
*/
void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
  * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
  * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
  * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
  * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

  tx_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #4 */
}


/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
  *ts += ts_field[i] << (i * 8);
  }
}

// ------------------------------ Responder ------------------------------------------------


/* Frames used in the ranging process for responder. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'E', 'V', 1, 0xE0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, // Frame control
                              0, // Sequence number
                              0xCA, 0xDE, // PAN ID
                              'V', 1, // Destination address
                              'D', 'E', // Source address
                              0xE1, // function code
                              0, 0, 0, 0, 0, 0, 0, 0, // placeholder id (8 byte) 
                              0, 0, 0, 0, // placeholder poll rx ts
                              0, 0, 0, 0, // placeholder resp tx ts
                              0, 0}; // grandezza = 28



/* Frame sequence number for responder */
static uint8 resp_frame_seq_nb = 0;

/* Declaration of responder static functions. */
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);

/* Timestamps for responder */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/**
 * @brief Responder mode function for SS TWR operation
 * 
 * @param dev_id The ID of this anchor
 * @return int Always returns 1
 */
int ss_resp_run(uint64_t dev_id)
{   
    
    //printf("Entro in modalità resp\r\n");
    /* Reset interrupt flags for responder mode */
    rx_int_flag = 0;
    tx_int_flag = 0;
    er_int_flag = 0;
    to_int_flag = 0;

    /* Set our ID in the response message */
    //tx_resp_msg[ALL_MSG_DEST_ID_IDX] = dev_id;
    
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    //printf("Responder %d: Attendo pacchetti, status: 0x%08x\r\n", dev_id, dwt_read32bitreg(SYS_STATUS_ID));
 
    
    // Attendo ricezione, errore o reset della task
    while (!(rx_int_flag || er_int_flag || to_int_flag || bool_mode_changed || new_spi_command_received || uart_new_command)) 
    { 
        nrf_delay_ms(1); // evita loop continui
    };

    if (new_spi_command_received || uart_new_command) {
        //printf("Responder %llu: Comando SPI ricevuto durante attesa UWB, ritorno al main loop.\r\n", (unsigned long long)dev_id);
        dwt_forcetrxoff(); // Disabilita transceiver DW1000 per sicurezza prima di uscire
        //LEDS_ON(BSP_LED_0_MASK);
        return 1; // Ritorna al main loop che gestirà il comando SPI
    }

    if(bool_mode_changed){
      dwt_forcetrxoff();
      return 1; // Ritorno dato che non devo più attendere risposte
    }
    

    if (rx_int_flag)
    {   
        
        printf("Responder: Ricevuto qualcosa, check del msg...\r\n");
        uint32 frame_len;

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is intended for us:
         * 1. Check first part of common header
         * 2. Controlla function code
         * 3. Estrai ID destinatario dal poll
         * 4. Confronta con ID (dev_id)
         */
         
        /*
        for(int i=0; i<ALL_MSG_ID_START_IDX + ID_LEN + 2; i++){
          printf("INDICE: %d \t", i);
          if(i<ALL_MSG_ID_START_IDX-1) printf("NORMALE: el: %d \t", rx_poll_msg[i]);
          printf("RICEVUTO: el: %d\r\n", rx_buffer[i]);
        }
        */
        
        
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if(frame_len >= (ALL_MSG_ID_START_IDX + ID_LEN) && // lunghezza minima
           memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0 &&
           rx_buffer[ALL_MSG_FC_IDX] == 0xE0) // Function code = poll
        { 
          //printf("Entro qui\r\n");
          uint64_t received_dest_id = bytes_to_uint64(&rx_buffer[ALL_MSG_ID_START_IDX]);
          uint32_t id_alta = (uint32_t)(received_dest_id >> 32);
          uint32_t id_bassa = (uint32_t)(received_dest_id & 0xFFFFFFFFUL);
          //printf("Received destination id: %08lX%08lX\r\n", id_alta, id_bassa);

          if(received_dest_id == dev_id)
          { 
            uint32 resp_tx_time;
            int ret;
            uint32_t id_alta = (uint32_t)(dev_id >> 32);
            uint32_t id_bassa = (uint32_t)(dev_id & 0xFFFFFFFFUL); // o solo (uint32_t)DEVICE_ID
            // Stampa le due parti in esadecimale, 8 cifre ciascuna con padding
            printf("Poll received from initiator for anchor %08lX%08lX\r\n", id_alta, id_bassa);
            
            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Compute final message transmission time. */
            uint32_t delay_to_use = nlos_mode ? POLL_RX_TO_RESP_TX_DLY_UUS_NLOS : POLL_RX_TO_RESP_TX_DLY_UUS;
            resp_tx_time = (poll_rx_ts + (delay_to_use * UUS_TO_DWT_TIME)) >> 8;
            //resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            
            /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
            resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* Write all timestamps in the final message. */
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            /* Write and send the response message. */
            tx_resp_msg[ALL_MSG_SN_IDX] = resp_frame_seq_nb;

            // Scrivo id nel messaggio
            uint64_to_bytes(dev_id, &tx_resp_msg[ALL_MSG_ID_START_IDX]);

            dwt_setdelayedtrxtime(resp_tx_time);

            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
            if (ret == DWT_SUCCESS)
            {
                
                
                // attendi completamento TX
                tx_int_flag = 0;
                while(!tx_int_flag && !bool_mode_changed) {};

                if(tx_int_flag){
                  printf("Response sent from anchor\r\n", dev_id);
                  resp_frame_seq_nb++;
                }
                else {
                  printf("Transmission wait interrupted\r\n");
                }

                
            }
            else
            {
                printf("Error scheduling delayed transmission (error=%d)\r\n", ret);


                 // provo trasmissione immediata
                 dwt_forcetrxoff();

                 ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

                 if (ret == DWT_SUCCESS){
                    printf("Immediate transmission started\r\n");

                    // wait for completition
                    tx_int_flag = 0;
                    while(!tx_int_flag && !bool_mode_changed) {};

                    if(tx_int_flag){
                       printf("Response sent\r\n");
                       resp_frame_seq_nb++;
                    }
                 } else {
                    printf("Could not start immediate transmission\r\n");
                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
                 }

                
            }
          }
          else 
          {
            printf("messaggio ricevuto non diretto a noi \r\n");
          }
        } 
        else
        {
        }
        
        /* Reset reception interrupt flag */
        rx_int_flag = 0;

        
    }
    if (er_int_flag){
      printf("Error receiving data \r\n");

      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

      dwt_rxreset();

      er_int_flag = 0;
    }

    if(to_int_flag){
      printf("Timeout receiving data as responder\r\n");

      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);

      dwt_rxreset();

      to_int_flag = 0;
    }

    return 1;
}

/**
 * @brief Get the RX timestamp in a 64-bit variable
 * 
 * @return uint64 timestamp value
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/**
 * @brief Fill a timestamp field in the response message
 * 
 * @param ts_field Pointer to timestamp field in the message
 * @param ts Timestamp value to store
 */
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
    int i;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}



/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);

  int i = 0;

  while (true)
  { 
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

    // Run appropriate function based on current mode
    if (device_mode == DEVICE_MODE_INITIATOR) {
          //printf("entro qui\r\n");
          // Communicate with all enabled responders in sequence
          for (int i = 0; i < MAX_RESPONDERS; i++) {
              if (anchor_enabled[i] && i != DEVICE_ID) {
                  ss_init_run(i);
                  vTaskDelay(RNG_DELAY_MS); // Small delay between anchors
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
        
        // Reset distance tracking
        for (int i = 0; i < MAX_RESPONDERS; i++) {
            distances[i].distance = 0.0;
            distances[i].valid = false;
        }
        
        bool_mode_changed = false;
    }
  }
}


/**
 * @brief Main task function that calls either initiator or responder based on mode
 * 
 * @param pvParameter Task parameters (unused)
 */
void ss_main_task_function(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    /* Initialize distance tracking for each anchor */
    for (int i = 0; i < MAX_RESPONDERS; i++) {
        distances[i].id = i;
        distances[i].distance = 0.0;
        distances[i].valid = false;
    }
    
    dwt_setleds(DWT_LEDS_ENABLE);
    
    // Configure for initial mode
    if (device_mode == DEVICE_MODE_INITIATOR) {
        // Initiator specific setup
        dwt_setrxtimeout(65000); // Maximum value timeout with DW1000 is 65ms
        dwt_setrxaftertxdelay(POLL_RX_TO_RESP_TX_DLY_UUS);
    } else {
        // Responder specific setup
        dwt_setrxtimeout(0); // set to NO receive timeout for responder
    }

    printf("Arrivo qui\r\n");
    
    while (true)
    {
        // Run appropriate function based on current mode
        if (device_mode == DEVICE_MODE_INITIATOR) {
              printf("entro qui\r\n");
              // Communicate with all enabled responders in sequence
              for (int i = 0; i < MAX_RESPONDERS; i++) {
                  if (anchor_enabled[i] && i != DEVICE_ID) {
                      ss_init_run(i);
                      vTaskDelay(RNG_DELAY_MS); // Small delay between anchors
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
            
            // Reset distance tracking
            for (int i = 0; i < MAX_RESPONDERS; i++) {
                distances[i].distance = 0.0;
                distances[i].valid = false;
            }
            
            bool_mode_changed = false;
        }
        
        
        /* Delay between operations */
        vTaskDelay(RNG_DELAY_MS);
    }
}
/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 5. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 6. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 7. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/