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

#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 100

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

/* Setting globali validi per entrambe le modalità */
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT 299702547

/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define ALL_MSG_DEST_ID_INDEX 8 // Indice da cui viene preso id del dispositivo nel messaggio inviato
#define ALL_MSG_DEST_ID_IDX ALL_MSG_DEST_ID_INDEX

/* Struttura e array in cui salvo informazioni dei risponditori */
typedef struct {
  uint8_t id;
  bool valid;
  double distance;
} responder_distance_t;

static responder_distance_t distances[MAX_RESPONDERS]; // Vettore delle distanze per il dispositivo


// -------------------- Initiator ------------------------------

/* Frames used in the ranging process. See NOTE 1,2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'E', 'V', 1, 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'D', 1, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);


/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter 


/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  dev_id of the device to comunicate with
*
* @return none
*/
int ss_init_run(uint8_t dev_id)
{


  // Imposto il valore dell'id del dispositivo con il quale voglio comunicare nel messaggio tx
  tx_poll_msg[ALL_MSG_DEST_ID_INDEX] = dev_id;

  /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  tx_count++;
  printf("Transmission to responder %d (#%d) \r\n",dev_id, tx_count);


  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 4 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
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

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG)
  {		
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the responder we contacted.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;
     if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN - 2) == 0 && 
            rx_buffer[ALL_MSG_DEST_ID_IDX] == dev_id) 
      {	
      rx_count++;
      printf("Reception from responder %d (#%d)\r\n",dev_id, rx_count);
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

      distances[dev_id].id = dev_id;
      distances[dev_id].distance = distance;
      distances[dev_id].valid = true;

      printf("Distance to responder %d: %f\r\n",dev_id, distance);


      /* Controllo se tutti i risponditori abilitati sono validi, in caso  */
      bool all_valid = true;
      for (int i = 0; i < MAX_RESPONDERS; i++) {
          if (anchor_enabled[i] && !distances[i].valid) {
              all_valid = false;
              break;
          }
      }
      
      /* If all anchors have valid measurements, print them together */
      if (all_valid) {
          printf("\n--- Distances to all anchors ---\n");
          for (int i = 0; i < MAX_RESPONDERS; i++) {
              if (anchor_enabled[i]) {
                  printf("Anchor %d: %f m\n", 
                         distances[i].id, 
                         distances[i].distance);
              }
          }
          printf("-------------------------------\n");
      }
    }
  }
  else
  { 
    printf("No response \r\n");

    // Imposto la risposta come non valida per questo dispositivo
    distances[dev_id].valid = false;

    /* Clear RX error/timeout events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }

  /* Execute a delay between ranging exchanges. */
  //     deca_sleep(RNG_DELAY_MS);

  //	return(1);
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
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'E', 'V', 0, 0xE0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'D', 0, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


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
int ss_resp_run(uint8_t dev_id)
{
    /* Set our ID in the response message */
    tx_resp_msg[ALL_MSG_DEST_ID_IDX] = dev_id;
    
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };

    if (status_reg & SYS_STATUS_RXFCG)
    {
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
         * 2. Check that the destination ID matches our anchor ID */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN - 2) == 0 && 
            rx_buffer[ALL_MSG_DEST_ID_IDX] == dev_id)
        {
            uint32 resp_tx_time;
            int ret;

            printf("Poll received from initiator for anchor %d\r\n", dev_id);
            
            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Compute final message transmission time. */
            resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
            resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* Write all timestamps in the final message. */
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            /* Write and send the response message. */
            tx_resp_msg[ALL_MSG_SN_IDX] = resp_frame_seq_nb;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
            if (ret == DWT_SUCCESS)
            {
                printf("Response sent from anchor %d\r\n", dev_id);
                /* Poll DW1000 until TX frame sent event set. */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };

                /* Clear TXFRS event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                resp_frame_seq_nb++;
            }
            else
            {
                printf("Error starting response transmission\r\n");
                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();
            }
        }
        else
        {
            printf("Ignoring poll not addressed to us (our ID: %d, request ID: %d)\r\n", 
                 dev_id, rx_buffer[ALL_MSG_DEST_ID_IDX]);
        }
    }
    else
    {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
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
    
    while (true)
    {
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
        
        // Run appropriate function based on current mode
        if (device_mode == DEVICE_MODE_INITIATOR) {
              // Communicate with all enabled responders in sequence
              for (int i = 0; i < MAX_RESPONDERS; i++) {
                  if (anchor_enabled[i]) {
                      ss_init_run(i);
                      vTaskDelay(10); // Small delay between anchors
                  }
              }
            
        } else {
            // Responder mode - respond using our anchor ID
            ss_resp_run(DEVICE_ID);
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