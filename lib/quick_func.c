#include "quick_func.h"

/*
File in cui sono definite le funzioni che, usando quelle di libreria,
permettono di eseguire task senza doverle chiamare una ad una.
*/

/*
Permette di settare in modo veloce il dispositivo come ancora.
Prende come parametro il valore initiator, che deve essere 1 per almeno una delle
ancore presenti nel network.

Ritorna 0 in caso di successo, -1 altrimenti.
*/
int set_as_anchor(int initiator){
    dwm_cfg_anchor_t cfg;
    int rv;

    if (initiator != 0 && initiator != 1){
        printf("Valore di initiator non valido\n");
        return -1;
    }

    cfg.initiator = initiator; //
    cfg.bridge = 0; 
    cfg.common.enc_en = 0; 
    cfg.common.led_en = 1; 
    cfg.common.ble_en = 1; 
    cfg.common.fw_update_en = 1; 
    cfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE; 
    rv = dwm_cfg_anchor_set(&cfg);
    if(rv < 0){
        printf("Errore in cfg_anchor_set\n");
        return -1;
    }

    usleep(10000); // attesa per permettere la scrittura dei dati

    rv = dwm_reset(); // per applicare le modifiche dopo aver settato come ancora
    if (rv < 0){
        printf("Errore in reset\n");
        return -1;
    }

    return 0;
}

/*
Funzione che permette di settare il dispositivo come tag.
*/
int set_as_tag(){
    dwm_cfg_tag_t cfg;
    int rv;
    
    //cfg.stnry_en = true; // Modalità stazionaria con rate di aggiornamento per questa modalità
    cfg.meas_mode = DWM_MEAS_MODE_TWR; // Modalità di misurazione Time of Flight
    cfg.loc_engine_en = true; // Abilita il motore di localizzazione
    cfg.low_power_en = false; // Disabilita la modalità di risparmio energetico
    cfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE; // Modalità UWB attiva
    cfg.common.ble_en = true; // Abilita il BLE
    cfg.common.enc_en = false; // Disabilita l'encryption
    cfg.common.led_en = true; // Abilita i LED
    cfg.common.fw_update_en = true; // Abilita l'aggiornamento del firmware per permettere sincronizzazione
    // con firmware di altri dispositivi (consigliato dalla documentazione)

    rv = dwm_cfg_tag_set(&cfg);
    if(rv < 0){
        printf("Errore in cfg_tag_set\n");
        return -1;
    }

    usleep(10000); // attesa per permettere la scrittura dei dati

    rv = dwm_reset(); // per applicare le modifiche dopo aver settato come tag
    if (rv < 0){
        printf("Errore in reset\n");
        return -1;
    }

    return 0;
    
}


