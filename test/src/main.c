//
// Created by f3m on 09/03/25.
//

#include "main.h"


int init()
{
    D_Print("Initializing BNet...\n");
    if (initBNet())
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}

int main()
{
#ifdef Debug
    D_Print("Launched in Debug Mode!\n");
#endif

    //TODO: init di BNet
    //TODO: conteggio nodi
    //TODO: init protocollo Heartbeat
    //TODO: stabilire ordine (leader election?ID?)
    //TODO: init di Distance con l'ordine di inizio
    //TODO: init di Ultra wideband con l'ordine di inizio

    if (init())
        return EXIT_FAILURE;

    /*---------------------DWM setup/use---------------------*/ 
    int ret = dwm_spi_init(SPI_DEVICE, SPI_SPEED, SPI_MODE); // Apro la comunicazione SPI in modalità 2
    if (ret != 0){
        fprintf(stderr, "Errore nell'apertura della comunicazione SPI\r\n");
        return -1;
    }

    // Chiamo funzione dwm_enter_config_mode per permettere al dispositivo di settare id
    // Per ogni id che passo chiamo, in un for loop, la funzione dwm_set_device_id_at (incrementando indice)
    // Imposto la modalità del dispositivo, che sia Iniziatore (per effettuare le misurazioni delle distanze), o risponditore
    // Esco dalla modalità configurazione

    // Chiamo la funzione dwm_measure_average passando id del dispositivo di cui voglio misurare distanza...
    // ... o dwm_measure_average_all, se voglio misurarla con tutti i dispositivi in modalità risponditore

    /*------------------------------------------------------*/



    
    dwm_spi_close(); // Al termine delle operazioni, chiudi la comunicazione SPI

    return EXIT_SUCCESS;
}
