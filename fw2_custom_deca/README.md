# Firmware FW2 Custom per DWM1001-Dev

## Introduzione
In questa cartella sono contenuti i file da sostituire all'esempio fornito da Qorvo nella [repository](https://github.com/Decawave/dwm1001-examples), in particolar modo negli esempi nella cartella *examples/ss_twr_init*.
Con questi file sarà possibile, a completamento del progetto, effettuare una misurazione 1-a-molti delle distanze con dispositivi DWM1001-dev. [TBU]

## Funzionamento
Come già anticipato, per far funzionare tutto occorre flashare in modo corretto il dispositivo con il firmware
fornito dalla Qorvo (consultare documentazione ufficiale) utilizzando JFlash.
Una volta eseguito questo passaggio è possibile usare il software SEGGER Embedded Studio (importante utilizzare la versione 5.70a per compatibilità) per poter flashare il dispositivo facendo uso dei codici forniti.
Una volta sostituiti i file è possibile aprire il progetto SEGGER collocandosi nella cartella *SES* e aprendo il file con estensione *emProject*. All'interno per poter far funzionare il tutto occorrerà:
- sostituire nel file nrf52.h alla linea 146 l'include con il seguente: #include "toolchain/cmsis/include/core_cm4.h"
- sostituire nel file retarget.c alla linea 112 la dicitura File * con __printf_tag_ptr

Per far funzionare SPI, inoltre, è necessario:
1) Aprire il file sdk_config.h
2) Cercare le diciture di definizione SPI
    1) Modificare SPIS_ENABLED a 1
    2) Modificare SPIS2_ENABLED a 1
Così facendo il dispositivo può essere configurato in modalità slave, così da permettere la comunicazione con un master.

Ogni dispositivo sarà configurabile tramite SPI in futuro. [TBU]

Una volta fatto questo sarà possibile effettuare una build del progetto, e per farlo occorrerà:
1) Cliccare "build" nella barra di utilità in alto
    - cliccare "rebuild nome_progetto"
2) Cliccare "target"
    - cliccare "connect j-link", scegliendo opportunamente il dispositivo che si vuole flashare
    - cliccare "download nome_progetto"

Una volta effettuati questi passaggi dovrebbe essere possibile utilizzare il dispositivo con il firmware custom.

## Ultimo aggiornamento - 12_04 15:30
Continua la configurazione di SPI, sto fixando alcuni problemi per cui non veniva configurato bene tutto di SPI, per farlo è stato necessario modificare il file sdk_config.h abilitando esplicitamente la comunicazione SPIS (SPI Slave) per il canale 2 di SPI, lasciando le altre configurazioni come stanno.
Inoltre, ho aggiunto il polling di dati SPI anche in modalità responder quando è in attesa di ricevere poll dall'iniziatore.


