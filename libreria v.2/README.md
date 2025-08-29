
# Libreria per invio comandi UART da un dispositivo master al DWM1001-Dev

In questa cartella sono contenuti i file libreria che permettono di gestire il DWM1001-Dev a partire da un dispositivo master, quale un Raspberry Pi.

Il Raspberry Pi deve essere configurato in modo appropriato da permettere il funzionamento dei dispositivi, per farlo occorre:
- Digitare, in una shell, il comando `sudo raspi-config`.
- Andare nella sezione di "Interface Options".
- Identificare la sezione di "Serial Port".
- Alla prima domanda selezionare "no".
- Alla seconda domanda selezionare "si".
- Riavviare il dispositivo