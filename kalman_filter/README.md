# Filtro di Kalman per la correzione misurazioni UWB

## Introduzione
L'idea dietro è quella di far uso del filtro di kalman per correggere le misurazioni derivanti dal sensore UWB che si discostano dalla realtà. Si fa uso del modello lineare in quanto non siamo a conoscenza, a priori, dell'angolo della misurazione effettuata. 

Il filtro si basa su un concetto semplice, ovvero quello di comparare la distanza predetta avendo il precedente stato `x` dato dalla distanza e velocità, con quella misurata con i sensori UWB. 
Fa uso di una serie di elementi teorici, riportati nel file `ukf.md` derivante da una precedente sperimentazione con un UKF, rilevatosi un "overkill".
