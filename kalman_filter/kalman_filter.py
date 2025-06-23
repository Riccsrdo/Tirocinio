import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time

# DA RIMUOVERE, UTILE PER TESTING
import matplotlib.pyplot as plt
import random

class KalmanFilterWrapper:
    def __init__(self, distanza_iniziale, r_std, q_std):
        """
        La funzione di inizializzazione del filtro.
        Prende:
            - distanza_iniziale: la distanza inizialmente stimata
            - r_std: deviazione standard del rumore di misura, da calibrare
            - q_std: deviazione standard del rumore di processo, da calibrare
        """

        # Inizializzo il filtro di Kalman
        # prende due parametri:
        #     - dim_x: dimensione dello stato, contiene solo distanza e velocità
        #     - dim_z: dimensione della misura uwb, contiene solo la distanza
        self.kf = KalmanFilter(dim_x=2, dim_z=1)

        # inizializzo lo stato iniziale, dato da [distanza, velocità]
        self.kf.x = np.array([distanza_iniziale, 0.])

        # Inizializzo la matrice di transizione dello stato F
        # viene aggiornata ad ogni passo dato che l'intervallo
        # di misurazione è variabile
        self.kf.F = np.array([[1., 1.], 
                              [0., 1.]])
        
        # Inizializzo la matrice di osservazione H
        self.kf.H = np.array([[1., 0.]])

        # inizializzo la matrice di covarianza del rumore di misura R
        self.kf.R = np.array([[r_std**2]])

        # inizializzo il valore q_std, utile per la matrice di covarianza del rumore di processo Q
        # viene calcolata in base al tempo di campionamento
        self.q_std = q_std

        # inizializzo la matrice di covarianza dello stato P
        # incertezza iniziale è alta per la velocità, bassa per la distanza
        self.kf.P = np.array([[r_std**2, 0.],
                              [0., 10.]])
        
        # Salvo l'ultimo valore del timestamp registrato
        self.timestamp = None
    
    def update(self, misura):
        """
        Funzione di calcolo della distanza filtrata. Da chiamare
        ogni volta che si ha una nuova misura.
        Prende:
            - misura: la misura della distanza, in metri
        """

        current_time = time.monotonic()

        # Controllo che non si tratti della prima misura
        if self.timestamp is None:
            # Se è la prima misura, salvo il timestamp e ritorno la distanza stimata
            # non ci possono essere predict
            self.timestamp = current_time
            return self.kf.x[0], self.kf.x[1]
    
        # calcolo dt sulla base del tempo trascorso
        dt = current_time - self.timestamp
        self.timestamp = current_time

        # aggiorno F e Q in base al dt corretto
        self.kf.F = np.array([[1., dt], [0., 1.]])
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=self.q_std**2)

        # chiamo il predict
        self.kf.predict()

        # e l'update
        self.kf.update(z=np.array([misura]))

        # ritorno i valori dello stato stimati
        distanza_filtrata = self.kf.x[0]
        velocita_filtrata = self.kf.x[1]

        return distanza_filtrata, velocita_filtrata
    

# esempio applicazione
if __name__=="__main__":

    R_STD = 0.15 # DA CALIBRARE
    Q_STD = 5.0 # DA CALIBRARE

    """
    ESEMPIO:

    # Inizializzo la classe
    kf = KalmanFilterWrapper(
        distanza_iniziale=1.0,  # Distanza iniziale stimata
        r_std=R_STD,  # Deviazione standard del rumore di misura
        q_std=Q_STD   # Deviazione standard del rumore di processo
    )

    distanza_filtrata, velocita_filtrata = kf.update(misura=1.2)  # Misura della distanza
    """

    # esempio applicazione
if __name__=="__main__":

    # --- Parametri in CENTIMETRI ---
    # R_STD: Deviazione standard del rumore di misura. 
    # 0.15 metri -> 15 centimetri
    R_STD = 15.0 # DA CALIBRARE

    # Q_STD: Deviazione standard del rumore di processo.
    # Riflette l'incertezza sulla nostra previsione (es. accelerazioni improvvise).
    # Scaliamo anche questo parametro. Un valore tra 1 e 10 cm/s^2 è un buon punto di partenza.
    Q_STD = 5.0 # DA CALIBRARE

    """
    ESEMPIO:

    # Inizializzo la classe (i valori ora sono in cm)
    kf = KalmanFilterWrapper(
        distanza_iniziale=100.0,  # Distanza iniziale stimata in cm
        r_std=R_STD,             # Deviazione standard del rumore di misura in cm
        q_std=Q_STD              # Deviazione standard del rumore di processo
    )

    distanza_filtrata, velocita_filtrata = kf.update(misura=120.0) # Misura in cm
    """


    