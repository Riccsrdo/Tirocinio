#include "dwm_func.h"
#include "quick_func.h"

/*
Per compilare digitare:
gcc -o main main.c dwm_func.c -pthread
*/

void* measure_distances(void* arg){
    uint32_t *distances = NULL; // Vettore delle distanze che verrà aggiornato da dwm_loc_get

    for(int i=0; i<10; i++){
        dwm_loc_get(&distances);
        usleep(1200000); // attendo 1,2 sec tra ogni misurazione

    }

    if(distances != NULL){ // libero la memoria se il vettore è stato allocato correttamente
        free(distances);
    }
}

int main() {
    if (spi_init() < 0) return -1;

    ensure_device_in_idle_state();
    
    read_status();
    usleep(10000);

    //dwm_upd_rate_set(4, 9);

    //Threads
    pthread_t thread_misurazioni;
    int rv;
    rv = pthread_create(&thread_misurazioni, NULL, measure_distances, NULL);
    if(rv){
        printf("Errore nella creazione del thread\n");
        return -1;
    }
    pthread_join(thread_misurazioni, NULL);
    

    //set_as_tag();

    spi_close();
    return 0;
}