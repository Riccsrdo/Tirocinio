#include "pers_func.h"

/*
Funzione utile allo svuotamento del file CSV.
*/
int empty_file(){
    FILE *fp;
    fp = fopen("data.csv", "w");
    fprintf(fp, "ID,Distance,Quality Factor,Anchor X,Anchor Y,Anchor Z,Anchor Quality Factor\n");
    fclose(fp);
    return 0;
}

/*
Funzione utile alla scrittura di dati di distanziamento in un file CSV, appendendo i dati alla fine.
Prende in input il 
*/
int write_data_in_file(uint16_t id, uint32_t dist, uint8_t qf, int32_t ax, int32_t ay, int32_t az, uint8_t aqf){
    FILE *fp;
    fp = fopen("data.csv", "a");
    fprintf(fp, "%d,%lu,%u,%ld,%ld,%ld,%u\n", id, dist, qf, ax, ay, az, aqf);
    fclose(fp);
    return 0;
}

/*
Funzione utile alla lettura di un file CSV, stampando a video i dati.
*/
int read_data_from_file(){
    FILE *fp;
    fp = fopen("data.csv", "r");
    char line[MAX_LINE_LENGTH];
    while (fgets(line, MAX_LINE_LENGTH, fp)) {
        char *token;
        char *fields[MAX_ITEMS];
        int field_count = 0;

        token = strtok(line, ","); // Divido la riga per virgole
        while (token != NULL && field_count < MAX_ITEMS) {
            fields[field_count++] = token;
            token = strtok(NULL, ",");
        }


        for (int i = 0; i < field_count; i++) {
            printf("Campo %d: %s\n", i, fields[i]);
        }
        printf("\n");
    }
    fclose(fp);
    return 0;
}