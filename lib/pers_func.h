#ifndef pers_func_h
#define pers_func_h

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define FILENAME "data.csv"
#define MAX_LINE_LENGTH 1024
#define MAX_ITEMS 7

int empty_file();
int write_data_in_file(uint16_t id, uint32_t dist, uint8_t qf, int32_t ax, int32_t ay, int32_t az, uint8_t aqf);
int read_data_from_file();

#endif