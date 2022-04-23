//
// Created by Kevin Williams on 4/22/22.
//

#ifndef PICO_SPIDER_PRINTUTILS_H
#define PICO_SPIDER_PRINTUTILS_H
#include <stdio.h>

void tblPrint(float * data, int cols, int rows) {
    float * p = data;
    for(int i=0; i < rows; i++) {
        printf("\n");
        for(int j=0; j < cols; j++) {
            printf("%f\t", *p++);
        }
    }
    printf("\n");
}
#endif //PICO_SPIDER_PRINTUTILS_H
