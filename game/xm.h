
#include "common.h"

typedef struct XData{
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t absSum;
}XData;

typedef struct MData{
    int16_t x;
    int16_t y;
    int16_t z;
}MData;
/*global variable*/

void XM_init();

void XM_int1_init();
void XM_update(uint8_t isPrint);
