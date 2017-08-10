
#include "common.h"

typedef struct GyroData{
    int16_t x;
    int16_t y;
    int16_t z;
}GyroData;

/*global variable*/

void GYRO_init();
void GYRO_update(uint8_t isPrint); 

void GYRO_int1_init();

void GYRO_int_on();
void GYRO_int_off();
