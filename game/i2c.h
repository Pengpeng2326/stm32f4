#include "common.h"


//I2C pin assignment
#define 	GPIO_I2C_SCL	(GPIO_Pin_6)
#define 	GPIO_I2C_SDA	(GPIO_Pin_7)
#define 	GPIO_I2C_CSG	(GPIO_Pin_4)
#define		GPIO_I2C_CSXM	(GPIO_Pin_5)

//global variables
void init_I2C1(void);
void I2C_stop(I2C_TypeDef* I2Cx);

uint8_t I2C_read_byte(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg);

void I2C_write_byte(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t val);
void I2C_read_buffer(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t* buff, uint16_t len);
void I2C_write_buffer(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t* buff, uint16_t len);
