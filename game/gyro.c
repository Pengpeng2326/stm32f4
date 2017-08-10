#include "gyro.h"
#include "i2c.h"
#include "my_printf.h"

/* this file contains everything about gyro */
static inline void GYRO_cs() 
{
    GPIO_SetBits(GPIOB, GPIO_I2C_CSG); 
    GPIO_ResetBits(GPIOB, GPIO_I2C_CSXM);

    delay_ms(1);

}

/* global variable */

GyroData gData;

void GYRO_init()
{

    GYRO_cs();


    while(I2C_read_byte(I2C1, SLAVE_ADDR_G, REG_WHO_AM_I) != WHO_AM_I_G)
    {
        //Err handler
        my_printf("ERR!! gyro failed to be selected \r\n");
    }



    //select x, y, z axis, and use 500dps (every 2ms)
    uint8_t cfgBuf[5] = {0xF, 0x0, 0x00, 0x10, 0x0};

    I2C_write_buffer(I2C1, SLAVE_ADDR_G, REG_G_CTRL1, cfgBuf, 5);
    I2C_read_buffer(I2C1, SLAVE_ADDR_G, REG_G_CTRL1, cfgBuf, 5);
    my_printf("gyro is configured 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \r\n",
              cfgBuf[0],cfgBuf[1],cfgBuf[2],cfgBuf[3],cfgBuf[4]);

    gData.x = 0; 
    gData.y = 0;
    gData.z = 0;
    
}
void GYRO_int_on()
{
    GYRO_cs();
    I2C_write_byte(I2C1, SLAVE_ADDR_G,REG_G_CTRL3, 0x80);
}

void GYRO_int_off()
{
    GYRO_cs();
    I2C_write_byte(I2C1, SLAVE_ADDR_G,REG_G_CTRL3, 0x00);
}
void GYRO_update(uint8_t isPrint)
{
    GYRO_cs();

    uint8_t dataBuf[6] = {0};
    //grab the data from the registers : X Y Z    
    I2C_read_buffer(I2C1, SLAVE_ADDR_G, REG_G_OUT_X_L, dataBuf, 6);

    gData.x = dataBuf[1]<<8|dataBuf[0];
    gData.y = dataBuf[3]<<8|dataBuf[2];
    gData.z = dataBuf[5]<<8|dataBuf[4];

    if(isPrint) my_printf("gyro_update: x %d, y %d, z %d \r\n", gData.x, gData.y, gData.z);
}



void GYRO_int1_init()
{

    GYRO_cs();
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /* Tell system that you will use PD1 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
    
    /* PD1 is connected to EXTI_Line1 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
 
    /* Add IRQ vector to NVIC */
    /* PD1 is connected to EXTI_Line1, which has EXTI1_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);


    //X: enable interrupt 1, 
    I2C_write_byte(I2C1, SLAVE_ADDR_G, REG_G_INT1_CFG, 0xAA);

    uint8_t cfgBuf[6] = {0x04,0x00,0x4,0x00,0x4,0x00};
    
    I2C_write_buffer(I2C1, SLAVE_ADDR_G, REG_G_INT1_THS_XH, cfgBuf, 6);
    my_printf("gyro int1 is turned on reg 0x%x \r\n", 
              I2C_read_byte(I2C1, SLAVE_ADDR_G, REG_G_INT1_CFG));

}

