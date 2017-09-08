#include "xm.h"
#include "i2c.h"
#include "my_printf.h"

/* local function declaration */


static inline void XM_cs();

/* this file contains everything about accelerator/meg */
static inline void XM_cs() 
{
    GPIO_ResetBits(GPIOB, GPIO_I2C_CSG); 
    GPIO_SetBits(GPIOB, GPIO_I2C_CSXM);

    delay_ms(1);

}


void XM_int1_init()
{
     XM_cs() ;
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
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
    
    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
 
    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);


    //X: enable interrupt 1, 
    I2C_write_byte(I2C1, SLAVE_ADDR_XM, REG_X_INT1_GEN_REG, 0x95);

    I2C_write_byte(I2C1, SLAVE_ADDR_XM, REG_X_INT1_GEN_THS, 0x7);
    my_printf("xm int1 is turned on reg 0x%x ths 0x%x\r\n",
              I2C_read_byte(I2C1, SLAVE_ADDR_XM, REG_X_INT1_GEN_REG), 
              I2C_read_byte(I2C1, SLAVE_ADDR_XM, REG_X_INT1_GEN_THS));

}

/* global variable */

XData xData;
MData mData;

void XM_init()
{

    XM_cs();


    while(I2C_read_byte(I2C1, SLAVE_ADDR_XM, REG_WHO_AM_I) != WHO_AM_I_XM)
    {
        //err handling
        my_printf("ERR!! xm failed to be selected \r\n");
    }

    //X: select x, y, z axis, and use 400hz (every 2.5ms); +/-4g
    //M: select sampling rate 25Hz, +/-4 gauss, countinues mode 
    uint8_t cfgBuf[8] = {0x0, 0x87, 0x08, 0x20, 0x0, 
                        0x0C, 0x20, 0x0};

    I2C_write_buffer(I2C1, SLAVE_ADDR_XM, REG_XM_CTRL1, cfgBuf, 8);
    I2C_read_buffer(I2C1, SLAVE_ADDR_XM, REG_XM_CTRL1, cfgBuf, 8);
    my_printf("xm is configured 0x%x, 0x%x, 0x%x, 0x%x, 0x%x,  0x%x, 0x%x, 0x%x \r\n",
              cfgBuf[0],cfgBuf[1],cfgBuf[2],cfgBuf[3],cfgBuf[4],cfgBuf[5],cfgBuf[6],cfgBuf[7]);
    


    xData.x = 0; 
    xData.y = 0;
    xData.z = 0;
    xData.absSum =0;


    mData.x = 0; 
    mData.y = 0;
    mData.z = 0;
 
}


void XM_update(uint8_t isPrint)
{
    XM_cs();

    uint8_t dataBuf[6] = {0};
    //grab the accelerator data from the registers : X Y Z    
    I2C_read_buffer(I2C1, SLAVE_ADDR_XM, REG_X_OUT_X_L, dataBuf, 6);

    xData.x = dataBuf[1]<<8|dataBuf[0];
    xData.y = dataBuf[3]<<8|dataBuf[2];
    xData.z = dataBuf[5]<<8|dataBuf[4];
    xData.absSum = abs(xData.x)+abs(xData.y)+abs(xData.z);

    delay_ms(1);
    //grab the meg data from the registers : X Y Z    
    I2C_read_buffer(I2C1, SLAVE_ADDR_XM, REG_M_OUT_X_L, dataBuf, 6);

    mData.x = dataBuf[1]<<8|dataBuf[0];
    mData.y = dataBuf[3]<<8|dataBuf[2];
    mData.z = dataBuf[5]<<8|dataBuf[4];




    if(isPrint) my_printf("accel_update: x %d, y %d, z %d absSum %d\r\n", xData.x, xData.y, xData.z, xData.absSum);
    if(isPrint) my_printf("meg_update: x %d, y %d, z %d \r\n", mData.x, mData.y, mData.z);
}

