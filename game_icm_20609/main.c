#include "my_printf.h"
#include "i2c.h"
#include "gyro.h"
#include "xm.h"

extern XData xData;
extern MData mData;
extern GyroData gData;

volatile static uint8_t int0Happened = 0, int1Happened=0;
static const uint8_t intBased = 0;


static const uint8_t gameOn = 0;
static const uint8_t freeFallOn = 0;


static uint8_t random_number_0_to_3()
{
    return (xData.absSum%4);
}


int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

    // Enable Usage Fault, Bus Fault, and MMU Fault, else it will default to HardFault handler
    //SCB->SHCSR |= 0x00070000; 

    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000); // tick every 1 ms, used by delay_ms()

    init_LED();
    init_blue_push_button();
    init_UART4();
    init_I2C1();

    my_printf("Begin ...\r\n");
    //while(1);
    GYRO_init();
    //XM_init();

    if (freeFallOn) XM_int1_init();
    if (gameOn) GYRO_int1_init();

    uint8_t ledOn = 0;
    while(1) {
        // Light up the LEDS when the user presses the blue button
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
            if (!ledOn) {
                ledOn=1;
                GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
            }



            my_printf("G: [%d %d %d]; X: [%d %d %d]; M[%d %d %d] \r\n", 
                      gData.x,gData.y, gData.z,
                      xData.x,xData.y, xData.z,
                      mData.x,mData.y, mData.z);

            //my_printf("check src 0x%x\r\n", I2C_read_byte(I2C1, SLAVE_ADDR_XM, REG_X_INT1_GEN_SRC));
            delay_ms(500);

    	} 
        else {
            if (ledOn) {
                ledOn=0;
                GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
            }
    	}

        {
            //regular update

            GYRO_update(0);
            delay_ms(10);
            //XM_update(0);
            delay_ms(10);
        }


        //update and printout the g/x/m value every 20ms (50Hz update rate)
        if(!intBased)
        {
            if (xData.absSum<FREE_FALL_TRSH) 
            {

                my_printf("freefall!! %d \r\n", xData.absSum);
                GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
                delay_ms(20);

                GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
            }
        }
        else if (int0Happened) 
        {
            my_printf("freefall!!! interrupted src 0x%x\r\n", I2C_read_byte(I2C1, SLAVE_ADDR_XM, REG_X_INT1_GEN_SRC));
            GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
            delay_ms(20);
            GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
            int0Happened = 0;
        }


        //game
        if (gameOn) 
        {
            uint8_t theNumber = random_number_0_to_3();
            switch (theNumber)
            {
            case 0:
                 GPIO_SetBits(GPIOD, GPIO_Pin_12 );//west
                 my_printf("0\r\n");
                 break;
            case 1:
                 GPIO_SetBits(GPIOD, GPIO_Pin_13 );//north
                 my_printf("1\r\n");
                 break;
            case 2:
                 GPIO_SetBits(GPIOD, GPIO_Pin_14 );//east
                 my_printf("2\r\n");
                 break;
            case 3:
                 GPIO_SetBits(GPIOD, GPIO_Pin_15 );//south
                 my_printf("3\r\n");
                 break;
            default:
                break;

            }
            delay_ms(100);
            GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);

            uint32_t stopage = 0x3FFFFFF;
            //game start!!
            my_printf("game start!! \r\n");
            int1Happened = 0;
            GYRO_int_on();
            while (--stopage)  
            {
                if (int1Happened)
                {
                    GYRO_update(0);
                    uint8_t src = I2C_read_byte(I2C1, SLAVE_ADDR_G, REG_G_INT1_SRC);
                    uint8_t guess = 4;

                    if ((gData.x>0)&&(gData.x>gData.y)) guess = 1;//north
                    else if ((gData.x<0)&&(gData.x<gData.y)) guess = 3;//south
                    else if ((gData.y>0)&&(gData.y>gData.x)) guess =0;//west
                    else if ((gData.y<0)&&(gData.y<gData.x)) guess =2;//east
                    my_printf("int1!!! interrupted src 0x%x %d %d %d guess %d\r\n", src, gData.x, gData.y, gData.z, guess);
                    if (guess == theNumber) 
                    {
                        my_printf("won!!\r\n");
                        GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
                        delay_ms(500);
                        GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
                        delay_ms(500);
                        GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
                        delay_ms(500);
                        GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
                    }
                    int1Happened = 0;

                    break;
                }
            }
            if (!stopage) 
            {

                my_printf("lost!! \r\n");
            }
            GYRO_int_off();
            delay_ms(2000);
        }


        


        

    }
}

/*ISR*/

/* Set interrupt handlers */
/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Do your stuff when PD0 is changed */
        
        int0Happened = 1;
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/* Set interrupt handlers */
/* Handle PD0 interrupt */
void EXTI1_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        /* Do your stuff when PD0 is changed */
        
        int1Happened = 1;
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: my_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

