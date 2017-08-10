#include "common.h"


static __IO uint32_t g_timing_delay;
__IO uint32_t g_ticks = 0; // increments every millisecond

void init_LED()
{
    GPIO_InitTypeDef gpio; // LEDS on GPIOD

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15;
	gpio.GPIO_Mode  = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOD, &gpio);
}

void init_blue_push_button()
{
    GPIO_InitTypeDef gpio; // push button on GPIOA

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	gpio.GPIO_Pin   = GPIO_Pin_0;
	gpio.GPIO_Mode  = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOA, &gpio);
}

void init_UART4()
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
 
    /* Configure USART Tx as alternate function  */
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* USART configuration */
    USART_Init(UART4, &usart);

    /* Enable USART */
    USART_Cmd(UART4, ENABLE);
}

void delay_ms(uint32_t t)
{
    g_timing_delay = t;

    while (g_timing_delay != 0);
}

void timing_delay_decrement(void)
{
    if (g_timing_delay != 0x00) { 
        g_timing_delay--;
    }
}

uint32_t get_ticks()
{
    return g_ticks;
}


int32_t abs(int32_t ip)
{
    return (ip>=0)?ip:-ip;
}
