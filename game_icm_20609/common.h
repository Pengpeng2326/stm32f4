
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

#include "stm32f4xx_i2c.h"
//ICM-20609 address/regs
#define         SLAVE_ADDR              (0x68<<1)
#define		SLAVE_ADDR_G		(0x68<<1)
#define		SLAVE_ADDR_XM		(0x3A)
//General
#define		REG_WHO_AM_I	    (0x75)
#define         REG_SMPLRT_DIV      (0x19)
#define         REG_CONFIG          (0x1A)
#define         REG_USER_CTRL       (0x6A)
#define         REG_PWR_MGMT_1      (0x6B)
#define         REG_GYRO_CFG        (0x1B)
#define         REG_ACC_CFG         (0x1C)
#define         REG_ACC_CFG2        (0x1D)
#define         REG_ACC_MEAS        (0x3B)
//Gyro
#define     REG_G_CTRL1         (0x20)
#define     REG_G_CTRL3         (0x22)
#define     REG_G_OUT_X_L       (0x28)
#define     REG_G_INT1_CFG      (0x30)
#define     REG_G_INT1_SRC      (0x31)
#define     REG_G_INT1_THS_XH   (0x32)
//X and M
#define     REG_XM_CTRL1        (0x1F)       
#define     REG_X_OUT_X_L       (0x28)
#define     REG_M_OUT_X_L       (0x08)
#define     REG_X_INT1_GEN_REG   (0x30)
#define     REG_X_INT1_GEN_SRC   (0x31)
#define     REG_X_INT1_GEN_THS   (0x32)

#define     WHO_AM_I_G          (0xA6)
#define     WHO_AM_I_XM         (0x49)

#define     FREE_FALL_TRSH      (5000)
/* global variables */


void timing_delay_decrement(void);
void delay_ms(uint32_t t);
void init_UART4();
void init_LED();
void init_blue_push_button();
uint32_t get_ticks();
int32_t abs(int32_t ip);
