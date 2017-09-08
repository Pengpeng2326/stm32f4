#include "i2c.h"


static void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);

static uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
static uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);

static void I2C_start(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t txRx);

void init_I2C1(void)
{
	
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /*init SCL and SDA pins */
    {
        GPIO_InitTypeDef GPIO_1;
        GPIO_1.GPIO_Pin = GPIO_I2C_SCL | GPIO_I2C_SDA; // we are going to use PB6 and PB7
        GPIO_1.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
        GPIO_1.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
        GPIO_1.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
        GPIO_1.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
        GPIO_Init(GPIOB, &GPIO_1);					// init GPIOB
        
        
        // Connect I2C1 pins to AF  
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
    }

    /*init XM and G CS pins */
    {
        GPIO_InitTypeDef GPIO_2;
        GPIO_2.GPIO_Pin = GPIO_I2C_CSG|GPIO_I2C_CSXM;
        GPIO_2.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_2.GPIO_OType = GPIO_OType_PP;
        GPIO_2.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_2.GPIO_Speed = GPIO_Speed_100MHz;

        GPIO_Init(GPIOB, &GPIO_2); 
    }


	// configure I2C1 
    {
        I2C_InitTypeDef I2C_InitStruct;
        I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
        I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
        I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
        I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
        I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
        I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
        I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
        
        // enable I2C1
        I2C_Cmd(I2C1, ENABLE);
    }
	
}

static void I2C_start(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t txRx)
{
    // Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	
    //tx mode
    if (txRx == I2C_Direction_Transmitter) 
    {
        // Send slave Address for write
        I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter);
            
        // wait for Slave acknowledge
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    //rx mode
    else
    {

        // Send slave Address for read 
        I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);

        // wait for Slave acknowledge
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    }

}

static void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}


static uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

static uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This is to receive one byte with ack from master */
uint8_t I2C_read_byte(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg)
{
  
	// waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    //step 1 send SAD+W
    I2C_start(I2Cx, addr, I2C_Direction_Transmitter);

	// Step 2, Send SUB slave register
    I2C_write(I2Cx, reg);
	
    //step 3 send SAD+R
    I2C_start(I2Cx, addr, I2C_Direction_Receiver);

	// receive the first byte
	uint8_t val = I2C_read_nack(I2Cx);

    //generate stop flag
    I2C_GenerateSTOP(I2Cx, ENABLE);

	// waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	return val;	
}


/* This is to receive multiples bytes with ack from master */
void I2C_read_buffer(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t* buff, uint16_t len)
{
    //nothing to do...
    if (len==0) 
        return;

	// waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

   //step 1 send SAD+W
    I2C_start(I2Cx, addr, I2C_Direction_Transmitter);

	// Step 2, Send SUB slave register
    I2C_write(I2Cx, 0x80|reg);
	
    //step 3 send SAD+R
    I2C_start(I2Cx, addr, I2C_Direction_Receiver);

	// receive the length-1 of data
    while (--len) {
        *buff++ = I2C_read_ack(I2Cx);
    }

    //receive the last data
    *buff = I2C_read_nack(I2Cx);

    //generate stop flag
	I2C_GenerateSTOP(I2Cx, ENABLE);

	// waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

}




/* This is to transmit one byte with ack from master */
void I2C_write_byte(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t val)
{
	// waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    //step 1 send SAD+W
    I2C_start(I2Cx, addr, I2C_Direction_Transmitter);

	// Step 2, Send SUB slave register
    I2C_write(I2Cx, reg);

	// Step 3, Send Data to program the register
    I2C_write(I2Cx, val);

    //generate stop flag
    I2C_GenerateSTOP(I2Cx, ENABLE);

	// waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
}

/* This is to transmit multiple byte with ack from master */
void I2C_write_buffer(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t* buff, uint16_t len)
{
    if (len==0) return;

    // waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    //step 1 send SAD+W
    I2C_start(I2Cx, addr, I2C_Direction_Transmitter);

	// Step 2, Send SUB slave register
    I2C_write(I2Cx, 0x80|reg);

	// Step 3, Send Data to program the register
    while (len--) {
        I2C_write(I2Cx, *buff++);
    }

    //generate stop flag
    I2C_GenerateSTOP(I2Cx, ENABLE);

	// waith while line is busy
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
}
/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

		
