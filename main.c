#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

int16_t received_i2c_data = 0;
uint16_t adc_value = 0;
uint16_t scaled_adc_value = 0;
uint16_t received_usart_data = 0;


void myDelay(uint32_t time) //It is for delaying
{
	for(uint32_t i = 0; i < time *6250; i++);
}

void LEDS_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable clock for GPIOD */
	//***********
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//***********

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;						 //GPIOD Pin 1 select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_OUT;        //GPIOD Pin 1 output
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;				 //GPIOD Pin 1 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			 //GPIOD Pin 1 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;    //GPIOD Pin 1  0 very high speed

	GPIO_Init(GPIOD,&GPIO_InitStruct);

}

void ADC_Config()
{
	GPIO_InitTypeDef GPIO_InitStruct;                            //GPIO init struct defined for pin.
	ADC_InitTypeDef ADC_InitStruct;                              //ADC init struct defined.
	ADC_CommonInitTypeDef ADC_CommonInitStruct;                  //ADC common init struct defined.

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);         //Port A clock opened, I used Port A pin 0.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);          //ADC clock opened.


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;                    //For adc usage, analog mode chosen.
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;                       //Pin 0 chosen.
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                  //Output type set as push pull.
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;                // No pull configuration set.
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;                // 2Mhz speed chosen.
    GPIO_Init(GPIOA, &GPIO_InitStruct);                          //Load the configuation

	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;        //Independent mode chosen, (we are going to read only 1 channel).
	ADC_CommonInitStruct.ADC_Prescaler =  ADC_Prescaler_Div6;    //Div 6 chosen for clock, APB2 line divided to 6.

    ADC_CommonInit(&ADC_CommonInitStruct);                       //Load the configuration

	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;          //Resolution set 12 bit.
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;              //For contiunuous analog read it's set to enable.
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;   //We don't need a trigger so none selected
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;          //Data aligned to left.

	ADC_DeInit();                                                 //ADC_de init.

	ADC_Init(ADC1,&ADC_InitStruct);                              //Configuration loaded

	ADC_Cmd(ADC1,ENABLE);                                        //ADC start command.

}

int read_ADC()
{
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_3Cycles);  //Regular channel configuration for ADC readings.

	ADC_SoftwareStartConv(ADC1);                                //ADC conversation start command.

	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC == RESET));      //Waiting for ADC flag.

	return ADC_GetConversionValue(ADC1);                       //Returning ADC value

}

void CONFIG_USART3()
{

	    GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);


		USART_InitStructure.USART_BaudRate = 19200;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_Init(USART3, &USART_InitStructure);

		USART_Cmd(USART3, ENABLE);
}


void Config_I2C()
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 I2C_InitTypeDef I2C_InitStructure;

 RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_Init(GPIOB, &GPIO_InitStructure);


 GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);


 I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
 I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
 I2C_InitStructure.I2C_OwnAddress1 = 0x00;
 I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
 I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
 I2C_InitStructure.I2C_ClockSpeed = 100000;

 I2C_Init(I2C1, &I2C_InitStructure);

//And start the I2C
 I2C_Cmd(I2C1, ENABLE);


}

uint8_t LM75_ReadConf(void) {
	uint8_t value;

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,0x01); // Send register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = I2C_ReceiveData(I2C1);

	return value;
}

void LM75_WriteConf(uint8_t value) {
	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,0x01); // Send register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C1,value);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTOP(I2C1,ENABLE);
}

void LM75_Shutdown(FunctionalState newstate) {
	uint8_t value;

	value = LM75_ReadConf();
	LM75_WriteConf(newstate == ENABLE ? value | 0x01 : value & 0xFE);
}



uint16_t Read_I2C_reg(uint8_t reg){

	uint16_t data;

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, 0x90, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,reg); // Send register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	data = (I2C_ReceiveData(I2C1) << 8); // Receive high byte
	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	data |= I2C_ReceiveData(I2C1); // Receive low byte

    return data;

}

int16_t LM75_Temperature(void) {
	uint16_t raw;
	int16_t temp;

	raw = Read_I2C_reg(0x00) >> 7;
	if (raw & 0x0100) {
		// Negative temperature
		temp = -10 * (((~(uint8_t)(raw & 0xFE) + 1) & 0x7F) >> 1) - (raw & 0x01) * 5;
	} else {
		// Positive temperature
		temp = ((raw & 0xFE) >> 1) * 10 + (raw & 0x01) * 5;
	}

	return temp;
}

int main()
{
	/* Init leds */
	LEDS_Init();

	/* Init ADC */
	ADC_Config();

	/*Init USART */
	CONFIG_USART3();

	/*Init I2C */
	Config_I2C();

	 LM75_Shutdown(DISABLE);

	while(1)
	{

	received_i2c_data = LM75_Temperature()/10;

    adc_value = read_ADC();
	scaled_adc_value = (0.01221)*adc_value;

    USART_SendData(USART3,received_i2c_data);

	received_usart_data = USART_ReceiveData(USART3);

	if(received_usart_data == '1' && received_i2c_data > scaled_adc_value)
	{
			GPIO_SetBits(GPIOD,GPIO_Pin_12);
	}

	if(received_usart_data == '0' || received_i2c_data <= scaled_adc_value)
	{
			GPIO_ResetBits(GPIOD,GPIO_Pin_12);
	}

	myDelay(1000);

	}


}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
