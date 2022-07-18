#include "stm32f4xx.h"                  // Device header
#include "uart_driver.h"
#include "stdarg.h"
#include "string.h"
#include <stdio.h>
#include "func.h"
 
usart_handle_t uart_handle;
usart_handle_t helmet;

uint8_t message1[] = "ENTER: \n";
uint8_t message2[] = "LED is toggled! \n";
uint8_t message3[] = "invalid operation! \n";

uint8_t rx_arr[32];
uint8_t rx_buffer[10];
int	chk =1;
int num=0;
volatile uint8_t cal[11];
volatile char buf[5];

#define delay  for(int i=0;i<10000;i++)

void active_mode_cmd(void);
void reply(uint8_t*);
void r_read_conc(uint8_t*);
void q_read_conc(void);


	
void rec_msg(uint8_t *msg){
	if(msg[0] == 't'){
      GPIOC->ODR ^= (1<<13);	
		usart_tx(&uart_handle,message2,sizeof(message2)-1);
	}
	else{
		usart_tx(&uart_handle,message3,sizeof(message3)-1);
	}
}	

//This callback will be called by the driver when driver finishes the transmission of data
void app_tx_cmp_callback(void *size)
{
}

void app_tx_cmp_callback_s1(void *size)
{
}
//This callback will be called by the driver when the application receives the command
void app_rx_cmp_callback(void *size)
{
	//we got a command, parse it
	rec_msg(rx_buffer);

}

void app_rx_cmp_callback_s1(void *size)
{
	if(chk ==0){
	reply(rx_arr);
	}
	else{
	r_read_conc(rx_arr);
	}
}


int main(void)
{
	//TIMER
	RCC->APB1ENR|=(1<<0); //RCC_APB1ENR_TIM2EN;
  TIM2->PSC=7;//8400;
  TIM2->ARR= 1999;//65535;
  TIM2->CNT=0;
	TIM2->CR1=1;
	
	//led
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER |=(1<<26);
	GPIOC->ODR = 0;

	//TX using PA9 & PA2
	GPIOA ->MODER |= (1<<19)| (1<<5);
	GPIOA->AFR[0] |= (0x07<<8);
  GPIOA->AFR[1]|= (1<<4)|(1<<5)|(1<<6);
  
	//RX using PA10 & PA3
	GPIOA ->MODER |= (1<<21)|(1<<7);
  GPIOA->AFR[0] |= (0x07<<12);
	GPIOA->AFR[1]|= (0x07<<8); 
	
	// enable clock for USART1 Peripheral
	_RCC_USART1_CLK_ENABLE();
	_RCC_USART2_CLK_ENABLE();
	
	uart_handle.Instance = USART_1;
	
	uart_handle.Init.BaudRate     = USART_BAUD_9600;
	uart_handle.Init.WordLength   = USART_WL_1S8B;
	uart_handle.Init.StopBits     = USART_STOPBITS_1;
	uart_handle.Init.Parity       = USART_PARITY_NONE;
//	uart_handle.Init.Mode         = USART_MODE_TX_RX;
	uart_handle.Init.Mode         = USART_MODE_TX;
	uart_handle.Init.OverSampling = USART_OVER16_ENABLE;
	
	helmet.Instance = USART_2; //for sensor
 
  helmet.Init.BaudRate     = USART_BAUD_9600;
	helmet.Init.WordLength   = USART_WL_1S8B;
	helmet.Init.StopBits     = USART_STOPBITS_1;
	helmet.Init.Parity       = USART_PARITY_NONE;
	helmet.Init.Mode         = USART_MODE_TX_RX;
	helmet.Init.OverSampling = USART_OVER16_ENABLE;
	
	//fill out the application callbasks
	uart_handle.tx_cmp_cb   = app_tx_cmp_callback;
	uart_handle.rx_cmp_cb   = app_rx_cmp_callback;
	
	//for sensor
	helmet.tx_cmp_cb = app_tx_cmp_callback_s1;
	helmet.rx_cmp_cb = app_rx_cmp_callback_s1;
	
	usart_init(&uart_handle);
	usart_init(&helmet);
	
	//enable the IRQ of USART1 Peripheral
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	
/*	while(uart_handle.tx_state != USART_STATE_READY);
	//Send the message*/
	usart_tx(&uart_handle,message1,sizeof(message1)-1);

// while(uart_handle.tx_state != USART_STATE_READY); 
//     active_mode_cmd();
//	 while(helmet.rx_state != USART_STATE_READY);	
//		 usart_rx(&helmet, rx_arr, 11);
		delay;
			q_read_conc();
		
		int n =0;

	while(1)
	{	 

  	while(helmet.rx_state != USART_STATE_READY);
			usart_rx(&helmet, rx_arr,11);
			delay;

		if(n==10){
			usart_tx(&uart_handle," ",sizeof(" ")-1);

				for(int i=0;i<100000;i++);
			usart_tx(&uart_handle,buf,sizeof(buf)-1);
    for(int i=0;i<100000;i++);
			//			delay;
			usart_tx(&uart_handle,"\n",sizeof("\n")-1);
		n=0;
			
	}
		usart_tx(&uart_handle," ",sizeof(" ")-1);
		n++;
	}

}

void USART1_IRQHandler(void){
	
	usart_handle_interrupt(&uart_handle);
}

void USART2_IRQHandler(void){
	
	usart_handle_interrupt(&helmet);
}

void active_mode_cmd(void) 
{
	uint8_t arrsend[9];
	
	arrsend[0] = 0xFF; // make sure to set wordlength to bits
	arrsend[1] = 0x01;
	arrsend[2] = 0x03;
	arrsend[3] = 0x01;
	arrsend[4] = 0x00;
	arrsend[5] = 0x00;
	arrsend[6] = 0x00;
	arrsend[7] = 0x00;
	arrsend[8] = 0x04;

	usart_tx(&helmet,arrsend,sizeof(arrsend)-1);
}

void reply (uint8_t *_buf )
{
		
	uint8_t buff[50];// = _buf;
	intToStr(*_buf,buff,4);
	usart_tx(&uart_handle,buff,4);
	
	cal[num] = *_buf;
  num++;
		if ((num == 10)&&((cal[2]+cal[3]+cal[4]+cal[5])==cal[sizeof(cal)-2])){
		  						
			num=0;
			
	if((cal[3] == 0x01)){
		usart_tx(&uart_handle,"\nactive mode ",sizeof("\nactive mode ")-1);
	}
}
	chk =1;

}

void q_read_conc(void){
	
	uint8_t arrsend[9];
	arrsend[0] = 0xFF; 
	arrsend[1] = 0x01;
	arrsend[2] = 0x07;
	arrsend[3] = 0x00;
	arrsend[4] = 0x00;
	arrsend[5] = 0x00;
	arrsend[6] = 0x00;
	arrsend[7] = 0x00;
	arrsend[8] = 0x07;
	usart_tx(&helmet,arrsend,sizeof(arrsend)-1);
}

void r_read_conc(uint8_t *_buf)
{

	uint8_t buff[50];// = _buf;
	intToStr(*_buf,buff,4);
	usart_tx(&uart_handle,buff,4);
	
	cal[num] = *_buf;
  num++;
		if ((num == 10)&&((cal[2]+cal[3]+cal[4]+cal[5])==cal[sizeof(cal)-2])){
		  						
			num=0;

	if(cal[3] == 0x01)//0x01 = 0.1 resolution
	{
    int gas_conc = (cal[4]*256)+cal[5];
	  float gas_Conc =gas_conc*0.1;
    ftoa(gas_Conc, buf, 2);

	}
	else if(cal[3] == 0x00)//0x00 = 1 resolution
	{
		int gas_conc = (cal[4]*256)+cal[5];
	  float gas_Conc =gas_conc*1;
    ftoa(gas_Conc, buf, 2);
		
	}
	else if(cal[3] == 0x02)//0x02 = 0.01 resolution
	{
	int gas_conc = (cal[4]*256)+cal[5];
	  float gas_Conc =gas_conc*0.01;
    ftoa(gas_Conc, buf, 3);
		
	}
}
	//chk =0;
}

