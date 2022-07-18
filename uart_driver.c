#include "uart_driver.h"
#include "stdarg.h"
#include "string.h"

/*****************************************************************************************/
/*                                                                                       */
/*                              HELPER FUNCTIONS                                         */
/*                                                                                       */
/*****************************************************************************************/

//uint8_t message2_[] = "LED is toggled! \n";
//uint8_t message3_[] = "invalid operation! \n";
usart_handle_t _uart_handle;

 uint8_t print_format(char *format,...){
	
		char buff[80];
		
		/*Extrating the argument list using VS apis*/
		va_list args;
		va_start(args,format);
		vsprintf(buff,format,args);
	  //return *buff;
	 usart_tx(&_uart_handle,buff,sizeof(buff)-1);
}

/**
 *@brief Enable the given USART peripheral
 *@param *usartx: base address of the USART peripheral
 *@retval None
 */
 void usart_enable(USART_TypeDef *usartx)
 {
	 usartx->CR1 |= USART_REG_CR1_USART_EN;
 }
	 
/**
 *@brief Disable the given USART peripheral
 *@param *usartx: base address of the USART peripheral
 *@retval None
 */
 void usart_disable(USART_TypeDef *usartx)
 {
	 usartx->CR1 &= ~USART_REG_CR1_USART_EN;
 }

 /**
 *@brief Enable/Disable the Transmit Block the given USART peripheral
 *@param *usartx: base address of the USART peripheral
 *@param  te     : if te=1, then enable the Transmit block.
 *@retval None
 */
 void usart_enable_disable_tx(USART_TypeDef *usartx, uint32_t te)
 {
	 if (te & USART_REG_CR1_TE)
	 {
		 usartx->CR1 |= USART_REG_CR1_TE;
	 }
	 else
	 {
		 usartx->CR1 &= ~USART_REG_CR1_TE;
	 }
 }
 
  /**
 *@brief Enable/Disable the Receive Block the given USART peripheral
 *@param *usartx: base address of the USART peripheral
 *@param  re     : if re=1, then enable the Receive block.
 *@retval None
 */
 void usart_enable_disable_rx(USART_TypeDef *usartx, uint32_t re)
 {
	 if (re & USART_REG_CR1_RE)
	 {
		 usartx->CR1 |= USART_REG_CR1_RE;
	 }
	 else
	 {
		 usartx->CR1 &= ~USART_REG_CR1_RE;
	 }
 }
 
  /**
 *@brief Configures the wors length for data transmission and reception
 *@param *usartx: base address of the USART peripheral
 *@param  wl     : if wl=1, then word length is 1s, 9 bits, nstop.
 *@retval None
 */
 void usart_config_word_length(USART_TypeDef *usartx, uint32_t wl)
 {
	 if(wl)
	 {
		 usartx->CR1 |= USART_REG_CR1_USART_WL;
	 }
	 else
	 {
		 usartx->CR1 &= ~USART_REG_CR1_USART_WL;
	 }
 }		 

 /**
 *@brief Configures the stop bits for data transmission and reception
 *@param *usartx: base address of the USART peripheral
 *@param  nstop : This value configures the stop bits
 *@retval None
 */
 void usart_config_stop_bits(USART_TypeDef *usartx, uint32_t nstop)
 {
	 usartx->CR2 &= ~USART_STOPBITS_1NHALF; //clearing stop bits
	 
	 if(nstop == USART_STOPBITS_HALF)       //0.5 stop bits
	 {
		 usartx->CR2 |= USART_STOPBITS_HALF;
	 }
	 else if(nstop == USART_STOPBITS_2)      //2 stop bits
	 {
		 usartx->CR2 |= USART_STOPBITS_2;
	 }
	 else if(nstop == USART_STOPBITS_1NHALF) //1.5 stop bits
	 {
		 usartx->CR2 |= USART_STOPBITS_1NHALF;
	 }
	 else{                                   //1 stop bit
		 usartx->CR2 |= USART_STOPBITS_1;
	 }
 }

 /**
 *@brief Configures te oversampling rate of USART Peripheral 
 *@param *usartx: base address of the USART peripheral
 *@param  over8 : if over8=1, then oversampling bt 8 will be used, otherwise default oversampling is by 16
 *@retval None
 */
 void usart_config_over_sampling(USART_TypeDef *usartx, uint32_t over8) 
 {
	 if (over8)
	 {
		 usartx-> CR1 |= USART_REG_CR1_OVER8;
	 }
 }
 
 
 
 /**
 *@brief Program the given Baud Rate 
 *@param *usartx: base address of the USART peripheral
 *@param  baud : baud rate value to be programed
 *@retval None
 */
 void usart_set_baud_rate(USART_TypeDef *usartx, uint32_t baud) 
 {
	 uint32_t val;
	 if (baud == USART_BAUD_9600)
	 {
		 val = 0x683;
	 }
	 else if ( val == USART_BAUD_115200)
	 {
		 val = 0x8B;
	 }
	 else{
		 val = 0x8B;
	 }
	 usartx->BRR = val;
 }
 
 
/**
 *@brief Enable/Disable the TXE interrupt
 *@param *usartx: base address of the USART peripheral
 *@param  txe_en : if txe_en=1, then enable the TXE interupt
 *@retval None
 */
 void usart_config_txe_interrupt(USART_TypeDef *usartx, uint32_t txe_en) 
 {
	 if(txe_en)
	 {
		 usartx->CR1 |= USART_REG_CR1_TXEIE;
	 }
	 else
	 {
		 usartx->CR1 &= ~USART_REG_CR1_TXEIE;
	 }
 }

/**
 *@brief Enable/Disable the RXNE interrupt
 *@param *usartx: base address of the USART peripheral
 *@param  rxne_en : if rxne_en=1, then enable the RXNE interupt
 *@retval None
 */
 void usart_config_rxne_interrupt(USART_TypeDef *usartx, uint32_t rxne_en) 
 {
	 if(rxne_en)
	 {
		 usartx->CR1 |= USART_REG_CR1_RXNEIE;
	 }
	 else
	 {
		 usartx->CR1 &= ~USART_REG_CR1_RXNEIE;
	 }
 }

/**
 *@brief Enable/Disable the ERROE interrupt
 *@param *usartx: base address of the USART peripheral
 *@param  er_en : if er_en=1, then enable the ERROR interupt
 *@retval None
 */
 void usart_config_error_interrupt(USART_TypeDef *usartx, uint32_t er_en) 
 {
	 if(er_en)
	 {
		 usartx->CR3 |= USART_REG_CR3_EIE;
	 }
	 else
	 {
		 usartx->CR3 &= ~USART_REG_CR3_EIE;
	 }
 }
 
 /**
 *@brief Enable/Disable the Parity Error interrupt
 *@param *usartx: base address of the USART peripheral
 *@param  pe_en : if pe_en=1, then enable the PARITY ERROR interrupt
 *@retval None
 */
 void usart_config_parity_error_interrupt(USART_TypeDef *usartx, uint32_t pe_en) 
 {
	 if(pe_en)
	 {
		 usartx->CR1 |= USART_REG_CR1_PEIE;
	 }
	 else
	 {
		 usartx->CR1 &= ~USART_REG_CR1_PEIE;
	 }
 }
 
  /**
  *@brief  Handle the TXE interrupt
  *@param husart : pointer to the handle structure that contains the confiduration information for
	*        the specified USART Module
  *@retval None
  */
 static void usart_handle_TXE_interrupt(usart_handle_t *husart)
 {
	 uint32_t tmp1 = 0;
	 uint8_t val =0;
	 
	 tmp1 = husart->tx_state;
	 
	 if(tmp1 == USART_STATE_BUSY_TX)
	 {
		 val = (uint8_t)(*husart->pTxBuffPtr++ & (uint8_t)0x00FF);
			husart->Instance->DR = val;
		 
		 if(--husart->TxXferCount == 0)
		 {
			 val = 0;
			 /*Disable the USART TXE Interrupt*/
			 husart->Instance->CR1 &= ~USART_REG_CR1_TXEIE;
			 
			 /*Enable the USART Transmit Complete Interrupt*/
      husart->Instance->CR1 |= USART_REG_CR1_TCIE;
			// husart->tx_state = USART_STATE_READY;
		 }
	 }
 }
 
/**
  *@brief  Handle the TC interrupt
  *@param husart : pointer to the handle structure that contains the confiduration information for
	*        the specified USART Module
  *@retval void
  */
 static void usart_handle_TC_interrupt(usart_handle_t *husart)
 {
 /*Disable the USART Transmit Complete Interrrupt*/
	 husart->Instance->CR1 &= ~USART_REG_CR1_TCIE;
	 husart->tx_state = USART_STATE_READY;
	 /* call the application callback */
	 if(husart->tx_cmp_cb){
		 husart->tx_cmp_cb(&husart->TxXferSize);
	 }
 }  

  /**
  *@brief  Handle the RXNE interrupt
  *@param husart : pointer to the handle structure that contains the confiduration information for
	*        the specified USART Module
  *@retval None
  */
 static void usart_handle_RXNE_interrupt(usart_handle_t *husart)
 {
	 uint32_t tmp1 = 0;
	 
	 tmp1 = husart->rx_state;
	 
	 if(tmp1 == USART_STATE_BUSY_RX)
	 { 
		 // is application using parity
		if(husart->Init.Parity == USART_PARITY_NONE)
		{
			//no parity
			*husart->pRxBuffPtr++ = (uint8_t)(husart->Instance->DR & (uint8_t)0x00FF);
			
		
		}
		else{
			
			//yes, do not read the most significant bit, because its a parity bit
			*husart->pRxBuffPtr++ = (uint8_t)(husart->Instance->DR & (uint8_t)0x007F);
	 }

   		// Is reception done? 
		 if(--husart->RxXferCount == 0)
		 {
			 /*Yes, Disable the USART TXE Interrupt*/
			 husart->Instance->CR1 &= ~USART_REG_CR1_RXNEIE;
			 
			 /*Disable the USART Parity Error Interrupt*/
			 husart->Instance->CR1 &= ~USART_REG_CR1_PEIE;
			 
			 /*Disable the USART Parity Error Interrupt*/
			 husart->Instance->CR3 &= ~USART_REG_CR3_EIE;
			 
			 //make the state ready for this handle
			 husart->rx_state = USART_STATE_READY;
			 
			 /*call the application callback*/
			 if(husart->rx_cmp_cb){
				 husart->rx_cmp_cb(&husart->RxXferSize);
			 }
		 }
	 }
 }
 
 /**
  *@brief  clear error flag
  *@param *handle : pointer to the handle structure that contains the confiduration information for
	*        the specified USART Module
  *@retval None
  */
 
 void usart_clear_error_flag(usart_handle_t *husart)
 {
	 uint32_t tmpreg = 0x00;
	 tmpreg = husart->Instance->SR;
	 tmpreg = husart->Instance->DR;
 }
 
 /**
  *@brief USART error callback
  *@param *handle : pointer to the handle structure that contains the confiduration information for
	*        the specified USART Module
  *@retval None
  */
 
 void usart_error_cd(usart_handle_t *handle)
 {
	 print_format("ERROR OCCURED IN RECEPTION");
	 //while(1){}
	 }
 

/*****************************************************************************************/
/*                                                                                       */
/*                              USART DRIVER APIS                                        */
/*                                                                                       */
/*****************************************************************************************/

/**
  *@brief API to USART initialization
  *@param *handle : pointer to the handle structure of the USART peripheral
  *@retval None
  */
void usart_init(usart_handle_t *usart_handle){
	
	/* Configure the Word Length */
	usart_config_word_length(usart_handle->Instance, usart_handle->Init.WordLength);

	/* Configure the number of STOP Bits */
  usart_config_stop_bits(usart_handle->Instance, usart_handle->Init.StopBits);

	/* Configurethe oversampling rate for the receive block */
	usart_config_over_sampling(usart_handle->Instance, usart_handle->Init.OverSampling);

	/* Set the Baud Rate */
  usart_set_baud_rate(usart_handle->Instance, usart_handle->Init.BaudRate);
	
	/* Enable the Transmission Block of USART peripheral */
	usart_enable_disable_tx(usart_handle->Instance, usart_handle->Init.Mode);
	
	/* Enable the Reception Block of USART Peripieral */
	usart_enable_disable_rx(usart_handle->Instance, usart_handle->Init.Mode);
	
	/* Enable the USART Peripheral */
	usart_enable(usart_handle->Instance);
	
	
	usart_handle->tx_state = USART_STATE_READY;
	usart_handle->rx_state = USART_STATE_READY;
	usart_handle->ErrorCode = USART_ERROR_NONE;	
	
}



/**
  *@brief API to USART data transmission
  *@param *handle : pointer to the handle structure of the USART peripheral
  *@param *buffer : holds the pointer to Tx buffer
  *@param  len    : length of data to be TXed
  *@retval None
  */

void usart_tx(usart_handle_t *usart_handle, uint8_t *buffer, uint32_t len)
{

	/*Populate the application given infos in to th USART handle structure*/
	usart_handle->pTxBuffPtr = buffer;
	usart_handle->TxXferCount = len;
	usart_handle->TxXferSize = len;
	
	/*This handle is busy in doing the TX*/
	usart_handle->tx_state = USART_STATE_BUSY_TX;
	
	/*Enable the USART Peripheral*/
	usart_enable(usart_handle->Instance);
	
	/*Enable the TXE interuppt */
	usart_config_txe_interrupt(usart_handle->Instance,1);
	
}



/**
  *@brief API to USART data reception
  *@param *handle : pointer to the handle structure of the USART peripheral
  *@param *buffer : holds the pointer to Rx buffer
  *@param  len    : length of data to be RXed
  *@retval None
  */

void usart_rx(usart_handle_t *usart_handle, uint8_t *buffer, uint32_t len)
{
	//uint32_t val;
	/*Populate the application given infos in to th USART handle structure*/
	usart_handle->pRxBuffPtr = 0;
	usart_handle->pRxBuffPtr = buffer;
	usart_handle->RxXferCount = len;
	usart_handle->RxXferSize = len;
	
	/*This handle is busy in doing the RX*/
	usart_handle->rx_state = USART_STATE_BUSY_RX;
	
	/*Enable the USART Peripheral*/
	usart_enable(usart_handle->Instance);
	
	/*Enable the Parity Error interuppt */
	usart_config_parity_error_interrupt(usart_handle->Instance,1);
	
	/*Enable the USART ERROR interuppt:(Frame Error, noise Error, Overrun Error) */
	usart_config_error_interrupt(usart_handle->Instance,1);
	
	//val = usart_handle->Instance->DR;
	
	/*Enable the RXNE interuppt */
	usart_config_rxne_interrupt(usart_handle->Instance,1);
}



/**
  *@brief API to USART interrupt request
  *@param *handle : pointer to the handle structure that contains the confiduration information for
	*        the specified USART Module
  *@retval None
  */


void usart_handle_interrupt(usart_handle_t *husart)
{
	uint32_t tmp1 = 0, tmp2 = 0;
	
	tmp1 = husart->Instance->SR & USART_REG_SR_PE_FLAG;
	tmp2  = husart->Instance->CR1 & USART_REG_CR1_PEIE;

	/* USART Parity interuppt occured */
	if((tmp1) && (tmp2))
	{
		usart_clear_error_flag(husart);
		husart->ErrorCode |= USART_ERROR_PE;
	}
	
	tmp1 = husart->Instance->SR & USART_REG_SR_FE_FLAG;
	tmp2  = husart->Instance->CR3 & USART_REG_CR3_EIE;

	/* USART Frame Error interuppt occured */
	if((tmp1) && (tmp2))
	{
		usart_clear_error_flag(husart);
		husart->ErrorCode |= USART_ERROR_FE;
	}

	tmp1 = husart->Instance->SR & USART_REG_SR_NF_FLAG;
	tmp2  = husart->Instance->CR3 & USART_REG_CR3_EIE;

	/* USART Noise Error interuppt occured */
	if((tmp1) && (tmp2))
	{
		usart_clear_error_flag(husart);
		husart->ErrorCode |= USART_ERROR_NE;
	}
	
	tmp1 = husart->Instance->SR & USART_REG_SR_ORE_FLAG;
	tmp2  = husart->Instance->CR3 & USART_REG_CR3_EIE;

	/* USART Overrun Error interuppt occured */
	if((tmp1) && (tmp2))
	{
		usart_clear_error_flag(husart);
		husart->ErrorCode |= USART_ERROR_ORE;
	}
	tmp1 = husart->Instance->SR & USART_REG_SR_RXNE_FLAG;
	tmp2  = husart->Instance->CR1 & USART_REG_CR1_RXNEIE;

	/* USART in mode Receiver */
	if((tmp1) && (tmp2))
	{
	 	usart_handle_RXNE_interrupt(husart);	
		
}
	tmp1 = husart->Instance->SR & USART_REG_SR_TXE_FLAG;
	tmp2  = husart->Instance->CR1 & USART_REG_CR1_TXEIE;

	/* USART in mode Transmitter */
	if((tmp1) && (tmp2))
	{
	usart_handle_TXE_interrupt(husart);
	}
	
	tmp1 = husart->Instance->SR & USART_REG_SR_TC_FLAG;
	tmp2  = husart->Instance->CR1 & USART_REG_CR1_TCIE;

	/* USART in mode Transmission End */
	if((tmp1) && (tmp2))
	{
	usart_handle_TC_interrupt(husart);
	}
	
	if(husart->ErrorCode != USART_ERROR_NONE)	
	{
		/* Set the USART state ready to be able to start again th process */
    husart->tx_state = USART_STATE_READY;
    husart->rx_state = USART_STATE_READY;

    usart_error_cd(husart);
	}		
}
