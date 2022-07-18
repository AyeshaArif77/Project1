#include "stm32f4xx.h" // Device header
//#include "stdint.h"
#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

/*UART state structures definitions */
typedef enum{
	USART_STATE_RESET        = 0x00,                /*Peripheral not initialized*/ 
	USART_STATE_READY        = 0x01,                /*Peripheral initialized and ready for use*/
	USART_STATE_BUSY         = 0x02,                /*internal process is going on*/
	USART_STATE_BUSY_TX      = 0x12,                /*Data Transmission process is going on*/
	USART_STATE_BUSY_RX      = 0x22,                /*Data Reception ptocess is going on*/
  USART_STATE_BUSY_TX_RX   = 0x32,                /*Data Transmission and Reception ptocess is going on*/
	
}usart_state_t;

/*UART possible error codes*/
#define USART_ERROR_NONE    ((uint32_t)0x00000000)  /*No Error*/
#define USART_ERROR_PE      ((uint32_t)0x00000001)  /*Parity Error*/
#define USART_ERROR_NE      ((uint32_t)0x00000002)  /*Noise Error*/
#define USART_ERROR_FE      ((uint32_t)0x00000004)  /*Frame Error*/
#define USART_ERROR_ORE     ((uint32_t)0x00000008)  /*Overrun Error*/
#define USART_ERROR_DMA     ((uint32_t)0x00000010)  /*DMA Error*/


/*UART and USART base addresses */
#define USART_1 USART1
#define USART_2 USART2
#define USART_6 USART6

/* Macros to enable clocks for USARTs */

#define _RCC_USART1_CLK_ENABLE() (RCC->APB2ENR |= (1<<4))
#define _RCC_USART2_CLK_ENABLE() (RCC->APB1ENR |= (1<<17))
#define _RCC_USART6_CLK_ENABLE() (RCC->APB2ENR |= (1<<5))

/*****************************************************************************************/
/*                                 USART                                                 */
/*                      Registers Bit Definitions                                        */
/*****************************************************************************************/

/* USART_SR Register*/
#define USART_REG_SR_TXE_FLAG     ((uint32_t)(1<<7))
#define USART_REG_SR_TC_FLAG      ((uint32_t)(1<<6))
#define USART_REG_SR_RXNE_FLAG    ((uint32_t)(1<<5))
#define USART_REG_SR_IDLE_FLAG    ((uint32_t)(1<<4))
#define USART_REG_SR_ORE_FLAG     ((uint32_t)(1<<3))
#define USART_REG_SR_NF_FLAG      ((uint32_t)(1<<2))
#define USART_REG_SR_FE_FLAG      ((uint32_t)(1<<1))
#define USART_REG_SR_PE_FLAG      ((uint32_t)(1<<0))


/* USART_BRR Register*/
#define USART_REG_BRR_MANTISSA     ((uint32_t)(1<<4))
#define USART_REG_BRR_FRACTION     ((uint32_t)(1<<0))


/* USART_CR1 Register*/
#define USART_REG_CR1_OVER8       ((uint32_t)(1<<15))
#define USART_OVER8_ENABLE         1
#define USART_OVER16_ENABLE        0

#define USART_REG_CR1_USART_EN     ((uint32_t)(1<<13))

#define USART_REG_CR1_USART_WL     ((uint32_t)(1<<12))
#define USART_WL_1S8B              0
#define USART_WL_1S9B              1

#define USART_REG_CR1_PEIE      ((uint32_t)(1<<8))
#define USART_REG_CR1_TXEIE     ((uint32_t)(1<<7))
#define USART_REG_CR1_TCIE      ((uint32_t)(1<<6))
#define USART_REG_CR1_RXNEIE    ((uint32_t)(1<<5))
#define USART_REG_CR1_TE        ((uint32_t)(1<<3))
#define USART_REG_CR1_RE        ((uint32_t)(1<<2))



/* USART_CR3 Register*/
#define USART_REG_CR3_EIE       ((uint32_t)(1<<0))

#define USART_STOPBITS_1        ((uint32_t)(0x00))
#define USART_STOPBITS_HALF     ((uint32_t)(0x01))
#define USART_STOPBITS_2        ((uint32_t)(0x02))
#define USART_STOPBITS_1NHALF   ((uint32_t)(0x03))

#define USART_PARITY_NONE       ((uint32_t)0x00000000)
#define USART_HWCONTROL_NONE    ((uint32_t)0x00000000)

#define USART_MODE_TX_RX        ((uint32_t)(USART_REG_CR1_TE | USART_REG_CR1_RE ))

#define USART_MODE_TX           ((uint32_t)(USART_REG_CR1_TE))

#define USART_MODE_RX           ((uint32_t)(USART_REG_CR1_RE))

#define USART_BAUD_9600         (uint32_t)9600
#define USART_BAUD_115200       (uint32_t)115200
#define USART_BAUD_2000000      (uint32_t)2000000

#define UNUSED(x)               ((void)(x))


/*****************************************************************************************/
/*                                                                                       */
/*                  DATA STRUCTURES USED BY USART DRIVER                                 */
/*****************************************************************************************/

/* USART Init Structure Definition*/
typedef struct
{
	uint32_t BaudRate;               /*This member configures the USART communication baud rate*/
	uint32_t WordLength;             /*This member specifies the number of data bits transmitted or recieved in a frame*/
	uint32_t StopBits;              /*This member specifies the number if stop bits transmitted*/
	uint32_t Parity;                /*This member specifies the parity mode*/
	uint32_t Mode;                  /*This member specifies whether the receive or transmit mode is enable or disable*/
	uint32_t OverSampling;          /*This member specifies whether oversampling 8 is enable or disable*/

}usart_init_t;

/*Application callbacks typedef*/
typedef void (TX_COMP_CB_t) (void *ptr);
typedef void (RX_COMP_CB_t) (void *ptr);

/*USART handle structure definition*/
typedef struct
	{
		USART_TypeDef                  *Instance;      /*USART register based address*/
		usart_init_t                   Init;          /*USART communication parameters*/
		uint8_t                       *pTxBuffPtr;    /*pointer to USART Tx transfer buffer*/
		uint16_t                       TxXferSize;     /*USART Tx Transfer size*/
		uint16_t                       TxXferCount;    /*USART Tx Transfer Count*/
		uint8_t                        *pRxBuffPtr;    /*Pointer to USART transfer Buffer*/
		uint16_t                       RxXferSize;     /*USART Rx Transfer size*/
		uint16_t                       RxXferCount;    /*USART Rx Transfer Count*/
		usart_state_t                  rx_state;       /*USART Rx Communication State */
		usart_state_t                  tx_state;       /* USART Tx Communication State */
		uint32_t                       ErrorCode;      /*USART Error Code*/
		TX_COMP_CB_t                   *tx_cmp_cb;     /*Application callback when Tx is completed*/
		RX_COMP_CB_t                   *rx_cmp_cb;     /*Application callback when Rx is completed*/
	
}usart_handle_t;


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
void usart_init(usart_handle_t *handle);

/**
  *@brief API to USART data transmission
  *@param *handle : pointer to the handle structure of the USART peripheral
  *@param *buffer : holds the pointer to Tx buffer
  *@param  len    : length of data to be TXed
  *@retval None
  */

void usart_tx(usart_handle_t *handle, uint8_t *buffer, uint32_t len);

/**
  *@brief API to USART data reception
  *@param *handle : pointer to the handle structure of the USART peripheral
  *@param *buffer : holds the pointer to Rx buffer
  *@param  len    : length of data to be RXed
  *@retval None
  */
	
void usart_rx(usart_handle_t *handle, uint8_t *buffer, uint32_t len);

/**
  *@brief API to USART interrupt request
  *@param *handle : pointer to the handle structure that contains the confiduration information for
	*        the specified USART Module
  *@retval None
  */

void usart_handle_interrupt(usart_handle_t *husart);



#endif
