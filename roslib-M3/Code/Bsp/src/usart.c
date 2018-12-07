/**
  ********************************************************************************
  * @File Name    : usart.c
  * @Author       : Jungle
  * @Mail         : Mail
  * @Created Time : 2018年3月19日 14:34:42
  * @Version      : V1.0
  * @Last Changed : 2018年3月19日 17:53:17
  * @Brief        : brief
  ********************************************************************************
  */

/* Inlcude ---------------------------------------------------------------------*/
#include "stdio.h"
#include <usart.h>
#include "bspInterruptPriority.h"

/** @addtogroup Cli_Project
  * @{
  */

/* Private typedef -------------------------------------------------------------*/
/* Private constants define ----------------------------------------------------*/
/* Private macro define --------------------------------------------------------*/
#define     UART_ROS_IRQHandler           USART1_IRQHandler
#define     UART_ROS_DMA_IRQHandler       DMA1_Channel4_IRQHandler

#define     ROS_UART_RX_PIN               GPIO_Pin_7
#define     ROS_UART_RX_PORT              GPIOB
#define     ROS_UART_RX_PORT_CLK          RCC_APB2Periph_GPIOB
#define     ROS_UART_TX_PIN               GPIO_Pin_6
#define     ROS_UART_TX_PORT              GPIOB
#define     ROS_UART_TX_PORT_CLK          RCC_APB2Periph_GPIOB
#define     ROS_UART_CLK                  RCC_APB2Periph_USART1
#define     ROS_UART_IRQn                 USART1_IRQn
#define     ROS_UART_DMA_TX_CHANNEL       DMA1_Channel4
#define     ROS_UART_DMA_IRQn             DMA1_Channel4_IRQn

/* Private variables -----------------------------------------------------------*/
/* Private function declaration ------------------------------------------------*/
static volatile void (* uartRosReceiveOneByte)(uint16_t) = NULL;
static volatile void (* uartRosDmaSendOver)(void) = NULL;

/* Private functions -----------------------------------------------------------*/
/**
  * @brief  : debug uart init: 8bits & 1 stop bit
  * @param  : baud Rate
  * @param  : pointer of usart receive interrupt
  * @retval : None
  */
void uart_ros_init(int baudRate, void (* const receive)(uint16_t))
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
	USART_InitTypeDef USART_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};

    if(receive == NULL) return;

	RCC_APB2PeriphClockCmd(ROS_UART_RX_PORT_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(ROS_UART_TX_PORT_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(ROS_UART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = ROS_UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROS_UART_RX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ROS_UART_TX_PIN;
	GPIO_Init(ROS_UART_RX_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(ROS_UART, &USART_InitStructure);

    uartRosReceiveOneByte = (void volatile(*)(uint16_t))receive;

    USART_Cmd(ROS_UART, ENABLE);
    USART_ITConfig(ROS_UART, USART_IT_TC, DISABLE);
    USART_ITConfig(ROS_UART, USART_IT_RXNE, DISABLE);
	USART_ClearFlag(ROS_UART, USART_FLAG_TC | USART_FLAG_RXNE);

    NVIC_InitStructure.NVIC_IRQChannel = ROS_UART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ROS_UART_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  : debug uart DMA init
  * @param  : pointer of DMA transmit complete callback function
  * @retval : None
  */
void uart_ros_dma_init(void (* const send)(void))
{
	DMA_InitTypeDef DMA_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};
    
    if(send == NULL) return;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);                         /* Enable DMA1 CLK */

    /* CH4，DMA Uart1 send */
    DMA_DeInit(ROS_UART_DMA_TX_CHANNEL);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);	        ///* Peripheral address */
    DMA_InitStructure.DMA_MemoryBaseAddr = NULL;	                            ///* Memory address */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;				            ///* DMA dir Memory To Peripheral */
    DMA_InitStructure.DMA_BufferSize = 0;			                            ///* buf size */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		    ///* Peripheral address add, disable */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			            ///* Memory address add, enable */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	    ///* Peripheral data length */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	    	    ///* Memory data length */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;				                ///* no circle */
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;				            ///* Priority */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                ///* Single */
    DMA_Init(ROS_UART_DMA_TX_CHANNEL, &DMA_InitStructure);

    uartRosDmaSendOver = (void volatile(*)(void))send;

    DMA_Cmd(ROS_UART_DMA_TX_CHANNEL, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_ITConfig(ROS_UART_DMA_TX_CHANNEL, DMA_IT_TC, ENABLE);
    USART_DMACmd(ROS_UART, USART_DMAReq_Tx, ENABLE);                          /* enable USART1 transe DMA */

    NVIC_InitStructure.NVIC_IRQChannel = ROS_UART_DMA_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ROS_UART_DMA_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  DEBUG_UART_IRQHandler:
  * @param  None
  * @retval None
  */
void UART_ROS_IRQHandler(void)
{
    if(USART_GetITStatus(ROS_UART, USART_IT_RXNE) != RESET) {

        uartRosReceiveOneByte(USART_ReceiveData(ROS_UART));

        USART_ClearITPendingBit(ROS_UART, USART_IT_RXNE);
    }
    else if(USART_GetITStatus(ROS_UART, USART_IT_TC) != RESET) {

        USART_ClearITPendingBit(ROS_UART, USART_IT_TC);
    }
}

/**
  * @brief  UART_DEBUG_DMA_IRQHandler:
  * @param  None
  * @retval None
  */
void UART_DEBUG_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC4) != RESET) {

        uartRosDmaSendOver();

        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}

/**
  * @brief  USART_SendBufByDMA
  * @param  None
  * @retval None
  */
void uart_send_data_by_dma(const USART_TypeDef* USARTx, uint8_t *buf, uint8_t length)
{
    if(USARTx == ROS_UART) {
        ROS_UART_DMA_TX_CHANNEL->CCR &= ((uint16_t)0xFFFE);    /* Disable DMA */
        ROS_UART_DMA_TX_CHANNEL->CMAR = (uint32_t)buf;         /* set memory addr */
        ROS_UART_DMA_TX_CHANNEL->CNDTR = length;
        ROS_UART_DMA_TX_CHANNEL->CCR |= ((uint16_t)0x0001);    /* Enable DMA */
    }
}

/**
  * @brief  enable USART1 receive interrupt:
  * @param  None
  * @retval None
  */
void uart_ros_rx_isr_enable(void)
{
    USART_ITConfig(ROS_UART, USART_IT_RXNE, ENABLE);//open receive
}

/**
  * @brief  ues printf to debug
  * @param  None
  * @retval None
  */
#if 0
#pragma import(__use_no_semihosting) /* 标准库需要的支持函数 */
struct __FILE
{
	int handle;
};
FILE __stdout;

void _sys_exit(int x)                /* 定义_sys_exit()以避免使用半主机模式 */
{
	x = x;
}

void _ttywrch(int ch)
{
    ch = ch;
}

int fputc(int ch, FILE *f)           /* 重定向fputc函数 */
{
	USART_SendData(DEBUG_UART, (uint8_t)ch);
    while(USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TXE) == RESET);    /* 循环发送,直到发送完毕 */

	return ch;
}
#endif

/**
  * @}
  */

/************************** Coopyright (C) Jungleeee 2018 *******END OF FILE*******/

