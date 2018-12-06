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
#define     UART_ROS_IRQHandler             UART4_IRQHandler
#define     UART_ROS_DMA_IRQHandler         DMA1_Stream4_IRQHandler

#define     UART_DEBUG_IRQHandler           USART1_IRQHandler
#define     UART_DEBUG_DMA_IRQHandler       DMA2_Stream7_IRQHandler

#define     ROS_UART_RX_PIN                 GPIO_Pin_11
#define     ROS_UART_RX_PORT                GPIOC
#define     ROS_UART_RX_PORT_CLK            RCC_AHB1Periph_GPIOC
#define     ROS_UART_TX_PIN                 GPIO_Pin_10
#define     ROS_UART_TX_PORT                GPIOC
#define     ROS_UART_TX_PORT_CLK            RCC_AHB1Periph_GPIOC
#define     ROS_UART_CLK                    RCC_APB1Periph_UART4
#define     ROS_UART_IRQn                   UART4_IRQn
#define     ROS_UART_DMA_TX_CHANNEL         DMA_Channel_4
#define     ROS_UART_DMA_TX_STREAM          DMA1_Stream4
#define     ROS_UART_DMA_IRQn               DMA1_Stream4_IRQn

#define     DEBUG_UART_RX_PIN               GPIO_Pin_10
#define     DEBUG_UART_RX_PORT              GPIOA
#define     DEBUG_UART_RX_PORT_CLK          RCC_AHB1Periph_GPIOA
#define     DEBUG_UART_TX_PIN               GPIO_Pin_9
#define     DEBUG_UART_TX_PORT              GPIOA
#define     DEBUG_UART_TX_PORT_CLK          RCC_AHB1Periph_GPIOA
#define     DEBUG_UART_CLK                  RCC_APB2Periph_USART1
#define     DEBUG_UART_IRQn                 USART1_IRQn
#define     DEBUG_UART_DMA_TX_CHANNEL       DMA_Channel_4
#define     DEBUG_UART_DMA_TX_STREAM        DMA2_Stream7
#define     DEBUG_UART_DMA_IRQn             DMA2_Stream7_IRQn

/* Private variables -----------------------------------------------------------*/
/* Private function declaration ------------------------------------------------*/
static volatile void (* uartRosReceiveOneByte)(uint16_t) = NULL;
static volatile void (* uartRosDmaSendOver)(void) = NULL;

static volatile void (* uartDebugReceiveOneByte)(uint16_t) = NULL;
static volatile void (* uartDebugDmaSendOver)(void) = NULL;

/* Private functions -----------------------------------------------------------*/
/**
  * @brief  : communciation uart init: 8bits & 1 stop bit
  * @param  : baud Rate
  * @param  : pointer of usart receive interrupt
  * @retval : None
  */
void uart_ros_init(int baudRate, void (* const receive)(uint16_t))
{
    if(receive == NULL) return;

    GPIO_InitTypeDef GPIO_InitStructure = {0};
	USART_InitTypeDef USART_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};

	RCC_AHB1PeriphClockCmd(ROS_UART_RX_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(ROS_UART_TX_PORT_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(ROS_UART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = ROS_UART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(ROS_UART_RX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ROS_UART_RX_PIN;
	GPIO_Init(ROS_UART_RX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(ROS_UART_RX_PORT, GPIO_PinSource11, GPIO_AF_UART4);
	GPIO_PinAFConfig(ROS_UART_TX_PORT, GPIO_PinSource10, GPIO_AF_UART4);

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
  * @brief  : com uart DMA init
  * @param  : pointer of DMA transmit complete callback function
  * @retval : None
  */
void uart_ros_dma_init(void (* const send)(void))
{
    if(send == NULL) return;

    DMA_InitTypeDef DMA_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                        /* Enable DMA2 CLK */

    /* CH4，DMA Uart send */
    DMA_DeInit(ROS_UART_DMA_TX_STREAM);						                    /* DMA2 */
    while(DMA_GetCmdStatus(ROS_UART_DMA_TX_STREAM) != DISABLE);

    DMA_InitStructure.DMA_Channel = ROS_UART_DMA_TX_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART4->DR);	        ///* Peripheral address */
    DMA_InitStructure.DMA_Memory0BaseAddr = NULL;	                            ///* Memory address */
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				        ///* DMA dir Memory To Peripheral */
    DMA_InitStructure.DMA_BufferSize = 0;			                            ///* buf size */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		    ///* Peripheral address add, disable */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			            ///* Memory address add, enable */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	    ///* Peripheral data length */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	    	    ///* Memory data length */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;				                ///* no circle */
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;				        ///* Priority */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                      ///* Direct mode */
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 ///* Single */
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         ///* Single */
    DMA_Init(ROS_UART_DMA_TX_STREAM, &DMA_InitStructure);

    uartRosDmaSendOver = (void volatile(*)(void))send;

    DMA_Cmd(ROS_UART_DMA_TX_STREAM, DISABLE);
    DMA_ClearFlag(ROS_UART_DMA_TX_STREAM, DMA_IT_TCIF4);
    DMA_ITConfig(ROS_UART_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
    USART_DMACmd(ROS_UART, USART_DMAReq_Tx, ENABLE);                          /* enable USART1 transe DMA */

    NVIC_InitStructure.NVIC_IRQChannel = ROS_UART_DMA_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ROS_UART_DMA_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  UART_ROS_IRQHandler:
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
  * @brief  UART_ROS__DMA_IRQHandler:
  * @param  None
  * @retval None
  */
void UART_ROS_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(ROS_UART_DMA_TX_STREAM, DMA_IT_TCIF4) != RESET) {

        uartRosDmaSendOver();

        DMA_ClearITPendingBit(ROS_UART_DMA_TX_STREAM, DMA_IT_TCIF4);
    }
}

/**
  * @brief  : debug uart init: 8bits & 1 stop bit
  * @param  : baud Rate
  * @param  : pointer of usart receive interrupt
  * @retval : None
  */
void uart_debug_init(int baudRate, void (* const receive)(uint16_t))
{
    if(receive == NULL) return;

    GPIO_InitTypeDef GPIO_InitStructure = {0};
	USART_InitTypeDef USART_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};

	RCC_AHB1PeriphClockCmd(DEBUG_UART_RX_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(DEBUG_UART_TX_PORT_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(DEBUG_UART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = DEBUG_UART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(DEBUG_UART_RX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DEBUG_UART_RX_PIN;
	GPIO_Init(DEBUG_UART_RX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(DEBUG_UART_RX_PORT, GPIO_PinSource10, GPIO_AF_USART1);
	GPIO_PinAFConfig(DEBUG_UART_TX_PORT, GPIO_PinSource9,  GPIO_AF_USART1);

	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_UART, &USART_InitStructure);

    uartDebugReceiveOneByte = (void volatile(*)(uint16_t))receive;

    USART_Cmd(DEBUG_UART, ENABLE);
    USART_ITConfig(DEBUG_UART, USART_IT_TC, DISABLE);
    USART_ITConfig(DEBUG_UART, USART_IT_RXNE, DISABLE);
	USART_ClearFlag(DEBUG_UART, USART_FLAG_TC | USART_FLAG_RXNE);

    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_UART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DEBUG_UART_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  : debug uart DMA init
  * @param  : pointer of DMA transmit complete callback function
  * @retval : None
  */
void uart_debug_dma_init(void (* const send)(void))
{
    if(send == NULL) return;

	DMA_InitTypeDef DMA_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                        /* Enable DMA2 CLK */

    /* CH4，DMA Uart send */
    DMA_DeInit(DEBUG_UART_DMA_TX_STREAM);						                    /* DMA2 */
    while(DMA_GetCmdStatus(DEBUG_UART_DMA_TX_STREAM) != DISABLE);

    DMA_InitStructure.DMA_Channel = DEBUG_UART_DMA_TX_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);	        ///* Peripheral address */
    DMA_InitStructure.DMA_Memory0BaseAddr = NULL;	                            ///* Memory address */
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				        ///* DMA dir Memory To Peripheral */
    DMA_InitStructure.DMA_BufferSize = 0;			                            ///* buf size */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		    ///* Peripheral address add, disable */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			            ///* Memory address add, enable */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	    ///* Peripheral data length */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	    	    ///* Memory data length */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;				                ///* no circle */
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;				            ///* Priority */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                      ///* Direct mode */
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 ///* Single */
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         ///* Single */
    DMA_Init(DEBUG_UART_DMA_TX_STREAM, &DMA_InitStructure);

    uartDebugDmaSendOver = (void volatile(*)(void))send;

    DMA_Cmd(DEBUG_UART_DMA_TX_STREAM, DISABLE);
    DMA_ClearFlag(DEBUG_UART_DMA_TX_STREAM, DMA_IT_TCIF7);
    DMA_ITConfig(DEBUG_UART_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
    USART_DMACmd(DEBUG_UART, USART_DMAReq_Tx, ENABLE);                          /* enable USART1 transe DMA */

    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_UART_DMA_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DEBUG_UART_DMA_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  DEBUG_UART_IRQHandler:
  * @param  None
  * @retval None
  */
void UART_DEBUG_IRQHandler(void)
{
    if(USART_GetITStatus(DEBUG_UART, USART_IT_RXNE) != RESET) {

        uartDebugReceiveOneByte(USART_ReceiveData(DEBUG_UART));

        USART_ClearITPendingBit(DEBUG_UART, USART_IT_RXNE);
    }
    else if(USART_GetITStatus(DEBUG_UART, USART_IT_TC) != RESET) {

        USART_ClearITPendingBit(DEBUG_UART, USART_IT_TC);
    }
}

/**
  * @brief  UART_DEBUG_DMA_IRQHandler:
  * @param  None
  * @retval None
  */
void UART_DEBUG_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(DEBUG_UART_DMA_TX_STREAM, DMA_IT_TCIF7) != RESET) {

        uartDebugDmaSendOver();

        DMA_ClearITPendingBit(DEBUG_UART_DMA_TX_STREAM, DMA_IT_TCIF7);
    }
}

/**
  * @brief  USART_SendBufByDMA
  * @param  None
  * @retval None
  */
void uart_send_data_by_dma(const USART_TypeDef* USARTx, uint8_t *buf, uint8_t length)
{
    if(USARTx == DEBUG_UART) {
        DEBUG_UART_DMA_TX_STREAM->CR &= ((uint32_t)0xFFFFFFFE);    /* Disable DMA */
        DEBUG_UART_DMA_TX_STREAM->M0AR = (uint32_t)buf;            /* set memory addr */
        DEBUG_UART_DMA_TX_STREAM->NDTR = length;
        DEBUG_UART_DMA_TX_STREAM->CR |= ((uint32_t)0x00000001);    /* Enable DMA */
    }
    else if(USARTx == ROS_UART) {
        ROS_UART_DMA_TX_STREAM->CR &= ((uint32_t)0xFFFFFFFE);    /* Disable DMA */
        ROS_UART_DMA_TX_STREAM->M0AR = (uint32_t)buf;            /* set memory addr */
        ROS_UART_DMA_TX_STREAM->NDTR = length;
        ROS_UART_DMA_TX_STREAM->CR |= ((uint32_t)0x00000001);    /* Enable DMA */
    }
}

/**
  * @brief  enable USART1 receive interrupt:
  * @param  None
  * @retval None
  */
void uart_debug_rx_isr_enable(void)
{
    USART_ITConfig(DEBUG_UART, USART_IT_RXNE, ENABLE);//open receive
}
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

