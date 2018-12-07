/**
  ********************************************************************************
  * @File Name    : usart.h
  * @Author       : Jungle
  * @Mail         : Mail
  * @Created Time : 2018年3月19日 14:35:19
  * @Version      : V1.0
  * @Last Changed : 2018年3月19日 17:04:06
  * @Brief        : brief
  ********************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------*/
#ifndef __USART_H_
#define __USART_H_

/* Inlcude ---------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported typedef ------------------------------------------------------------*/
/* Exported constants define ---------------------------------------------------*/
/* Exported macro define -------------------------------------------------------*/
#define     ROS_UART                        USART1

/* Exported variables ----------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------*/
void uart_ros_init(int baudRate, void (* const receive)(uint16_t));
void uart_ros_dma_init(void (* const send)(void));

void uart_send_data_by_dma(const USART_TypeDef* USARTx, uint8_t *buf, uint8_t length);

void uart_ros_rx_isr_enable(void);

#endif /* __USART_H_ */

/************************** Coopyright (C) Jungleeee 2018 *******END OF FILE*******/

