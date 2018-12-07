/**
  ********************************************************************************
  * @File Name    : serial.c
  * @Author       : Jungle
  * @Mail         : Mail
  * @Created Time : 2018年3月19日 17:06:47
  * @Version      : V1.0
  * @Last Changed : 2018年3月23日 17:45:36
  * @Brief        : brief
  ********************************************************************************
  */

/* Inlcude ---------------------------------------------------------------------*/
#include <serial.h>
#include <usart.h>

/** @addtogroup Cli_Project
  * @{
  */

/* Private typedef -------------------------------------------------------------*/
/* Private constants define ----------------------------------------------------*/
/* Private macro define --------------------------------------------------------*/
/* Private variables -----------------------------------------------------------*/
/* Private function declaration ------------------------------------------------*/
/* Private functions -----------------------------------------------------------*/
/**
  * @brief  : commond line serial init
  * @param  : baud Rate
  * @param  : the pointer of receive callback function
  * @retval : None
  */
void serial_cli_init(int baudRate, void (* const cliReceiveOneCharCallBack)(uint16_t))
{

}

/**
  * @brief  : commond line serial print info enable
  * @param  : the pointer of print info complete callback function
  * @retval : None
  */
void serial_cli_printInfo_init(void (* const cliPrintInfoOverCallBack)(void))
{

}

/**
  * @brief  : print info
  * @param  : pointer of buf: data buffer
  * @param  : length: data buffer length
  * @retval : None
  */
void serial_cli_print_info(uint8_t *buf, int length)
{

}

/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
void serial_cli_recv_enable(void)
{

}

/**
  * @brief  : commond line serial init
  * @param  : baud Rate
  * @param  : the pointer of receive callback function
  * @retval : None
  */
void serial_ros_init(int baudRate, void (* const rosReceiveOneByteCallBack)(uint16_t))
{
    uart_ros_init(baudRate, rosReceiveOneByteCallBack);
}

/**
  * @brief  : commond line serial print info enable
  * @param  : the pointer of print info complete callback function
  * @retval : None
  */
void serial_ros_send_data_init(void (* const rosSendDataOverCallBack)(void))
{
    uart_ros_dma_init(rosSendDataOverCallBack);
}

/**
  * @brief  : print info
  * @param  : pointer of buf: data buffer
  * @param  : length: data buffer length
  * @retval : None
  */
void serial_ros_send_data(uint8_t *buf, int length)
{
    uart_send_data_by_dma(ROS_UART, buf, length);
}

/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
void serial_ros_recv_enable(void)
{
    uart_ros_rx_isr_enable();
}

/**
  * @}
  */

/************************** Coopyright (C) Jungleeee 2018 *******END OF FILE*******/

