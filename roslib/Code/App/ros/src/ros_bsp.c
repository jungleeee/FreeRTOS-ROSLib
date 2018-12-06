/**
  ********************************************************************************
  * @File Name    : ros_bsp.cpp
  * @Author       : Jungle
  * @Mail         : Mail
  * @Created Time : 2018/7/12 14:23:16
  * @Version      : V1.0
  * @Last Changed : 2018/7/12 14:23:16
  * @Brief        : brief
  ********************************************************************************
  */

/* Inlcude ---------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include <serial.h>

#include "stm32f4xx.h"

/** @addtogroup C:\Users\Administrator\Desktop\cli\_Source Insight 3.5\ros_Project
  * @{
  */

/* Private function declaration ------------------------------------------------*/
/* Private typedef -------------------------------------------------------------*/
typedef struct ROS_DATA_BLOCK {
    int xLength;
    uint8_t xData[256];//ros_MAX_TX_QUEUE_LEN
} Ros_Data_Block_Definition_t;

/* Private constants define ----------------------------------------------------*/
#define ros_MAX_TX_QUEUE_LEN            256
#define ros_MAX_TX_QUEUE_DEPTH          100

#define ros_MAX_RX_QUEUE_LEN            1
#define ros_MAX_RX_QUEUE_DEPTH          1024

#define ros_MAX_RX_QUEUE_WAIT           0

#define ros_MAX_TX_QUEUE_WAIT           1000
#define ros_MAX_TX_BINARY_DMA_WAIT      500

/* Private macro define --------------------------------------------------------*/
/* Private variables -----------------------------------------------------------*/
static xQueueHandle xRosSerialRecvCharsQueue = NULL;
static xQueueHandle xRosSerialSendCharsQueue = NULL;
static xSemaphoreHandle xRosSerialSendCompleteBinary = NULL;

/* Private functions -----------------------------------------------------------*/
/**
  * @brief  : Fetch the message from the queue to the serial port.
  * @param  : None
  * @retval : None
  */
static void ros_task_send(void *pvParameters)
{
    static Ros_Data_Block_Definition_t cOutputQueuePull;

	for( ;; ) {

        if(xQueueReceive(xRosSerialSendCharsQueue, (uint8_t *)&cOutputQueuePull, ros_MAX_TX_QUEUE_WAIT) == pdTRUE) {

            xSemaphoreTake(xRosSerialSendCompleteBinary, 0);

            taskENTER_CRITICAL();
            serial_ros_send_data(cOutputQueuePull.xData, cOutputQueuePull.xLength);
            taskEXIT_CRITICAL();

            /* Wait the DMA send over & Start next */
            xSemaphoreTake(xRosSerialSendCompleteBinary, ros_MAX_TX_BINARY_DMA_WAIT);
        }
    }
}

/**
  * @brief  : The serial port receives the character callback.
  * @param  : None
  * @retval : None
  */
static void ros_receive_char_callback(uint16_t ch)
{
    uint8_t cRxedChar = (uint8_t)ch;

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xRosSerialRecvCharsQueue, &cRxedChar, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  : The serial port output completes the callback.
  * @param  : None
  * @retval : None
  */
static void ros_send_frame_over_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xRosSerialSendCompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
static void ros_serial_init(int baudRate)
{
    serial_ros_init(baudRate, ros_receive_char_callback);
    serial_ros_send_data_init(ros_send_frame_over_callback);
}

/**
  * @brief  : ros_uart_recv_enable_init
  * @param  : None
  * @retval : None
  */
void ros_serial_recv_enable_init(void)
{
    serial_ros_recv_enable();
}

/**
  * @brief  : ros send data length.
  * @param  : pointer of data
  * @param  : data length
  * @retval : None
  */
void ros_serial_write_length(uint8_t *data, int length)
{
    static Ros_Data_Block_Definition_t cOutputQueuePush;

    taskENTER_CRITICAL();
    cOutputQueuePush.xLength = length;
    memcpy(cOutputQueuePush.xData, data, length);

    xQueueSend(xRosSerialSendCharsQueue, (uint8_t *)&cOutputQueuePush, ros_MAX_TX_QUEUE_WAIT);
    taskEXIT_CRITICAL();
}

/**
  * @brief  : read
  * @param  : None
  * @retval : char
  */
int ros_serial_read_char(void)
{
    uint8_t cRxedChar;

    if(xQueueReceive(xRosSerialRecvCharsQueue, &cRxedChar, ros_MAX_RX_QUEUE_WAIT) == pdTRUE) {

        return (int)cRxedChar;
    }
    else
        return -1;
}

/**
  * @brief  : get system time
  * @param  : None
  * @retval : char
  */
unsigned int ros_get_system_time(void)
{
    return xTaskGetTickCount();
}

/**
  * @brief  : dalay ms
  * @param  : None
  * @retval : char
  */
void ros_dealy_ms(unsigned int ms)
{
    vTaskDelay(ms);
}

/**
  * @brief  : communication usart init
  * @param  : None
  * @retval : None
  */
void ros_communication_bsp_init(void)
{
    if(xRosSerialRecvCharsQueue == NULL) {
        xRosSerialRecvCharsQueue = xQueueCreate(ros_MAX_RX_QUEUE_DEPTH, sizeof(uint8_t) * ros_MAX_RX_QUEUE_LEN);
        configASSERT(xRosSerialRecvCharsQueue);
    }

    if(xRosSerialSendCharsQueue == NULL) {
        xRosSerialSendCharsQueue = xQueueCreate(ros_MAX_TX_QUEUE_DEPTH, sizeof(uint8_t) * ros_MAX_TX_QUEUE_LEN);
        configASSERT(xRosSerialSendCharsQueue);
    }

    if(xRosSerialSendCompleteBinary == NULL) {
    	xRosSerialSendCompleteBinary = xSemaphoreCreateBinary();
    	configASSERT(xRosSerialSendCompleteBinary);
    }

    xSemaphoreTake(xRosSerialSendCompleteBinary, 0);

    ros_serial_init(460800);

    xTaskCreate(ros_task_send,    (const char *)"rosSendTask",    128, NULL, 14, NULL);
}

/**
  * @}
  */

/************************** Coopyright (C) Jungleeee 2018 *******END OF FILE********/

