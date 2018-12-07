/**
  ********************************************************************************
  * @File Name    : ros_test.cpp
  * @Author       : Jungle
  * @Mail         : Mail
  * @Created Time : 2018/7/16 17:39:11
  * @Version      : V1.0
  * @Last Changed : 2018/7/16 17:39:11
  * @Brief        : brief
  ********************************************************************************
  */

/* Inlcude ---------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

#include <ros.h>
#include <stm32Hardware.h>

#include "std_msgs/String.h"

#include "ros_test.h"

/** @addtogroup
  * @{
  */

/* Private function declaration ------------------------------------------------*/
static void CallBack(const std_msgs::String &str);

static void node_test_subscribe(void *pvParameters);
static void node_test_publish(void *pvParameters);

/* Private typedef -------------------------------------------------------------*/
typedef void (* pfNODE_TASK_CALLBACK)(void *p);
typedef struct sROS_NODE_DESCRIPTOR {

	const pfNODE_TASK_CALLBACK pfNodeTask;

	const char cNodeName[configMAX_TASK_NAME_LEN];

	const int xTaskPriority;
} Ros_Node_Definition_t;

/* Private constants define ----------------------------------------------------*/
/* Private macro define --------------------------------------------------------*/
/* Private variables -----------------------------------------------------------*/
static Ros_Node_Definition_t xRegisteredNode[] =
{
	{node_test_subscribe, "subscribe",      9},
	{node_test_publish,	  "publish",        10},
	{NULL,				  ""},
};
static Ros_Node_Definition_t *psNode = xRegisteredNode;

static ros::NodeHandle n;

static std_msgs::String Msg;
static ros::Publisher stm_publish("stmPublish", &Msg);

static ros::Subscriber<std_msgs::String> stm_subscribe("stmSubscribe", CallBack);

/* Private functions -----------------------------------------------------------*/
/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
static void CallBack(const std_msgs::String &str)
{

}

/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
static void node_test_subscribe(void *pvParameters)
{
    n.subscribe(stm_subscribe);

    while(1)
    {
        n.spinOnce();
        ros_dealy_ms(10);
    }
}

/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
static void node_test_publish(void *pvParameters)
{
	n.advertise(stm_publish);
        
    Msg.data = "stm publish";

    for( ;; ) {
        
        stm_publish.publish(&Msg);
        
        ros_dealy_ms(1000);
	}
}

/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
static void ros_node_init(void *pvParameters)
{
	n.initNode();

	for( ;psNode->pfNodeTask != NULL; psNode++) {

    	xTaskCreate(psNode->pfNodeTask,  psNode->cNodeName,  256,  NULL, psNode->xTaskPriority, NULL);
	}

	vTaskDelete(NULL);
}

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  :
  * @param  : None
  * @retval : None
  */
	void ros_task_create(void)
	{
		ros_communication_bsp_init();

    	xTaskCreate(ros_node_init,    (const char *)"rosNodeTask",    256, NULL, 2, NULL);
	}

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/************************** Coopyright (C) Jungleeee 2018 *******END OF FILE********/

