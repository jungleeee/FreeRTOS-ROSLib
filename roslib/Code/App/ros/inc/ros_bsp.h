/**
  ********************************************************************************
  * @File Name    : ros_bsp.h
  * @Author       :
  * @Mail         : Mail
  * @Created Time : 2018/7/12 14:23:42
  * @Version      : V1.0
  * @Last Changed : 2018/7/12 14:23:42
  * @Brief        : brief
  ********************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------*/
#ifndef __ROS_BSP_H_
#define __ROS_BSP_H_

/* Inlcude ---------------------------------------------------------------------*/

/* Exported typedef ------------------------------------------------------------*/
/* Exported constants define ---------------------------------------------------*/
/* Exported macro define -------------------------------------------------------*/
/* Exported variables ----------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
    void ros_serial_recv_enable_init(void);
    void ros_serial_write_length(unsigned char *data, int length);
    int  ros_serial_read_char(void);
    unsigned int ros_get_system_time(void);
    void ros_dealy_ms(unsigned int ms);

    void ros_communication_bsp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ROS_BSP_H_ */

/************************** Coopyright (C) Jungleee 2018 *******END OF FILE******/

