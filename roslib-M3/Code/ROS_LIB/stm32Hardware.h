/**
  ********************************************************************************
  * @File Name    : stm32Hardware.h
  * @Author       : Jungle
  * @Mail         : Mail
  * @Created Time : 2018/7/16 17:29:18
  * @Version      : V1.0
  * @Last Changed : 2018/7/16 17:29:18
  * @Brief        : brief
  ********************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------*/
#ifndef __STM32HARDWARE_H_
#define __STM32HARDWARE_H_

/* Inlcude ---------------------------------------------------------------------*/
#include "stdio.h"
#include "ros_bsp.h"

/* Exported typedef ------------------------------------------------------------*/
class Stm32Hardware
{
	public:
		Stm32Hardware(){};

		void init(void)
        {
            baud_ = 115200;
            ros_serial_recv_enable_init();
        }

        int getBaud() {return baud_;}

        void setBaud(long baud)
        {
            this->baud_= baud;
        }

		int read(void)
        {
            return ros_serial_read_char();
        }

		void write(unsigned char* data, int length)
        {
            ros_serial_write_length(data, length);
        }

		unsigned long time(void) {return ros_get_system_time();}

    protected:
        long baud_;
};

/* Exported constants define ---------------------------------------------------*/
/* Exported macro define -------------------------------------------------------*/
/* Exported variables ----------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------*/

#endif /* __STM32HARDWARE_H_ */

/************************** Coopyright (C) Jungleeee 2018 *******END OF FILE*****/
