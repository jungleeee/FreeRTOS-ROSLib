# FreeRTOS-ROSLib

---
        author      : Jungle
        Creat Time  : 2018/12/6
        Mail        : yjxxx168@qq.com
        Last Change : 2018/12/6

---
### 前言

```
        学习、总结、交流、进步！
```

测试环境为Keil 5.20 && STM32F405VGT6 && FreeRtos 10.0.0；晶振频率为12Mhz。<br />
请根据实际情况修改后进行测试。<br />

原创，转载请注明出处。<br />

---
### 介绍

ROS(Robot Operating System）是一个机器人软件平台(本人不够熟悉，只有简单使用。不多哔哔）。<br />
这里主要介绍如何实现SMT32控制器如何实现跟ROS进行同行和数据交互：<br />
1、ROS端和STM32端自主定义协议，在ROS端（如ubuntu）做串口驱动，实现协议解析，再将数据发送到相应topic。<br />
2、STM32端解析和封装ROS协议。<br />
3、移植ROSLib，将STM32作为节点加入到ROS系统中（就是库本身实现了ROS协议的解析的封装）。<br />

使用方案3。<br />

<!-- more -->

---
### ROSLIB

ROSlib是arduino下支持ROS的库，C++编写。（至于如何获取arduino IDE下ROSlib库，本人没实际操作，不多哔哔）。<br />
在做移植过程中，主要参考 [基于STM32的rosserial_client的节点开发](https://blog.csdn.net/wubaobao1993/article/details/70808959)。(主要移植步骤文中阐述的比较清楚，不多哔哔)。<br />

建议一步一步搭建，而不是一股脑复制粘贴，因为系统或者Keil版本或者其他因素，都有可能导致错误出来，最好是一点点来。优先搭建好STM32和FreeRTOS，再加入ROSlib(C++)。<br />

因为加入了FreeRTOS系统，系统的Task和queue机制，使得在串口收发处理时有更加便捷和完善的方式。<br />

需要注意的是，由于ROS自定义的消息类型，大部分存在一个消息占用的数据量比较大(几百字节)，所以在设置串口发送任务时的队列大小，需要仔细评估，以免内存浪费或者一帧数据内存不足。<br />
在`ros_bsp.c`文件中，定义为256个字节，根据自身情况调整：。<br />


```
    typedef struct ROS_DATA_BLOCK {
        int xLength;
        uint8_t xData[256];//ros_MAX_TX_QUEUE_LEN
    } Ros_Data_Block_Definition_t;
```

`ros_bsp.c`中提供了`stm32Hardware.h`需要定义的函数接口，具体查看源码。<br />

`ros_bsp.c`中`ros_communication_bsp_init()`函数初始化整个通信需要的任务和串口初始化，默认波特率406800。<br />

`ros_test.cpp`中创建了两个任务： <br />


```
    static void node_test_subscribe(void *pvParameters)
    {//ros端可以通过这个话题像STM32发送消息，收到消息后，调用回调函数
        n.subscribe(stm_subscribe);

        while(1)
        {
            n.spinOnce();
            ros_dealy_ms(10);
        }
    }

    static void node_test_publish(void *pvParameters)
    {//STM每一秒中通过这个话题像ROS发送Msg消息：stm publish
        n.advertise(stm_publish);

        Msg.data = "stm publish";

        for( ;; ) {
            stm_publish.publish(&Msg);

            ros_dealy_ms(1000);
        }
    }

```

在使用过程中，请不要忘了根据自己的实际情况配置工程：芯片型号、晶振频率（包括调整程序中的时钟频率）、调试工具等。

---
### 参考

1、[基于STM32的rosserial_client的节点开发](https://blog.csdn.net/wubaobao1993/article/details/70808959) <br />
2、[git 上的例子](https://github.com/stoneyang159/rosserial_client_stm32f103) <br />
3、[ros rosserial_python 用法](http://wiki.ros.org/rosserial_python) <br />


