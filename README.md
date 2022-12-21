### 资料和视频正在上传中。。。。


一、机械臂简介

机械制作方面采用铝型材、3D打印、同步带和步进电机，低成本的制作方案。控制器方面，使用esp32s3芯片，带有WiFi蓝牙功能。可以输出100KHZ脉冲，支持关节运动和直线运动指令，150mA 8路输出，6~36V 8路输入，带光耦隔离。编程方面，采用blockly可视化编程，使用手机或平板就可以完成编程。

![输入图片说明](%E5%9B%BE%E7%89%87/%E6%9C%BA%E6%A2%B0%E8%87%82%E7%94%B5%E6%9C%BA%E7%BB%93%E6%9E%84.png)


二、制作过程




1.机械部分

首先使用FreeCAD软件进行3D模型图绘制。FreeCAD是一个开源的3D设计软件，可以免费使用，方便二次开发，支持运动仿真和简单的有限元仿真。我使用它来设计我的机械臂。其中前三关节采用双电机结构，也就是说一个关节有两个电机，这样平衡的设计既提高了负载能力，也提高了精度和稳定性。现在这台机械臂拥有3kg负载能力和0.5mm的重複定位精度，以及765mm的臂长。


![输入图片说明](%E5%9B%BE%E7%89%87%E6%9C%AA%E5%91%BD%E5%90%8D.png)



2.控制器部分




2.1 主控制器

主控制器采用ESP32-S3芯片，ESP32-S3芯片带有WiFi蓝牙功能，双核，240Mhz频率，45 个可编程 GPIO。性能可以满足机械臂运行算法，设计了6个脉冲控制接口，可以控制6轴机械臂。8路输入和8路输出，带光电耦合起到保护电路的作用，方便外接电磁阀。

![输入图片说明](%E5%9B%BE%E7%89%87/SCH_Schematic1_0-P1_2022-07-15.png)

![输入图片说明](%E5%9B%BE%E7%89%87/IMG_20220505_170758.jpg)

2.2 电机驱动部分

步进电机驱动采用TB67S109AFTG芯片，支持最大4A电流。控制芯片使用ESP32-C3，支持脉冲控制和CAN通信控制。磁编码器使用MT6816CT-ACD，14位单圈绝度编码器，最高精度可达0.02°。


![输入图片说明](%E5%9B%BE%E7%89%87/SCH_Schematic2_0-P1_2022-07-15.png)

![输入图片说明](%E5%9B%BE%E7%89%87/%E6%9C%AA%E5%91%BD%E5%90%8D3.png)


2.3 软件部分




主控板程序是基于开源micropython的，它实现了使用python语言就可完成单片机程序开发。驱动板编程是基于ESP-IDF库编写的C语言程序。手机控制软件是基于开源blockly和bipes编写的。

![输入图片说明](%E5%9B%BE%E7%89%87/%E6%9C%AA%E5%91%BD%E5%90%8D5.png)


三、使用说明


![输入图片说明](%E5%9B%BE%E7%89%87/%E6%9C%AA%E5%91%BD%E5%90%8D4.png)

### 四、项目收入

| 时间        | 收入类型         | 金额（元） |
|-----------|--------------|-------|
| 2021-2022   |     嘉立创-火星计划金奖        |   10000|
| 2021-2022   |     用户咨询赞助         |   500    |
| 2021-2022 | 哔哩哔哩视频       |  100   |
| 2020-2021 | 贸泽电子-短视频大赛金奖 | 2000  |
| 2020-2021 | 用户咨询赞助       | 200   |
| 2020-2021 | 哔哩哔哩视频       | 200   |






