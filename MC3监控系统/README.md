# 农业监控系统

## 介绍
使用树莓派3做的一套农业监控系统，可以监控植物的温度、湿度、光照强度、重量、录像，全方面记录种植过程中的情况
制作视频已上传到哔哩哔哩：[https://www.bilibili.com/video/BV1cR4y1H7LY/](https://www.bilibili.com/video/BV1cR4y1H7LY/)

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/123850_808c896b_2117144.png "屏幕截图.png")

## 软件架构
树莓派需要安装64位系统，32位无法安装OpenCV，本人试过。需要安装的应用有，docker、node-red、grafana、timescaledb、pgadmin4、OpenCV、jupyterlab


## 安装教程

### 1.  docker

#使用脚本安装

sudo curl -sSL https://get.docker.com | sh

#查看 Docker 版本

docker -v

#重启 systemctl 守护进程

sudo systemctl daemon-reload

#设置 Docker 开机启动

sudo systemctl enable docker

#开启 Docker 服务

sudo systemctl start docker
 
#下载 Docker 图形化界面 portainer

sudo docker pull portainer/portainer

#创建 portainer 容器

sudo docker volume create portainer_data

#运行 portainer

sudo docker run -d -p 9000:9000 --name portainer --restart always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer

浏览器中输入树莓派IP:9000 进入界面

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/122524_2dd7cba1_2117144.png "屏幕截图.png")

### 2.  node-red

首先在docker内创建一个用于容器与外部互交的东西

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/130250_b3949b0a_2117144.png "屏幕截图.png")

安装 node-red，可以到docker仓库查看 [链接](https://registry.hub.docker.com/r/nodered/node-red)

docker run -it -p 1880:1880 -v node_red_data:/data --name mynodered nodered/node-red

需要添加驱动映射dev，以及root用户，和打开Privileged mode ，usb串口才可以用

为了解决多个usb串口随机问题，需要绑定接口。终端输入：sudo vi /etc/udev/rules.d/10-local.rules

添加  ACTION=="add",KERNELS=="1-2:1.0",SUBSYSTEMS=="usb",MODE:="0777",SYMLINK+="name"

要查看usb的接口KERNELS，输入命令：ls /sys/class/tty/ttyUSB* -l

![输入图片说明](%E5%9B%BE%E7%89%87.png)

浏览器中输入树莓派IP:1880进入界面

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/131314_adf94fb1_2117144.png "屏幕截图.png")

然后再安装两个插件node-red-contrib-modbus 和 node-red-contrib-postgresql 用于读传感器值和保存数据到数据库

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/131609_1f42fc66_2117144.png "屏幕截图.png")


### 3.  grafana

可以到docker仓库查看[链接](https://registry.hub.docker.com/r/grafana/grafana)

docker run -d --name=grafana -p 3000:3000 grafana/grafana

浏览器中输入树莓派IP:3000进入界面

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/133837_eadb6bd8_2117144.png "屏幕截图.png")

然后安装插件 grafana-cli plugins install dalvany-image-panel 用于显示图片

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/133941_92f6f15b_2117144.png "屏幕截图.png")


### 4.  timescaledb


可以到docker仓库查看[链接](https://registry.hub.docker.com/r/timescale/timescaledb/tags?page=1&ordering=last_updated)

拉取镜像 

docker pull timescale/timescaledb:latest-pg12

部署容器是有添加环境变量POSTGRES_PASSWORD 和密码

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/134042_0b9eaa51_2117144.png "屏幕截图.png")


### 5.  pgadmin4

可以到docker仓库查看[链接](https://registry.hub.docker.com/r/dpage/pgadmin4)

拉取镜像 

docker pull dpage/pgadmin4

部署容器是有添加环境变量PGADMIN_DEFAULT_EMAIL 和 PGADMIN_DEFAULT_PASSWORD 作为用户名和密码

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/134930_f8b59d0c_2117144.png "屏幕截图.png")

浏览器中输入树莓派IP:5000进入界面

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/135032_2d519644_2117144.png "屏幕截图.png")


### 6.  OpenCV 和jupyterlab

到docker仓库拉取一个基础python镜像

docker pull python

部署python容器，进入容器使用pip安装 OpenCV和jupyterlab

pip install opencv-python

pip install jupyterlab

pip install psycopg2

pip install matplotlib

apt install libgl1-mesa-glx


保存为镜像

部署jupyterlab

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/140354_76b5bff8_2117144.png "屏幕截图.png")

部署OpenCV

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/140432_896b2795_2117144.png "屏幕截图.png")

## 使用说明


3D打印零件

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/125043_1f02a5e8_2117144.png "屏幕截图.png")

安如图接好线路图

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/124709_a8425cbc_2117144.png "屏幕截图.png")

node-red导入文件 ： nodered程序.json

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/140914_8ecae655_2117144.png "屏幕截图.png")

grafana导入文件 ： grafana界面.json

![输入图片说明](https://images.gitee.com/uploads/images/2021/0921/122833_9078efbb_2117144.png "屏幕截图.png")




## 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


## 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
