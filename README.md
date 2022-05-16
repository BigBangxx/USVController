# USVController

2022.05.16

#### 介绍

无人艇控制程序，也可控无人小车（python语言）。

#### 实验图片

仿真环境测试

![image-20220221163002292](https://gitee.com/sttdo/picture/raw/master/img/2022/02/image-20220221163002292.png)

部署测试

![73c60abc92266048fad7dac99ac41d4](https://gitee.com/sttdo/picture/raw/master/img/2022/05/73c60abc92266048fad7dac99ac41d4.jpg)

路点追踪（任务模式）

![image-20220226224850116](https://gitee.com/sttdo/picture/raw/master/img/2022/02/image-20220226224850116.png)

​		

#### 使用说明

支持遥控器和地面站控制，支持双体船模型和单体船模型。

遥控器通道三控制前进倒退，通道一控制转向。

**硬件：**航模遥控器、接收机、接收机信号转换模块和串口转USB模块（必须，连接见下图，可联系购买定制板子）、组合导航、数传电台

**硬件连接：**所有外设均使用USB连接至控制器。左（主）推进器连接接收机信号转换模块Channel3、右（舵）推进器连接收机信号转换模块Channel1。



![硬件链接](https://gitee.com/sttdo/picture/raw/master/img/2022/05/硬件链接.png)

使用*python main.py*运行船控，首次运行后增加AppData文件夹，其中settings.ini为配置文件（重启后生效）、csv格式为日志文件、xml格式为任务信息文件。

**仿真环境**

仿真环境使用Airsim，车模型，代码在Airsim文件夹，相关配置教程请自行百度，navigation_type设置为airsim



#### 开发计划

1. 定位保持
2. 编队



#### 项目合作、开发、交流，联系sangtongtong@qq.com



#### **配置文件信息**

```
[usv]
	usv_id				//USV编号
	los_distance  	 	//视线法制导视线距离
	is_catamaran   		//True为双体船模型、False为单体船模型
	heading_p   		//航向P参数
	heading_i  		 	//航向I参数
	heading_d  		 	//航向D参数
	speed_p   			//速度P参数
	speed_i  			//速度I参数
	speed_d   			//速度D参数
	position_p  		//位置P参数
	position_i   		//位置I参数
	position_d  		//位置D参数

[rcu]
	sbus_com 			//串口转USB模块连接端口号

[navigation]
	navigation_type		//使用导航的协议类型，目前支持ANPP、FDI、Rion、Wit
	navigation_com		//导航连接端口号
	navigation_baudrate //导航使用波特率
	airsim_ip 			//Airsim所在IP
	airsim_port 		//Airsim使用端口号

[gcs]
	communication_type 			// 与地面站通讯的方式，udp或serial
	gcs_com						//串口通信连接端口号
	server_ip 					//udp通讯使用透传服务器ip
	server_port					//透传服务器端口号
	gcs_disconnect_time_allow 	//地面站连接超时时间
	gcs_waypoint_err 			//路点误差
```

