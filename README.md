# USVControl

2022.03.02

#### 介绍

无人艇控制程序，也可控无人小车（python语言）。

#### 使用说明
​		支持遥控器和地面站控制，支持差速模型和舵模型（通过配置文件is_catamaran参数配置）。

​		遥控器接收机S.BUS2接口连接至COM50，同时COM50向外发送修改后的S.BUS2协议控制推进器或舵。中间信号转换可借助FT232模块、接收机信号转换模块，切记S.BUS协议需硬件取反。

​		组合导航目前支持维特智能品牌和瑞芬IMU560（非标准协议），连接至COM51。

​		地面站地面站可通过数传电台（COM52）或4G连接，地面站暂不公开分享。

​		遥控器通道三控制前进倒退，通道一控制转向。

​		以上参数均可通过AppData/settings.ini文件配置

​		AppData中csv格式文件为日志信息

​		仿真环境使用Airsim，车模型，代码在Airsim文件夹，相关配置教程请自行百度，navigation_type设置为airsim

​		




#### 更新记录

2021.11.26 第一版，仅支持遥控功能

2021.11.27 添加维特组合导航协议解析 COM51

2021.11.30

1. 增加PID类
2. 模式增加”锁定，手动，直线（角速度与舵量闭环）“，由通道5控制
3. 增加LOS制导方法（路点模式使用）
4. 增加与地面站通讯功能（协议解析及通讯维持） 

2021.12.1

1. RemoterControlUnit类增加记录上一周期数据
2. Control类增加地面站部分控制模式（gcs、heading、speed）、遥控器和地面站控制逻辑为遥控器为主
3. GroundControlStation类修复参数名错误和serial  read字节数BUG
4. 打印数据，供调试使用，暂时放弃共享内存方法，更改主函数名和Timer

2021.12.2

1. Add Mission class
2. Improve ground control station protocol analysis

2021.12.3

1. Add 'waypoint' 'trajectory' 'mission' control mode
2. Fix the mission upload function
3. Pack EXE

2021.12.4

1. Add the log writing function 

2021.12.9

1. Add simulation features(airsim)

2021.12.10

1. Add ini feature
2. Change the mode of communication with the airsim to non-blocking
3. The simulation environment can be used, but the ground control station data has a large delay after connection

2022.02.21

1. Supports riven(rion) IMU560 integrated navigation

2022.02.25

1. Communication with the ground station supports network port communication.
2. Simulation code optimization

2022.02.26

1. Fix rcu delay bug
1. Optimize the code
1. Set PID parameters by Settings.ini
1. Support the usv that have a propeller and a rudder

2022.03.02

1. Implement qGeoCoordinate, cancel Import PySide2
2. Simulated geographic coordinates update



#### 开发计划

1. 代码部署测试

3. 增加Ascii和Binary标准协议解析，整合重构协议内容

4. 共享内存

   

#### 实验图片

仿真环境测试

![image-20220221163002292](https://gitee.com/sttdo/picture/raw/master/img/2022/02/image-20220221163002292.png)

部署测试

![557dbde9521f06c42232a0a7b7bd5fb](https://gitee.com/sttdo/picture/raw/master/img/2022/02/557dbde9521f06c42232a0a7b7bd5fb.jpg)

路点追踪（任务模式）

![image-20220226224850116](https://gitee.com/sttdo/picture/raw/master/img/2022/02/image-20220226224850116.png)



#### 参与贡献

1. Fork 本仓库

2. 新建 Feat_xxx 分支

3. 提交代码

4. 新建 Pull Request
