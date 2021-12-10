# USVControl

2021.12.10

#### 介绍

python版船控，也可控两轮无人车，作者初学python练手。

#### 使用说明
​		支持遥控器和地面站控制，程序跑在工控机或树莓派上。

​		遥控器接收机S.BUS2接口连接COM50，同时COM50向外发送修改后的S.BUS2协议控制电机。中间信号转换可借助FT232模块、接收机信号转换模块，切记S.BUS协议需硬件取反。

​		组合导航目前仅支持维特智能品牌（非标准协议），借助TTL转串口连接在COM51。通过串口连接地面站，COM52，地面站暂不对外分享。遥控器通道三控制前进倒退，通道一控制转向。

​		以上参数均可通过AppData/settings.ini文件配置

​		AppData中csv格式文件为日志信息

​		Airsim仿真功能待完善，暂不对外分享。




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



#### 开发计划

1. 添加支持瑞芬组合导航
2. 完善Arisim仿真功能
3. 代码重构
4. 共享内存
5. 组合导航EKF


#### 实验图片

​		待添加

#### 参与贡献

1. Fork 本仓库

2. 新建 Feat_xxx 分支

3. 提交代码

4. 新建 Pull Request
