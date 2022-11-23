# Baize_HexapodRobot

实际上这个机械结构是国外老哥做的一款arduino六足机器人。

这里我们把他改造成ros和arduino两个版本的，提供更多丰富的功能。

目前主要用arduino和ros两种软件平台来做，可以用于学习arduino或者ros编程，同时也可以将ros于arduino结合。

白泽六足机器人ROS版，软件平台基于ROS。通过ROS主从机的方式来进行机器人的监测和控制。自己的PC主机用作ROS主机，机器人机身上有一块Nanopi K2作为ROS从机。

Nanopi k2安装ubuntu16.04和ros kinetic。

在运行过程中ROS主机上运行rviz可视化上位机软件进行六足机器人运行状态的实时显示，ROS从机负责控制机器人运动。

通过track_ik运动学求解器进行正逆运动学解。

![实地测试例程](https://github.com/Allen953/Baize_HexapodRobot_ROS/blob/main/7.Photos%20%26%20Videos/Baize_HexapodRobot_ROS.gif)

里面有ROS下的机器人正逆学解例程，如下图。

![正逆学解](https://github.com/Allen953/Baize_HexapodRobot_ROS/blob/main/7.Photos%20%26%20Videos/202208041650.gif)

接线教程

按照如下图进行舵机连线

![舵机接线图](https://github.com/Allen953/Baize_HexapodRobot_ROS/commit/1ecb608f0c5ea4e534e1632581796b394cbf035f)


