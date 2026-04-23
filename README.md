# ROBOCON_Volleyball_Robot
Volleyball robot for Robocon competition

我基于 ROS2 Humble 搭建了一套全向轮移动机器人控制系统。系统使用 Xbox 手柄输入，通过 teleop 节点发布速度指令，底盘控制节点完成四 omni 轮运动学解算，电机驱动节点通过 SocketCAN 控制达妙 DM-H6215 轮毂电机。同时实现了接球板俯仰控制和击球机构控制，并加入了速度限幅、加减速保护、CAN 超时保护和 launch 一键启动。
