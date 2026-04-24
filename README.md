# ROBOCON_Volleyball_Robot

## Volleyball robot for Robocon competition

# 排球二车: ROS2 全向轮接球机器人控制工程

我基于 ROS2 Humble 搭建了一套全向轮移动机器人控制系统。系统使用 Xbox 手柄输入，通过 teleop 节点发布速度指令，底盘控制节点完成四 omni 轮运动学解算，电机驱动节点通过 SocketCAN 控制达妙 DM-H6215 轮毂电机。同时实现了接球板俯仰控制和击球机构控制，并加入了速度限幅、加减速保护、CAN 超时保护和 launch 一键启动。

## 1. 项目简介

本工程用于控制一台基于 ROS2 Humble 的全向轮接球机器人。

机器人主要功能包括：

- 使用手柄遥控四个全向轮底盘运动
- 通过 CAN 总线控制四个达妙 DM-H6215 轮毂电机
- 控制一个 DM-J4310 电机调整接球板俯仰角度
- 控制三个 DM-J4310 电机完成击球动作

## 2. 硬件组成

- 主控：RDK X5
- 底盘电机：4 个 DM-H6215 轮毂电机
- 接球板俯仰电机：1 个 DM-J4310
- 击球电机：3 个 DM-J4310
- 输入设备：Xbox 手柄
- 通信方式：CAN 总线，SocketCAN，can0，1Mbps

## 3. 软件环境

- Ubuntu
- ROS2 Humble
- C++
- SocketCAN

## 4. 工程包说明

| 功能包 | 作用 |
|---|---|
| dm_msgs | 自定义消息 |
| base_controller | 将 /cmd_vel 转换为四个轮子的速度 |
| vobot_dm_driver | 通过 CAN 控制达妙电机 |
| pitch_controller | 根据手柄输入控制接球板俯仰 |
| hit_controller | 处理击球触发逻辑 |
| robot_bringup | 一键启动整车系统 |

## 5. 系统数据流

```text
手柄
  ↓
/joy
  ↓
teleop_twist_joy
  ↓
/cmd_vel
  ↓
base_controller
  ↓
/wheel_speeds
  ↓
vobot_dm_driver
  ↓
CAN 总线
  ↓
达妙底盘电机
```

## 6. 启动步骤
1. 启动CAN
```
sudo ip link set can0 down

sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

ip -details link show can0
```
2. 编译工程
```
cd /home/sunrise/ros2_vo_ws

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
```
3. 启动整车
```
ros2 launch robot_bringup full_stack.launch.py
```

## 7. 常用调试命令
 查看topic:
```
ros2 topic list
```
  查看手柄数据:
```
ros2 topic echo /joy
```
 查看速度指令:
```
ros2 topic echo /cmd_vel
```
 查看四轮速度:
```
ros2 topic echo /wheel_speeds
```
 手动触发击球:
```
ros2 topic pub --once /hit_trigger std_msgs/msg/Bool "{data: true}"
```

## 9. 当前状态
目前已经实现：
- 手柄遥控底盘运动
- CAN 控制四个底盘电机
- 手柄控制接球板俯仰
- 手柄按钮触发击球