# OCS2 Quadruped Controller

This is a ros2-control controller based on [legged_control](https://github.com/qiayuanl/legged_control)
and [ocs2_ros2](https://github.com/legubiao/ocs2_ros2).

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

[![](http://i0.hdslb.com/bfs/archive/e758ce019587032449a153cf897a543443b64bba.jpg)](https://www.bilibili.com/video/BV1UcxieuEmH/)

## 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
    * joint velocity
    * joint effort
    * KP
    * KD
* state:
    * joint effort
    * joint position
    * joint velocity
    * imu sensor
        * linear acceleration
        * angular velocity
        * orientation
    * feet force sensor

## 2. Build

### 2.1 Build Dependencies

* OCS2 ROS2 Libraries
  ```bash
  colcon build --packages-up-to ocs2_legged_robot_ros
  colcon build --packages-up-to ocs2_self_collision
  ```

### 2.2 Build OCS2 Quadruped Controller

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadruped_controller
```

## 3. Launch

supported robot description:

* Unitree
    * go2_description
    * go1_description
    * a1_description
    * aliengo_description
    * b2_description
* Xiaomi
    * cyberdog_description
* DeepRobotics
    * lite3_description
    * x30_description
* Anybotics
    * anymal_c_description

### 3.1 Mujoco Simulation
> **Warm Reminder**: You need to launch [Unitree Mujoco C++ Simulation](https://github.com/legubiao/unitree_mujoco) before launch the controller.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```

At the first launch, controller may compile the OCS2 model and generate the shared library. The compilation process may take a few minutes. After the compilation, restart the controller and the robot should stand up. Then you can use the keyboard or joystick to control the robot (Keyboard 2 or Joystick LB+A to Trot mode).