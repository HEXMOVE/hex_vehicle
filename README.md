# hex_vehicle

**hex_vehicle** is the minial implementation to use socketcan and ROS to control HEXMOVE chassis. This demo is make as easy to read as possible and then as simple as possible, so you can modify to fit your needs, or remove the ROS part to use it as a standalone python script, or write code in other languages based on this demo.

> WARNING: This is a demo project. It is not intended to be used in real world. The code here could be buggy. This code it is not optimized for performance at all. This code does not implenment all fuctions of our product.
> 
> DO NOT DIRECTLY USE THIS CODE IN YOUR PROJECTS.
> 
>  It is provided as a reference for developers to understand how to use the HEXMOVE chassis with CAN, so you could write your own code based on this demo. 
> 
> - If you just want to use the robot, please just use the ROS package provided by HEXMOVE.
> 
> - If you do not plan to write your own code, please just use ROS package provided by HEXMOVE.
> 
> If you are really determined to use write your own code, please note that we do not provide any technical support for that except this demo and the CAN bus document.

## Pre requisites

- A linux machine supporting socketcan
- A socketcan interface, for example the can device we provide or physical can device, USB2CAN etc.
- ROS1 or ROS2 installed. Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)
- ROS socketcan package:
   - For ROS1: [socketcan_bridge](https://wiki.ros.org/socketcan_bridge) `apt install ros-<distro>-socketcan-bridge`
      > `sudo ip link set can0 type can bitrate 500000`, `sudo ip link set can0 up`, `rosrun socketcan_bridge socketcan_bridge_node`
   - For ROS2: Get [ros2-socketcan](https://github.com/autowarefoundation/ros2_socketcan/releases/tag/1.3.0) from GITHUB(v1.3.0). DO NOT INSTALL FROM APT. Foxy and humble have breaking changes. 
      > `sudo ip link set can0 type can bitrate 500000`, `sudo ip link set can0 up`, `ros2 run ros2_socketcan socketcan_bridge_node`

## Supported Platforms and Systems

Currently, only Ubuntu is supported. For architectures, only x64 and ARM are supported. There is no plan to support other architectures at the moment.

For users using other systems, you can choose to run ROS inside a Docker container. However, there will be no technical support for this.

### Verified Platforms
* [x] **X64**
* [X] **Jetson Orin Nano**
* [x] **Jetson Orin NX**
* [X] **Jetson AGX Orin**
* [ ] **Horizon RDK X5**
* [ ] **Rockchip RK3588**

---

## Public APIs

### Published Topics

| Topic      | Msg Type                | Description           |
| ---------- | ----------------------- | --------------------- |
| `/odom` | `nav_msgs/(msg/)Odometry` | The odom of the robot |
| `/hex_topic/vehicle_state` | `std_msgs/(msg/)String`  | The state of robot in JSON form. |

> In the demo, we only published part of the state of the robot. You can always refer to the datasheet to get more information. 
> 
> Or, just use the ROS package provided by us.

What's in the JSON message:

- (string) device_type        
   > name of vehicle
- (bool) brake_state
   > 0=off 1=on
- (int) work_mode
   > 0=standby 1=remote 2=CAN 3=free
- (float) battery_vol
   > (V) voltage of battery
- (string) err_state
   > Err info. Only shows one of the errors. You can improve this code to show more errors by refering to the datasheet, or use the ROS package provided by us to get more error info.



### Subscribed Topics

| Topic     | Msg Type                | Description           |
| --------- | ----------------------- | --------------------- |
| `/cmd_vel` | `geometry_msgs/(msg/)Twist` | Target speed for the robot |
| `/hex_topic/vehicle_control` | `std_msgs/(msg/)String`  | Controls vehicle in a json form. |

What can be put into the json message:

- (bool) brake_state
   > 0=off 1=on
- (int) target_work_mode
   > 0=standby 1=remote 2=CAN 3=free
- (bool) clear_err
   > 0=off 1=on, clears error status

### Parameters

| Name        | Data Type     | Description                  |
| ----------- | ------------- | ---------------------------- |
| `odom_from_speed`  | `bool` | Calculates odom from speed instead of encorder values. |

## Getting Started

Follow these steps to set up the project for development and testing on your local machine:

1. Create a workspace `catkin_ws` and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone the repository:

   ```shell
   git clone https://github.com/hexfellow/hex_python_template.git
   ```

3. Navigate back to the `catkin_ws` directory and build the workspace:

   For ROS 1:
   ```shell
   cd ../
   catkin_make
   ```

   For ROS 2:
   ```shell
   cd ../
   colcon build
   ```

4. Source the `setup.bash` file and run the tests:

   For ROS 1:
   ```shell
   source devel/setup.bash --extend
   ```

   For ROS 2:
   ```shell
   source install/setup.bash --extend
   ```



### Usage

1. Launch the `python_template` node:

   For ROS 1:
   ```shell
   roslaunch hex_python_template python_template.launch
   ```

   For ROS 2:
   ```shell
   ros2 launch hex_python_template python_template.launch.py
   ```
