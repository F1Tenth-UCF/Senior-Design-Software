# Senior Design F1Tenth Gazebo Simulator

This branch contains the code for the physical F1Tenth race vehicle.

# Setup

> Note: Follow these instructions prior to cloning this repository.

## Setting up the ROS 2 environment
Following the F1Tenth standard, we use ROS 2 Foxy Fitzroy. Begin by following the installation instructions for linux [here](https://docs.ros.org/en/foxy/Installation.html). Be sure to install the `ros-foxy-desktop` package, as you will need `rviz2` to aid in visualizing the car's sensor readings as you continue to set up the vehicle.

Now, you need to create a new ROS 2 workspace. Follow the tutorial [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to create a new workspace. In the rest of this guide, we will assume that your workspace is located at `~/f1tenth_ws`.

## Setting up the lidar

> Note: We use a Hokuyo UST-10LX lidar connected to the ethernet port of a Seed Studio reComputer J20 running [Jetson Linux 35.6.0](https://developer.nvidia.com/embedded/jetson-linux-r3560). Instructions for flashing the Jetson with that version of Jetpack can be found [here](https://wiki.seeedstudio.com/reComputer_J2021_J202_Flash_Jetpack/#enterforce-recovery-mode).

Source your ROS 2 environment, and then run the following commands to install dependencies for the lidar:

```bash
sudo apt-get update
sudo apt-get install ros-foxy-laser-proc
sudo apt-get install ros-foxy-diagnostic-updater
```

Then, follow the instructions [here](https://github.com/Hokuyo-aut/urg_node2) to install the lidar driver. Be sure to install the `urg_node2` package in the `src` directory of your workspace.

After this is installed, you will need to enter Ubuntu settings > Network and click the + icon nect to wired. In the IPV4 tab, set the method to `manual` and add the following to the Addresses table:

- Address: 192.168.0.15
- Netmask: 255.255.255.0
- Gateway: 192.168.0.1

**The Jetson must be connected to this network in order to read data from the lidar.**

TODO: add instructions for changing the transform of the lidar depending on its mounting position.

## Setting up the PixHawk

> Note: In this section, we are using a PixHawk 6C flight controller running PX4, connected to a VESC 6MK VI electronic speed controller. 

TODO: add configuration instructions for the PixHawk and VESC.

> Note: The remainder of these instructions assume that the PixHawk and VESC are configured such that the car can be driven and steered from a ground control station connected to the PixHawk such as [QGroundControl](https://docs.qgroundcontrol.com/master/en/).

Install mavros by following the instructions [here](https://github.com/mavlink/mavros/blob/master/mavros/README.md).

## Setting up SLAM

Install slam-toolbox. In our case, the command was

```bash
sudo apt-get install ros-foxy-slam-toolbox
```

# Running the car

Run each of the following commands **in their own sourced terminal window**.

## Launching the lidar

```bash
ros2 launch urg_node2 urg_node2.launch.py
```

## Launching the MAVROS bridge

```bash
sudo chmod 777 /dev/ttyTHS0
ros2 run mavros mavros_node --ros-args --param fcu_url:=/dev/ttyTHS0:921600 > mavros_out.txt 2>&1 &
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 20, on_off: 1}"
ros2 param set /mavros/imu frame_id imu
```

## Launching the car

```bash
ros2 launch f1tenth_racer simlaunch.py
```

At this point, the TF tree should contain the odom->base_link transform, as well as the base_link->imu and base_link->laser transforms.

## Running SLAM

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

## Running NAV2

```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=src/Senior-Design-Software/f1tenth_racer/config/nav2_params.yaml
```
