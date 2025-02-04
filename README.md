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

After this is installed, run the following command to enter the network configuration menu:

```bash
nm-connection-editor
```

Create a new ethernet connection named 'ROS' with the following settings:

- General settings:
    - All users may connect to this network: **checked**
- Ethernet settings:
    - MTU: **automatic**
    - Wake on LAN: **Default**
    - Link negotiation: **Ignore**
- 802.1X Security settings:
    - Use 802.1X security: **unchecked**
- DCB Settings:
    - Enable DCB: **unchecked**
- Proxy Settings:
    - Method: **None**
- IPv4 Settings:
    - Method: **Manual**
    - Address: **192.168.0.15**
    - Netmask: **32**
    - Gateway: **Blank**
    - Routes:
        - Address: **192.168.0.10**
        - Netmask: **255.255.255.255**
        - Gateway: **Blank**
        - Metric: **Blank**
        - Use this connection only for resources on its network: **checked**
- IPv6 Settings:
    - Method: **Automatic**

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

Run the following commands to start the full stack (lidar + FCU + transforms + SLAM + NAV2):

```bash
sudo chmod 777 /dev/ttyTHS0
ros2 launch f1tenth_racer stacklaunch.py
```

> Note: One of the commands in this launch file uses sudo to open the FCU serial port. You may need to enter the car's password to continue. However, it is likely that the prompt to do so will be buried under the other output. If it seems to hang, try entering the password and pressing enter again.