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

<!-- ## Setting up the PixHawk

> Note: In this section, we are using a PixHawk 6C flight controller running PX4, connected to a VESC 6MK VI electronic speed controller. 

TODO: add configuration instructions for the PixHawk and VESC.

> Note: The remainder of these instructions assume that the PixHawk and VESC are configured such that the car can be driven and steered from a ground control station connected to the PixHawk such as [QGroundControl](https://docs.qgroundcontrol.com/master/en/).

Install mavros by following the instructions [here](https://github.com/mavlink/mavros/blob/master/mavros/README.md). -->

## Setting up the F1Tenth System

Install the F1Tenth system by following the instructions [here](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#doc-drive-workspace). If you experience issues with the command `rosdep install --from-paths src -i -y`, try simply installing the dependencies with `sudo apt-get install ros-foxy-[package_name]` for each dependency for which the rosdep definition is not found.

> Be sure to follow the `Calibrating the Odometry` tutorial before running.

## Setting up SLAM

Install slam-toolbox. In our case, the command was

```bash
sudo apt-get install ros-foxy-slam-toolbox
```

## Setting up Raceline Optimization

Clone the following repository into the base directory of your workspace:

```bash
git clone https://github.com/TUMFTM/global_racetrajectory_optimization.git
```

Make the following changes within the repository:
- in `global_racetrajectory_optimization/requirements.txt`, remove all the versions from the package names.
- replace `global_racetrajectory_optimization/params/racecar.ini` with `src/Senior-Design-Software/f1tenth_racer/config/racecar.ini`
- in `/home/cavrel/f1tenth_ws/global_racetrajectory_optimization/main_globaltraj.py`, make the following changes
    - Set `debug` to `False`
    - Set all `plot_opts` to `False`
    - Set `file_paths["track_name"]` to `hec_track`
    - Set `opt_type` to `mincurv`
    - Comment out `pkg_resources.require(dependencies)`

Then run `pip show trajectory_planning_helpers`. Take the location displayed as `TRAJ_HELP_PATH`. Navigate to `TRAJ_HELP_PATH/trajectory_planning_helpers/spline_approximation.py` and within the `dist_to_p` function, add the following line immediately before the return statement:

```python
s = np.array(s).flatten()
```

Then, run `pip install -r global_racetrajectory_optimization/requirements.txt` to install the dependencies.

Finally, in `src/Senior-Design-Software/f1tenth_racer/f1tenth_racer/nav2_intermediary.py`, change `OPTIMIZER_PATH` to the full path of the `global_racetrajectory_optimization` repository.

## Additional setup

Install tf transformations to enable the `pose_broadcaster.py` script to publish the target pose to the car during the exploration phase of the race.

```bash
sudo apt-get install ros-foxy-tf-transformations
```

In `src/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp`, change `double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;` to `double current_speed = -(-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;`

In `src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`, comment out `device_name: /dev/input/joypad-f710`.

# Running the car

Run the following commands to start the full stack (lidar + F1Tenth system + SLAM + NAV2):

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=5
ros2 launch f1tenth_racer f1tenth_sys_stacklaunch.py
# ros2 run f1tenth_racer pose_broadcaster_2
ros2 run f1tenth_racer wall_follower
```

> Note: If difficulties are encountered, try running `ros2 daemon stop` followed by `ros2 daemon start` to restart the ROS 2 daemon, while the script is not running.

> Note: The environment variables are set to enable remote communication between ROS 2 Foxy on the car and a different ROS 2 distro (e.g. Humble) running on a different Ubuntu machine on the same network. This is crucial if you want to visualize the car's topics in Rviz on a different machine while it is driving.

<!-- > Note: One of the commands in this launch file uses sudo to open the FCU serial port. You may need to enter the car's password to continue. However, it is likely that the prompt to do so will be buried under the other output. If it seems to hang, try entering the password and pressing enter again. -->

> Note: You can run the following command to serialize the pose graph:

```
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/cavrel/f1tenth_ws/src/Senior-Design-Software/saved_graph.posegraph'}"
```

```
ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/cavrel/f1tenth_ws/src/Senior-Design-Software/saved_map'}}"
```