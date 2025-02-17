# Senior Design F1Tenth Gazebo Simulator

This repository contains the Gazebo simulator for the F1Tenth platform. The simulator is based on the [F1Tenth Gazebo Simulator](https://github.com/haritsahm/simulator/tree/code_refactor) but adapted to ROS2 and Cartographer.

# Info about the simulator

This simulator is built using Gazebo, and was set up following the nav2 tutorial [here](https://docs.nav2.org/setup_guides/index.html). However, instead of using the Gazebo differential drive plugin, we use the ros2_controllers ackermann plugin. This is what publishes the odometry data from the wheel encoders; however, we use the gz sim [imu](https://gazebosim.org/docs/latest/sensors/).

# Setup

## Setting up the docker application
run
```bash
docker-compose up --build
```
to build the docker image and run the container. The container will start the simulator and the web interface.

The remaining commands need to each be run in their own terminal. To set up the new terminal window, run the following command:

```bash
./setup_terminal_window.sh
```

This will open a new terminal window and source the ROS 2 environment.

The simulator can be controlled using the web interface. The web interface can be accessed at [`localhost:8080`](http://localhost:8080/vnc.html). The web interface allows the user to control the car and visualize the sensor data.

## Running the simulator

In a new terminal, run
```bash
ros2 launch ros2_gazebo_sim simlaunch.py
```

> Note: To verify that the simulator is working properly, create a new terminal and run the following command after running the setup commands above:
> ```bash
> ros2 topic pub /ackermann_controller/reference geometry_msgs/msg/TwistStamped "{
>   header: {
>     stamp: { sec: 0, nanosec: 0 },
>     frame_id: 'base_link'
>   },
>   twist: {
>     linear: { x: 1.0, y: 0.0, z: 0.0 },
>     angular: { x: 0.0, y: 0.0, z: 0.0 }
>   }
> }"
> ```
> This will publish a velocity of 1 m/s to the car. If the car does not move, there may be an issue with the simulation or the controllers.

and then navigate to the web interface at [`localhost:8080`](http://localhost:8080/vnc.html) and quickly unpause the simulator (bottom left button). If you do not do this within 5 seconds, the controllers will time out waiting for the simulation to start, and you'll have to run the launch file again.

Then, run
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/ros2_gazebo_sim/config/slam_params.yaml
```

In another terminal to start the SLAM algorithm.

Finally, run
```
po
```

to start the navigation stack.

At this point, the car is ready to control! Publish `geometry_msgs/msg/TwistStamped` messages to control speed and steering.

> It may be helpful to use rviz to visualize the map and the car's path. You can start it by running `rviz2` in a new terminal.

## Controlling the car
TODO