# Senior Design F1Tenth Gazebo Simulator

This repository contains the Gazebo simulator for the F1Tenth platform. The simulator is based on the [F1Tenth Gazebo Simulator](https://github.com/haritsahm/simulator/tree/code_refactor) but adapted to ROS2 and Cartographer.

# Setup

## Setting up the docker application
run
```bash
docker-compose up
```
to build the docker image and run the container. The container will start the simulator and the web interface.

Then, run
```bash
docker ps
docker exec -it $container_id /bin/bash
```
to enter the container.

> Note: Ensure to source ROS 2 in every new terminal window. Run `source /opt/ros/jazzy/setup.bash` to do so.

The simulator can be controlled using the web interface. The web interface can be accessed at [`localhost:8080`](http://localhost:8080/vnc.html). The web interface allows the user to control the car and visualize the sensor data.

## Building the simulator

Navigate to `sim_ws/` and run
```bash
colcon build
```

## Running the simulator

In a terminal, run
```bash
ros2 launch ros2_gazebo_sim simlaunch.py
```

and then navigate to the web interface at [`localhost:8080`](http://localhost:8080/vnc.html) and quickly unpause the simulator (bottom left button). If you do not do this within 5 seconds, the controllers will time out waiting for the simulation to start, and you'll have to run the launch file again.

At this point, the car is ready to control! Publish `geometry_msgs/msg/TwistStamped` messages to control speed and steering.

## Controlling the car
TODO