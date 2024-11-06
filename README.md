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
ros2 launch ros2_gazebo_sim simlaunch.xml
```

## Controlling the car
TODO