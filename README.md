# Senior Design F1Tenth Gazebo Simulator

This repository contains the Gazebo simulator for the F1Tenth platform. The simulator is based on the [F1Tenth Gazebo Simulator](https://github.com/haritsahm/simulator/tree/code_refactor) but adapted to ROS2 and Cartographer.

## Installation
run
```bash
docker-compose up
```
to build the docker image and run the container. The container will start the simulator and the web interface.

## Startup
run
```bash
docker ps
docker exec -it $container_id /bin/bash
```
to enter the container.

The simulator can be controlled using the web interface. The web interface can be accessed at [`localhost:8080`](http://localhost:8080/vnc.html). The web interface allows the user to control the car and visualize the sensor data.

## Running the simulator

> Note: Ensure to source ROS 2 in every new terminal window. Run `source /opt/ros/jazzy/setup.bash` to do so.

In one terminal, run
```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="world/race_track.world"
```

Then in another terminal, run
```bash
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "urdf/model.urdf", name: "urdf_model"'
```
to spawn in the car

## Controlling the car
TODO