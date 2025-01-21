CONTAINER_ID=$(docker ps | grep f1tenth_gazebo_sim | awk '{print $1}')

docker exec -it "$CONTAINER_ID" /bin/bash -c "
  source /opt/ros/jazzy/setup.bash
  cd sim_ws
  colcon build
  source install/setup.bash
  exec /bin/bash
"